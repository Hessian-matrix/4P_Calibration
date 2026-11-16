const PROTOCOL = 'fourp.worker.v1';
const MAX_READ = 16 * 1024 * 1024;
const RANGE_WINDOW_BYTES = 256 * 1024;
const MAX_CACHED_BYTES_PER_FILE = 1024 * 1024;
const LARGE_READ_BYTES = 64 * 1024;
const SQLITE_PAGE_BYTES = 4096;
const DEFAULT_PAGE_BYTES = 64 * 1024;
const INSPECTION_PROGRESS_INTERVAL_MS = 100;
const RUN_EVENT_BATCH_LIMIT = 32;
const RUN_EVENT_BATCH_INTERVAL_MS = 50;
let generation = null;
let engine = null;
let fileReader = null;
let rangeReader = null;
let wasmMemory = null;
let sourceBindings = new Map();
let cancelView = null;
let activeInspection = null;
let activeAuthorization = null;

class BoundedRangeReader {
  constructor(readFile, now) {
    this.readFile = readFile;
    this.now = now;
    this.sourceKey = null;
    this.format = null;
    this.files = null;
    this.caches = new Map();
    this.clock = 0;
    this.totals = { requests: 0, requestedBytes: 0, fetchedBytes: 0, cacheHits: 0, readMs: 0 };
  }

  activate(authorization) {
    if (this.sourceKey !== authorization.key) {
      this.caches.clear();
      this.sourceKey = authorization.key;
    }
    this.format = authorization.format;
    this.files = authorization.files;
  }

  deactivate() {
    this.files = null;
    this.format = null;
  }

  invalidate() {
    this.deactivate();
    this.sourceKey = null;
    this.caches.clear();
  }

  snapshot() { return { ...this.totals }; }

  read(handle, offset, length) {
    if (!validFileHandle(handle) || typeof offset !== 'bigint' || !Number.isSafeInteger(length) || length < 0 || length > MAX_READ) throw new RangeError('Invalid or oversized source range');
    const file = this.files?.get(handle);
    if (!file) throw new Error(`File handle ${handle} is not authorized for the active source`);
    const size = BigInt(file.size);
    if (offset < 0n || offset > size || BigInt(length) > size - offset) throw new RangeError('Source range is outside file');
    this.totals.requests += 1;
    this.totals.requestedBytes += length;
    if (length === 0) {
      this.totals.cacheHits += 1;
      return new Uint8Array();
    }
    const start = Number(offset);
    if (!Number.isSafeInteger(start)) throw new RangeError('Source offset exceeds the browser integer range');
    const pageBytes = this.format === 'ros2_sqlite3' ? SQLITE_PAGE_BYTES : DEFAULT_PAGE_BYTES;
    const windowStart = Math.floor(start / pageBytes) * pageBytes;
    const windowSpan = start + length - windowStart;
    if (length >= LARGE_READ_BYTES || windowSpan > RANGE_WINDOW_BYTES) return this.fetch(file, start, length);

    const cache = this.caches.get(handle) ?? { windows: [], bytes: 0 };
    this.caches.set(handle, cache);
    const end = start + length;
    const hit = cache.windows.find((window) => window.start <= start && end <= window.start + window.bytes.byteLength);
    if (hit) {
      hit.used = ++this.clock;
      this.totals.cacheHits += 1;
      return hit.bytes.subarray(start - hit.start, end - hit.start);
    }

    const fetchLength = Math.min(RANGE_WINDOW_BYTES, file.size - windowStart);
    const bytes = this.fetch(file, windowStart, fetchLength);
    cache.windows.push({ start: windowStart, bytes, used: ++this.clock });
    cache.bytes += bytes.byteLength;
    while (cache.bytes > MAX_CACHED_BYTES_PER_FILE) {
      let oldest = 0;
      for (let index = 1; index < cache.windows.length; index += 1) if (cache.windows[index].used < cache.windows[oldest].used) oldest = index;
      cache.bytes -= cache.windows[oldest].bytes.byteLength;
      cache.windows.splice(oldest, 1);
    }
    return bytes.subarray(start - windowStart, end - windowStart);
  }

  fetch(file, start, length) {
    const began = this.now();
    let bytes;
    try {
      bytes = new Uint8Array(this.readFile(file, start, length));
    } finally {
      this.totals.readMs += Math.max(0, this.now() - began);
    }
    if (bytes.byteLength !== length) throw new RangeError('Browser file read was short');
    this.totals.fetchedBytes += bytes.byteLength;
    return bytes;
  }
}
function send(type, payload = {}, transfer = []) { postMessage({ protocol: PROTOCOL, generation, type, ...payload }, transfer); }
function parseJson(value, label) {
  if (typeof value !== 'string') throw new TypeError(`${label} did not return a JSON string`);
  try { return JSON.parse(value); } catch (cause) { throw new Error(`${label} returned invalid JSON: ${cause.message}`); }
}
function serializeError(error, code = 'RUN_FAILED') {
  return { code, message: error instanceof Error ? error.message : String(error), details: error?.details ?? null, retryable: false };
}
function createRunDelivery(runId) {
  return { runId, nextSeq: 0n, pending: [], transfers: [], transferSet: new Set(), lastSentAt: performance.now() };
}

function flushRunEvents(state, now = performance.now()) {
  if (!state || state.pending.length === 0) return;
  const events = state.pending;
  const transfers = state.transfers;
  state.pending = [];
  state.transfers = [];
  state.transferSet = new Set();
  state.lastSentAt = now;
  send('run-events', { runId: state.runId, events }, transfers);
}

function attachPreviewPixels(event, pixels) {
  const preview = event?.kind === 'detectionProgress' ? event.payload?.preview : null;
  if (pixels == null) {
    if (preview && Object.hasOwn(preview, 'pixels')) throw new TypeError('Preview pixels must not be encoded in run event JSON');
    if (preview) throw new TypeError('Preview metadata is missing its pixel callback argument');
    return null;
  }
  if (!(pixels instanceof Uint8Array) || !preview || Object.hasOwn(preview, 'pixels')) throw new TypeError('Invalid preview pixel callback argument');
  const width = Number(preview.width), height = Number(preview.height);
  if (!Number.isSafeInteger(width) || !Number.isSafeInteger(height) || width <= 0 || height <= 0 || pixels.byteLength !== width * height) throw new RangeError('Preview pixel length does not match its metadata');
  const shared = typeof SharedArrayBuffer !== 'undefined' && pixels.buffer instanceof SharedArrayBuffer;
  let owned = pixels;
  if (shared || pixels.buffer === wasmMemory?.buffer) {
    owned = new Uint8Array(pixels.byteLength);
    owned.set(pixels);
  }
  if (!(owned.buffer instanceof ArrayBuffer)) throw new TypeError('Preview pixels do not have a transferable ArrayBuffer');
  preview.pixels = owned;
  return owned.buffer;
}

function enqueueRunEvent(state, event, pixels) {
  const now = performance.now();
  if (event.kind === 'stage') {
    flushRunEvents(state, now);
    const buffer = attachPreviewPixels(event, pixels);
    state.nextSeq += 1n;
    send('run-events', { runId: state.runId, events: [{ seq: String(state.nextSeq), event }] }, buffer ? [buffer] : []);
    state.lastSentAt = now;
    return;
  }
  const buffer = attachPreviewPixels(event, pixels);
  state.nextSeq += 1n;
  state.pending.push({ seq: String(state.nextSeq), event });
  if (buffer && !state.transferSet.has(buffer)) {
    state.transferSet.add(buffer);
    state.transfers.push(buffer);
  }
  if (state.pending.length >= RUN_EVENT_BATCH_LIMIT || now - state.lastSentAt >= RUN_EVENT_BATCH_INTERVAL_MS) flushRunEvents(state, now);
}


function emitInspectionProgress(state, force = false) {
  if (!state || activeInspection !== state) return;
  const now = performance.now();
  const changed = state.bytesRead !== state.sentBytesRead || state.readCalls !== state.sentReadCalls;
  if (!changed || (!force && now - state.lastSentAt < INSPECTION_PROGRESS_INTERVAL_MS)) return;
  state.lastSentAt = now;
  state.sentBytesRead = state.bytesRead;
  state.sentReadCalls = state.readCalls;
  send('inspection-progress', { inspectionId: state.inspectionId, bytesRead: state.bytesRead, readCalls: state.readCalls });
}


function validFileHandle(value) {
  return Number.isSafeInteger(value) && value >= 0 && value <= 0xffff_ffff;
}

function bindingKey(sourceId, sourceRevision) {
  if (typeof sourceId !== 'string' || !sourceId || !Number.isSafeInteger(sourceRevision) || sourceRevision <= 0) throw new TypeError('Invalid source authorization identity');
  return JSON.stringify([sourceId, sourceRevision]);
}

function resolveAuthorization(source) {
  if (!source || typeof source !== 'object') throw new TypeError('Missing source descriptor');
  const key = bindingKey(source.sourceId, source.sourceRevision);
  const binding = sourceBindings.get(key);
  if (!binding) throw new Error('Source identity or revision is not bound');
  if (!Array.isArray(source.files) || source.files.length !== binding.files.size) throw new Error('Source descriptor does not match its authorization');
  const handles = new Set();
  for (const descriptor of source.files) {
    if (!descriptor || !validFileHandle(descriptor.handle) || handles.has(descriptor.handle)) throw new TypeError('Invalid source file descriptor');
    handles.add(descriptor.handle);
    const file = binding.files.get(descriptor.handle);
    if (!file) throw new Error(`File handle ${descriptor.handle} is not bound to this source`);
    if (typeof descriptor.size !== 'string' || !/^(?:0|[1-9]\d*)$/.test(descriptor.size)) throw new TypeError('Invalid source file size');
    const descriptorSize = BigInt(descriptor.size);
    if (descriptorSize !== BigInt(file.size)) throw new Error(`Authorized file size changed for handle ${descriptor.handle}`);
  }
  return { key, sourceId: source.sourceId, sourceRevision: source.sourceRevision, format: source.format, files: binding.files };
}

function beginSourceOperation(source) {
  const start = rangeReader.snapshot();
  const authorization = resolveAuthorization(source);
  rangeReader.activate(authorization);
  activeAuthorization = authorization;
  return { start, authorization };
}

function finishSourceOperation(operation) {
  const end = rangeReader.snapshot();
  rangeReader.deactivate();
  activeAuthorization = null;
  const difference = (name) => end[name] - operation.start[name];
  return {
    requests: difference('requests'),
    requestedBytes: difference('requestedBytes'),
    fetchedBytes: difference('fetchedBytes'),
    cacheHits: difference('cacheHits'),
    readMs: difference('readMs'),
    windowBytes: RANGE_WINDOW_BYTES,
    maxCachedBytesPerFile: MAX_CACHED_BYTES_PER_FILE,
    largeReadBytes: LARGE_READ_BYTES,
    sourceId: operation.authorization?.sourceId ?? null,
    sourceRevision: operation.authorization?.sourceRevision ?? null,
  };
}

function invalidateSourceCaches(invalidateEngine = true) {
  activeAuthorization = null;
  rangeReader?.invalidate();
  if (invalidateEngine && typeof engine?.invalidate_source_cache === 'function') {
    try { engine.invalidate_source_cache(); } catch { /* a failed engine is discarded by the controller */ }
  }
}

function browserPerformance(ioStats) {
  return {
    ...ioStats,
    wasmMemoryBytes: wasmMemory?.buffer?.byteLength ?? null,
    fetchedBytesSemantics: 'FileReaderSync bytes returned to the range cache; not physical disk I/O',
  };
}

function attachBrowserPerformance(value, ioStats) {
  if (!value || !Array.isArray(value.artifacts)) throw new TypeError('run_calibration result is missing its artifacts array');
  const performance = browserPerformance(ioStats);
  value.ioStats = performance;
  value.artifacts.push({ path: 'browser-performance.json', mediaType: 'application/json', text: `${JSON.stringify(performance, null, 2)}\n` });
  return performance;
}

function installHost() {
  fileReader = new FileReaderSync();
  rangeReader = new BoundedRangeReader(
    (file, start, length) => fileReader.readAsArrayBuffer(file.slice(start, start + length)),
    () => performance.now(),
  );
  globalThis.fourPHost = Object.freeze({
    readRange(handle, offset, length) {
      const bytes = rangeReader.read(handle, offset, length);
      if (activeInspection) {
        activeInspection.bytesRead += bytes.byteLength;
        activeInspection.readCalls += 1;
        emitInspectionProgress(activeInspection);
      }
      return bytes;
    },
    fileLength(handle) {
      const file = activeAuthorization?.files.get(handle);
      if (!file) throw new Error(`File handle ${handle} is not authorized for the active source`);
      return BigInt(file.size);
    },
    nowMillis() { return performance.timeOrigin + performance.now(); },
    workerCount() { return currentWorkerCount; },
  });
}

let currentWorkerCount = 1;

async function initialize(message) {
  generation = message.generation;
  sourceBindings = new Map();
  activeAuthorization = null;
  installHost();
  send('init-stage', { stage: 'compile', message: '编译与实例化 WASM…' });
  const module = await import(message.glueUrl);
  const required = ['default', 'engine_version', 'validate_config', 'inspect_source', 'run_calibration'];
  for (const name of required) if (typeof module[name] !== 'function') throw new Error(`Engine export missing: ${name}`);
  if (message.profile === 'cpu-rayon' && (typeof module.initThreadPool !== 'function' || typeof module.cancellation_flag_ptr !== 'function')) throw new Error('Thread engine cancellation exports missing');
  const wasm = await module.default(message.wasmBytes);
  wasmMemory = wasm.memory ?? null;
  engine = module;
  if (message.profile === 'cpu-rayon') { send('init-stage', { stage: 'pool', message: `初始化 ${message.requestedWorkers} 个 Rayon Worker…`, ready: 0, planned: message.requestedWorkers }); await module.initThreadPool(message.requestedWorkers); currentWorkerCount = message.requestedWorkers; }
  let cancelMemory = null, cancelPointer = null;
  if (message.profile === 'cpu-rayon') { cancelMemory = wasm.memory?.buffer; cancelPointer = Number(module.cancellation_flag_ptr()); if (!(cancelMemory instanceof SharedArrayBuffer) || !Number.isSafeInteger(cancelPointer) || cancelPointer < 0 || cancelPointer % 4) throw new Error('Thread engine did not expose an aligned shared cancellation flag'); cancelView = new Int32Array(cancelMemory, cancelPointer, 1); Atomics.store(cancelView, 0, 0); }
  const version = String(engine.engine_version());
  if (!version.endsWith(message.profile)) throw new Error(`Engine profile mismatch: expected ${message.profile}, got ${version}`);
  send('init-stage', { stage: 'pool', message: `计算 Worker 已就绪 · ${currentWorkerCount} 个工作线程`, ready: currentWorkerCount, planned: message.requestedWorkers });
  send('ready', { version, cancelMemory, cancelPointer, capabilities: { profile: message.profile, threads: currentWorkerCount, cooperativeCancel: message.profile === 'cpu-rayon' } });
}

function bindFiles(message) {
  const key = bindingKey(message.sourceId, message.sourceRevision);
  if (!Array.isArray(message.files) || message.files.length === 0) throw new TypeError('Invalid bound file collection');
  const files = new Map();
  for (const item of message.files) {
    if (!validFileHandle(item?.handle) || !(item.file instanceof File) || !Number.isSafeInteger(item.file.size) || item.file.size < 0 || files.has(item.handle)) throw new TypeError('Invalid bound file');
    files.set(item.handle, item.file);
  }
  invalidateSourceCaches();
  for (const [existingKey, binding] of sourceBindings) if (binding.sourceId === message.sourceId) sourceBindings.delete(existingKey);
  sourceBindings.set(key, { sourceId: message.sourceId, sourceRevision: message.sourceRevision, files });
  send('bound', { requestId: message.requestId, sourceId: message.sourceId, sourceRevision: message.sourceRevision });
}

onmessage = async ({ data: message }) => {
  if (!message || message.protocol !== PROTOCOL) return;
  if (message.type !== 'init' && message.generation !== generation) return;
  let runDelivery = null;
  let sourceOperation = null;
  try {
    if (message.type === 'init') return await initialize(message);
    if (!engine) throw new Error('Engine is not initialized');
    if (message.type === 'bind') return bindFiles(message);
    if (message.type === 'validate') {
      const value = parseJson(engine.validate_config(message.config), 'validate_config');
      return send('validated', { requestId: message.requestId, value });
    }
    if (message.type === 'inspect') {
      sourceOperation = beginSourceOperation(message.source);
      const inspection = { inspectionId: message.inspectionId, bytesRead: 0, readCalls: 0, sentBytesRead: 0, sentReadCalls: 0, lastSentAt: -Infinity };
      activeInspection = inspection;
      let value;
      let ioStats;
      try {
        value = parseJson(engine.inspect_source(JSON.stringify(message.source)), 'inspect_source');
      } finally {
        emitInspectionProgress(inspection, true);
        if (activeInspection === inspection) activeInspection = null;
        ioStats = finishSourceOperation(sourceOperation);
        sourceOperation = null;
      }
      return send('inspection-completed', { inspectionId: message.inspectionId, value, ioStats: browserPerformance(ioStats) });
    }
    if (message.type === 'run') {
      sourceOperation = beginSourceOperation(message.source);
      runDelivery = createRunDelivery(message.runId);
      let output;
      let ioStats;
      try {
        output = engine.run_calibration(message.config, JSON.stringify(message.source), (eventJson, previewPixels) => {
          const event = parseJson(eventJson, 'run event');
          if (event.kind === 'stage') event.stage = event.payload;
          enqueueRunEvent(runDelivery, event, previewPixels);
        });
      } finally {
        ioStats = finishSourceOperation(sourceOperation);
        sourceOperation = null;
      }
      flushRunEvents(runDelivery);
      const value = parseJson(output, 'run_calibration');
      const performance = attachBrowserPerformance(value, ioStats);
      return send('run-completed', { runId: message.runId, value, ioStats: performance, lastSeq: String(runDelivery.nextSeq) });
    }
  } catch (error) {
    if (sourceOperation) {
      try { finishSourceOperation(sourceOperation); } catch { /* preserve the operation failure */ }
    }
    const trapped = error instanceof WebAssembly.RuntimeError;
    if (message.type === 'inspect' || message.type === 'run') invalidateSourceCaches(!trapped);
    // Synchronous WASM prevents timer callbacks; all queued telemetry must be posted before
    // any error/cancellation terminal. Worker message ordering then preserves the sequence tail.
    try { flushRunEvents(runDelivery); } catch (flushError) { error = flushError; }
    if (message.type === 'init') return send('init-failed', { error: serializeError(error, 'ENGINE_NOT_READY') });
    if (trapped) return send('error', { requestId: message.requestId, inspectionId: message.inspectionId, runId: message.runId, operation: message.type, error: serializeError(error, 'ENGINE_TRAP') });
    if (message.type === 'run' && cancelView && Atomics.load(cancelView, 0) !== 0) return send('run-cancelled', { runId: message.runId });
    const requestId = message.requestId;
    const code = message.type === 'validate' ? 'INVALID_REQUEST' : 'RUN_FAILED';
    send('error', { requestId, inspectionId: message.inspectionId, runId: message.runId, operation: message.type, error: serializeError(error, code) });
  }
};
