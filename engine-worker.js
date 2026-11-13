const PROTOCOL = 'fourp.worker.v1';
const MAX_READ = 16 * 1024 * 1024;
const INSPECTION_PROGRESS_INTERVAL_MS = 100;
let generation = null;
let engine = null;
let reader = null;
let filesByHandle = new Map();
let cancelView = null;
let activeInspection = null;
function send(type, payload = {}, transfer = []) { postMessage({ protocol: PROTOCOL, generation, type, ...payload }, transfer); }
function parseJson(value, label) {
  if (typeof value !== 'string') throw new TypeError(`${label} did not return a JSON string`);
  try { return JSON.parse(value); } catch (cause) { throw new Error(`${label} returned invalid JSON: ${cause.message}`); }
}
function serializeError(error, code = 'RUN_FAILED') {
  return { code, message: error instanceof Error ? error.message : String(error), details: error?.details ?? null, retryable: false };
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


function installHost() {
  reader = new FileReaderSync();
  globalThis.fourPHost = Object.freeze({
    readRange(handle, offset, length) {
      if (!Number.isInteger(handle) || typeof offset !== 'bigint' || !Number.isSafeInteger(length) || length < 0 || length > MAX_READ) throw new RangeError('Invalid or oversized source range');
      const file = filesByHandle.get(handle);
      if (!file) throw new Error(`Unknown file handle ${handle}`);
      const size = BigInt(file.size);
      if (offset < 0n || offset > size || BigInt(length) > size - offset) throw new RangeError('Source range is outside file');
      const start = Number(offset);
      const bytes = new Uint8Array(reader.readAsArrayBuffer(file.slice(start, start + length)));
      if (activeInspection) {
        activeInspection.bytesRead += bytes.byteLength;
        activeInspection.readCalls += 1;
        emitInspectionProgress(activeInspection);
      }
      return bytes;
    },
    fileLength(handle) {
      const file = filesByHandle.get(handle);
      if (!file) throw new Error(`Unknown file handle ${handle}`);
      return BigInt(file.size);
    },
    nowMillis() { return performance.timeOrigin + performance.now(); },
    workerCount() { return currentWorkerCount; },
  });
}

let currentWorkerCount = 1;

async function initialize(message) {
  generation = message.generation;
  installHost();
  send('init-stage', { stage: 'compile', message: '编译与实例化 WASM…' });
  const module = await import(message.glueUrl);
  const required = ['default', 'engine_version', 'validate_config', 'inspect_source', 'run_calibration'];
  for (const name of required) if (typeof module[name] !== 'function') throw new Error(`Engine export missing: ${name}`);
  if (message.profile === 'cpu-rayon' && (typeof module.initThreadPool !== 'function' || typeof module.cancellation_flag_ptr !== 'function')) throw new Error('Thread engine cancellation exports missing');
  const wasm = await module.default(message.wasmBytes);
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
  for (const item of message.files) {
    if (!Number.isInteger(item.handle) || !(item.file instanceof File)) throw new TypeError('Invalid bound file');
    filesByHandle.set(item.handle, item.file);
  }
  send('bound', { requestId: message.requestId, sourceId: message.sourceId, sourceRevision: message.sourceRevision });
}

onmessage = async ({ data: message }) => {
  if (!message || message.protocol !== PROTOCOL) return;
  if (message.type !== 'init' && message.generation !== generation) return;
  try {
    if (message.type === 'init') return await initialize(message);
    if (!engine) throw new Error('Engine is not initialized');
    if (message.type === 'bind') return bindFiles(message);
    if (message.type === 'validate') {
      const value = parseJson(engine.validate_config(message.config), 'validate_config');
      return send('validated', { requestId: message.requestId, value });
    }
    if (message.type === 'inspect') {
      const inspection = { inspectionId: message.inspectionId, bytesRead: 0, readCalls: 0, sentBytesRead: 0, sentReadCalls: 0, lastSentAt: -Infinity };
      activeInspection = inspection;
      let value;
      try {
        value = parseJson(engine.inspect_source(JSON.stringify(message.source)), 'inspect_source');
      } finally {
        emitInspectionProgress(inspection, true);
        if (activeInspection === inspection) activeInspection = null;
      }
      return send('inspection-completed', { inspectionId: message.inspectionId, value });
    }
    if (message.type === 'run') {
      let eventSeq = 0n;
      const value = parseJson(engine.run_calibration(message.config, JSON.stringify(message.source), (eventJson) => {
        const event = parseJson(eventJson, 'run event');
        if (event.kind === 'stage') event.stage = event.payload;
        eventSeq += 1n;
        send('run-event', { runId: message.runId, seq: String(eventSeq), event });
      }), 'run_calibration');
      return send('run-completed', { runId: message.runId, value, lastSeq: String(eventSeq) });
    }
  } catch (error) {
    if (message.type === 'init') return send('init-failed', { error: serializeError(error, 'ENGINE_NOT_READY') });
    if (error instanceof WebAssembly.RuntimeError) return send('error', { requestId: message.requestId, inspectionId: message.inspectionId, runId: message.runId, operation: message.type, error: serializeError(error, 'ENGINE_TRAP') });
    if (message.type === 'run' && cancelView && Atomics.load(cancelView, 0) !== 0) return send('run-cancelled', { runId: message.runId });
    const requestId = message.requestId;
    const code = message.type === 'validate' ? 'INVALID_REQUEST' : 'RUN_FAILED';
    send('error', { requestId, inspectionId: message.inspectionId, runId: message.runId, operation: message.type, error: serializeError(error, code) });
  }
};
