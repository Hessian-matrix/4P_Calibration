import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFile } from 'node:fs/promises';

const workerSource = await readFile(new URL('../app/engine-worker.js', import.meta.url), 'utf8');
const PROTOCOL = 'fourp.worker.v1';
let fixtureNumber = 0;

function fixtureModule({ runBody, profile = 'cpu-single' }) {
  const shared = profile === 'cpu-rayon';
  return `
const memory = new WebAssembly.Memory({ initial: 1, maximum: 1, shared: ${shared} });
let lastEventJson = null;
let lastPixels = null;
export default async function initialize() { return { memory }; }
export function engine_version() { return '4p/test;${profile}'; }
export function validate_config() { return '{}'; }
export function inspect_source() { return '{}'; }
export async function initThreadPool() {}
export function cancellation_flag_ptr() { return 0; }
export function callbackState() {
  return { eventJson: lastEventJson, pixelByteLength: lastPixels?.byteLength ?? null, memoryByteLength: memory.buffer.byteLength };
}
export function run_calibration(config, source, emitCallback) {
  const emit = (eventJson, pixels) => {
    lastEventJson = eventJson;
    lastPixels = pixels ?? null;
    emitCallback(eventJson, pixels);
  };
  ${runBody}
}
`;
}

async function createHarness({ runBody, profile = 'cpu-single', times = [] }) {
  const posts = [];
  let clockIndex = 0;
  let currentTime = times[0] ?? 0;
  const performance = {
    timeOrigin: 1_700_000_000_000,
    now() {
      if (clockIndex < times.length) currentTime = times[clockIndex++];
      return currentTime;
    },
  };
  const context = vm.createContext({
    ArrayBuffer,
    Atomics,
    File,
    FileReaderSync: class FileReaderSync {},
    SharedArrayBuffer,
    Uint8Array,
    WebAssembly,
    performance,
    postMessage(message, transfer = []) {
      const uniqueTransfers = new Set(transfer);
      const transferKinds = Array.from(transfer, (buffer) => buffer instanceof SharedArrayBuffer ? 'shared' : buffer instanceof ArrayBuffer ? 'array' : 'other');
      const cloned = structuredClone(message, { transfer });
      posts.push({ message: cloned, transferCount: transfer.length, uniqueTransferCount: uniqueTransfers.size, transferKinds });
    },
  });

  const moduleUrl = `data:text/javascript;charset=utf-8,${encodeURIComponent(fixtureModule({ runBody, profile }))}#fixture-${fixtureNumber++}`;
  const fixture = await import(moduleUrl);
  const script = new vm.Script(workerSource, {
    filename: 'engine-worker.js',
    importModuleDynamically: vm.constants.USE_MAIN_CONTEXT_DEFAULT_LOADER,
  });
  script.runInContext(context);
  await context.onmessage({ data: {
    protocol: PROTOCOL,
    type: 'init',
    generation: 7,
    profile,
    requestedWorkers: profile === 'cpu-rayon' ? 2 : 1,
    glueUrl: moduleUrl,
    wasmBytes: new Uint8Array(),
  } });
  assert.ok(posts.some(({ message }) => message.type === 'ready'), JSON.stringify(posts));
  await context.onmessage({ data: {
    protocol: PROTOCOL, generation: 7, type: 'bind', requestId: 'bind-test',
    sourceId: 'test-source', sourceRevision: 1,
    files: [{ handle: 1, file: new File([], 'empty.bag') }],
  } });
  assert.equal(posts.at(-1).message.type, 'bound');
  posts.length = 0;

  return {
    posts,
    fixture,
    async run(runId = 'run-1') {
      await context.onmessage({ data: { protocol: PROTOCOL, generation: 7, type: 'run', runId, config: '', source: {
        sourceId: 'test-source', sourceRevision: 1, format: 'ros1_v2',
        files: [{ handle: 1, relativePath: 'empty.bag', size: '0', role: 'data' }],
      } } });
    },
  };
}

function runEventPosts(posts) {
  return posts.filter(({ message }) => message.type === 'run-events');
}

function flattenedEntries(posts) {
  return runEventPosts(posts).flatMap(({ message }) => message.events);
}

const numericEvent = (iteration) => JSON.stringify({ kind: 'solveProgress', payload: { iteration } });

test('binary preview pixels bypass JSON, retain metadata, and transfer exactly once', async () => {
  const harness = await createHarness({ runBody: `
    const eventJson = JSON.stringify({
      kind: 'detectionProgress',
      payload: {
        camera_id: 'cam0',
        frames_processed: 10,
        preview: { width: 3, height: 2, corners: [[0.5, 1.5], [2, 1]] }
      }
    });
    emit(eventJson, new Uint8Array([1, 2, 3, 4, 5, 6]));
    return JSON.stringify({ result: { ok: true }, artifacts: [] });
  ` });

  await harness.run();

  const rawCallback = harness.fixture.callbackState();
  assert.equal(Object.hasOwn(JSON.parse(rawCallback.eventJson).payload.preview, 'pixels'), false);
  assert.equal(rawCallback.pixelByteLength, 0, 'the standalone callback buffer must be detached by transfer');

  const [batch] = runEventPosts(harness.posts);
  assert.equal(batch.transferCount, 1);
  assert.equal(batch.uniqueTransferCount, 1);
  assert.deepEqual(batch.transferKinds, ['array']);
  const preview = batch.message.events[0].event.payload.preview;
  assert.deepEqual({ width: preview.width, height: preview.height, corners: preview.corners }, {
    width: 3,
    height: 2,
    corners: [[0.5, 1.5], [2, 1]],
  });
  assert.ok(preview.pixels instanceof Uint8Array);
  assert.deepEqual([...preview.pixels], [1, 2, 3, 4, 5, 6]);
  assert.deepEqual(harness.posts.map(({ message }) => message.type), ['run-events', 'run-completed']);
});

test('a WASM-memory pixel view is copied before transfer', async () => {
  const harness = await createHarness({ runBody: `
    const pixels = new Uint8Array(memory.buffer, 16, 4);
    pixels.set([9, 8, 7, 6]);
    emit(JSON.stringify({ kind: 'detectionProgress', payload: { preview: { width: 2, height: 2, corners: [] } } }), pixels);
    return JSON.stringify({ result: {}, artifacts: [] });
  ` });

  await harness.run();

  assert.equal(harness.fixture.callbackState().memoryByteLength, 65_536);
  const [batch] = runEventPosts(harness.posts);
  assert.deepEqual(batch.transferKinds, ['array']);
  assert.deepEqual([...batch.message.events[0].event.payload.preview.pixels], [9, 8, 7, 6]);
});

test('numeric telemetry flushes at the 32-event bound', async () => {
  const harness = await createHarness({ runBody: `
    for (let iteration = 0; iteration < 33; iteration += 1) emit(JSON.stringify({ kind: 'solveProgress', payload: { iteration } }));
    return JSON.stringify({ result: {}, artifacts: [] });
  ` });

  await harness.run();

  assert.deepEqual(runEventPosts(harness.posts).map(({ message }) => message.events.length), [32, 1]);
  assert.deepEqual(flattenedEntries(harness.posts).map(({ seq }) => seq), Array.from({ length: 33 }, (_, index) => String(index + 1)));
  assert.equal(harness.posts.at(-1).message.type, 'run-completed');
  assert.equal(harness.posts.at(-1).message.lastSeq, '33');
});

test('every callback checks the 50 ms boundary without relying on a timer', async () => {
  const harness = await createHarness({
    times: [0, 0, 50, 50, 50],
    runBody: `
      emit(${JSON.stringify(numericEvent(1))});
      emit(${JSON.stringify(numericEvent(2))});
      emit(${JSON.stringify(numericEvent(3))});
      return JSON.stringify({ result: {}, artifacts: [] });
    `,
  });

  await harness.run();

  assert.deepEqual(runEventPosts(harness.posts).map(({ message }) => message.events.map(({ seq }) => seq)), [['1', '2'], ['3']]);
});

test('a stage event flushes preceding numeric telemetry before its own message', async () => {
  const harness = await createHarness({ runBody: `
    emit(${JSON.stringify(numericEvent(1))});
    emit(${JSON.stringify(numericEvent(2))});
    emit(JSON.stringify({ kind: 'stage', payload: 'solve' }));
    emit(${JSON.stringify(numericEvent(3))});
    return JSON.stringify({ result: {}, artifacts: [] });
  ` });

  await harness.run();

  const batches = runEventPosts(harness.posts).map(({ message }) => message.events);
  assert.deepEqual(batches.map((entries) => entries.map(({ seq }) => seq)), [['1', '2'], ['3'], ['4']]);
  assert.equal(batches[1][0].event.kind, 'stage');
  assert.equal(batches[1][0].event.stage, 'solve');
});

test('completion flushes the final batch before publishing its last sequence', async () => {
  const harness = await createHarness({ runBody: `
    emit(${JSON.stringify(numericEvent(1))});
    emit(${JSON.stringify(numericEvent(2))});
    return JSON.stringify({ result: { complete: true }, artifacts: [] });
  ` });

  await harness.run();

  assert.deepEqual(harness.posts.map(({ message }) => message.type), ['run-events', 'run-completed']);
  assert.deepEqual(flattenedEntries(harness.posts).map(({ seq }) => seq), ['1', '2']);
  assert.equal(harness.posts[1].message.lastSeq, '2');
});

test('run errors flush the final sequence tail before the error terminal', async () => {
  const harness = await createHarness({ runBody: `
    emit(${JSON.stringify(numericEvent(1))});
    emit(${JSON.stringify(numericEvent(2))});
    throw new Error('fixture failure');
  ` });

  await harness.run();

  assert.deepEqual(harness.posts.map(({ message }) => message.type), ['run-events', 'error']);
  assert.deepEqual(flattenedEntries(harness.posts).map(({ seq }) => seq), ['1', '2']);
  assert.equal(harness.posts[1].message.error.code, 'RUN_FAILED');
});

test('cooperative cancellation flushes the final sequence tail before cancellation', async () => {
  const harness = await createHarness({
    profile: 'cpu-rayon',
    runBody: `
      emit(${JSON.stringify(numericEvent(1))});
      emit(${JSON.stringify(numericEvent(2))});
      Atomics.store(new Int32Array(memory.buffer, 0, 1), 0, 1);
      throw new Error('fixture cooperative cancellation');
    `,
  });

  await harness.run();

  assert.deepEqual(harness.posts.map(({ message }) => message.type), ['run-events', 'run-cancelled']);
  assert.deepEqual(flattenedEntries(harness.posts).map(({ seq }) => seq), ['1', '2']);
});
