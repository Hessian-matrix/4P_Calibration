import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { readFile } from 'node:fs/promises';

const workerSource = await readFile(new URL('../app/engine-worker.js', import.meta.url), 'utf8');
const PROTOCOL = 'fourp.worker.v1';
let fixtureNumber = 0;

class FakeFile {
  constructor(name, bytes, lastModified = 1) {
    this.name = name;
    this.bytes = Uint8Array.from(bytes);
    this.size = this.bytes.byteLength;
    this.lastModified = lastModified;
    this.type = 'application/octet-stream';
  }

  slice(start, end) { return { file: this, start, end }; }
}

class FakeFileReaderSync {
  readAsArrayBuffer(slice) {
    return slice.file.bytes.slice(slice.start, slice.end).buffer;
  }
}

function fixtureModule(hostName) {
  return `
const memory = new WebAssembly.Memory({ initial: 1, maximum: 2 });
let invalidations = 0;
const read = ({ handle, offset, length }) => Array.from(globalThis[${JSON.stringify(hostName)}].readRange(handle, BigInt(offset), length));
const perform = (reads = []) => reads.map(read);
export default async function initialize() { return { memory }; }
export function engine_version() { return '4p/test;cpu-single'; }
export function validate_config() { return '{}'; }
export function invalidate_source_cache() { invalidations += 1; }
export function invalidation_count() { return invalidations; }
export function inspect_source(descriptor) {
  const source = JSON.parse(descriptor);
  const samples = perform(source.testReads);
  if (source.throwAfterRead) throw new Error('fixture inspection failure');
  return JSON.stringify({ samples });
}
export function run_calibration(config, descriptor) {
  const plan = config ? JSON.parse(config) : {};
  const samples = perform(plan.reads);
  if (plan.throwAfterRead) throw new Error('fixture run failure');
  return JSON.stringify({ result: { samples }, artifacts: [] });
}
`;
}

async function createHarness() {
  const posts = [];
  let tick = 0;
  const context = vm.createContext({
    ArrayBuffer,
    Atomics,
    File: FakeFile,
    FileReaderSync: FakeFileReaderSync,
    SharedArrayBuffer,
    Uint8Array,
    WebAssembly,
    performance: { timeOrigin: 1_700_000_000_000, now: () => ++tick },
    postMessage(message, transfer = []) {
      posts.push(structuredClone(message, { transfer }));
    },
  });
  const hostName = `__fourPRangeHost${fixtureNumber}`;
  const moduleUrl = `data:text/javascript;charset=utf-8,${encodeURIComponent(fixtureModule(hostName))}#fixture-${fixtureNumber++}`;
  const fixture = await import(moduleUrl);
  new vm.Script(workerSource, {
    filename: 'engine-worker.js',
    importModuleDynamically: vm.constants.USE_MAIN_CONTEXT_DEFAULT_LOADER,
  }).runInContext(context);
  await context.onmessage({ data: {
    protocol: PROTOCOL,
    type: 'init',
    generation: 11,
    profile: 'cpu-single',
    requestedWorkers: 1,
    glueUrl: moduleUrl,
    wasmBytes: new Uint8Array(),
  } });
  assert.ok(posts.some(({ type }) => type === 'ready'), JSON.stringify(posts));
  posts.length = 0;
  globalThis[hostName] = context.fourPHost;

  const dispatch = async (message) => {
    posts.length = 0;
    await context.onmessage({ data: { protocol: PROTOCOL, generation: 11, ...message } });
    return posts;
  };
  const bind = async (sourceId, sourceRevision, entries) => {
    const result = await dispatch({
      type: 'bind',
      requestId: `bind-${sourceId}-${sourceRevision}`,
      sourceId,
      sourceRevision,
      files: entries.map(({ handle, file }) => ({ handle, file })),
    });
    assert.equal(result.at(-1)?.type, 'bound', JSON.stringify(result));
  };
  const source = (sourceId, sourceRevision, entries, extra = {}) => ({
    format: extra.format ?? 'ros1_v2',
    sourceId,
    sourceRevision,
    files: entries.map(({ handle, file, relativePath = file.name, role = 'data' }) => ({ handle, relativePath, size: String(file.size), role })),
    ...extra,
  });
  const inspect = async (descriptor, inspectionId = 'inspection-1') => {
    const result = await dispatch({ type: 'inspect', inspectionId, source: descriptor });
    return result.at(-1);
  };
  const run = async (descriptor, plan, runId = 'run-1') => {
    const result = await dispatch({ type: 'run', runId, source: descriptor, config: JSON.stringify(plan) });
    return result.at(-1);
  };
  const close = () => { delete globalThis[hostName]; };
  return { bind, close, fixture, inspect, run, source };
}

const patterned = (length, seed = 0) => Uint8Array.from({ length }, (_, index) => (index + seed) % 251);

function expected(bytes, offset, length) {
  return Array.from(bytes.slice(offset, offset + length));
}

test('small windows span boundaries and EOF while large reads always bypass cache', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const bytes = patterned(700_003);
  const entry = { handle: 1, file: new FakeFile('recording.bag', bytes) };
  await harness.bind('source-a', 1, [entry]);
  const reads = [
    { handle: 1, offset: 262_142, length: 4 },
    { handle: 1, offset: 262_142, length: 4 },
    { handle: 1, offset: bytes.length - 3, length: 3 },
    { handle: 1, offset: 400_000, length: 65_536 },
    { handle: 1, offset: 400_000, length: 65_536 },
  ];
  const terminal = await harness.run(harness.source('source-a', 1, [entry]), { reads });
  assert.equal(terminal.type, 'run-completed');
  assert.deepEqual(terminal.value.result.samples, reads.map(({ offset, length }) => expected(bytes, offset, length)));
  assert.equal(terminal.ioStats.requests, 5);
  assert.equal(terminal.ioStats.requestedBytes, 4 + 4 + 3 + 65_536 + 65_536);
  assert.equal(terminal.ioStats.cacheHits, 1);
  assert.ok(terminal.ioStats.fetchedBytes >= 3 + 65_536 * 2);
  const artifact = terminal.value.artifacts.find(({ path }) => path === 'browser-performance.json');
  assert.ok(artifact);
  const metrics = JSON.parse(artifact.text);
  assert.equal(metrics.wasmMemoryBytes, 65_536);
  assert.match(metrics.fetchedBytesSemantics, /not physical disk I\/O/);
});

test('inspect and repeated runs reuse the same source window and expose operation deltas', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const bytes = patterned(400_000);
  const entry = { handle: 7, file: new FakeFile('repeat.db3', bytes) };
  await harness.bind('repeat-source', 3, [entry]);
  const read = { handle: 7, offset: 8_192, length: 32 };
  const descriptor = harness.source('repeat-source', 3, [entry], { format: 'ros2_sqlite3', testReads: [read] });
  const inspected = await harness.inspect(descriptor);
  assert.equal(inspected.type, 'inspection-completed');
  assert.equal(inspected.ioStats.fetchedBytes, 262_144);
  const firstRun = await harness.run({ ...descriptor, previewEnabled: false }, { reads: [read] }, 'run-first');
  const secondRun = await harness.run({ ...descriptor, previewEnabled: true }, { reads: [read] }, 'run-second');
  assert.equal(firstRun.ioStats.cacheHits, 1);
  assert.equal(firstRun.ioStats.fetchedBytes, 0);
  assert.equal(secondRun.ioStats.cacheHits, 1);
  assert.equal(secondRun.ioStats.fetchedBytes, 0);
  assert.deepEqual(secondRun.value.result.samples[0], expected(bytes, read.offset, read.length));
});

test('each file owns an independent bounded cache budget', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const firstBytes = patterned(2_000_000, 3);
  const secondBytes = patterned(2_000_000, 19);
  const entries = [
    { handle: 1, file: new FakeFile('part-0.db3', firstBytes) },
    { handle: 2, file: new FakeFile('part-1.db3', secondBytes) },
  ];
  await harness.bind('multi-file', 1, entries);
  const repeated = [
    { handle: 1, offset: 0, length: 16 },
    { handle: 2, offset: 0, length: 16 },
    { handle: 1, offset: 0, length: 16 },
    { handle: 2, offset: 0, length: 16 },
  ];
  const terminal = await harness.run(harness.source('multi-file', 1, entries, { format: 'ros2_sqlite3' }), { reads: repeated });
  assert.equal(terminal.ioStats.cacheHits, 2);
  assert.equal(terminal.ioStats.fetchedBytes, 2 * 262_144);

  const spaced = [0, 300_000, 600_000, 900_000, 1_200_000, 0].map((offset) => ({ handle: 1, offset, length: 8 }));
  const eviction = await harness.run(harness.source('multi-file', 1, entries, { format: 'ros2_sqlite3' }), { reads: spaced }, 'run-eviction');
  assert.equal(eviction.ioStats.cacheHits, 1, 'only the window retained from the preceding run should hit; the fifth new window evicts the oldest');
  assert.ok(eviction.ioStats.fetchedBytes <= 5 * 262_144);
});

test('same-name reauthorization and revision changes invalidate stale bytes and reader state', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const oldBytes = patterned(300_000, 1);
  const newBytes = patterned(300_000, 101);
  const oldEntry = { handle: 9, file: new FakeFile('same.bag', oldBytes, 1) };
  const newEntry = { handle: 9, file: new FakeFile('same.bag', newBytes, 2) };
  const read = { handle: 9, offset: 1234, length: 12 };
  await harness.bind('replaceable', 1, [oldEntry]);
  const oldResult = await harness.inspect(harness.source('replaceable', 1, [oldEntry], { testReads: [read] }), 'old');
  assert.deepEqual(oldResult.value.samples[0], expected(oldBytes, read.offset, read.length));

  await harness.bind('replaceable', 1, [newEntry]);
  const replacement = await harness.inspect(harness.source('replaceable', 1, [newEntry], { testReads: [read] }), 'replacement');
  assert.deepEqual(replacement.value.samples[0], expected(newBytes, read.offset, read.length));
  assert.equal(replacement.ioStats.cacheHits, 0);
  assert.ok(replacement.ioStats.fetchedBytes > 0);

  await harness.bind('replaceable', 2, [newEntry]);
  const stale = await harness.inspect(harness.source('replaceable', 1, [newEntry], { testReads: [read] }), 'stale');
  assert.equal(stale.type, 'error');
  assert.match(stale.error.message, /identity or revision is not bound/);
  assert.ok(harness.fixture.invalidation_count() >= 3);
});

test('source errors discard cached windows before a retry', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const bytes = patterned(350_000);
  const entry = { handle: 4, file: new FakeFile('failure.mcap', bytes) };
  const descriptor = harness.source('error-source', 1, [entry]);
  const read = { handle: 4, offset: 500, length: 20 };
  await harness.bind('error-source', 1, [entry]);
  const failed = await harness.run(descriptor, { reads: [read], throwAfterRead: true }, 'failed');
  assert.equal(failed.type, 'error');
  const retried = await harness.run(descriptor, { reads: [read] }, 'retried');
  assert.equal(retried.type, 'run-completed');
  assert.equal(retried.ioStats.cacheHits, 0);
  assert.ok(retried.ioStats.fetchedBytes > 0);
});

test('missing authorization is rejected even when an engine would not read a file', async (t) => {
  const harness = await createHarness();
  t.after(harness.close);
  const result = await harness.run({}, { reads: [] });
  assert.equal(result.type, 'error');
  assert.match(result.error.message, /Invalid source authorization identity/);
});
