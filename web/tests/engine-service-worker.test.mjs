import test from 'node:test';
import assert from 'node:assert/strict';
import vm from 'node:vm';
import { createHash, webcrypto } from 'node:crypto';
import { readFile } from 'node:fs/promises';

const workerSource = await readFile(new URL('../app/coi-serviceworker.js', import.meta.url), 'utf8');
const origin = 'https://calibration.test';
const scope = `${origin}/tools/4p/`;
const engine = `${scope}engine/`;
const CSP = "default-src 'none'; script-src 'self' 'wasm-unsafe-eval'; worker-src 'self' blob:; connect-src 'self' blob:; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; base-uri 'none'; form-action 'none'; object-src 'none'";

function sha256(bytes) {
  return createHash('sha256').update(bytes).digest('hex');
}

function manifest(profile, files) {
  const described = files.map(({ relative, role, bytes }) => ({ relative, role, bytes, sha256: sha256(bytes) }));
  const digest = createHash('sha256');
  for (const file of [...described].sort((left, right) => left.relative < right.relative ? -1 : left.relative > right.relative ? 1 : 0)) {
    digest.update(file.relative);
    digest.update(Buffer.from([0]));
    digest.update(Buffer.from(file.sha256, 'hex'));
  }
  const buildId = digest.digest('hex');
  return {
    value: {
      manifestVersion: 1,
      releaseId: 'test-release',
      buildId,
      engineApi: 'fourp.engine.v1',
      executionProfile: profile,
      resources: described.map((file) => ({
        role: file.role,
        url: `${buildId}/${file.relative}`,
        bytes: String(file.bytes.byteLength),
        sha256: file.sha256,
      })),
    },
    files: described,
  };
}

function fixture() {
  const single = manifest('cpu-single', [
    { relative: 'four_p_calibration.js', role: 'glue', bytes: Buffer.from('export default 1;') },
    { relative: 'four_p_calibration_bg.wasm', role: 'wasm', bytes: Buffer.from([0, 97, 115, 109, 1]) },
  ]);
  const rayon = manifest('cpu-rayon', [
    { relative: 'four_p_calibration.js', role: 'glue', bytes: Buffer.from('import "./snippets/rayon/worker.js";') },
    { relative: 'four_p_calibration_bg.wasm', role: 'wasm', bytes: Buffer.from([0, 97, 115, 109, 2]) },
    { relative: 'snippets/rayon/worker.js', role: 'module', bytes: Buffer.from('export const worker = true;') },
  ]);
  const routes = new Map();
  const addManifest = (name, record) => {
    routes.set(`${engine}${name}`, {
      bytes: Buffer.from(JSON.stringify(record.value)),
      headers: { 'Content-Type': 'application/json' },
    });
    for (const [index, resource] of record.value.resources.entries()) {
      routes.set(`${engine}${resource.url}`, {
        bytes: record.files[index].bytes,
        headers: { 'Content-Type': resource.role === 'wasm' ? 'application/wasm' : 'text/javascript' },
      });
    }
  };
  addManifest('manifest.json', single);
  addManifest('manifest-threads.json', rayon);
  return { routes, single, rayon };
}

function createHarness(routes) {
  const listeners = new Map();
  const calls = [];
  const self = {
    registration: { scope },
    clients: { claim: async () => undefined },
    skipWaiting: async () => undefined,
    addEventListener(type, listener) { listeners.set(type, listener); },
  };
  const fetch = async (input) => {
    const href = typeof input === 'string' ? input : input.url;
    calls.push(href);
    const route = routes.get(href);
    if (!route) return new Response('missing fixture', { status: 404, headers: { 'Content-Type': 'text/plain' } });
    return new Response(route.bytes, { status: route.status ?? 200, headers: route.headers });
  };
  const context = vm.createContext({
    self,
    fetch,
    crypto: webcrypto,
    URL,
    Request,
    Response,
    Headers,
    TextEncoder,
    TextDecoder,
    Uint8Array,
    ArrayBuffer,
    BigInt,
  });
  new vm.Script(workerSource, { filename: 'coi-serviceworker.js' }).runInContext(context);

  return {
    calls,
    async request(href, init = undefined) {
      let response;
      listeners.get('fetch')({
        request: new Request(href, init),
        respondWith(value) { response = Promise.resolve(value); },
      });
      assert.ok(response, `service worker did not handle ${href}`);
      return response;
    },
  };
}

test('rejects tampered bytes on a second JavaScript fetch at the service-worker boundary', async () => {
  const { routes, single } = fixture();
  const glue = single.value.resources.find((resource) => resource.role === 'glue');
  const href = `${engine}${glue.url}`;
  const expected = routes.get(href).bytes;
  const harness = createHarness(routes);

  const verified = await harness.request(href);
  assert.equal(verified.status, 200);
  assert.deepEqual(Buffer.from(await verified.arrayBuffer()), expected);

  routes.get(href).bytes = Buffer.from('export default 2;');
  const tampered = await harness.request(href);

  assert.equal(tampered.status, 502);
  assert.equal(tampered.headers.get('X-Content-Type-Options'), 'nosniff');
  assert.match(await tampered.text(), /verification failed/i);
  assert.equal(harness.calls.filter((url) => url === href).length, 2);
});

test('rejects undeclared engine resources without fetching them', async () => {
  const { routes, single } = fixture();
  const unknown = `${engine}${single.value.buildId}/not-declared.js`;
  const harness = createHarness(routes);

  const response = await harness.request(unknown);

  assert.equal(response.status, 404);
  assert.equal(await response.text(), 'Unknown engine resource');
  assert.equal(harness.calls.includes(unknown), false);
  assert.equal(harness.calls.includes(`${engine}manifest.json`), true);
  assert.equal(harness.calls.includes(`${engine}manifest-threads.json`), true);
});

test('returns the exact verified bytes with isolation and content headers', async () => {
  const { routes, rayon } = fixture();
  const helper = rayon.value.resources.find((resource) => resource.role === 'module');
  const expected = routes.get(`${engine}${helper.url}`).bytes;
  const harness = createHarness(routes);

  const response = await harness.request(`${engine}${helper.url}`);
  const actual = Buffer.from(await response.arrayBuffer());

  assert.equal(response.status, 200);
  assert.deepEqual(actual, expected);
  assert.equal(response.headers.get('X-FourP-Verified-SHA256'), helper.sha256);
  assert.equal(response.headers.get('Content-Security-Policy'), CSP);
  assert.equal(response.headers.get('Cross-Origin-Embedder-Policy'), 'require-corp');
  assert.equal(response.headers.get('Cross-Origin-Opener-Policy'), 'same-origin');
  assert.equal(response.headers.get('X-Content-Type-Options'), 'nosniff');
  assert.equal(response.headers.get('Referrer-Policy'), 'no-referrer');
});
