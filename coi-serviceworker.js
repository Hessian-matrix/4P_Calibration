const ISOLATION_WORKER_VERSION = 'fourp-coi-v2';
const ENGINE_API = 'fourp.engine.v1';
const HEX_64 = /^[0-9a-f]{64}$/;
const DECIMAL = /^(0|[1-9][0-9]*)$/;
const PATH_SEGMENT = /^[A-Za-z0-9._-]+$/;
const VERIFIED_BUILD_HEADER = 'X-FourP-Verified-Build';
const VERIFIED_ASSET_HEADER = 'X-FourP-Verified-SHA256';
const CONTENT_SECURITY_POLICY = "default-src 'none'; script-src 'self' 'wasm-unsafe-eval'; worker-src 'self' blob:; connect-src 'self' blob:; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; base-uri 'none'; form-action 'none'; object-src 'none'";

const scopeUrl = new URL(self.registration.scope);
const engineUrl = new URL('engine/', scopeUrl);
const manifests = new Map([
  [new URL('manifest.json', engineUrl).href, 'cpu-single'],
  [new URL('manifest-threads.json', engineUrl).href, 'cpu-rayon'],
]);
const validatedBuilds = new Map();
const pendingManifests = new Map();
const encoder = new TextEncoder();
const decoder = new TextDecoder('utf-8', { fatal: true });

function secureHeaders(source = undefined) {
  const headers = new Headers(source);
  headers.set('Cross-Origin-Embedder-Policy', 'require-corp');
  headers.set('Cross-Origin-Opener-Policy', 'same-origin');
  headers.set('Content-Security-Policy', CONTENT_SECURITY_POLICY);
  headers.set('X-Content-Type-Options', 'nosniff');
  headers.set('Referrer-Policy', 'no-referrer');
  headers.set('Cache-Control', 'no-store');
  return headers;
}

function rejection(status, message) {
  const headers = secureHeaders({ 'Content-Type': 'text/plain; charset=utf-8' });
  return new Response(message, { status, headers });
}

function hexBytes(value) {
  const bytes = new Uint8Array(value.length / 2);
  for (let index = 0; index < bytes.length; index += 1) {
    bytes[index] = Number.parseInt(value.slice(index * 2, index * 2 + 2), 16);
  }
  return bytes;
}

async function sha256(bytes) {
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join('');
}

function canonicalResource(resource, buildId, manifestUrl) {
  if (!resource || typeof resource !== 'object' || Array.isArray(resource)) throw new Error('manifest resource is not an object');
  const { role, url, bytes, sha256: declaredHash } = resource;
  if (typeof url !== 'string' || !url || url.includes('\\') || url.includes('%')) throw new Error('manifest resource URL is not canonical');
  const parts = url.split('/');
  if (parts.length < 2 || parts[0] !== buildId || parts.some((part) => !PATH_SEGMENT.test(part) || part === '.' || part === '..')) {
    throw new Error('manifest resource path does not match its build');
  }
  const resolved = new URL(url, manifestUrl);
  if (resolved.origin !== scopeUrl.origin || resolved.search || resolved.hash || !resolved.pathname.startsWith(engineUrl.pathname)) {
    throw new Error('manifest resource escapes the engine directory');
  }
  if (resolved.href !== new URL(parts.join('/'), manifestUrl).href) throw new Error('manifest resource URL is not canonical');
  if (typeof bytes !== 'string' || !DECIMAL.test(bytes)) throw new Error('manifest resource byte length is invalid');
  if (typeof declaredHash !== 'string' || !HEX_64.test(declaredHash)) throw new Error('manifest resource SHA-256 is invalid');
  const relative = parts.slice(1).join('/');
  const expectedRole = relative === 'four_p_calibration.js' ? 'glue' : relative === 'four_p_calibration_bg.wasm' ? 'wasm' : 'module';
  if (role !== expectedRole) throw new Error('manifest resource role does not match its canonical path');
  return { role, url, relative, href: resolved.href, bytes, sha256: declaredHash };
}

async function validateManifest(value, expectedProfile, manifestUrl) {
  if (!value || typeof value !== 'object' || Array.isArray(value)
    || value.manifestVersion !== 1
    || value.engineApi !== ENGINE_API
    || value.executionProfile !== expectedProfile
    || typeof value.releaseId !== 'string' || !value.releaseId
    || typeof value.buildId !== 'string' || !HEX_64.test(value.buildId)
    || !Array.isArray(value.resources) || value.resources.length === 0) {
    throw new Error('engine manifest schema, API, profile, or build is invalid');
  }

  const resources = value.resources.map((resource) => canonicalResource(resource, value.buildId, manifestUrl));
  const byUrl = new Map();
  for (const resource of resources) {
    if (byUrl.has(resource.href)) throw new Error('engine manifest contains a duplicate resource URL');
    byUrl.set(resource.href, resource);
  }
  if (resources.filter((resource) => resource.role === 'glue').length !== 1
    || resources.filter((resource) => resource.role === 'wasm').length !== 1) {
    throw new Error('engine manifest lacks one canonical glue and WASM resource');
  }

  const digestParts = [];
  let digestLength = 0;
  for (const resource of [...resources].sort((left, right) => left.relative < right.relative ? -1 : left.relative > right.relative ? 1 : 0)) {
    const path = encoder.encode(resource.relative);
    const hash = hexBytes(resource.sha256);
    digestParts.push(path, new Uint8Array([0]), hash);
    digestLength += path.byteLength + 1 + hash.byteLength;
  }
  const digestInput = new Uint8Array(digestLength);
  let offset = 0;
  for (const part of digestParts) {
    digestInput.set(part, offset);
    offset += part.byteLength;
  }
  if (await sha256(digestInput) !== value.buildId) throw new Error('engine manifest build digest is invalid');
  return { buildId: value.buildId, profile: expectedProfile, resources: byUrl };
}

function sameBuild(left, right) {
  if (left.resources.size !== right.resources.size) return false;
  for (const [href, resource] of left.resources) {
    const other = right.resources.get(href);
    if (!other || other.bytes !== resource.bytes || other.sha256 !== resource.sha256 || other.role !== resource.role) return false;
  }
  return true;
}

function cacheManifest(manifest) {
  const cached = validatedBuilds.get(manifest.buildId);
  if (cached && !sameBuild(cached, manifest)) throw new Error('the same build ID declared conflicting resources');
  if (cached) {
    cached.profiles.add(manifest.profile);
    return cached;
  }
  const record = { ...manifest, profiles: new Set([manifest.profile]) };
  validatedBuilds.set(record.buildId, record);
  return record;
}

async function fetchManifest(url, expectedProfile) {
  if (pendingManifests.has(url.href)) return pendingManifests.get(url.href);
  const pending = (async () => {
    const response = await fetch(url.href, { cache: 'no-store', redirect: 'error', credentials: 'same-origin' });
    if (!response.ok || response.redirected || (response.url && response.url !== url.href)) throw new Error('engine manifest fetch was rejected');
    const body = await response.arrayBuffer();
    const value = JSON.parse(decoder.decode(body));
    const manifest = cacheManifest(await validateManifest(value, expectedProfile, url));
    return { response, body, manifest };
  })();
  pendingManifests.set(url.href, pending);
  try {
    return await pending;
  } finally {
    pendingManifests.delete(url.href);
  }
}

async function findResource(url) {
  const parts = url.pathname.slice(engineUrl.pathname.length).split('/');
  const buildId = parts[0];
  if (!HEX_64.test(buildId)) return null;
  let build = validatedBuilds.get(buildId);
  if (!build) {
    await Promise.all([...manifests].map(([href, profile]) => fetchManifest(new URL(href), profile)));
    build = validatedBuilds.get(buildId);
  }
  return build?.resources.get(url.href) ?? null;
}

function verifiedResponse(response, body, verifiedHeader, verifiedValue) {
  const headers = secureHeaders(response.headers);
  headers.delete('Content-Encoding');
  headers.delete('Transfer-Encoding');
  headers.set('Content-Length', String(body.byteLength));
  headers.set(verifiedHeader, verifiedValue);
  return new Response(body, { status: response.status, statusText: response.statusText, headers });
}

async function serveEngine(request, requestUrl) {
  if (request.method !== 'GET') return rejection(405, 'Engine resources only support GET');
  if (requestUrl.search || requestUrl.hash) return rejection(404, 'Unknown engine resource');

  const expectedProfile = manifests.get(requestUrl.href);
  if (expectedProfile) {
    try {
      const { response, body, manifest } = await fetchManifest(requestUrl, expectedProfile);
      return verifiedResponse(response, body, VERIFIED_BUILD_HEADER, manifest.buildId);
    } catch {
      return rejection(502, 'Engine manifest verification failed');
    }
  }

  let resource;
  try {
    resource = await findResource(requestUrl);
  } catch {
    return rejection(502, 'Engine manifest verification failed');
  }
  if (!resource) return rejection(404, 'Unknown engine resource');

  try {
    const response = await fetch(request, { cache: 'no-store', redirect: 'error' });
    if (!response.ok || response.status !== 200 || response.redirected || (response.url && response.url !== requestUrl.href)) throw new Error('engine resource fetch was rejected');
    const body = await response.arrayBuffer();
    if (BigInt(body.byteLength) !== BigInt(resource.bytes) || await sha256(body) !== resource.sha256) throw new Error('engine resource bytes do not match the manifest');
    return verifiedResponse(response, body, VERIFIED_ASSET_HEADER, resource.sha256);
  } catch {
    return rejection(502, 'Engine resource verification failed');
  }
}

self.addEventListener('install', (event) => {
  event.waitUntil(self.skipWaiting());
});

self.addEventListener('activate', (event) => {
  event.waitUntil(self.clients.claim());
});

self.addEventListener('message', (event) => {
  if (event.data?.type === 'fourp:isolation-version' && event.ports[0]) {
    event.ports[0].postMessage({ version: ISOLATION_WORKER_VERSION });
  }
});

self.addEventListener('fetch', (event) => {
  const requestUrl = new URL(event.request.url);
  if (requestUrl.origin !== scopeUrl.origin || !requestUrl.pathname.startsWith(scopeUrl.pathname)) return;

  event.respondWith((async () => {
    if (requestUrl.pathname.startsWith(engineUrl.pathname)) return serveEngine(event.request, requestUrl);
    const response = await fetch(event.request, { cache: 'no-store' });
    return new Response(response.body, {
      status: response.status,
      statusText: response.statusText,
      headers: secureHeaders(response.headers),
    });
  })());
});
