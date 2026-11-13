const ISOLATION_WORKER_VERSION = 'fourp-coi-v1';

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
  const scopeUrl = new URL(self.registration.scope);
  if (requestUrl.origin !== scopeUrl.origin || !requestUrl.pathname.startsWith(scopeUrl.pathname)) return;

  event.respondWith((async () => {
    // The worker owns no CacheStorage cache. Bypass the HTTP cache too, so a
    // newly packaged shell and its content-addressed engine cannot be mixed.
    const response = await fetch(event.request, { cache: 'no-store' });
    if (response.type === 'opaque' || response.type === 'opaqueredirect') return response;

    const headers = new Headers(response.headers);
    headers.set('Cross-Origin-Embedder-Policy', 'require-corp');
    headers.set('Cross-Origin-Opener-Policy', 'same-origin');
    return new Response(response.body, {
      status: response.status,
      statusText: response.statusText,
      headers,
    });
  })());
});
