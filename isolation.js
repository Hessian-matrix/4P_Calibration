export const ISOLATION_WORKER_VERSION = 'fourp-coi-v1';

export class IsolationBootstrapError extends Error {
  constructor(code, message, details = {}) {
    super(message);
    this.name = 'IsolationBootstrapError';
    this.code = code;
    this.details = details;
  }
}

function fail(code, message, details, cause) {
  const error = new IsolationBootstrapError(code, message, details);
  if (cause !== undefined) error.cause = cause;
  throw error;
}

function assertLocalDeployment(workerUrl, scopeUrl) {
  if (workerUrl.origin !== location.origin || scopeUrl.origin !== location.origin) {
    fail('INVALID_ISOLATION_URL', 'The isolation worker and scope must be same-origin');
  }
  if (!scopeUrl.pathname.endsWith('/') || !workerUrl.pathname.startsWith(scopeUrl.pathname)) {
    fail('INVALID_ISOLATION_SCOPE', 'The isolation worker must be inside a directory scope', {
      workerUrl: workerUrl.href,
      scopeUrl: scopeUrl.href,
    });
  }
  if (workerUrl.search || workerUrl.hash || scopeUrl.search || scopeUrl.hash) {
    fail('INVALID_ISOLATION_URL', 'The isolation worker and scope cannot contain a query or fragment');
  }
}

async function activatedWorker(registration) {
  // Prefer a newly installing generation over the currently active one.
  const candidate = registration.installing || registration.waiting || registration.active;
  if (!candidate) fail('ISOLATION_WORKER_MISSING', 'The isolation worker registration has no worker');
  if (candidate.state === 'activated') return candidate;
  if (candidate.state === 'redundant') fail('ISOLATION_WORKER_REDUNDANT', 'The isolation worker became redundant');

  await new Promise((resolve, reject) => {
    const changed = () => {
      if (candidate.state === 'activated') {
        candidate.removeEventListener('statechange', changed);
        resolve();
      } else if (candidate.state === 'redundant') {
        candidate.removeEventListener('statechange', changed);
        reject(new IsolationBootstrapError('ISOLATION_WORKER_REDUNDANT', 'The isolation worker became redundant'));
      }
    };
    candidate.addEventListener('statechange', changed);
    changed();
  });
  return candidate;
}

function workerVersion(worker) {
  return new Promise((resolve, reject) => {
    const channel = new MessageChannel();
    const timeout = setTimeout(() => {
      channel.port1.close();
      reject(new IsolationBootstrapError('ISOLATION_VERSION_TIMEOUT', 'The isolation worker did not report its version'));
    }, 3000);
    channel.port1.onmessage = (event) => {
      clearTimeout(timeout);
      channel.port1.close();
      const version = event.data?.version;
      if (typeof version !== 'string') {
        reject(new IsolationBootstrapError('INVALID_ISOLATION_VERSION', 'The isolation worker reported an invalid version'));
      } else {
        resolve(version);
      }
    };
    worker.postMessage({ type: 'fourp:isolation-version' }, [channel.port2]);
  });
}

function reloadMarker(key) {
  try {
    const storage = sessionStorage;
    storage.getItem(key);
    return {
      get: () => storage.getItem(key),
      set: (value) => storage.setItem(key, value),
      clear: () => storage.removeItem(key),
    };
  } catch (cause) {
    fail('ISOLATION_SESSION_UNAVAILABLE', 'Session storage is required to prevent isolation reload loops', {}, cause);
  }
}

/**
 * Establishes optional cross-origin isolation before engine/controller setup.
 * It resolves only when setup may continue, rejects on a permanent failure, or
 * remains pending after scheduling the one allowed reload for this worker version.
 */
export async function bootstrapCrossOriginIsolation({
  enabled = true,
  workerUrl = new URL('./coi-serviceworker.js', import.meta.url),
  scopeUrl = new URL('./', import.meta.url),
} = {}) {
  workerUrl = new URL(workerUrl, import.meta.url);
  scopeUrl = new URL(scopeUrl, import.meta.url);
  assertLocalDeployment(workerUrl, scopeUrl);
  const marker = reloadMarker(`fourp:coi-reload:${scopeUrl.pathname}`);

  if (!enabled) {
    marker.clear();
    return { status: 'disabled', isolated: crossOriginIsolated, reloaded: false };
  }
  if (!isSecureContext || !('serviceWorker' in navigator)) {
    fail('ISOLATION_UNAVAILABLE', 'Cross-origin isolation requires a secure context with Service Worker support');
  }

  let registration;
  try {
    registration = await navigator.serviceWorker.register(workerUrl, {
      scope: scopeUrl.href,
      updateViaCache: 'none',
    });
  } catch (cause) {
    fail('ISOLATION_REGISTRATION_FAILED', 'Unable to register the cross-origin isolation worker', {
      workerUrl: workerUrl.href,
      scopeUrl: scopeUrl.href,
    }, cause);
  }

  const active = await activatedWorker(registration);
  const activeVersion = await workerVersion(active);
  if (activeVersion !== ISOLATION_WORKER_VERSION) {
    fail('ISOLATION_PACKAGE_MISMATCH', 'The page and isolation worker versions do not match', {
      expected: ISOLATION_WORKER_VERSION,
      actual: activeVersion,
    });
  }

  const controller = navigator.serviceWorker.controller;
  let controllerVersion = null;
  if (controller && new URL(controller.scriptURL).href === workerUrl.href) {
    controllerVersion = await workerVersion(controller);
  }
  if (crossOriginIsolated && controllerVersion === ISOLATION_WORKER_VERSION) {
    marker.clear();
    return {
      status: 'isolated',
      isolated: true,
      reloaded: false,
      version: ISOLATION_WORKER_VERSION,
      scope: registration.scope,
    };
  }

  const attempt = JSON.stringify({ workerUrl: workerUrl.href, version: ISOLATION_WORKER_VERSION });
  if (marker.get() === attempt) {
    fail('ISOLATION_RELOAD_FAILED', 'Cross-origin isolation is still unavailable after the allowed reload', {
      controlled: controllerVersion !== null,
      isolated: crossOriginIsolated,
      expectedVersion: ISOLATION_WORKER_VERSION,
      controllerVersion,
    });
  }
  try {
    marker.set(attempt);
  } catch (cause) {
    fail('ISOLATION_SESSION_UNAVAILABLE', 'Unable to record the bounded isolation reload', {}, cause);
  }
  location.reload();
  await new Promise(() => {});
}
