
function fail(message, details) {
  const error = new Error(message);
  error.details = details;
  throw error;
}

async function sha256(bytes) {
  const digest = await crypto.subtle.digest('SHA-256', bytes);
  return [...new Uint8Array(digest)].map((value) => value.toString(16).padStart(2, '0')).join('');
}

async function fetchResource(resource, manifestUrl, signal, onChunk) {
  const url = new URL(resource.url, manifestUrl);
  const manifestDirectory = new URL('.', manifestUrl);
  if (url.origin !== location.origin || !url.pathname.startsWith(manifestDirectory.pathname) || url.search || url.hash || resource.url.includes('\\')) fail('引擎 manifest 包含越界或非静态资源', { url: url.href });
  const response = await fetch(url, { signal, cache: 'no-cache' });
  if (!response.ok || !response.body) fail(`无法下载 ${resource.url}`, { status: response.status });
  const headerLength = response.headers.get('Content-Length');
  const encoding = response.headers.get('Content-Encoding');
  const trustedLength = (!encoding || encoding === 'identity') && headerLength && /^\d+$/.test(headerLength) && Number(headerLength) === Number(resource.bytes) ? Number(headerLength) : null;
  const reader = response.body.getReader();
  const chunks = [];
  let received = 0;
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    received += value.byteLength;
    onChunk(value.byteLength, trustedLength);
  }
  const bytes = new Uint8Array(received);
  let offset = 0;
  for (const chunk of chunks) { bytes.set(chunk, offset); offset += chunk.byteLength; }
  if (resource.bytes != null && BigInt(received) !== BigInt(resource.bytes)) fail(`资源长度不符: ${resource.url}`, { expected: String(resource.bytes), actual: String(received) });
  if (resource.sha256) {
    const actual = await sha256(bytes);
    if (actual !== resource.sha256.toLowerCase()) fail(`资源哈希不符: ${resource.url}`, { expected: resource.sha256, actual });
  }
  return bytes;
}

export async function loadEngineResources({ profile = 'cpu-single', signal, onState }) {
  const manifestUrl = new URL(profile === 'cpu-rayon' ? './engine/manifest-threads.json' : './engine/manifest.json', import.meta.url);
  onState({ stage: 'manifest', message: '获取发布 manifest…', determinate: false });
  const response = await fetch(manifestUrl, { signal, cache: 'no-cache' });
  if (!response.ok) fail('无法获取引擎 manifest', { status: response.status, url: manifestUrl.href });
  const manifest = await response.json();
  if (!manifest || manifest.manifestVersion !== 1 || manifest.engineApi !== 'fourp.engine.v1' || typeof manifest.releaseId !== 'string' || typeof manifest.buildId !== 'string' || !Array.isArray(manifest.resources)) fail('引擎 manifest 格式或 API 版本无效');
  if (manifest.executionProfile !== profile) fail('manifest 执行档与用户选择不一致');
  const glue = manifest.resources.find((item) => item.role === 'glue' && item.url.split('/').at(-1) === 'four_p_calibration.js');
  const wasm = manifest.resources.find((item) => item.role === 'wasm' && item.url.split('/').at(-1) === 'four_p_calibration_bg.wasm');
  if (!glue || !wasm) fail('manifest 必须包含规范名称的 WASM glue 与二进制');
  if (manifest.resources.some((item) => typeof item.url !== 'string' || !/^\d+$/.test(String(item.bytes)) || !/^[a-fA-F0-9]{64}$/.test(String(item.sha256)))) fail('manifest 每个资源必须声明 URL、十进制字节数与 SHA-256');
  if (!/^[a-fA-F0-9]{64}$/.test(manifest.buildId) || manifest.resources.some((item) => !item.url.startsWith(`${manifest.buildId}/`)) || new Set(manifest.resources.map((item) => item.url)).size !== manifest.resources.length) fail('manifest 资源必须位于唯一的 content-addressed build 目录');
  const aggregateTotal = manifest.resources.reduce((sum, resource) => sum + BigInt(resource.bytes), 0n);
  let aggregateReceived = 0n;
  onState({ stage: 'download', message: '下载引擎资源…', received: '0', total: String(aggregateTotal), aggregateReceived: '0', aggregateTotal: String(aggregateTotal), determinate: aggregateTotal > 0n });
  const loaded = new Map();
  for (const resource of manifest.resources) {
    let resourceReceived = 0n;
    const bytes = await fetchResource(resource, manifestUrl, signal, (count, trustedLength) => {
      resourceReceived += BigInt(count); aggregateReceived += BigInt(count);
      onState({ stage: 'download', message: `下载 ${resource.url}`, received: String(aggregateReceived), total: String(aggregateTotal), aggregateReceived: String(aggregateReceived), aggregateTotal: String(aggregateTotal), resourceReceived: String(resourceReceived), resourceTotal: trustedLength == null ? null : String(trustedLength), determinate: aggregateTotal > 0n });
    });
    loaded.set(resource.url, bytes);
  }
  return { manifest, glueUrl: new URL(glue.url, manifestUrl).href, wasmBytes: loaded.get(wasm.url) };
}
