'use strict';
const $ = (id) => document.getElementById(id);
const pages = [...document.querySelectorAll('.page')];
let boardValid = false;
let capabilities = { worker: '尚未检测' };

function navigate(focus = true) {
  const requested = location.hash.slice(1);
  const current = pages.find((page) => page.id === requested) || pages[0];
  for (const page of pages) page.hidden = page !== current;
  for (const link of document.querySelectorAll('[data-page]')) {
    if (link.dataset.page === current.id) link.setAttribute('aria-current', 'step');
    else link.removeAttribute('aria-current');
  }
  if (current.id === 'review') preflight();
  if (focus) current.querySelector('h1').focus({ preventScroll: true });
}
window.addEventListener('hashchange', () => navigate());
navigate(false);

$('files').addEventListener('change', () => {
  const files = [...$('files').files];
  $('file-list').replaceChildren();
  for (const file of files) {
    const item = document.createElement('li');
    item.textContent = `${file.name} · ${(file.size / 1048576).toFixed(2)} MiB`;
    $('file-list').append(item);
  }
  if (!files.length) {
    const item = document.createElement('li');
    item.textContent = '尚未选择文件';
    $('file-list').append(item);
  }
  $('file-status').textContent = files.length
    ? `已选择 ${files.length} 个文件 · 仅元信息，尚未解析或扫描`
    : '未扫描 · 不从扩展名推断话题或标定模式';
});
$('format').addEventListener('change', () => {
  $('files').value = '';
  $('files').dispatchEvent(new Event('change'));
  $('file-status').textContent = '格式已更改，请重新选择文件；不会沿用旧数据绑定。';
});

function updateBoard() {
  const size = Number($('tag-size').value);
  const gap = Number($('tag-gap').value);
  const source = $('board-ids').value.trim();
  const rows = source.split('\n').map((line) => line.trim().split(/\s+/));
  const values = rows.flat();
  let error = '';
  if (!source || values.some((value) => !/^\d+$/.test(value) || !Number.isSafeInteger(Number(value)))) error = 'ID 必须是非负安全整数；不能有空行。';
  else if (rows.some((row) => row.length !== rows[0].length)) error = '每行 ID 数量必须一致。';
  else if (new Set(values.map(Number)).size !== values.length) error = 'ID 重复；请核对实物，不会自动重新编号。';
  else if (values.length > 400) error = '设计预览最多显示 400 个 ID；这不是引擎的标定板限制。';
  else if (!Number.isFinite(size) || size <= 0 || !Number.isFinite(gap) || gap < 0 || $('tag-gap').value === '') error = '边长必须为正数，间隙必须为非负数。';
  boardValid = !error;
  $('board-preview').replaceChildren();
  $('board-status').classList.toggle('error', !boardValid);
  $('board-status').textContent = error || `${rows.length} 行 × ${rows[0].length} 列 · 布局格式有效，尚未校验 Tag family ID 范围`;
  if (!boardValid) return;
  const preview = $('board-preview');
  preview.style.gridTemplateColumns = `repeat(${rows[0].length}, minmax(0, 1fr))`;
  // Percentage gaps use the same total physical width as the grid container.
  const width = rows[0].length * size + (rows[0].length - 1) * gap;
  preview.style.columnGap = `${gap / width * 100}%`;
  preview.style.rowGap = `${gap / width * 100}%`;
  // CSS percentage row gaps can overflow an intrinsic-height grid; use an aspect ratio.
  const height = rows.length * size + (rows.length - 1) * gap;
  preview.style.aspectRatio = `${width} / ${height}`;
  preview.style.rowGap = `${gap / height * 100}%`;
  preview.style.gridTemplateRows = `repeat(${rows.length}, minmax(0, 1fr))`;
  for (const value of values) {
    const cell = document.createElement('span');
    cell.textContent = value;
    preview.append(cell);
  }
}
for (const id of ['tag-size', 'tag-gap', 'board-ids']) $(id).addEventListener('input', updateBoard);
updateBoard();

const profileNotes = {
  cpu: '无需共享内存隔离；计算放在 Worker，CPU 使用 f64。此原型尚未验证核心库 WASM 可编译性。',
  threads: '需要 crossOriginIsolated、SharedArrayBuffer、WASM atomics 和线程版引擎；还必须真正初始化 Rayon 线程池。'
};
function updateProfile() { $('profile-note').textContent = profileNotes[$('profile').value]; }
$('profile').addEventListener('change', updateProfile);
updateProfile();

function preflight() {
  const items = [
    ['阻断', '标定引擎未接入：不能扫描、检测、求解或导出标定结果。'],
    [$('files').files.length ? '未检查' : '阻断', $('files').files.length ? '文件已选择，但容器、话题、时间质量及数据内容尚未解析。' : '尚未选择输入数据文件。'],
    [boardValid ? '未检查' : '阻断', boardValid ? '板布局格式有效；Tag family、角点映射及实物一致性尚未检查。' : $('board-status').textContent],
    ['未检查', '传感器绑定、模型匹配、IMU 覆盖和可观测性须由实际引擎检查。']
  ];
  const parallel = $('profile').value === 'threads';
  if (parallel && !(globalThis.crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined')) items.push(['阻断', '所选方案需要共享内存；当前页面未满足跨源隔离条件。']);
  if (!$('workers').checkValidity()) items.push(['阻断', '并发预算须为 1–32 的整数（设计范围）。']);
  items.push(['说明', '原型检查不是生产 RunSettings 校验，不作为标定启动依据。']);
  $('checks').replaceChildren();
  for (const [severity, message] of items) {
    const li = document.createElement('li');
    const label = document.createElement('strong');
    label.textContent = severity;
    if (severity === '阻断') label.className = 'error';
    const text = document.createElement('span');
    text.textContent = message;
    li.append(label, text);
    $('checks').append(li);
  }
}
$('check').addEventListener('click', preflight);

const loadingStates = {
  download: ['正在下载引擎资源', '设计状态：总字节未知时不显示百分比，只显示实际已接收字节。'],
  compile: ['正在编译与实例化 WASM', '设计状态：编译没有可靠百分比，显示阶段与真实耗时；完成后才进入下一阶段。'],
  pool: ['正在启动 Worker 池', '设计状态：显示实际已握手数量 / 计划数量，不用计时器虚构完成数。'],
  failure: ['资源加载失败 · 等待用户处理', '设计状态：保留具体错误、资源名和阶段；提供重试。选择 CPU 应创建新尝试，不静默降级。'],
  ready: ['初始化完成 · 可以选择文件', '设计状态：全部必要资源完成后停止动画；显示实际后端、线程数与引擎版本。']
};
$('loading-state').addEventListener('change', () => {
  const state = $('loading-state').value;
  $('loading-preview').dataset.state = state;
  $('loading-title').textContent = loadingStates[state][0];
  $('loading-copy').textContent = loadingStates[state][1];
});

function showCapabilities() {
  const entries = [
    ['安全上下文', globalThis.isSecureContext ? '是（HTTPS / localhost）' : '否'],
    ['WebAssembly API', typeof WebAssembly === 'object' ? '存在 · 未加载核心引擎' : '不可用'],
    ['Worker 握手', capabilities.worker],
    ['共享内存隔离', globalThis.crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined' ? '前置条件满足 · 未验证 WASM 线程' : '未满足 · 仍可使用独立 Workers'],
    ['逻辑并发提示', navigator.hardwareConcurrency ? String(navigator.hardwareConcurrency) : '未知'],
  ];
  $('capabilities').replaceChildren();
  for (const [name, value] of entries) {
    const dt = document.createElement('dt');
    const dd = document.createElement('dd');
    dt.textContent = name;
    dd.textContent = value;
    $('capabilities').append(dt, dd);
  }
}

async function probe() {
  const started = performance.now();
  const boot = document.querySelector('.boot');
  $('probe').disabled = true;
  $('probe-motion').hidden = false;
  boot.setAttribute('aria-busy', 'true');
  $('probe-status').textContent = '正在检查 Worker 握手；未下载或编译标定引擎。';
  capabilities = { worker: '检查中' };
  showCapabilities();
  const tick = () => { $('elapsed').textContent = `环境检查耗时 ${((performance.now() - started) / 1000).toFixed(1)} s`; };
  tick();
  const timer = setInterval(tick, 100);
  const workerCheck = new Promise((resolve) => {
    let worker;
    let timeout;
    const finish = (message) => {
      clearTimeout(timeout);
      worker?.terminate();
      capabilities.worker = message;
      resolve();
    };
    try {
      worker = new Worker('./capability-worker.js');
      timeout = setTimeout(() => finish('握手超时 · 可重试，不判为不支持'), 8000);
      worker.onmessage = (event) => finish(event.data === '4p-ready' ? '握手成功 · 非 WASM 线程测试' : '握手协议错误');
      worker.onerror = () => finish('加载失败 · 请检查网络 / CSP');
      worker.postMessage('4p-probe');
    } catch (error) {
      finish(`创建失败：${error.message}`);
    }
  });
  try {
    await workerCheck;
    showCapabilities();
    $('probe-status').textContent = '环境检查已结束，详情见本机能力。设计原型可以浏览；标定引擎未接入。';
    preflight();
  } finally {
    clearInterval(timer);
    tick();
    $('probe-motion').hidden = true;
    boot.setAttribute('aria-busy', 'false');
    $('probe').disabled = false;
  }
}
$('probe').addEventListener('click', probe);
probe();
