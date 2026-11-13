import { TaskController } from './controller.js';
import { installBrowserApi } from './browser-api.js';
import { bootstrapCrossOriginIsolation } from './isolation.js';
import { ConfigurationForms } from './configuration-forms.js';
import { ResultsView } from './results-view.js';
import { FrameHistory } from './frame-history.js';
import { ConvergenceView } from './convergence-view.js';
import { StageTiming } from './stage-timing.js';

const $ = (id) => document.getElementById(id);
const controller = new TaskController();
const api = installBrowserApi(controller);
const reduceMotion = matchMedia('(prefers-reduced-motion: reduce)');
const frameHistory = new FrameHistory($('frame-bars'), $('frame-bars-label'), { reducedMotion: reduceMotion });
const convergenceView = new ConvergenceView($('convergence-view'));
const stageTiming = new StageTiming($('stage-timing'));
let source = null, inspectionId = null, draft = null, preflight = null, runId = null;
let configDirty = true, settingCanonical = false, busy = false, bindingGeneration = 0;
let profilePromise = null, profileTimer = null;
const pretty = (value) => JSON.stringify(value, null, 2);
const profileNames = { 'cpu-single': '兼容模式', 'cpu-rayon': '多核模式' };
const stageNames = { starting: '正在准备', scan_detect: '读取图像并检测标定板', solve: '计算标定参数', export: '整理结果文件', completed: '标定完成', cancelled: '已停止', failed: '标定失败', cancelling: '正在停止' };
const finite = (value) => value != null && value !== '' && Number.isFinite(Number(value));
const displayNumber = (value, digits = 1) => finite(value) ? Number(value).toFixed(digits) : '—';
const formatBytes = (value) => {
  if (value == null) return '—';
  const n = Number(value);
  if (!Number.isFinite(n)) return '—';
  return n < 1024 ? `${n} B` : n < 1048576 ? `${(n / 1024).toFixed(1)} KB` : n < 1073741824 ? `${(n / 1048576).toFixed(1)} MB` : `${(n / 1073741824).toFixed(2)} GB`;
};
async function request(method, params = {}) {
  const response = await api.request({ protocol: 'fourp.browser.v1', id: crypto.randomUUID(), method, params });
  if (response.error) throw Object.assign(new Error(response.error.message), { apiError: response.error });
  return response.result;
}
function selectSection(id) {
  for (const link of document.querySelectorAll('[data-section]')) link.toggleAttribute('aria-current', link.dataset.section === id);
}
function milestone(id) {
  const section = $(id);
  if (!section) return;
  history.replaceState(null, '', `#${id}`);
  selectSection(id);
  section.scrollIntoView({ behavior: reduceMotion.matches ? 'auto' : 'smooth', block: 'start' });
}
const sectionObserver = new IntersectionObserver((entries) => {
  const visible = entries.filter((entry) => entry.isIntersecting).sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0];
  if (visible) selectSection(visible.target.id);
}, { rootMargin: '-15% 0px -65% 0px', threshold: [0, .2, .5] });
for (const section of document.querySelectorAll('.work-block')) sectionObserver.observe(section);
for (const link of document.querySelectorAll('[data-section]')) link.addEventListener('click', () => selectSection(link.dataset.section));
selectSection(location.hash.slice(1) || 'boot');

const forms = new ConfigurationForms({
  sensors: $('sensor-forms'), target: $('board-forms'), solver: $('solver-forms'),
  onChange(config) {
    $('config-editor').value = pretty(config);
    if (!settingCanonical) { configDirty = true; preflight = null; $('config-status').textContent = '设置已更新，开始前会自动检查。'; }
    renderOverview();
  },
  onIssues() { renderOverview(); },
});
const resultView = new ResultsView($('result-summary'));
function setCanonical(config) {
  settingCanonical = true;
  try { forms.setConfig(config); $('config-editor').value = pretty(config); }
  finally { settingCanonical = false; }
}
function renderOverview() {
  if (!source) { $('run-overview').textContent = '请先导入数据并完成解析。'; return; }
  const config = forms.getConfig();
  const cameras = config?.cameras?.length || 0;
  $('run-overview').textContent = `${source.files.length} 个文件 · ${cameras} 路相机${config?.source?.imu_topic ? ' + IMU' : ''} · ${profileNames[controller.executionProfile] || '尚未准备'}`;
}

function setProgress(element, received, total, complete = false) {
  const known = total != null && BigInt(total) > 0n;
  element.className = `progress ${complete ? 'complete' : known ? '' : 'indeterminate'}`;
  if (known && !complete) {
    const ratio = BigInt(received || 0) * 10000n / BigInt(total);
    element.querySelector('span').style.width = `${Math.min(100, Number(ratio) / 100)}%`;
  } else element.querySelector('span').style.removeProperty('width');
}
controller.addEventListener('init', ({ detail }) => {
  const ready = detail.stage === 'ready', failed = detail.stage === 'failed';
  $('boot-title').textContent = ready ? '本地引擎已就绪' : failed ? '引擎准备未完成' : '正在准备本地引擎';
  $('boot-status').textContent = ready ? '计算组件已加载。文件只在本机浏览器中读取。' : detail.message || '正在加载计算组件…';
  const total = detail.aggregateTotal ?? detail.total;
  const received = detail.aggregateReceived ?? detail.received;
  setProgress($('boot-progress'), received ?? 0, total, ready);
  $('boot-bytes').textContent = received != null ? `${formatBytes(received)}${total == null ? ' 已下载' : ` / ${formatBytes(total)}`}` : (ready ? '下载与编译完成' : '编译阶段不提供虚构百分比');
  $('retry-init').hidden = !failed;
  $('boot-status').setAttribute('role', failed ? 'alert' : 'status');
  $('engine-badge').textContent = ready ? `${profileNames[detail.capabilities.profile]} · 已就绪` : failed ? '准备失败' : '正在准备';
  $('engine-badge').className = `badge ${ready ? 'ok' : failed ? 'error' : ''}`;
  $('boot').setAttribute('aria-busy', String(!ready && !failed));
  for (const id of ['source-files', 'source-directory', 'choose-source', 'validate-config', 'start-run']) $(id).disabled = !ready;
  $('file-zone').setAttribute('aria-disabled', String(!ready));
  updateProfileControls();
  if (ready) { $('version').textContent = detail.version.split(';')[0]; if (!source) milestone('source'); }
});
async function initializeApplication() {
  try {
    $('boot-status').textContent = '正在准备浏览器运行环境…';
    await bootstrapCrossOriginIsolation({ enabled: !crossOriginIsolated });
    await applyProfile();
  } catch (error) { controller.emit('init', { stage: 'failed', message: error.message }); }
}
$('retry-init').addEventListener('click', initializeApplication);
void initializeApplication();

function updateProfileControls() {
  const locked = Boolean(profilePromise || controller.activeRunId || [...controller.inspections.values()].some(item => item.status === 'running'));
  $('profile').disabled = locked;
  $('worker-count').disabled = locked;
  $('worker-count').closest('label').hidden = $('profile').value !== 'cpu-rayon';
}
async function applyProfile() {
  clearTimeout(profileTimer);
  if (profilePromise) await profilePromise;
  const profile = $('profile').value, workers = profile === 'cpu-rayon' ? Number($('worker-count').value) : 1;
  if (!Number.isInteger(workers) || workers < 1 || workers > 32) throw new Error('线程数须为 1–32 的整数');
  if (controller.ready && controller.executionProfile === profile && controller.requestedWorkers === workers) return;
  if (controller.activeRunId || [...controller.inspections.values()].some(item => item.status === 'running')) throw new Error('运行或解析期间不能更改计算方式');
  preflight = null;
  $('profile-status').textContent = `正在自动应用${profileNames[profile]}${profile === 'cpu-rayon' ? ` · ${workers} 线程` : ''}…`;
  profilePromise = controller.switchProfile(profile, workers);
  updateProfileControls();
  try {
    await profilePromise;
    $('profile-status').textContent = `已自动应用${profileNames[controller.executionProfile]} · 实际 ${controller.capabilities.threads} 个工作线程`;
    renderOverview();
  } catch (error) {
    $('profile-status').textContent = `无法启用：${error.message}。未自动降档，可手动选择兼容模式。`;
    throw error;
  } finally { profilePromise = null; updateProfileControls(); }
}
function scheduleProfile() {
  preflight = null;
  clearTimeout(profileTimer);
  updateProfileControls();
  if ($('profile').value === 'cpu-rayon' && !$('worker-count').checkValidity()) {
    $('profile-status').textContent = '请输入 1–32 的整数线程数'; return;
  }
  profileTimer = setTimeout(() => { void applyProfile().catch(() => {}); }, 350);
}
$('profile').addEventListener('change', scheduleProfile);
$('worker-count').addEventListener('input', scheduleProfile);
for (const event of ['run', 'complete', 'failed', 'cancelled']) controller.addEventListener(event, updateProfileControls);

function sourceUsesDirectory() { return $('source-format').value === 'ros2_sqlite3'; }
function updateSourceEntry() {
  const directory = sourceUsesDirectory();
  $('file-zone-title').textContent = directory ? '选择 ROS2 数据目录' : '选择数据文件';
  $('file-zone-hint').textContent = directory ? '点击选择目录，或把完整目录拖到这里（需包含 metadata.yaml）' : '点击选择，或把文件拖到这里';
  $('choose-source').textContent = directory ? '选择目录' : '选择文件';
}
function openSourcePicker() {
  if (!controller.ready) return;
  $(sourceUsesDirectory() ? 'source-directory' : 'source-files').click();
}
$('choose-source').addEventListener('click', (event) => { event.stopPropagation(); openSourcePicker(); });
$('file-zone').addEventListener('click', (event) => { if (event.target !== $('choose-source')) openSourcePicker(); });
$('file-zone').addEventListener('keydown', (event) => { if (event.key === 'Enter' || event.key === ' ') { event.preventDefault(); openSourcePicker(); } });
for (const name of ['dragenter', 'dragover']) $('file-zone').addEventListener(name, (event) => { event.preventDefault(); if (controller.ready) $('file-zone').classList.add('is-dragging'); });
for (const name of ['dragleave', 'drop']) $('file-zone').addEventListener(name, (event) => { event.preventDefault(); $('file-zone').classList.remove('is-dragging'); });
function readEntryFile(entry) { return new Promise((resolve, reject) => entry.file(resolve, reject)); }
function readDirectoryBatch(reader) { return new Promise((resolve, reject) => reader.readEntries(resolve, reject)); }
async function collectEntry(entry, prefix = '') {
  if (entry.isFile) return [{ file: await readEntryFile(entry), relativePath: `${prefix}${entry.name}` }];
  if (!entry.isDirectory) return [];
  const reader = entry.createReader(), children = [];
  while (true) { const batch = await readDirectoryBatch(reader); if (!batch.length) break; children.push(...batch); }
  const nested = await Promise.all(children.map((child) => collectEntry(child, `${prefix}${entry.name}/`)));
  return nested.flat();
}
$('file-zone').addEventListener('drop', async (event) => {
  if (!controller.ready) return;
  try {
    const entries = [...event.dataTransfer.items].map((item) => item.webkitGetAsEntry?.()).filter(Boolean);
    const selected = entries.length ? (await Promise.all(entries.map((entry) => collectEntry(entry)))).flat() : [...event.dataTransfer.files];
    await bindSelectedFiles(selected);
  } catch (error) { $('source-status').textContent = `无法读取拖入内容：${error.message}`; }
});
function resetSource() {
  bindingGeneration++;
  source = null; inspectionId = null; draft = null; preflight = null; configDirty = true;
  $('inspect-source').disabled = true;
  $('inspection-output').textContent = '请解析新数据中的话题。';
  $('inspection-status').textContent = '导入数据后自动解析容器和话题。';
  $('inspection-bytes').textContent = '';
  $('inspection-progress').className = 'progress idle';
  forms.setInspection({ topics: [] }, { format: $('source-format').value });
  renderOverview();
}
async function bindSelectedFiles(fileList) {
  resetSource();
  const generation = bindingGeneration;
  const entries = [...fileList].map((entry) => entry?.file instanceof File ? entry : { file: entry, relativePath: entry.webkitRelativePath || entry.name });
  $('file-list').replaceChildren();
  if (!entries.length) { $('file-list').innerHTML = '<li>尚未选择文件</li>'; return; }
  const allRooted = entries.every(({ relativePath }) => relativePath.includes('/'));
  const roots = new Set(entries.map(({ relativePath }) => relativePath.split('/')[0]));
  const stripRoot = allRooted && roots.size === 1;
  const normalized = entries.map(({ file, relativePath }) => ({ file, relativePath: stripRoot ? relativePath.slice(relativePath.indexOf('/') + 1) : relativePath || file.name }));
  for (const { file, relativePath } of normalized) { const li = document.createElement('li'); li.textContent = `${relativePath} · ${formatBytes(file.size)}`; $('file-list').append(li); }
  $('source-status').textContent = '正在绑定所选数据…';
  try {
    const bound = await request('sources.bind', { format: $('source-format').value, files: normalized.map(({ file, relativePath }) => ({ file, relativePath, role: /\.(ya?ml|json)$/i.test(file.name) && !($('source-format').value === 'ros2_sqlite3' && file.name.toLowerCase() === 'metadata.yaml') ? 'auxiliary' : 'data' })) });
    if (generation !== bindingGeneration) return;
    source = bound;
    $('source-status').textContent = `已绑定 ${source.files.length} 个文件，正在自动解析。`;
    $('inspect-source').disabled = false;
    milestone('inspection');
    await inspectSource(generation);
  } catch (error) { if (generation === bindingGeneration) $('source-status').textContent = `文件准备失败：${error.message}`; }
}
for (const id of ['source-files', 'source-directory']) $(id).addEventListener('change', (event) => {
  $(id === 'source-files' ? 'source-directory' : 'source-files').value = '';
  void bindSelectedFiles(event.currentTarget.files);
});
$('source-format').addEventListener('change', () => {
  $('source-files').value = ''; $('source-directory').value = ''; resetSource(); updateSourceEntry();
  $('file-list').innerHTML = '<li>数据格式已变化，请重新选择。</li>';
  $('source-status').textContent = '请重新选择数据。';
});
updateSourceEntry();
function renderInspectionBytes(progress = {}) {
  const fileBytes = source.files.reduce((total, file) => total + BigInt(file.size), 0n);
  $('inspection-bytes').textContent = `所选文件总大小 ${formatBytes(fileBytes)} · 累计读取 ${formatBytes(progress.bytesRead ?? 0)}（含重复读取，可超过文件大小）· ${progress.readCalls ?? 0} 次读取`;
}
async function inspectSource(generation = bindingGeneration) {
  if (!source) return;
  $('inspect-source').disabled = true; $('cancel-inspection').hidden = false; $('inspection-status').textContent = '正在解析容器与话题…';
  setProgress($('inspection-progress'), 0, null);
  renderInspectionBytes();
  try {
    ({ inspectionId } = await request('sources.inspect', { sourceId: source.sourceId, sourceRevision: source.sourceRevision }));
    updateProfileControls();
    const id = inspectionId;
    while (id === inspectionId && generation === bindingGeneration) {
      const item = await request('inspections.get', { inspectionId: id });
      if (item.status === 'completed') {
        $('source-status').textContent = '数据已绑定并解析完成。'; $('inspection-status').textContent = '解析完成'; $('inspection-output').textContent = pretty(item.result);
        setProgress($('inspection-progress'), 0, null, true);
        forms.setInspection(item.result, { format: source.format, path: source.files.find((file) => file.role === 'data')?.relativePath || 'authorized-source', files: source.files });
        $('config-editor').value = pretty(forms.getConfig()); configDirty = true;
        milestone('sensors'); break;
      }
      if (item.status === 'failed' || item.status === 'cancelled') throw new Error(item.error?.message || '解析已取消');
      await new Promise((resolve) => setTimeout(resolve, 250));
    }
  } catch (error) { if (generation === bindingGeneration) $('inspection-status').textContent = `解析未完成：${error.message}`; }
  finally { if (generation === bindingGeneration) { $('cancel-inspection').hidden = true; $('inspect-source').disabled = !source; } updateProfileControls(); }
}
$('inspect-source').addEventListener('click', () => void inspectSource());
$('cancel-inspection').addEventListener('click', async () => {
  if (!inspectionId) return;
  try { await request('inspections.cancel', { inspectionId }); $('inspection-status').textContent = '已取消解析'; }
  catch (error) { $('inspection-status').textContent = error.message; }
});
controller.addEventListener('inspection-progress', ({ detail }) => {
  if (!inspectionId || detail.inspectionId !== inspectionId) return;
  setProgress($('inspection-progress'), 0, null);
  renderInspectionBytes(detail);
});

const tabs = [...document.querySelectorAll('[role=tab]')];
function activateTab(tab) {
  for (const item of tabs) { const active = item === tab; item.setAttribute('aria-selected', String(active)); item.tabIndex = active ? 0 : -1; $(item.getAttribute('aria-controls')).hidden = !active; }
}
for (const [index, tab] of tabs.entries()) {
  tab.addEventListener('click', () => activateTab(tab));
  tab.addEventListener('keydown', (event) => { if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return; event.preventDefault(); const next = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1 : (index + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length; activateTab(tabs[next]); tabs[next].focus(); });
}
async function validateSettings() {
  if (!source) throw new Error('请先导入并解析数据。');
  await applyProfile();
  const issues = forms.getIssues();
  if (issues.length) throw new Error(issues.map((item) => typeof item === 'string' ? item : item.message).join('；'));
  $('config-status').textContent = '正在检查设置…';
  if (!draft || configDirty) {
    const config = pretty(forms.getConfig());
    draft = draft ? await request('drafts.replace', { draftId: draft.draftId, expectedRevision: draft.revision, config }) : await request('drafts.create', { sourceId: source.sourceId, config });
  }
  preflight = await request('drafts.validate', { draftId: draft.draftId, expectedRevision: draft.revision, executionProfile: $('profile').value });
  draft = await request('drafts.get', { draftId: draft.draftId });
  setCanonical(draft.canonicalConfig); configDirty = false;
  $('config-revision').textContent = `配置版本 ${draft.revision}`;
  $('config-status').textContent = preflight.blocking.length ? '部分设置需要调整，请查看计算区提示。' : '设置检查通过，可以开始标定。';
  return preflight;
}
$('validate-config').addEventListener('click', async () => { try { await validateSettings(); } catch (error) { preflight = null; $('config-status').textContent = `请调整设置：${error.message}`; } });
async function checkSettings() {
  const list = $('preflight'); list.replaceChildren();
  try {
    const checked = await validateSettings();
    for (const [kind, label] of [['blocking', '需要调整'], ['warnings', '请留意'], ['unchecked', '尚未检查']]) for (const item of checked[kind] || []) { const li = document.createElement('li'); li.className = kind; li.textContent = `${label}：${item.message || item.code}`; list.append(li); }
    if (checked.blocking.length || checked.unchecked.length) return false;
    const li = document.createElement('li'); li.className = 'ok'; li.textContent = '检查通过，将使用当前设置开始标定。'; list.append(li); return true;
  } catch (error) { preflight = null; const li = document.createElement('li'); li.className = 'blocking'; li.textContent = error.message; list.append(li); $('config-status').textContent = `请调整设置：${error.message}`; return false; }
}
$('preflight-button').addEventListener('click', () => void checkSettings());
function resetTelemetry() {
  frameHistory.reset(forms.getConfig().target.id_rows_front_view_left_to_right.flat().length);
  convergenceView.reset();
  stageTiming.start(forms.getConfig().cameras, controller.inspections.get(inspectionId)?.result?.topics ?? []);
  for (const id of ['metric-fps', 'metric-average-fps', 'metric-workers', 'metric-iteration']) $(id).textContent = '—';
  $('frame-rows').innerHTML = '<tr class="placeholder"><td colspan="8">等待真实检测事件</td></tr>';
  $('camera-previews').replaceChildren();
  for (const camera of forms.getConfig().cameras) {
    const figure = document.createElement('figure'); figure.className = 'preview'; figure.dataset.cameraId = camera.id;
    const canvas = document.createElement('canvas'); canvas.width = 320; canvas.height = 240;
    canvas.setAttribute('aria-label', `${camera.id} 图像与检测角点`);
    const caption = document.createElement('figcaption'); caption.textContent = `${camera.id} · 等待真实图像`;
    figure.append(canvas, caption); $('camera-previews').append(figure);
  }
  $('solve-phase').textContent = '求解尚未开始'; $('solve-work').textContent = '';
  $('imu-values').querySelectorAll('dd').forEach((item) => { item.textContent = '—'; });
  if (!forms.getConfig()?.source?.imu_topic) $('imu-values').querySelector('dd').textContent = '未选用 IMU';
}
$('start-run').addEventListener('click', async () => {
  if (busy || controller.activeRunId) return;
  busy = true; $('start-run').disabled = true;
  try {
    if (!await checkSettings()) return;
    const accepted = await request('runs.start', { draftId: draft.draftId, expectedRevision: draft.revision, preflightId: preflight.preflightId, idempotencyKey: crypto.randomUUID() });
    runId = accepted.runId; const workers = controller.runs.get(runId)?.workerCount ?? controller.capabilities.threads;
    $('run-state').textContent = '进行中'; $('run-stage').textContent = `正在准备标定… · ${workers} 个工作线程`; $('metric-workers').textContent = String(workers);
    $('cancel-run').hidden = false; $('cancel-run').textContent = '停止标定'; $('event-log').replaceChildren(); $('compute-activity').classList.add('running'); resetTelemetry(); $('metric-workers').textContent = String(workers); milestone('run');
  } catch (error) { $('run-stage').textContent = `无法开始：${error.message}`; }
  finally { busy = false; $('start-run').disabled = !!controller.activeRunId; }
});
$('cancel-run').addEventListener('click', async () => {
  if (!runId) return;
  const cooperative = controller.capabilities.profile === 'cpu-rayon' && controller.runs.get(runId)?.state === 'running';
  if (!cooperative && !confirm('停止会丢弃本次尚未完成的结果。确定停止？')) return;
  try { const result = await request('runs.cancel', { runId, mode: cooperative ? 'cooperative' : 'force' }); if (result.state === 'cancelling') { $('run-state').textContent = '正在停止'; $('run-stage').textContent = '正在等待当前计算安全结束…'; $('cancel-run').textContent = '立即强制停止'; } }
  catch (error) { $('run-stage').textContent = `停止未完成：${error.message}`; }
});
function renderPreview(preview, payload) {
  const width = Number(preview?.width), height = Number(preview?.height), pixels = preview?.pixels;
  if (!Number.isSafeInteger(width) || !Number.isSafeInteger(height) || width <= 0 || height <= 0 || !pixels || pixels.length !== width * height) return;
  const figure = [...$('camera-previews').children].find(node => node.dataset.cameraId === payload.camera_id);
  if (!figure) return;
  const canvas = figure.querySelector('canvas'), context = canvas.getContext('2d'); canvas.width = width; canvas.height = height;
  const image = context.createImageData(width, height);
  for (let index = 0; index < pixels.length; index++) { const value = pixels[index]; const offset = index * 4; image.data[offset] = value; image.data[offset + 1] = value; image.data[offset + 2] = value; image.data[offset + 3] = 255; }
  context.putImageData(image, 0, 0);
  if (Array.isArray(preview.corners)) { context.fillStyle = '#4cff8d'; for (const corner of preview.corners) { const x = Number(Array.isArray(corner) ? corner[0] : corner?.x), y = Number(Array.isArray(corner) ? corner[1] : corner?.y); if (finite(x) && finite(y)) { context.beginPath(); context.arc(x, y, 2, 0, Math.PI * 2); context.fill(); } } }
  figure.querySelector('figcaption').textContent = `${payload.camera_id} · ${payload.timestamp_ns} ns · ${payload.points} 角点 · 抽样非同步画面`;
}
function vectorText(value) { return Array.isArray(value) && value.every(finite) ? `[${value.map((item) => Number(item).toPrecision(6)).join(', ')}]` : '—'; }
function renderImu(payload) {
  if (!payload) return;
  const values = $('imu-values').querySelectorAll('dd');
  const samples = payload.sample_count ?? payload.samples;
  values[0].textContent = samples == null ? '—' : String(samples);
  values[1].textContent = payload.timestamp_ns == null ? '—' : String(payload.timestamp_ns);
  values[2].textContent = vectorText(payload.gyro_rad_s);
  values[3].textContent = vectorText(payload.specific_force_m_s2);
}
function appendDetection(payload) {
  frameHistory.push(payload);
  $('frame-rows').querySelector('.placeholder')?.remove();
  const row = document.createElement('tr');
  const cells = [payload.camera_id, payload.frames_processed, payload.timestamp_ns, payload.tags, payload.points, finite(payload.frames_per_second) ? Number(payload.frames_per_second).toFixed(1) : null, finite(payload.average_frames_per_second) ? Number(payload.average_frames_per_second).toFixed(1) : null, payload.workers];
  for (const value of cells) { const cell = document.createElement('td'); cell.textContent = value == null ? '—' : String(value); row.append(cell); }
  $('frame-rows').append(row); while ($('frame-rows').rows.length > 200) $('frame-rows').deleteRow(0);
}
controller.addEventListener('event', ({ detail }) => {
  if (detail.runId !== runId) return;
  convergenceView.event(detail);
  stageTiming.event(detail);
  const payload = detail.payload || {}, workerCount = payload.workers ?? controller.runs.get(runId)?.workerCount ?? controller.capabilities?.threads;
  let progress = workerCount == null ? '' : ` · ${workerCount} 个工作线程`;
  if (detail.kind === 'detectionProgress') {
    $('metric-fps').textContent = displayNumber(payload.frames_per_second); $('metric-average-fps').textContent = displayNumber(payload.average_frames_per_second); $('metric-workers').textContent = payload.workers == null ? String(workerCount ?? '—') : String(payload.workers);
    appendDetection(payload); if (payload.preview) renderPreview(payload.preview, payload); if (payload.latest_imu) renderImu(payload.latest_imu);
    progress += `${payload.frames_processed == null ? '' : ` · 已处理 ${payload.frames_processed} 帧`}${finite(payload.average_frames_per_second) ? ` · 平均 ${Number(payload.average_frames_per_second).toFixed(1)} FPS` : ''}`;
  } else if (detail.kind === 'imuProgress') renderImu(payload);
  else if (detail.kind === 'solveProgress') { $('metric-iteration').textContent = `${payload.iteration ?? '—'} / ${payload.max_iterations ?? '—'}`; $('solve-phase').textContent = 'LM 求解迭代'; $('solve-work').textContent = `iter ${payload.iteration ?? '—'} / ${payload.max_iterations ?? '—'} · cost ${finite(payload.cost) ? Number(payload.cost).toExponential(4) : '—'}`; }
  else if (detail.kind === 'solveActivity') { $('solve-phase').textContent = payload.phase == null ? '求解活动' : String(payload.phase); $('solve-work').textContent = payload.completed != null && payload.total != null ? `${payload.completed} / ${payload.total}${payload.iteration == null ? '' : ` · iter ${payload.iteration}`}` : payload.iteration == null ? '工作量不可计数' : `iter ${payload.iteration} · 工作量不可计数`; }
  $('run-stage').textContent = `${stageNames[detail.stage] || detail.stage || '正在计算'} · ${(detail.elapsedMs / 1000).toFixed(1)} 秒${progress}`;
  const { preview, ...numericPayload } = payload;
  if (preview) numericPayload.preview = { width: preview.width, height: preview.height, corners: preview.corners?.length ?? 0, pixels: '图像仅在预览显示，不转储像素' };
  const li = document.createElement('li'), entry = document.createElement('details'), summary = document.createElement('summary');
  summary.textContent = `#${detail.seq} ${detail.stage} / ${detail.kind}${detail.kind === 'detectionProgress' ? ` · ${payload.camera_id} · ${payload.tags} Tags · ${payload.points} 角点` : ''}`;
  entry.append(summary);
  entry.addEventListener('toggle', () => {
    if (entry.open && entry.children.length === 1) { const pre = document.createElement('pre'); pre.textContent = pretty(numericPayload); entry.append(pre); }
  });
  li.append(entry); $('event-log').append(li);
  while ($('event-log').children.length > 100) $('event-log').firstElementChild.remove();
});
async function artifactBytes(artifact) {
  const chunks = []; let offset = 0n; const total = BigInt(artifact.length);
  while (offset < total) { const length = Number(total - offset > 4194304n ? 4194304n : total - offset); const part = await request('artifacts.read', { artifactId: artifact.artifactId, offset: String(offset), length }); chunks.push(part.data); offset += BigInt(length); }
  return new Blob(chunks, { type: artifact.mediaType });
}
function downloadBlob(blob, name) { const url = URL.createObjectURL(blob), anchor = document.createElement('a'); anchor.href = url; anchor.download = name; anchor.click(); setTimeout(() => URL.revokeObjectURL(url), 0); }
controller.addEventListener('complete', async ({ detail }) => {
  if (detail.runId !== runId) return;
  frameHistory.stop();
  convergenceView.stop('求解完成');
  stageTiming.stop('已完成');
  $('run-state').textContent = '已完成'; $('run-stage').textContent = '标定与结果文件已生成'; $('cancel-run').hidden = true; $('start-run').disabled = false; $('compute-activity').classList.remove('running');
  $('result-status').textContent = '结果已生成。请检查误差和参数，应用前仍需人工验收。'; $('result-output').textContent = pretty(detail.result);
  try {
    const artifacts = await request('artifacts.list', { runId });
    const summaryArtifact = artifacts.find((item) => item.path === 'result/summary.json');
    const summary = summaryArtifact ? JSON.parse(await (await artifactBytes(summaryArtifact)).text()) : null;
    resultView.render({ result: detail.result, summary });
    const list = $('artifact-list'); list.replaceChildren();
    const names = { 'result/calibration.yaml': '标定参数', 'result/summary.json': '误差与收敛报告', 'result/extrinsics_3d.html': '三维结果查看器' };
    const extra = document.createElement('details'); extra.className = 'advanced'; const title = document.createElement('summary'); title.textContent = '其他诊断与完整日志文件'; extra.append(title);
    for (const artifact of artifacts) { const li = document.createElement('li'), button = document.createElement('button'); button.textContent = `下载${names[artifact.path] || ` ${artifact.path}`} · ${formatBytes(artifact.length)}`; button.addEventListener('click', async () => { try { downloadBlob(await artifactBytes(artifact), artifact.path.split('/').pop()); } catch (error) { $('result-status').textContent = `下载失败：${error.message}`; } }); li.append(button); (names[artifact.path] ? list : extra).append(li); }
    if (extra.children.length > 1) { const more = document.createElement('li'); more.append(extra); list.append(more); }
  } catch (error) { resultView.render({ result: detail.result, summary: null }); $('result-status').textContent = `结果已生成，但摘要读取失败：${error.message}`; }
  milestone('results');
});
function finishRun(state, message) { frameHistory.stop(); convergenceView.stop(state); stageTiming.stop(state); $('run-state').textContent = state; $('run-stage').textContent = message; $('cancel-run').hidden = true; $('start-run').disabled = false; $('compute-activity').classList.remove('running'); }
controller.addEventListener('failed', ({ detail }) => { if (detail.runId === runId) finishRun('未完成', `标定失败：${detail.error.message}`); });
controller.addEventListener('cancelled', ({ detail }) => { if (detail.runId === runId) finishRun('已停止', '本次计算已停止，没有发布未完成的结果。'); });
async function importConfiguration(text) { const value = await controller.callWorker({ type: 'validate', requestId: crypto.randomUUID(), config: text }); setCanonical(value.config); configDirty = true; preflight = null; $('config-status').textContent = '配置已导入，可继续调整。'; }
$('import-config-file').addEventListener('change', async (event) => { try { const file = event.target.files[0]; if (file) await importConfiguration(await file.text()); } catch (error) { $('config-status').textContent = `配置导入失败：${error.message}`; } });
$('apply-config-text').addEventListener('click', async () => { try { await importConfiguration($('config-editor').value); } catch (error) { $('config-status').textContent = `配置未应用：${error.message}`; } });
$('export-config').addEventListener('click', () => downloadBlob(new Blob([pretty(forms.getConfig())], { type: 'application/json' }), 'calibration-config.json'));
