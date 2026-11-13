import { loadEngineResources } from './loader.js';

const WORKER_PROTOCOL = 'fourp.worker.v1';
const API_PROTOCOL = 'fourp.browser.v1';
const EVENT_LIMIT = 1000;
const MAX_ARTIFACT_CHUNK = 4 * 1024 * 1024;
const randomId = (prefix) => `${prefix}-${crypto.randomUUID()}`;
const stableJson = (value) => value && typeof value === 'object' ? Array.isArray(value) ? `[${value.map(stableJson).join(',')}]` : `{${Object.keys(value).sort().map((key) => `${JSON.stringify(key)}:${stableJson(value[key])}`).join(',')}}` : JSON.stringify(value);

export class TaskController extends EventTarget {
  constructor() {
    super();
    this.generation = 0; this.worker = null; this.resources = null; this.ready = false; this.initState = { stage: 'shell' };
    this.sources = new Map(); this.inspections = new Map(); this.drafts = new Map(); this.preflights = new Map(); this.runs = new Map(); this.artifacts = new Map(); this.idempotency = new Map();
    this.pending = new Map(); this.activeRunId = null; this.nextFileHandle = 1; this.executionProfile = 'cpu-single'; this.requestedWorkers = 1;
  }
  emit(name, detail) { this.dispatchEvent(new CustomEvent(name, { detail })); }
  async initialize(profile = this.executionProfile, requestedWorkers = this.requestedWorkers) {
    if (!['cpu-single','cpu-rayon'].includes(profile)) throw this.error('CAPABILITY_UNAVAILABLE','该执行档尚不可用');
    if (profile === 'cpu-rayon' && (!crossOriginIsolated || typeof SharedArrayBuffer === 'undefined')) throw this.error('CAPABILITY_UNAVAILABLE','cpu-rayon 需要跨源隔离与 SharedArrayBuffer');
    this.executionProfile = profile; this.requestedWorkers = profile === 'cpu-rayon' ? Math.max(1, Math.min(32, Number(requestedWorkers) || 1)) : 1;
    const generation = ++this.generation;
    this.ready = false;
    this.worker?.terminate();
    this.worker = null;
    const abort = new AbortController(); this.initAbort?.abort(); this.initAbort = abort;
    try {
      this.resources = await loadEngineResources({ profile: this.executionProfile, signal: abort.signal, onState: (state) => { if (generation === this.generation) { this.initState = state; this.emit('init', state); } } });
      if (generation !== this.generation) return;
      await this.createWorker(generation);
    } catch (error) {
      if (generation !== this.generation || error.name === 'AbortError') return;
      this.initState = { stage: 'failed', message: error.message, error: error.apiError ?? { code: 'ENGINE_NOT_READY', message: error.message, details: error.details ?? null, retryable: true } }; this.emit('init', this.initState); throw error;
    }
  }
  async switchProfile(profile, requestedWorkers) { if (this.activeRunId) throw this.error('BUSY','运行期间不能切换执行档'); const sources=[...this.sources.values()]; await this.initialize(profile, requestedWorkers); for (const source of sources) await this.callWorker({ type:'bind',requestId:randomId('bind'),sourceId:source.id,sourceRevision:source.revision,files:source.files.map(({handle,file})=>({handle,file})) }); }
  createWorker(generation) {
    return new Promise((resolve, reject) => {
      const worker = new Worker(new URL('./engine-worker.js', import.meta.url), { type: 'module', name: `fourp-${this.executionProfile}` });
      this.worker = worker;
      const onError = (event) => { reject(new Error(event.message || 'Worker initialization failed')); };
      worker.addEventListener('error', onError, { once: true });
      worker.onmessage = ({ data }) => {
        if (!data || data.protocol !== WORKER_PROTOCOL || data.generation !== this.generation) return;
        if (data.type === 'ready') {
          worker.removeEventListener('error', onError); this.ready = true; this.version = data.version; this.capabilities = data.capabilities; this.cancelView = data.cancelMemory ? new Int32Array(data.cancelMemory, data.cancelPointer, 1) : null;
          this.initState = { stage: 'ready', message: '引擎就绪', version: data.version, capabilities: data.capabilities }; this.emit('init', this.initState); resolve();
          return;
        }
        if (data.type === 'init-failed') { worker.removeEventListener('error', onError); worker.terminate(); reject(Object.assign(new Error(data.error.message), { apiError: data.error })); return; }
        if (data.type === 'init-stage') { this.initState = data; this.emit('init', data); return; }
        this.handleWorkerMessage(data);
      };
      worker.postMessage({ protocol: WORKER_PROTOCOL, type: 'init', generation, profile: this.executionProfile, requestedWorkers: this.requestedWorkers, glueUrl: this.resources.glueUrl, wasmBytes: this.resources.wasmBytes }, [this.resources.wasmBytes.buffer]);
    });
  }
  handleWorkerMessage(message) {
    const pendingId = message.requestId || message.inspectionId;
    if (message.type === 'error') {
      if (message.runId) this.failRun(message.runId, message.error);
      else { const pending = this.pending.get(pendingId); if (pending) { this.pending.delete(pendingId); pending.reject(Object.assign(new Error(message.error.message), { apiError: message.error })); } if (message.inspectionId) { const item = this.inspections.get(message.inspectionId); if (item) Object.assign(item, { status: 'failed', error: message.error }); } }
      if (message.error.code === 'ENGINE_TRAP') void this.recoverAfterTrap(message.error);
      return;
    }
    if (message.type === 'validated') return this.resolvePending(message.requestId, message.value);
    if (message.type === 'bound') return this.resolvePending(message.requestId, message);
    if (message.type === 'inspection-progress') {
      const inspection = this.inspections.get(message.inspectionId);
      if (inspection?.status === 'running') {
        const progress = { bytesRead: message.bytesRead, readCalls: message.readCalls };
        inspection.progress = progress;
        this.emit('inspection-progress', { inspectionId: message.inspectionId, ...progress });
      }
      return;
    }
    if (message.type === 'inspection-completed') {
      const inspection = this.inspections.get(message.inspectionId); if (inspection) Object.assign(inspection, { status: 'completed', result: message.value, completedAt: Date.now() });
      return this.resolvePending(message.inspectionId, message.value);
    }
    if (message.type === 'run-events') {
      if (!Array.isArray(message.events)) return;
      for (const item of message.events) this.appendRunEvent(message.runId, item.seq, item.event);
      return;
    }
    if (message.type === 'run-cancelled') { const run = this.runs.get(message.runId); if (run && (run.state === 'running' || run.state === 'cancelling')) { run.state = 'running'; this.appendRunEvent(run.id, run.lastSeq + 1n, { stage: 'cancelled', kind: 'state', payload: { state: 'cancelled', mode: 'cooperative' } }); run.state = 'cancelled'; run.terminationReason = 'cooperative'; this.activeRunId = null; this.emit('cancelled', { runId: run.id }); } return; }
    if (message.type === 'run-completed') return this.completeRun(message.runId, message.value, message.lastSeq);
  }
  resolvePending(id, value) { const pending = this.pending.get(id); if (pending) { this.pending.delete(id); pending.resolve(value); } }
  callWorker(message, transfer = []) { return new Promise((resolve, reject) => { const id = message.requestId || message.inspectionId; this.pending.set(id, { resolve, reject }); this.worker.postMessage({ protocol: WORKER_PROTOCOL, generation: this.generation, ...message }, transfer); }); }
  descriptor(source) { return { format: source.format, sourceId: source.id, sourceRevision: source.revision, files: source.files.map(({ handle, relativePath, size, role }) => ({ handle, relativePath, size, role })) }; }
  safePath(path) { if (typeof path !== 'string' || !path || path.startsWith('/') || path.split(/[\\/]/).some((part) => !part || part === '.' || part === '..')) throw this.error('INVALID_REQUEST', 'relativePath 无效'); return path.replaceAll('\\', '/'); }
  async bindSource(params) {
    if (!this.ready) throw this.error('ENGINE_NOT_READY', '引擎尚未就绪');
    if (this.activeRunId) throw this.error('BUSY', '运行期间不能绑定新数据源');
    if ([...this.inspections.values()].some((item)=>item.status==='running')) throw this.error('BUSY','数据源扫描正在运行');
    if (!['ros1_v2','ros2_mcap','ros2_sqlite3'].includes(params?.format)) throw this.error('UNSUPPORTED_FORMAT', '不支持的容器格式');
    if (!Array.isArray(params.files) || !params.files.length) throw this.error('SOURCE_NOT_BOUND', '没有已授权文件');
    const id = randomId('source'), revision = 1;
    const files = params.files.map((entry) => { const file = entry instanceof File ? entry : entry.file; if (!(file instanceof File)) throw this.error('SOURCE_NOT_BOUND', 'sources.bind 只接受已授权 File'); const relativePath = this.safePath(entry.relativePath || file.webkitRelativePath || file.name); return { handle: this.nextFileHandle++, file, relativePath, size: String(file.size), mediaType: file.type || 'application/octet-stream', role: entry.role || 'data', lastModified: String(file.lastModified) }; });
    if (new Set(files.map((f) => f.relativePath)).size !== files.length) throw this.error('INVALID_REQUEST', 'relativePath 重名');
    const source = { id, revision, format: params.format, files };
    await this.callWorker({ type: 'bind', requestId: randomId('bind'), sourceId: id, sourceRevision: revision, files: files.map(({ handle, file }) => ({ handle, file })) });
    this.sources.set(id, source); return this.publicSource(source);
  }
  publicSource(source) { return { sourceId: source.id, sourceRevision: source.revision, format: source.format, files: source.files.map(({ relativePath, size, mediaType, role, lastModified }) => ({ relativePath, size, mediaType, role, lastModified })) }; }
  async inspectSource(params) { if (!this.ready) throw this.error('ENGINE_NOT_READY','引擎尚未就绪'); if (this.activeRunId || [...this.inspections.values()].some((item)=>item.status==='running')) throw this.error('BUSY','已有活动任务'); const source = this.requireSource(params.sourceId, params.sourceRevision); const id = randomId('inspection'); const record = { inspectionId: id, sourceId: source.id, sourceRevision: source.revision, status: 'running', startedAt: Date.now(), progress: { bytesRead: 0, readCalls: 0 } }; this.inspections.set(id, record); this.callWorker({ type: 'inspect', inspectionId: id, source: this.descriptor(source) }).catch(() => {}); return { inspectionId: id }; }
  requireSource(id, revision) { const source = this.sources.get(id); if (!source) throw this.error('SOURCE_NOT_BOUND', '数据源不存在'); if (revision != null && revision !== source.revision) throw this.error('STALE_REVISION', '数据源 revision 已变化'); return source; }
  async createDraft(params) { const source = this.requireSource(params.sourceId); const id = randomId('draft'); const draft = { id, revision: 1, sourceId: source.id, config: String(params.config ?? '') }; this.drafts.set(id, draft); return this.publicDraft(draft); }
  publicDraft(draft) { return { draftId: draft.id, revision: draft.revision, sourceId: draft.sourceId, config: draft.config, canonicalConfig: draft.canonicalConfig ?? null }; }
  requireDraft(id, revision) { const draft = this.drafts.get(id); if (!draft) throw this.error('NOT_FOUND', '草稿不存在'); if (revision != null && draft.revision !== revision) throw this.error('STALE_REVISION', '草稿 revision 已变化'); return draft; }
  async replaceDraft(params) { const draft = this.requireDraft(params.draftId, params.expectedRevision); draft.config = String(params.config ?? ''); draft.revision++; delete draft.canonicalConfig; delete draft.canonicalYaml; return this.publicDraft(draft); }
  async validateDraft(params) { if (!this.ready) throw this.error('ENGINE_NOT_READY','引擎尚未就绪'); if (this.activeRunId || [...this.inspections.values()].some((item)=>item.status==='running')) throw this.error('BUSY','已有活动任务'); const draft = this.requireDraft(params.draftId, params.expectedRevision); if (params.executionProfile !== this.executionProfile) throw this.error('CAPABILITY_UNAVAILABLE', '所选执行档尚未初始化'); const requestId = randomId('validate'); const value = await this.callWorker({ type: 'validate', requestId, config: draft.config }); draft.canonicalConfig = value.config; draft.canonicalYaml = value.yaml; draft.config = value.yaml; const source = this.requireSource(draft.sourceId); const inspection = [...this.inspections.values()].find((item) => item.sourceId === source.id && item.sourceRevision === source.revision && item.status === 'completed'); const blocking = []; if (!inspection) blocking.push({ code: 'SOURCE_NOT_INSPECTED', message: '数据源尚未完成扫描' }); const id = randomId('preflight'); const snapshotHash = await this.hashText(value.yaml); const preflight = { id, draftId: draft.id, revision: draft.revision, sourceRevision: source.revision, buildId: this.resources.manifest.buildId, profile: this.executionProfile, snapshotHash, blocking, warnings: [], unchecked: [] }; this.preflights.set(id, preflight); return { preflightId: id, blocking, warnings: [], unchecked: [], snapshotHash, buildId: preflight.buildId }; }
  async hashText(text) { const bytes = new TextEncoder().encode(text); const digest = await crypto.subtle.digest('SHA-256', bytes); return [...new Uint8Array(digest)].map((v) => v.toString(16).padStart(2,'0')).join(''); }
  async startRun(params) {
    const draft = this.requireDraft(params.draftId, params.expectedRevision), preflight = this.preflights.get(params.preflightId);
    if (!this.ready) throw this.error('ENGINE_NOT_READY', '引擎尚未就绪');
    if ([...this.inspections.values()].some((item)=>item.status==='running')) throw this.error('BUSY','数据源扫描正在运行');
    if (typeof params.idempotencyKey !== 'string' || !params.idempotencyKey) throw this.error('INVALID_REQUEST', 'idempotencyKey 必须是非空字符串');
    if (!preflight || preflight.draftId !== draft.id || preflight.revision !== draft.revision || preflight.buildId !== this.resources.manifest.buildId || preflight.profile !== this.executionProfile) throw this.error('STALE_PREFLIGHT', '启动检查已过期');
    if (preflight.blocking.length) throw this.error('PREFLIGHT_BLOCKED', '启动检查存在阻断项', { blocking: preflight.blocking });
    const requestKey = stableJson({ draftId: draft.id, revision: draft.revision, preflightId: preflight.id }); const previous = this.idempotency.get(params.idempotencyKey);
    if (previous) { if (previous.requestKey !== requestKey) throw this.error('IDEMPOTENCY_CONFLICT', '幂等键已用于不同请求'); return { runId: previous.runId, status: 'accepted', configHash: preflight.snapshotHash }; }
    if (this.activeRunId) throw this.error('BUSY', '已有活动运行');
    const source = this.requireSource(draft.sourceId); const id = randomId('run'); if (this.cancelView) Atomics.store(this.cancelView, 0, 0); const run = { id, state: 'running', stage: 'starting', startedAt: performance.now(), events: [], firstSeq: 1n, lastSeq: 0n, profile: this.executionProfile, workerCount: this.capabilities.threads, configHash: preflight.snapshotHash, sourceId: source.id };
    this.runs.set(id, run); this.activeRunId = id; this.idempotency.set(params.idempotencyKey, { requestKey, runId: id });
    this.worker.postMessage({ protocol: WORKER_PROTOCOL, generation: this.generation, type: 'run', runId: id, config: draft.canonicalYaml, source: this.descriptor(source) }); this.emit('run', { runId: id });
    return { runId: id, status: 'accepted', configHash: preflight.snapshotHash };
  }
  appendRunEvent(id, seqValue, event) {
    const run = this.runs.get(id), seq = BigInt(seqValue);
    if (!run || !['running','cancelling'].includes(run.state) || seq <= run.lastSeq) return;
    run.lastSeq = seq; run.stage = event.stage || run.stage;
    if (event.kind === 'state' && ['completed','failed','cancelled'].includes(event.payload?.state)) run.completedAt = performance.now();
    const normalized = { runId: id, seq: String(seq), stage: run.stage, kind: event.kind || 'progress', elapsedMs: Math.round((run.completedAt ?? performance.now()) - run.startedAt), payload: event.payload ?? event };
    let retained = normalized;
    if (normalized.kind === 'detectionProgress' && normalized.payload?.preview) {
      const { preview: _livePreview, ...diagnosticPayload } = normalized.payload;
      retained = { ...normalized, payload: diagnosticPayload };
    }
    run.events.push(retained);
    if (run.events.length > EVENT_LIMIT) { run.events.shift(); run.firstSeq += 1n; }
    this.emit('event', normalized);
  }
  completeRun(id, value, lastSeqValue) { const run = this.runs.get(id); if (!run || !['running','cancelling'].includes(run.state)) return; const lastSeq = BigInt(lastSeqValue); this.appendRunEvent(id, (run.lastSeq > lastSeq ? run.lastSeq : lastSeq) + 1n, { stage: 'completed', kind: 'state', payload: { state: 'completed' } }); run.state = 'completed'; run.result = value.result; run.completedAt = performance.now(); for (const artifact of value.artifacts || []) { if (typeof artifact.path !== 'string' || typeof artifact.text !== 'string') continue; const bytes = new TextEncoder().encode(artifact.text); const artifactId = randomId('artifact'); const record = { id: artifactId, runId: id, path: artifact.path, mediaType: artifact.mediaType || 'text/plain;charset=utf-8', bytes, complete: true }; this.artifacts.set(artifactId, record); (run.artifactIds ||= []).push(artifactId); } this.activeRunId = null; this.emit('complete', { runId: id, result: run.result }); }
  failRun(id, error) { const run = this.runs.get(id); if (!run || !['running','cancelling'].includes(run.state)) return; this.appendRunEvent(id, run.lastSeq + 1n, { stage: 'failed', kind: 'state', payload: { state: 'failed', error } }); run.state = 'failed'; run.error = error; this.activeRunId = null; this.emit('failed', { runId: id, error }); }
  async cancelRun(params) { const run = this.runs.get(params.runId); if (!run) throw this.error('NOT_FOUND','运行不存在'); if (!['running','cancelling'].includes(run.state)) return { accepted: false, state: run.state }; if (params.mode === 'cooperative') { if (run.profile !== 'cpu-rayon' || !this.cancelView) throw this.error('CANCEL_MODE_UNSUPPORTED','当前执行档不支持协作取消'); Atomics.store(this.cancelView,0,1); run.state='cancelling'; run.stage='cancelling'; return {accepted:true,state:'cancelling'}; } if (params.mode !== 'force') throw this.error('INVALID_REQUEST','取消 mode 必须为 cooperative 或 force'); this.worker.terminate(); run.state='running'; this.appendRunEvent(run.id, run.lastSeq + 1n, { stage: 'cancelled', kind: 'state', payload: { state: 'cancelled', mode: 'force' } }); run.state = 'cancelled'; run.terminationReason = 'force'; run.artifactIds = []; this.activeRunId = null; this.ready = false; this.emit('cancelled',{runId:run.id}); await this.recreateAfterTermination(); return { accepted: true, state: 'cancelled' }; }
  async recreateAfterTermination() { const sources = [...this.sources.values()]; await this.initialize(); for (const source of sources) await this.callWorker({ type:'bind',requestId:randomId('bind'),sourceId:source.id,sourceRevision:source.revision,files:source.files.map(({handle,file})=>({handle,file})) }); }
  async recoverAfterTrap(error) { this.worker?.terminate(); this.ready=false; this.cancelView=null; for (const [id,pending] of this.pending) { pending.reject(this.error('ENGINE_NOT_READY','WASM trap 后引擎正在重新初始化',{cause:error.message})); this.pending.delete(id); } this.initState={stage:'failed',message:'WASM trap；正在重建计算 Worker',error}; this.emit('init',this.initState); try { await this.recreateAfterTermination(); } catch {} }
  error(code, message, details = null) { const error = new Error(message); error.apiError = { code, message, details, retryable: false }; return error; }
}

export { API_PROTOCOL, MAX_ARTIFACT_CHUNK };
