import { API_PROTOCOL, MAX_ARTIFACT_CHUNK } from './controller.js';

const METHODS = new Set(['capabilities.get','sources.list','sources.bind','sources.inspect','inspections.get','inspections.cancel','drafts.create','drafts.replace','drafts.get','drafts.validate','runs.start','runs.get','runs.events','runs.cancel','runs.result','artifacts.list','artifacts.read','runs.release']);

function publicRun(run) {
  return { runId: run.id, state: run.state, stage: run.stage, elapsedMs: Math.round((run.completedAt ?? performance.now()) - run.startedAt), lastSeq: String(run.lastSeq), terminationReason: run.terminationReason ?? null, backend: run.profile, workerCount: run.workerCount, error: run.error ?? null };
}
async function hashBytes(bytes) { const digest = await crypto.subtle.digest('SHA-256', bytes); return [...new Uint8Array(digest)].map((v)=>v.toString(16).padStart(2,'0')).join(''); }

export function installBrowserApi(controller) {
  async function dispatch(method, params) {
    switch (method) {
      case 'capabilities.get': return { apiVersion: API_PROTOCOL, engineVersion: controller.version ?? null, buildId: controller.resources?.manifest.buildId ?? null, initialization: controller.initState, executionProfile: controller.capabilities?.profile ?? null, executionProfiles: { 'cpu-single': { status: controller.executionProfile === 'cpu-single' && controller.ready ? 'ready' : 'available' }, 'cpu-rayon': { status: controller.executionProfile === 'cpu-rayon' && controller.ready ? 'ready' : crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined' ? 'unchecked' : 'unavailable', reason: crossOriginIsolated ? null : 'cross-origin isolation required' } }, threads: controller.capabilities?.threads ?? 0, formats: ['ros1_v2','ros2_mcap','ros2_sqlite3'], calibrationModes: { status: 'engine-validated', values: null }, permissions: { fileAccess: 'explicit-File-binding', uploads: false } };
      case 'sources.list': return [...controller.sources.values()].map((item) => controller.publicSource(item));
      case 'sources.bind': return controller.bindSource(params);
      case 'sources.inspect': return controller.inspectSource(params);
      case 'inspections.get': { const item = controller.inspections.get(params.inspectionId); if (!item) throw controller.error('NOT_FOUND','扫描不存在'); return structuredClone(item); }
      case 'inspections.cancel': {
        const item = controller.inspections.get(params.inspectionId); if (!item) throw controller.error('NOT_FOUND','扫描不存在');
        if (item.status !== 'running') return { accepted: false, status: item.status };
        controller.worker.terminate(); item.status = 'cancelled'; const pending=controller.pending.get(item.inspectionId); if(pending) pending.reject(controller.error('RUN_FAILED','扫描已取消')); controller.pending.delete(item.inspectionId); controller.ready = false; await controller.recreateAfterTermination(); return { accepted: true, status: 'cancelled' };
      }
      case 'drafts.create': return controller.createDraft(params);
      case 'drafts.replace': return controller.replaceDraft(params);
      case 'drafts.get': return controller.publicDraft(controller.requireDraft(params.draftId));
      case 'drafts.validate': return controller.validateDraft(params);
      case 'runs.start': return controller.startRun(params);
      case 'runs.get': { const run = controller.runs.get(params.runId); if (!run) throw controller.error('NOT_FOUND','运行不存在'); return publicRun(run); }
      case 'runs.events': {
        const run = controller.runs.get(params.runId); if (!run) throw controller.error('NOT_FOUND','运行不存在'); let afterSeq; try { afterSeq = BigInt(params.afterSeq ?? '0'); } catch { throw controller.error('INVALID_REQUEST','afterSeq 无效'); } const limit = Math.min(500, Math.max(1, Number(params.limit ?? 100)));
        if (afterSeq < 0n || !Number.isSafeInteger(limit)) throw controller.error('INVALID_REQUEST','事件游标或 limit 无效');
        if (afterSeq + 1n < run.firstSeq) throw controller.error('EVENT_GAP','事件游标已落后于保留窗口',{firstAvailableSeq:String(run.firstSeq)});
        const events = run.events.filter((event)=>BigInt(event.seq)>afterSeq).slice(0,limit); return { events: structuredClone(events), nextSeq: events.at(-1)?.seq ?? String(afterSeq), lastSeq: String(run.lastSeq) };
      }
      case 'runs.cancel': return controller.cancelRun(params);
      case 'runs.result': { const run=controller.runs.get(params.runId); if(!run) throw controller.error('NOT_FOUND','运行不存在'); if(run.released) throw controller.error('NOT_FOUND','运行结果资源已释放'); if(run.state==='failed') throw controller.error('RUN_FAILED','运行失败',run.error); if(run.state!=='completed') throw controller.error('RESULT_NOT_READY','结果尚未就绪'); return structuredClone(run.result); }
      case 'artifacts.list': {
        const run=controller.runs.get(params.runId); if(!run) throw controller.error('NOT_FOUND','运行不存在'); const result=[];
        for(const id of run.artifactIds||[]){const item=controller.artifacts.get(id); if(!item.hash)item.hash=await hashBytes(item.bytes); result.push({artifactId:id,path:item.path,mediaType:item.mediaType,length:String(item.bytes.byteLength),hash:item.hash,complete:item.complete});} return result;
      }
      case 'artifacts.read': {
        const item=controller.artifacts.get(params.artifactId); if(!item) throw controller.error('NOT_FOUND','产物不存在'); const offset=BigInt(params.offset??'0'), length=Number(params.length);
        if(offset<0n||offset>BigInt(item.bytes.byteLength)||!Number.isSafeInteger(length)||length<0||length>MAX_ARTIFACT_CHUNK||BigInt(length)>BigInt(item.bytes.byteLength)-offset) throw controller.error('INVALID_REQUEST','产物读取范围无效',{maxLength:MAX_ARTIFACT_CHUNK});
        return {artifactId:item.id,offset:String(offset),length:String(length),data:item.bytes.slice(Number(offset),Number(offset)+length)};
      }
      case 'runs.release': {
        const run=controller.runs.get(params.runId); if(!run) throw controller.error('NOT_FOUND','运行不存在'); if(run.state==='running'||run.state==='cancelling') throw controller.error('BUSY','运行中不能释放');
        for(const id of run.artifactIds||[]) controller.artifacts.delete(id); delete run.result; run.artifactIds=[]; run.released=true; return {released:true};
      }
      default: throw controller.error('INVALID_REQUEST','未知方法');
    }
  }
  const api = Object.freeze({
    async request(envelope) {
      const id = envelope?.id;
      if (!envelope || envelope.protocol !== API_PROTOCOL || typeof id !== 'string' || !id) return { protocol: API_PROTOCOL, id: id ?? null, error: { code: envelope?.protocol && envelope.protocol !== API_PROTOCOL ? 'UNSUPPORTED_PROTOCOL' : 'INVALID_REQUEST', message: '请求信封无效', details: null, retryable: false } };
      if (!METHODS.has(envelope.method)) return { protocol: API_PROTOCOL, id, error: { code:'INVALID_REQUEST',message:'未知方法',details:{method:envelope.method},retryable:false } };
      try { return { protocol: API_PROTOCOL, id, result: await dispatch(envelope.method, envelope.params ?? {}) }; }
      catch (error) { return { protocol: API_PROTOCOL, id, error: error.apiError ?? { code:'RUN_FAILED',message:error.message || String(error),details:null,retryable:false } }; }
    },
    subscribe({ runId, afterSeq = '0' }, callback) {
      if (typeof callback !== 'function') throw new TypeError('callback 必须是函数'); let cursor; try { cursor=BigInt(afterSeq); } catch { throw new TypeError('afterSeq 必须是十进制序号字符串'); } let active=true;
      const deliver=(event)=>{const seq=BigInt(event.seq);if(active&&event.runId===runId&&seq>cursor){cursor=seq;callback(structuredClone(event));}};
      const existing=controller.runs.get(runId); if(!existing) throw new Error('运行不存在'); if(cursor+1n<existing.firstSeq) throw Object.assign(new Error('事件游标已落后于保留窗口'),{code:'EVENT_GAP'}); for(const event of existing.events) deliver(event);
      const listener=({detail})=>deliver(detail); controller.addEventListener('event',listener); return ()=>{active=false;controller.removeEventListener('event',listener);};
    }
  });
  Object.defineProperty(window,'fourP',{value:api,writable:false,configurable:false});
  window.dispatchEvent(new CustomEvent('fourp:api-ready',{detail:{protocol:API_PROTOCOL}}));
  return api;
}
