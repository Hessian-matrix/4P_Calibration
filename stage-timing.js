const names = { starting: '准备计算', scan_detect: '读取与检测', solve: '标定求解', export: '整理结果' };
const duration = (seconds) => {
  const value = Math.max(0, Math.ceil(seconds));
  return value < 60 ? `${value} 秒` : `${Math.floor(value / 60)} 分 ${value % 60} 秒`;
};

export class StageTiming {
  constructor(root) {
    root.innerHTML = '<div class="metric"><span>当前阶段</span><strong data-stage>尚未开始</strong></div><div class="metric"><span>当前阶段耗时</span><strong data-elapsed>—</strong></div><div class="metric"><span>本阶段预计剩余</span><strong data-remaining>—</strong></div><p class="small" data-note>预计时间不包含后续阶段。</p>';
    this.stageLabel = root.querySelector('[data-stage]');
    this.elapsed = root.querySelector('[data-elapsed]');
    this.remaining = root.querySelector('[data-remaining]');
    this.note = root.querySelector('[data-note]');
  }

  start(cameras, topics) {
    clearInterval(this.timer);
    this.frames = new Map(cameras.map(camera => [camera.id, 0]));
    const counts = cameras.map(camera => topics.find(topic => topic.name === camera.topic)?.message_count);
    this.total = counts.length && counts.every(count => Number.isSafeInteger(count) && count >= 0)
      ? counts.reduce((sum, count) => sum + count, 0) : null;
    this.rate = null;
    this.lastDetection = null;
    this.ended = null;
    this.stage = null;
    this.setStage('starting');
    this.timer = setInterval(() => this.render(), 1000);
  }

  setStage(stage) {
    if (!stage || stage === this.stage || ['cancelling', 'cancelled', 'completed', 'failed'].includes(stage)) return;
    this.stage = stage;
    this.started = performance.now();
    this.render();
  }

  event(detail) {
    if (this.started == null || this.ended != null) return;
    if (detail.kind === 'state') return;
    this.setStage(detail.kind === 'stage' ? detail.payload : detail.stage);
    if (detail.kind !== 'detectionProgress') return;
    const p = detail.payload;
    if (this.frames.has(p.camera_id) && Number.isSafeInteger(p.frames_processed) && p.frames_processed >= 0) {
      this.frames.set(p.camera_id, Math.max(this.frames.get(p.camera_id), p.frames_processed));
      this.lastDetection = performance.now();
      this.rate = p.average_frames_per_second;
    }
  }

  stop(state) {
    if (this.started == null) return;
    this.ended = performance.now();
    this.state = state;
    clearInterval(this.timer);
    this.render();
  }

  render() {
    const now = this.ended ?? performance.now();
    this.stageLabel.textContent = `${names[this.stage] || this.stage}${this.ended == null ? '' : ` · ${this.state}`}`;
    this.elapsed.textContent = duration((now - this.started) / 1000);
    this.note.textContent = '预计时间不包含后续阶段。';
    if (this.ended != null) { this.remaining.textContent = '—'; return; }
    if (this.stage !== 'scan_detect') {
      this.remaining.textContent = '暂无法估算';
      this.note.textContent = this.stage === 'solve' ? '迭代上限不是完成进度，收敛时间未知。' : '此阶段没有可信的剩余工作量。';
      return;
    }
    const done = [...this.frames.values()].reduce((sum, count) => sum + count, 0);
    this.note.textContent = `已检测 ${done}${this.total == null ? '' : ` / ${this.total}`} 帧；按全相机累计平均速度估算，不含求解时间。`;
    if (this.total == null || done > this.total) { this.remaining.textContent = '暂无法估算'; return; }
    if (done === this.total) { this.remaining.textContent = '检测完成，等待阶段结束'; return; }
    if (done < 8 || !(this.rate > 0) || !Number.isFinite(this.rate)) { this.remaining.textContent = '估算中…'; return; }
    if (now - this.lastDetection > 5000) { this.remaining.textContent = '等待进度更新'; return; }
    this.remaining.textContent = `约 ${duration((this.total - done) / this.rate)}`;
  }
}
