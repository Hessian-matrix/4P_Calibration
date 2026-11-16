const LIMIT = 100;
const MAX_BATCH = 12;
const RISE_MS = 240;
const SHIFT_MS = 300;
const STYLE_ID = 'frame-history-styles';

function installStyles() {
  if (document.getElementById(STYLE_ID)) return;
  const style = document.createElement('style');
  style.id = STYLE_ID;
  style.textContent = `
#frame-bars.fh-root {
  position: relative;
  display: block;
  width: 100%;
  height: 100%;
  overflow: hidden;
  contain: layout paint;
}
#frame-bars.fh-root .fh-bar {
  position: absolute;
  right: 0;
  bottom: 0;
  width: max(1px, calc(1% - 2px));
  height: max(2px, var(--fh-height));
  min-width: 0;
  min-height: 2px;
  border-radius: 2px 2px 0 0;
  background: var(--green, #155f35);
  transform: translateX(calc(-1 * var(--fh-slot, 0) * var(--fh-step, 0px))) scaleY(var(--fh-rise, 1));
  transform-origin: center bottom;
  will-change: auto;
}
#frame-bars.fh-root .fh-bar[data-tags="0"] {
  background: var(--muted, #607066);
  opacity: .55;
}
#frame-bars.fh-root[data-phase="rise"] .fh-bar {
  transition: transform ${RISE_MS}ms cubic-bezier(.16, 1, .3, 1);
  will-change: transform;
}
#frame-bars.fh-root[data-phase="shift"] .fh-bar {
  transition: transform ${SHIFT_MS}ms cubic-bezier(.22, 1, .36, 1);
  will-change: transform;
}
#frame-bars.fh-root[data-reduced-motion="true"] .fh-bar {
  transition: none !important;
}
@media (max-width: 420px) {
  #frame-bars.fh-root .fh-bar { width: max(1px, calc(1% - 1px)); }
}
`;
  document.head.append(style);
}

function validCount(value) {
  const number = typeof value === 'number' ? value : Number(value);
  return Number.isFinite(number) && number >= 0 ? number : null;
}

function display(value) {
  return value === undefined || value === null || value === '' ? '—' : String(value);
}

/**
 * A bounded, real-event-only frame histogram.
 *
 * Animated updates have two deliberately separate phases: old bars slide left
 * to clear space, then new bars rise in those slots. Rendering is scheduled on
 * the main thread and never acknowledges or blocks the compute Worker. Bursts
 * are coalesced; the pending queue retains only the newest LIMIT events.
 */
export class FrameHistory {
  constructor(root, label, { reducedMotion = globalThis.matchMedia?.('(prefers-reduced-motion: reduce)') } = {}) {
    if (!(root instanceof Element)) throw new TypeError('FrameHistory root must be an Element');
    if (label != null && !(label instanceof Element)) throw new TypeError('FrameHistory label must be an Element');

    installStyles();
    this.root = root;
    this.label = label;
    this.reducedMotion = reducedMotion;
    this.entries = [];
    this.queue = [];
    this.activeBatch = [];
    this.scale = 1;
    this.stopped = false;
    this.frameRequest = 0;
    this.phaseTimer = 0;

    root.classList.add('fh-root');
    this.resizeObserver = new ResizeObserver(([entry]) => {
      root.style.setProperty('--fh-step', `${entry.contentRect.width / LIMIT}px`);
    });
    this.resizeObserver.observe(root);
    root.dataset.phase = 'idle';
    this.onMotionChange = () => {
      this.root.dataset.reducedMotion = String(this.motionReduced());
      if (this.motionReduced() && (this.activeBatch.length || this.queue.length)) this.finishImmediately();
    };
    reducedMotion?.addEventListener?.('change', this.onMotionChange);
    this.onMotionChange();
    this.reset(1);
  }

  /** Clear the previous run and use the configured board tag count as scale. */
  reset(tagCount) {
    this.cancelScheduledWork();
    this.entries = [];
    this.queue = [];
    this.activeBatch = [];
    this.scale = Math.max(1, validCount(tagCount) ?? 1);
    this.stopped = false;
    this.root.dataset.phase = 'idle';
    this.root.dataset.scale = String(this.scale);
    this.root.replaceChildren();
    this.updateLabel(true);
  }

  /** Queue one genuine detection payload; no placeholder frames are generated. */
  push(payload) {
    if (this.stopped) return;
    const tags = validCount(payload?.tags);
    if (tags === null) return;

    const { camera_id, camera, frame_id, frame, frames_processed, timestamp_ns, timestamp } = payload;
    this.queue.push({ payload: { camera_id, camera, frame_id, frame, frames_processed, timestamp_ns, timestamp }, tags, element: null });
    if (this.queue.length > LIMIT) this.queue.splice(0, this.queue.length - LIMIT);
    if (!this.activeBatch.length && !this.frameRequest) {
      this.frameRequest = requestAnimationFrame(() => {
        this.frameRequest = 0;
        this.startBatch();
      });
    }
  }

  /** Stop future updates and synchronously expose every newest retained frame. */
  stop() {
    if (this.stopped) return;
    this.stopped = true;
    this.finishImmediately();
  }

  motionReduced() {
    return this.reducedMotion === true || this.reducedMotion?.matches === true;
  }

  cancelScheduledWork() {
    if (this.frameRequest) cancelAnimationFrame(this.frameRequest);
    clearTimeout(this.phaseTimer);
    this.frameRequest = 0;
    this.phaseTimer = 0;
  }

  startBatch() {
    if (this.stopped || this.activeBatch.length || !this.queue.length) return;
    if (this.motionReduced()) {
      this.finishImmediately();
      return;
    }

    this.activeBatch = this.queue.splice(0, MAX_BATCH);
    this.raiseScale(this.activeBatch);
    if (!this.entries.length) { this.riseBatch(); return; }
    this.root.dataset.phase = 'shift';
    for (let index = 0; index < this.entries.length; index += 1) {
      this.position(this.entries[this.entries.length - index - 1], index + this.activeBatch.length);
    }
    this.phaseTimer = setTimeout(() => this.riseBatch(), SHIFT_MS);
  }

  riseBatch() {
    this.phaseTimer = 0;
    const batchSize = this.activeBatch.length;
    this.root.dataset.phase = 'rise';
    for (let index = 0; index < batchSize; index += 1) {
      const entry = this.activeBatch[index];
      entry.element = this.makeBar(entry);
      this.position(entry, batchSize - index - 1);
      entry.element.style.setProperty('--fh-rise', '0');
      this.root.append(entry.element);
    }

    // Two frames guarantee the zero-height state is painted without a layout read.
    this.frameRequest = requestAnimationFrame(() => {
      this.frameRequest = requestAnimationFrame(() => {
        this.frameRequest = 0;
        for (const entry of this.activeBatch) entry.element.style.setProperty('--fh-rise', '1');
        this.phaseTimer = setTimeout(() => this.completeBatch(), RISE_MS);
      });
    });
  }


  completeBatch() {
    this.phaseTimer = 0;
    const combined = this.entries.concat(this.activeBatch);
    this.entries = combined.slice(-LIMIT);
    const keep = new Set(this.entries);
    for (const entry of combined) if (!keep.has(entry)) entry.element?.remove();
    this.activeBatch = [];
    this.root.dataset.phase = 'idle';
    this.positionAll();
    this.updateLabel(false);

    if (this.queue.length && !this.stopped) {
      this.frameRequest = requestAnimationFrame(() => {
        this.frameRequest = 0;
        this.startBatch();
      });
    }
  }

  finishImmediately() {
    this.cancelScheduledWork();
    const combined = this.entries.concat(this.activeBatch, this.queue).slice(-LIMIT);
    this.entries = combined;
    this.activeBatch = [];
    this.queue = [];
    this.raiseScale(combined);
    this.root.dataset.phase = 'idle';
    this.root.replaceChildren();
    for (const entry of this.entries) {
      entry.element = this.makeBar(entry);
      entry.element.style.setProperty('--fh-rise', '1');
      this.root.append(entry.element);
    }
    this.positionAll();
    this.updateLabel(false);
  }

  raiseScale(entries) {
    let nextScale = this.scale;
    for (const entry of entries) nextScale = Math.max(nextScale, entry.tags);
    if (nextScale === this.scale) return;
    this.scale = nextScale;
    this.root.dataset.scale = String(nextScale);
    for (const entry of this.entries.concat(this.activeBatch)) this.setHeight(entry);
  }

  makeBar(entry) {
    const { payload, tags } = entry;
    const bar = document.createElement('span');
    bar.className = 'fh-bar';
    bar.dataset.tags = String(tags);
    const camera = display(payload?.camera_id ?? payload?.camera);
    const frame = display(payload?.frame_id ?? payload?.frame ?? payload?.frames_processed);
    const time = display(payload?.timestamp_ns ?? payload?.timestamp);
    const tooltip = `相机 ${camera} · 帧 ${frame} · 时间 ${time}${time === '—' ? '' : ' ns'} · ${tags} Tags`;
    bar.title = tooltip;
    bar.setAttribute('aria-label', tooltip);
    this.setHeight(entry, bar);
    return bar;
  }

  setHeight(entry, element = entry.element) {
    element?.style.setProperty('--fh-height', `${entry.tags / this.scale * 100}%`);
  }

  position(entry, slot) {
    entry.element?.style.setProperty('--fh-slot', String(slot));
  }

  positionAll() {
    for (let index = 0; index < this.entries.length; index += 1) {
      this.position(this.entries[this.entries.length - index - 1], index);
    }
  }

  updateLabel(waiting) {
    if (!this.label) return;
    this.label.textContent = waiting
      ? `一柱一帧 · 纵轴 0–${this.scale} Tags · 右侧最新 · 等待真实检测事件`
      : `最近 ${this.entries.length} 帧 · 一柱一帧 · 纵轴 0–${this.scale} Tags · 右侧最新（完成顺序，非同步时间轴）`;
  }
}

export default FrameHistory;
