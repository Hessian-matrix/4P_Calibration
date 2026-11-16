import test from 'node:test';
import assert from 'node:assert/strict';
import { RunStatus } from '../app/stage-timing.js';

const labels = { scan_detect: '读取图像并检测标定板', solve: '计算标定参数' };
const detection = (frames, rate) => ({ stage: 'scan_detect', kind: 'detectionProgress', elapsedMs: 1000, payload: { frames_processed: frames, average_frames_per_second: rate } });

test('log and IMU events retain frames and average FPS until the next detection update', () => {
  const root = { textContent: '' };
  const status = new RunStatus(root, labels);
  status.event(detection(120, 32.45), 4);
  for (const kind of ['message', 'imuProgress', 'stage']) {
    status.event({ stage: 'scan_detect', kind, elapsedMs: 1100, payload: {} }, 4);
    assert.match(root.textContent, /已处理 120 帧 · 平均 32\.5 FPS$/);
  }
  status.event(detection(121, 32.8), 4);
  assert.match(root.textContent, /已处理 121 帧 · 平均 32\.8 FPS$/);
});

test('phase changes and new runs cannot retain stale detection summaries', () => {
  const root = { textContent: '' };
  const status = new RunStatus(root, labels);
  status.event(detection(120, 32.45), 4);
  status.event({ stage: 'solve', kind: 'stage', elapsedMs: 2000, payload: 'solve' }, 4);
  assert.equal(root.textContent, '计算标定参数 · 2.0 秒 · 4 个工作线程');
  status.event(detection(122, 32.9), 4);
  status.reset();
  status.event({ stage: 'scan_detect', kind: 'message', elapsedMs: 0, payload: 'opening' }, 2);
  assert.equal(root.textContent, '读取图像并检测标定板 · 0.0 秒 · 2 个工作线程');
  status.event(detection(0, 0), 2);
  assert.match(root.textContent, /已处理 0 帧 · 平均 0\.0 FPS$/);
});
