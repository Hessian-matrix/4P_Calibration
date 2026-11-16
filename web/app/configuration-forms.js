const IMAGE_TYPES = new Set(['sensor_msgs/Image', 'sensor_msgs/CompressedImage']);
const IMU_TYPE = 'sensor_msgs/Imu';
const CAMERA_MODELS = [
  ['double-sphere', 'Double Sphere'],
  ['kannala-brandt4', 'Kannala–Brandt 4'],
  ['extended-unified', 'Extended Unified'],
  ['pinhole-none', 'Pinhole（无畸变）'],
  ['pinhole-radtan5', 'Pinhole RadTan 5'],
  ['pinhole-radtan8', 'Pinhole RadTan 8'],
  ['pinhole-d12', 'Pinhole D12'],
];
const TAG_FAMILIES = ['tag16h5', 'tag25h9', 'tag36h10', 'tag36h11'];
const IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
const DEFAULT_IDS = [
  [30, 31, 32, 33, 34, 35],
  [24, 25, 26, 27, 28, 29],
  [18, 19, 20, 21, 22, 23],
  [12, 13, 14, 15, 16, 17],
  [6, 7, 8, 9, 10, 11],
  [0, 1, 2, 3, 4, 5],
];

const clone = (value) => typeof structuredClone === 'function'
  ? structuredClone(value)
  : JSON.parse(JSON.stringify(value));
const escapeHtml = (value) => String(value).replace(/[&<>"']/g, (character) => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
})[character]);
const finite = (value) => Number.isFinite(Number(value));
const number = (value) => value === '' ? Number.NaN : Number(value);
const integer = (value) => value === '' ? Number.NaN : Number.parseInt(value, 10);
const checked = (value) => value ? ' checked' : '';
const selected = (value, candidate) => value === candidate ? ' selected' : '';
const identity = () => clone(IDENTITY);
const diagonal = (value) => [[value, 0, 0], [0, value, 0], [0, 0, value]];

function defaultTarget() {
  return {
    type: 'aprilgrid', family: 'tag36h11', border_bits: 2,
    tag_size_m: 0.088, tag_gap_m: 0.0264,
    id_rows_front_view_left_to_right: clone(DEFAULT_IDS),
    decoded_corner_to_front_corner: [2, 3, 0, 1],
  };
}

function defaultDetector() {
  return {
    max_hamming: 0,
    refinement: { method: 'saddle', max_iterations: 20 },
    budgets: { max_pixels: 16777216, max_clusters: 16384, max_candidates: 512 },
  };
}

function defaultSolver() {
  return { max_iterations: 1000, huber_delta: 3, point_rejection_px: 6 };
}

function defaultImu() {
  return {
    gravity_magnitude_m_s2: 9.80665,
    noise: {
      gyro_covariance_rad2_s_hz: diagonal(0.000001),
      accelerometer_covariance_m2_s4_hz: diagonal(0.000001),
      gyro_bias_walk_rad_s2_sqrt_hz: 0.00001,
      accelerometer_bias_walk_m_s3_sqrt_hz: 0.00001,
    },
    integration: { method: 'midpoint', max_gap_s: 0.02, max_samples: 2000000 },
    axis_calibration: {
      refine: false,
      gyro_calibrated_from_raw: identity(),
      accelerometer_calibrated_from_raw: identity(),
    },
    continuous_time: {
      knot_interval_s: 0.01,
      camera_time_offsets: { refine: false, minimum_s: -0.005, maximum_s: 0.005 },
      clock_skews: { refine_global: false, refine_cameras: false, minimum_ppm: -200, maximum_ppm: 200 },
      visual_motion_anisotropy: { enabled: false, variance_ratio_per_motion_squared: 0.015 },
    },
  };
}

function initialIntrinsics(path = null) {
  return path ? { path } : {};
}

function automaticCalibration(cameras, imuTopic) {
  const reference = cameras[0]?.id || '';
  if (imuTopic) {
    return {
      type: 'multi_camera_imu', method: 'continuous_time', visual_reference_camera: reference,
      initial_intrinsics: {},
      time_offset: { type: 'estimate', initial_s: 0, minimum_s: -0.1, maximum_s: 0.1, initialization_steps: 11 },
      imu: defaultImu(),
    };
  }
  if (cameras.length >= 2) {
    return {
      type: 'multi_camera', reference_camera: reference, refine_intrinsics: true,
      initial_intrinsics: {}, max_closure_rotation_deg: 2, max_closure_translation_m: 0.05,
    };
  }
  return { type: 'single_camera', camera: reference, initial_intrinsics: {} };
}

function topicUsable(topic) {
  return topic && topic.status === 'ok' && topic.header_time_valid === true && Number(topic.message_count) > 0;
}

function cameraId(index) {
  return `cam${index}`;
}

const topicOrder = new Intl.Collator('en', { numeric: true, sensitivity: 'variant' });
function compareTopicPaths(a, b) {
  return topicOrder.compare(a, b) || (a < b ? -1 : a > b ? 1 : 0);
}

function sortedTopics(inspection) {
  return Array.isArray(inspection?.topics) ? [...inspection.topics].sort((a, b) => compareTopicPaths(a.name, b.name)) : [];
}

export function createDefaultConfig(inspection = null, options = {}) {
  const topics = sortedTopics(inspection);
  const imageTopics = topics.filter((topic) => IMAGE_TYPES.has(topic.type_name) && topicUsable(topic));
  const cameras = imageTopics.map((topic, index) => ({
    id: cameraId(index), topic: topic.name, model: 'double-sphere', pixel_sigma: 1,
    timing: { timestamp_reference: 'exposure_midpoint', fixed_exposure_duration_s: null, exposure_metadata_topic: null },
  }));
  const imuTopic = topics.find((topic) => topic.type_name === IMU_TYPE && topicUsable(topic))?.name ?? null;
  return {
    source: {
      path: options.path || inspection?.source || 'authorized-source',
      format: options.format || 'ros1_v2', imu_topic: imuTopic,
      synchronization: { max_delta_ns: 2000000, require_all_cameras: true },
    },
    cameras,
    target: defaultTarget(),
    detector: defaultDetector(),
    calibration: automaticCalibration(cameras, imuTopic),
    solver: defaultSolver(),
  };
}

function normalizeConfig(value) {
  const base = createDefaultConfig();
  const input = value && typeof value === 'object' ? clone(value) : {};
  const output = {
    ...base,
    ...input,
    source: {
      ...base.source,
      ...(input.source || {}),
      synchronization: { ...base.source.synchronization, ...(input.source?.synchronization || {}) },
    },
    cameras: Array.isArray(input.cameras) ? input.cameras.map((camera) => ({
      id: '', topic: '', model: 'double-sphere', pixel_sigma: 1,
      timing: { timestamp_reference: 'exposure_midpoint', fixed_exposure_duration_s: null, exposure_metadata_topic: null },
      ...camera,
      timing: {
        timestamp_reference: 'exposure_midpoint', fixed_exposure_duration_s: null, exposure_metadata_topic: null,
        ...(camera?.timing || {}),
      },
    })) : base.cameras,
    target: { ...base.target, ...(input.target || {}) },
    detector: {
      ...base.detector, ...(input.detector || {}),
      refinement: { ...base.detector.refinement, ...(input.detector?.refinement || {}) },
      budgets: { ...base.detector.budgets, ...(input.detector?.budgets || {}) },
    },
    solver: { ...base.solver, ...(input.solver || {}) },
  };
  const calibration = input.calibration || automaticCalibration(output.cameras, output.source.imu_topic);
  output.calibration = normalizeCalibration(calibration, output.cameras);
  return output;
}

function normalizeCalibration(calibration, cameras) {
  const reference = cameras[0]?.id || '';
  const intrinsics = initialIntrinsics(calibration?.initial_intrinsics?.path || null);
  if (calibration?.type === 'multi_camera') {
    return {
      type: 'multi_camera', reference_camera: calibration.reference_camera ?? reference,
      refine_intrinsics: calibration.refine_intrinsics ?? true, initial_intrinsics: intrinsics,
      max_closure_rotation_deg: calibration.max_closure_rotation_deg ?? 2,
      max_closure_translation_m: calibration.max_closure_translation_m ?? 0.05,
    };
  }
  if (calibration?.type === 'multi_camera_imu') {
    const imu = calibration.imu || {};
    const defaults = defaultImu();
    return {
      type: 'multi_camera_imu', method: calibration.method || 'continuous_time',
      visual_reference_camera: calibration.visual_reference_camera ?? reference,
      initial_intrinsics: intrinsics,
      time_offset: calibration.time_offset?.type === 'fixed'
        ? { type: 'fixed', value_s: calibration.time_offset.value_s ?? 0 }
        : {
            type: 'estimate', initial_s: calibration.time_offset?.initial_s ?? 0,
            minimum_s: calibration.time_offset?.minimum_s ?? -0.1,
            maximum_s: calibration.time_offset?.maximum_s ?? 0.1,
            initialization_steps: calibration.time_offset?.initialization_steps ?? 11,
          },
      imu: {
        ...defaults, ...imu,
        noise: { ...defaults.noise, ...(imu.noise || {}) },
        integration: { ...defaults.integration, ...(imu.integration || {}) },
        axis_calibration: { ...defaults.axis_calibration, ...(imu.axis_calibration || {}) },
        continuous_time: {
          ...defaults.continuous_time, ...(imu.continuous_time || {}),
          camera_time_offsets: { ...defaults.continuous_time.camera_time_offsets, ...(imu.continuous_time?.camera_time_offsets || {}) },
          clock_skews: { ...defaults.continuous_time.clock_skews, ...(imu.continuous_time?.clock_skews || {}) },
          visual_motion_anisotropy: { ...defaults.continuous_time.visual_motion_anisotropy, ...(imu.continuous_time?.visual_motion_anisotropy || {}) },
        },
      },
    };
  }
  return { type: 'single_camera', camera: calibration?.camera ?? reference, initial_intrinsics: intrinsics };
}

function matrixInputs(name, matrix, labels = ['X', 'Y', 'Z']) {
  const rows = matrix || identity();
  return `<div class="cf-matrix" role="group" aria-label="${escapeHtml(name)}">${rows.map((row, rowIndex) =>
    `<div class="row"><span class="small">${labels[rowIndex]}</span>${row.map((value, columnIndex) =>
      `<label class="small">${labels[rowIndex]}${labels[columnIndex]}<input type="number" step="any" data-matrix="${escapeHtml(name)}" data-row="${rowIndex}" data-column="${columnIndex}" value="${escapeHtml(value)}"></label>`).join('')}</div>`).join('')}</div>`;
}

function topicRate(topic) {
  const first = Number(topic.first_record_time_ns);
  const last = Number(topic.last_record_time_ns);
  const count = Number(topic.message_count);
  if (!Number.isFinite(first) || !Number.isFinite(last) || count < 2 || last <= first) return '—';
  return `${((count - 1) * 1e9 / (last - first)).toFixed(1)} Hz（记录时间）`;
}

export class ConfigurationForms {
  constructor({ sensors, target, solver, onChange = null, onIssues = null, files = [], sourceFiles = [], auxiliaryFiles = [] } = {}) {
    if (!(sensors instanceof Element) || !(target instanceof Element) || !(solver instanceof Element)) {
      throw new TypeError('ConfigurationForms requires sensors, target, and solver container Elements');
    }
    this.containers = { sensors, target, solver };
    this.onChange = typeof onChange === 'function' ? onChange : null;
    this.onIssues = typeof onIssues === 'function' ? onIssues : null;
    this.inspection = null;
    this.config = createDefaultConfig();
    this._calibrationDrafts = new Map([[this.config.calibration.type, clone(this.config.calibration)]]);
    this.advancedCameraTopic = null;
    this.auxiliaryFiles = [];
    this._boundInput = (event) => this._handleInput(event);
    for (const container of Object.values(this.containers)) container.classList.add('cf-form');
    for (const container of Object.values(this.containers)) {
      container.addEventListener('change', this._boundInput);
      container.addEventListener('input', this._boundInput);
    }
    this.setAuxiliaryFiles(auxiliaryFiles.length ? auxiliaryFiles : sourceFiles.length ? sourceFiles : files, false);
    this.render();
  }

  destroy() {
    for (const container of Object.values(this.containers)) {
      container.removeEventListener('change', this._boundInput);
      container.removeEventListener('input', this._boundInput);
      container.classList.remove('cf-form');
      container.replaceChildren();
    }
  }

  setAuxiliaryFiles(files, rerender = true) {
    this.auxiliaryFiles = (Array.isArray(files) ? files : []).map((file) => typeof file === 'string' ? file : file?.relativePath)
      .filter((path, index, all) => typeof path === 'string' && path && all.indexOf(path) === index)
      .filter((path) => /\.(?:ya?ml|json)$/i.test(path));
    if (rerender) this.render();
  }

  setInspection(inspection, options = {}) {
    const old = this.config;
    const preserveSelection = old.cameras.length > 0 || Boolean(this.inspection?.topics?.length);
    this.inspection = inspection && typeof inspection === 'object' ? clone(inspection) : null;
    if (options.files || options.sourceFiles || options.auxiliaryFiles) {
      this.setAuxiliaryFiles(options.auxiliaryFiles || options.sourceFiles || options.files, false);
    }
    const generated = createDefaultConfig(this.inspection, {
      format: options.format || old.source?.format,
      path: options.path || old.source?.path || this.inspection?.source,
    });
    const topics = Array.isArray(this.inspection?.topics) ? this.inspection.topics : [];
    const retainedCameras = (old.cameras || []).filter((camera) => topics.some((topic) => topic.name === camera.topic && IMAGE_TYPES.has(topic.type_name) && topicUsable(topic)));
    if (preserveSelection) {
      generated.cameras = clone(retainedCameras);
      const retainedImu = topics.some((topic) => topic.name === old.source?.imu_topic && topic.type_name === IMU_TYPE && topicUsable(topic));
      generated.source.imu_topic = retainedImu ? old.source.imu_topic : null;
    }
    generated.target = clone(old.target || generated.target);
    generated.detector = clone(old.detector || generated.detector);
    generated.solver = clone(old.solver || generated.solver);
    generated.calibration = clone(old.calibration || generated.calibration);
    this.config = normalizeConfig(generated);
    this._calibrationDrafts.set(old.calibration.type, clone(old.calibration));
    this._deriveCalibrationMode();
    this.render();
    this._notify();
  }

  setConfig(config) {
    this.config = normalizeConfig(config);
    this._calibrationDrafts = new Map([[this.config.calibration.type, clone(this.config.calibration)]]);
    this._deriveCalibrationMode();
    this.render();
    this._notifyIssues();
  }

  getConfig() {
    return clone(this.config);
  }

  getIssues() {
    const config = this.config;
    const issues = [];
    const add = (field, message, code = 'INVALID_CONFIGURATION') => issues.push({ code, field, message, blocking: true });
    if (!config.source?.path || !String(config.source.path).trim()) add('source.path', '数据源路径/授权绑定不能为空。');
    if (!['ros1_v2', 'ros2_mcap', 'ros2_sqlite3'].includes(config.source?.format)) add('source.format', '请选择支持的输入格式。');
    if (!Number.isSafeInteger(Number(config.source?.synchronization?.max_delta_ns)) || Number(config.source.synchronization.max_delta_ns) < 0) add('source.synchronization.max_delta_ns', '同步容差必须是非负安全整数纳秒值。');

    if (!Array.isArray(config.cameras) || !config.cameras.length) add('cameras', '至少选择一个可用的图像话题。');
    const ids = new Set();
    const cameraTopics = new Set();
    for (const [index, camera] of (config.cameras || []).entries()) {
      if (!camera.id?.trim()) add(`cameras.${index}.id`, '相机 ID 不能为空。');
      else if (ids.has(camera.id)) add(`cameras.${index}.id`, `相机 ID ${camera.id} 重复。`);
      ids.add(camera.id);
      if (!camera.topic?.trim()) add(`cameras.${index}.topic`, '图像话题不能为空。');
      else if (cameraTopics.has(camera.topic)) add(`cameras.${index}.topic`, `图像话题 ${camera.topic} 重复。`);
      cameraTopics.add(camera.topic);
      if (!CAMERA_MODELS.some(([value]) => value === camera.model)) add(`cameras.${index}.model`, '相机模型不受支持。');
      if (!finite(camera.pixel_sigma) || Number(camera.pixel_sigma) <= 0) add(`cameras.${index}.pixel_sigma`, '像素标准差必须为正数。');
      const timing = camera.timing || {};
      if (!['exposure_midpoint', 'trigger'].includes(timing.timestamp_reference)) add(`cameras.${index}.timing.timestamp_reference`, '曝光时间戳语义不受支持。');
      if (timing.fixed_exposure_duration_s != null && (!finite(timing.fixed_exposure_duration_s) || Number(timing.fixed_exposure_duration_s) <= 0)) add(`cameras.${index}.timing.fixed_exposure_duration_s`, '固定曝光时长必须为正数。');
      if (timing.fixed_exposure_duration_s != null && timing.exposure_metadata_topic) add(`cameras.${index}.timing`, '固定曝光时长和曝光元数据话题只能选择一个。');
      if (timing.timestamp_reference === 'trigger' && timing.fixed_exposure_duration_s == null && !timing.exposure_metadata_topic) add(`cameras.${index}.timing`, 'Trigger 时间戳必须提供曝光时长来源。');
    }
    const inspected = Array.isArray(this.inspection?.topics) ? this.inspection.topics : [];
    if (inspected.length) {
      for (const camera of config.cameras || []) {
        const topic = inspected.find((item) => item.name === camera.topic && IMAGE_TYPES.has(item.type_name));
        if (!topic) add('cameras', `扫描结果中不存在图像话题 ${camera.topic}。`, 'TOPIC_NOT_FOUND');
        else if (!topicUsable(topic)) add('cameras', `图像话题 ${camera.topic} 没有可用的 header 时间或消息。`, 'TOPIC_UNUSABLE');
      }
      if (config.source.imu_topic) {
        const topic = inspected.find((item) => item.name === config.source.imu_topic && item.type_name === IMU_TYPE);
        if (!topic) add('source.imu_topic', `扫描结果中不存在 IMU 话题 ${config.source.imu_topic}。`, 'TOPIC_NOT_FOUND');
        else if (!topicUsable(topic)) add('source.imu_topic', `IMU 话题 ${config.source.imu_topic} 没有可用的 header 时间或消息。`, 'TOPIC_UNUSABLE');
      }
    }

    const calibration = config.calibration || {};
    if (calibration.type === 'single_camera') {
      if (config.cameras.length !== 1) add('calibration.type', '单相机模式必须且只能启用一路相机。', 'UNSUPPORTED_COMBINATION');
      if (config.source.imu_topic) add('source.imu_topic', '单相机模式不能绑定 IMU。', 'UNSUPPORTED_COMBINATION');
      if (!ids.has(calibration.camera)) add('calibration.camera', '单相机选择必须引用已启用的相机 ID。');
    } else if (calibration.type === 'multi_camera') {
      if (config.cameras.length < 2) add('calibration.type', '多相机模式至少需要两路相机。', 'UNSUPPORTED_COMBINATION');
      if (config.source.imu_topic) add('source.imu_topic', '纯视觉多相机模式不能绑定 IMU。', 'UNSUPPORTED_COMBINATION');
      if (!ids.has(calibration.reference_camera)) add('calibration.reference_camera', '参考相机必须属于已启用相机。');
      if (calibration.refine_intrinsics === false && !calibration.initial_intrinsics?.path) add('calibration.initial_intrinsics', '不精化内参时必须选择已授权的初始内参文件。');
      if (!finite(calibration.max_closure_rotation_deg) || Number(calibration.max_closure_rotation_deg) <= 0) add('calibration.max_closure_rotation_deg', '闭环旋转容差必须为正数。');
      if (!finite(calibration.max_closure_translation_m) || Number(calibration.max_closure_translation_m) <= 0) add('calibration.max_closure_translation_m', '闭环平移容差必须为正数。');
    } else if (calibration.type === 'multi_camera_imu') {
      if (config.cameras.length < 2) add('calibration.type', '相机 + IMU 模式至少需要两路相机。', 'UNSUPPORTED_COMBINATION');
      if (!config.source.imu_topic) add('source.imu_topic', '相机 + IMU 模式必须选择一个扫描所得 IMU 话题。');
      if (!ids.has(calibration.visual_reference_camera)) add('calibration.visual_reference_camera', '视觉参考相机必须属于已启用相机。');
      this._validateInertial(calibration, add);
    } else add('calibration.type', '请选择受支持的标定模式。');

    this._validateTarget(config.target, add);
    this._validateDetector(config.detector, add);
    if (!Number.isInteger(Number(config.solver?.max_iterations)) || Number(config.solver.max_iterations) < 1 || Number(config.solver.max_iterations) > 1000) add('solver.max_iterations', '求解迭代预算必须在 1..1000。');
    if (!finite(config.solver?.huber_delta) || Number(config.solver.huber_delta) <= 0) add('solver.huber_delta', 'Huber 转折必须为正数。');
    if (config.solver?.point_rejection_px == null && !(calibration.type === 'multi_camera_imu' && calibration.method === 'continuous_time')) add('solver.point_rejection_px', '仅连续时间联合模式支持不做硬剔除。', 'UNSUPPORTED_COMBINATION');
    if (config.solver?.point_rejection_px != null && (!finite(config.solver.point_rejection_px) || Number(config.solver.point_rejection_px) <= 0)) add('solver.point_rejection_px', '硬剔除阈值必须为正数。');

    const path = calibration.initial_intrinsics?.path;
    if (path && this.auxiliaryFiles.length && !this.auxiliaryFiles.includes(path)) add('calibration.initial_intrinsics.path', '初始内参文件不在当前已授权的辅助文件中。', 'FILE_NOT_AUTHORIZED');
    return issues;
  }

  render() {
    this._renderSensors();
    this._renderTarget();
    this._renderSolver();
    this._notifyIssues();
  }

  _renderSensors() {
    const topics = sortedTopics(this.inspection);
    const selectedByTopic = new Map(this.config.cameras.map((camera) => [camera.topic, camera]));
    const reason = (topic) => {
      if (!IMAGE_TYPES.has(topic.type_name) && topic.type_name !== IMU_TYPE) return `不支持的消息类型：${topic.type_name || '未知'}`;
      if (topic.status !== 'ok') return `扫描状态：${topic.status || '未知错误'}`;
      if (topic.header_time_valid !== true) return 'Header 时间不可用';
      if (!(Number(topic.message_count) > 0)) return '没有消息';
      return '';
    };
    const rows = topics.length ? topics.map((topic) => {
      const camera = selectedByTopic.get(topic.name);
      const kind = IMAGE_TYPES.has(topic.type_name) ? '图像' : topic.type_name === IMU_TYPE ? 'IMU' : '不支持';
      const unavailable = reason(topic);
      const cameraControls = IMAGE_TYPES.has(topic.type_name) ? `<input aria-label="启用 ${escapeHtml(topic.name)}" type="checkbox" data-camera-enabled="${escapeHtml(topic.name)}"${checked(Boolean(camera))}${unavailable ? ' disabled' : ''}>` : topic.type_name === IMU_TYPE ? `<input aria-label="选择 ${escapeHtml(topic.name)}" type="radio" name="cf-imu-topic" data-imu-choice="${escapeHtml(topic.name)}"${checked(this.config.source.imu_topic === topic.name)}${unavailable ? ' disabled' : ''}>` : '<input aria-label="不支持" type="checkbox" disabled>';
      return `<tr data-topic-row="${escapeHtml(topic.name)}"><td data-label="启用">${cameraControls}</td><td data-label="Topic / 类型" class="cf-topic-name"><span class="mono">${escapeHtml(topic.name)}</span><br><span class="small">${escapeHtml(topic.type_name || '未知类型')}</span></td><td data-label="类别 / 数量 / 频率">${kind}<br><span class="small">${escapeHtml(topic.message_count ?? '—')} 条 · ${topicRate(topic)}</span></td><td data-label="状态">${unavailable ? `<span class="small" title="${escapeHtml(unavailable)}">${escapeHtml(unavailable)}</span>` : '<span class="badge ok">可用</span>'}</td><td data-label="相机 ID">${camera ? `<input aria-label="相机 ID" data-camera-field="id" data-camera-topic="${escapeHtml(topic.name)}" value="${escapeHtml(camera.id)}" autocomplete="off">` : '—'}</td><td data-label="标定模型">${camera ? `<select aria-label="相机模型" data-camera-field="model" data-camera-topic="${escapeHtml(topic.name)}">${CAMERA_MODELS.map(([value, label]) => `<option value="${value}"${selected(camera.model, value)}>${label}</option>`).join('')}</select>` : '—'}</td></tr>`;
    }).join('') : '<tr class="cf-topic-empty"><td colspan="6" class="small">尚无扫描所得话题。不会生成示例观测或虚构话题。</td></tr>';
    const advancedCamera = this.config.cameras.find((camera) => camera.topic === this.advancedCameraTopic) || this.config.cameras[0] || null;
    this.advancedCameraTopic = advancedCamera?.topic || null;
    const timing = advancedCamera?.timing || {};
    const durationSource = timing.exposure_metadata_topic ? 'metadata' : timing.fixed_exposure_duration_s != null ? 'fixed' : 'none';
    const advancedSettings = advancedCamera ? `<details class="cf-camera-advanced" open><summary>相机高级设置：像素噪声、曝光与时间戳</summary><label>编辑相机<select data-camera-advanced-select>${this.config.cameras.map((camera) => `<option value="${escapeHtml(camera.topic)}"${selected(advancedCamera.topic, camera.topic)}>${escapeHtml(camera.id)} · ${escapeHtml(camera.topic)}</option>`).join('')}</select></label><div class="cf-camera-advanced-grid"><label>角点像素标准差（px）<input type="number" min="0" step="any" data-camera-field="pixel_sigma" data-camera-topic="${escapeHtml(advancedCamera.topic)}" value="${escapeHtml(advancedCamera.pixel_sigma)}"></label><label>Header 时间含义<select data-camera-field="timestamp_reference" data-camera-topic="${escapeHtml(advancedCamera.topic)}"><option value="exposure_midpoint"${selected(timing.timestamp_reference, 'exposure_midpoint')}>曝光中点</option><option value="trigger"${selected(timing.timestamp_reference, 'trigger')}>曝光开始 Trigger</option></select></label><label>曝光时长来源<select data-camera-field="duration_source" data-camera-topic="${escapeHtml(advancedCamera.topic)}"><option value="none"${selected(durationSource, 'none')}>不提供（中点时间戳可用）</option><option value="fixed"${selected(durationSource, 'fixed')}>固定时长</option><option value="metadata"${selected(durationSource, 'metadata')}>std_msgs/String 元数据话题</option></select></label><label${durationSource === 'fixed' ? '' : ' hidden'}>固定曝光时长（s）<input type="number" min="0" step="any" data-camera-field="fixed_exposure_duration_s" data-camera-topic="${escapeHtml(advancedCamera.topic)}" value="${escapeHtml(timing.fixed_exposure_duration_s ?? 0.01)}"></label><label${durationSource === 'metadata' ? '' : ' hidden'}>曝光元数据话题<input data-camera-field="exposure_metadata_topic" data-camera-topic="${escapeHtml(advancedCamera.topic)}" value="${escapeHtml(timing.exposure_metadata_topic || '')}" placeholder="/camera/frame_metadata"></label></div></details>` : '';
    this.containers.sensors.innerHTML = `<div class="panel"><div class="row"><h2>扫描所得传感器</h2><span class="badge">${topics.length} 个话题</span></div><p class="small">勾选图像或单选 IMU 后自动推断标定模式；不可用与不支持的话题保持只读并说明原因。</p><div class="cf-topic-table-wrap"><table class="cf-topic-table"><thead><tr><th>启用</th><th>Topic / 类型</th><th>类别 / 数量 / 频率</th><th>状态</th><th>相机 ID</th><th>标定模型</th></tr></thead><tbody>${rows}</tbody></table></div><label><input type="radio" name="cf-imu-topic" data-imu-choice=""${checked(!this.config.source.imu_topic)}> 不使用 IMU</label>${advancedSettings}<details><summary>高级同步设置</summary><label>最大近同步差（ns）<input type="number" min="0" step="1" data-field="max_delta_ns" value="${escapeHtml(this.config.source.synchronization.max_delta_ns)}"></label><label><input type="checkbox" data-field="require_all_cameras"${checked(this.config.source.synchronization.require_all_cameras)}> 每个近同步组要求所有相机都有原始帧</label></details></div>`;
    const hint = document.createElement('p'); hint.className = 'small';
    const imuCount = topics.filter((topic) => topic.type_name === IMU_TYPE && topicUsable(topic)).length;
    hint.textContent = '首次解析默认启用全部可用图像话题，按路径自然排序分配 cam0、cam1…；重新解析保留已有选择与 ID。' + (imuCount > 1 ? ` 当前有 ${imuCount} 路可用 IMU；求解器只支持一路，首次默认选择排序后的第一路，可在表中切换。` : ' 有可用 IMU 时默认启用，并自动推断联合标定模式。');
    this.containers.sensors.querySelector('.cf-topic-table-wrap').before(hint);
  }

  _renderTarget() {
    const target = this.config.target;
    const rows = target.id_rows_front_view_left_to_right || [];
    const rowCount = rows.length || 1;
    const columnCount = rows[0]?.length || 1;
    const values = rows.flat();
    const previewValid = rows.length > 0 && rows.every((row) => row.length === columnCount) && values.every((value) => Number.isInteger(Number(value)) && Number(value) >= 0) && new Set(values.map(Number)).size === values.length;
    const width = columnCount * Number(target.tag_size_m) + Math.max(0, columnCount - 1) * Number(target.tag_gap_m);
    const height = rowCount * Number(target.tag_size_m) + Math.max(0, rowCount - 1) * Number(target.tag_gap_m);
    const boardCells = rows.map((row, rowIndex) => row.map((value, columnIndex) => `<label class="small">R${rowIndex + 1} C${columnIndex + 1}<input type="number" min="0" max="4294967295" step="1" data-board-id data-row="${rowIndex}" data-column="${columnIndex}" value="${escapeHtml(value)}"></label>`).join('')).join('');
    const cornerStart = Number(target.decoded_corner_to_front_corner?.[0] ?? 0);
    this.containers.target.innerHTML = `<div class="cards"><div class="panel"><h2>AprilGrid 几何</h2>
      <label>Tag family<select data-target="family">${TAG_FAMILIES.map((family) => `<option${selected(target.family, family)}>${family}</option>`).join('')}</select></label>
      <label>黑边宽度（bit）<select data-target="border_bits"><option value="1"${selected(Number(target.border_bits), 1)}>1</option><option value="2"${selected(Number(target.border_bits), 2)}>2</option></select></label>
      <label>Tag 边长（m）<input type="number" min="0" step="any" data-target="tag_size_m" value="${escapeHtml(target.tag_size_m)}"></label>
      <label>相邻 Tag 间隙（m，不是比例）<input type="number" min="0" step="any" data-target="tag_gap_m" value="${escapeHtml(target.tag_gap_m)}"></label>
      <div class="row"><label>行数<input type="number" min="1" max="64" step="1" data-board-dimension="rows" value="${rowCount}"></label><label>列数<input type="number" min="1" max="64" step="1" data-board-dimension="columns" value="${columnCount}"></label></div>
      <label>解码角点 0 对应正视角点<select data-target="corner_start"><option value="0"${selected(cornerStart, 0)}>0 · 左上（映射 0,1,2,3）</option><option value="1"${selected(cornerStart, 1)}>1 · 右上（映射 1,2,3,0）</option><option value="2"${selected(cornerStart, 2)}>2 · 右下（映射 2,3,0,1）</option><option value="3"${selected(cornerStart, 3)}>3 · 左下（映射 3,0,1,2）</option></select></label>
      </div><div class="panel"><div class="row"><h2>正视 ID 布局 · 直接编辑</h2><span class="badge${previewValid ? ' ok' : ' error'}">${rowCount} × ${columnCount}</span></div><p class="small">每个格子对应一个 Tag，直接修改其中的 ID。正视从左到右、从上到下；示意不可打印。</p><div class="cf-id-grid cf-board-editor" aria-label="可编辑标定板 ID 布局" style="display:grid;grid-template-columns:repeat(${columnCount},minmax(60px,1fr));gap:8px;overflow:auto">${boardCells}</div><p class="small">板尺寸：${finite(width) ? width.toFixed(4) : '—'} × ${finite(height) ? height.toFixed(4) : '—'} m</p>${previewValid ? '' : '<p class="alert">布局必须为非负 u32 ID 且不能重复。</p>'}</div></div>
      <details class="panel"><summary>高级检测设置</summary><label>最大 Hamming 纠错<select data-detector="max_hamming">${[0, 1, 2, 3].map((value) => `<option value="${value}"${selected(Number(this.config.detector.max_hamming), value)}>${value}</option>`).join('')}</select></label><label>角点细化<select data-detector="refinement_method"><option value="saddle"${selected(this.config.detector.refinement.method, 'saddle')}>Saddle</option><option value="outer_contour"${selected(this.config.detector.refinement.method, 'outer_contour')}>Outer contour</option><option value="disabled"${selected(this.config.detector.refinement.method, 'disabled')}>禁用</option></select></label><label>细化迭代上限<input type="number" min="1" step="1" data-detector="refinement_iterations" value="${escapeHtml(this.config.detector.refinement.max_iterations)}"></label><label>最大图像像素<input type="number" min="1" step="1" data-detector="max_pixels" value="${escapeHtml(this.config.detector.budgets.max_pixels)}"></label><label>最大聚类数<input type="number" min="1" step="1" data-detector="max_clusters" value="${escapeHtml(this.config.detector.budgets.max_clusters)}"></label><label>最大候选数<input type="number" min="1" step="1" data-detector="max_candidates" value="${escapeHtml(this.config.detector.budgets.max_candidates)}"></label></details>`;
  }

  _renderSolver() {
    const calibration = this.config.calibration;
    const reference = calibration.type === 'single_camera' ? calibration.camera : calibration.type === 'multi_camera' ? calibration.reference_camera : calibration.visual_reference_camera;
    const cameraOptions = this.config.cameras.map((camera) => `<option value="${escapeHtml(camera.id)}"${selected(reference, camera.id)}>${escapeHtml(camera.id)} · ${escapeHtml(camera.topic)}</option>`).join('');
    const path = calibration.initial_intrinsics?.path || '';
    const fileOptions = [`<option value=""${path ? '' : ' selected'}>请选择已授权文件</option>`, ...this.auxiliaryFiles.map((file) => `<option value="${escapeHtml(file)}"${selected(path, file)}>${escapeHtml(file)}</option>`), ...(path && !this.auxiliaryFiles.includes(path) ? [`<option value="${escapeHtml(path)}" selected>${escapeHtml(path)}（当前配置）</option>`] : [])].join('');
    const solver = this.config.solver;
    const modeLabel = calibration.type === 'single_camera' ? '单相机内参' : calibration.type === 'multi_camera' ? '多相机纯视觉' : '多相机 + IMU';
    this.containers.solver.innerHTML = `<div class="panel"><div class="row"><h2>标定配置</h2><span class="badge">自动模式：${modeLabel}</span></div>
      <label>${calibration.type === 'single_camera' ? '标定相机' : calibration.type === 'multi_camera' ? '参考相机' : '视觉参考相机'}<select data-calibration="reference">${cameraOptions}</select></label>
      <label>内参初值策略<select data-calibration="intrinsics_strategy"><option value="automatic"${path ? '' : ' selected'}>自动初始化</option><option value="file"${path ? ' selected' : ''}${this.auxiliaryFiles.length || path ? '' : ' disabled'}>使用已授权结果文件</option></select></label>
      <label${path ? '' : ' hidden'}>初始内参文件<select data-calibration="intrinsics_path">${fileOptions}</select></label>
      ${calibration.type === 'multi_camera' ? `<label><input type="checkbox" data-calibration="refine_intrinsics"${checked(calibration.refine_intrinsics)}> 联合精化内参</label><details><summary>多相机闭环初始化容差</summary><label>旋转（deg）<input type="number" min="0" step="any" data-calibration="max_closure_rotation_deg" value="${escapeHtml(calibration.max_closure_rotation_deg)}"></label><label>平移（m）<input type="number" min="0" step="any" data-calibration="max_closure_translation_m" value="${escapeHtml(calibration.max_closure_translation_m)}"></label></details>` : ''}
      </div>${calibration.type === 'multi_camera_imu' ? this._inertialHtml(calibration) : ''}
      <details class="panel" open><summary>求解器与鲁棒损失</summary><label>最大 LM 迭代数（1..1000）<input type="number" min="1" max="1000" step="1" data-solver="max_iterations" value="${escapeHtml(solver.max_iterations)}"></label><label>Huber 转折（白化残差单位）<input type="number" min="0" step="any" data-solver="huber_delta" value="${escapeHtml(solver.huber_delta)}"></label><label>硬剔除策略<select data-solver="rejection_mode"><option value="threshold"${solver.point_rejection_px == null ? '' : ' selected'}>使用像素阈值</option><option value="huber_only"${solver.point_rejection_px == null ? ' selected' : ''}${calibration.type === 'multi_camera_imu' && calibration.method === 'continuous_time' ? '' : ' disabled'}>仅 Huber，不硬剔除（仅连续时间）</option></select></label><label${solver.point_rejection_px == null ? ' hidden' : ''}>硬剔除阈值（px）<input type="number" min="0" step="any" data-solver="point_rejection_px" value="${escapeHtml(solver.point_rejection_px ?? 6)}"></label></details>
      <div class="panel"><h2>当前配置状态</h2><ul class="checks">${this.getIssues().length ? this.getIssues().map((issue) => `<li class="blocking"><strong>阻断</strong>${escapeHtml(issue.message)}</li>`).join('') : '<li class="ok"><strong>表单完整</strong>仍需 Rust 权威验证与数据预检。</li>'}</ul></div>`;
  }

  _inertialHtml(calibration) {
    const imu = calibration.imu;
    const time = calibration.time_offset;
    const continuous = imu.continuous_time;
    const isContinuous = calibration.method === 'continuous_time';
    return `<div class="panel"><h2>IMU、时间对齐与积分</h2>
      <label>惯性方法<select data-calibration="method"><option value="continuous_time"${selected(calibration.method, 'continuous_time')}>连续时间四次 B 样条</option><option value="preintegration"${selected(calibration.method, 'preintegration')}>预积分</option></select></label>
      <label>相机—IMU 主时差<select data-time-offset="type"><option value="estimate"${selected(time.type, 'estimate')}>估计</option><option value="fixed"${selected(time.type, 'fixed')}>固定</option></select></label>
      ${time.type === 'fixed' ? `<label>固定时差 value_s（t_imu = t_camera + Δt）<input type="number" step="any" data-time-offset="value_s" value="${escapeHtml(time.value_s)}"></label>` : `<div class="cards"><label>初值（s）<input type="number" step="any" data-time-offset="initial_s" value="${escapeHtml(time.initial_s)}"></label><label>初始化搜索步数（2..101）<input type="number" min="2" max="101" step="1" data-time-offset="initialization_steps" value="${escapeHtml(time.initialization_steps)}"></label><label>下界（s）<input type="number" step="any" data-time-offset="minimum_s" value="${escapeHtml(time.minimum_s)}"></label><label>上界（s）<input type="number" step="any" data-time-offset="maximum_s" value="${escapeHtml(time.maximum_s)}"></label></div>`}
      <label>当地重力模长（m/s²）<input type="number" min="0" step="any" data-imu="gravity" value="${escapeHtml(imu.gravity_magnitude_m_s2)}"></label>
      <label>积分方法<select data-imu="integration_method"><option value="midpoint">Midpoint（当前唯一支持）</option></select></label><label>IMU 最大样本间隔（s）<input type="number" min="0" step="any" data-imu="max_gap_s" value="${escapeHtml(imu.integration.max_gap_s)}"></label><label>IMU 最大样本数<input type="number" min="2" step="1" data-imu="max_samples" value="${escapeHtml(imu.integration.max_samples)}"></label></div>
      <details class="panel"><summary>IMU 噪声 PSD 与 bias random walk</summary><p class="small">矩阵位于校正后的物理坐标系；连续时间使用 covariance = Q / dt。预积分只支持各向同性对角 PSD。</p><h3>陀螺白噪声 PSD（rad²/s/Hz）</h3>${matrixInputs('gyro_covariance', imu.noise.gyro_covariance_rad2_s_hz)}<h3>加速度计白噪声 PSD（m²/s⁴/Hz）</h3>${matrixInputs('accelerometer_covariance', imu.noise.accelerometer_covariance_m2_s4_hz)}<label>陀螺 bias walk（rad/s²/√Hz）<input type="number" min="0" step="any" data-imu="gyro_bias_walk" value="${escapeHtml(imu.noise.gyro_bias_walk_rad_s2_sqrt_hz)}"></label><label>加速度计 bias walk（m/s³/√Hz）<input type="number" min="0" step="any" data-imu="accelerometer_bias_walk" value="${escapeHtml(imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz)}"></label></details>
      <details class="panel"${isContinuous ? '' : ' hidden'}><summary>连续时间、相机时钟与曝光运动</summary><label>四次 B 样条 knot 间隔（s）<input type="number" min="0" step="any" data-continuous="knot_interval_s" value="${escapeHtml(continuous.knot_interval_s)}"></label><label><input type="checkbox" data-continuous="camera_offsets_refine"${checked(continuous.camera_time_offsets.refine)}> 精化非参考相机残余时间偏移</label><div class="cards"><label>相机偏移下界（s）<input type="number" step="any" data-continuous="camera_offsets_minimum_s" value="${escapeHtml(continuous.camera_time_offsets.minimum_s)}"></label><label>相机偏移上界（s）<input type="number" step="any" data-continuous="camera_offsets_maximum_s" value="${escapeHtml(continuous.camera_time_offsets.maximum_s)}"></label></div><label><input type="checkbox" data-continuous="refine_global_skew"${checked(continuous.clock_skews.refine_global)}> 精化 IMU 全局时钟 skew</label><label><input type="checkbox" data-continuous="refine_camera_skews"${checked(continuous.clock_skews.refine_cameras)}> 精化各相机时钟 skew</label><div class="cards"><label>Skew 下界（ppm）<input type="number" step="any" data-continuous="skew_minimum_ppm" value="${escapeHtml(continuous.clock_skews.minimum_ppm)}"></label><label>Skew 上界（ppm）<input type="number" step="any" data-continuous="skew_maximum_ppm" value="${escapeHtml(continuous.clock_skews.maximum_ppm)}"></label></div><label><input type="checkbox" data-continuous="motion_anisotropy_enabled"${checked(continuous.visual_motion_anisotropy.enabled)}> 二次求解使用曝光运动各向异性视觉协方差</label><label>每平方像素运动的方差比增量（px⁻²）<input type="number" min="0" step="any" data-continuous="motion_variance_ratio" value="${escapeHtml(continuous.visual_motion_anisotropy.variance_ratio_per_motion_squared)}"></label></details>
      <details class="panel"${isContinuous ? '' : ' hidden'}><summary>IMU 轴响应矩阵</summary><label><input type="checkbox" data-axis="refine"${checked(imu.axis_calibration.refine)}> 在连续时间第二阶段精化轴参数</label><p class="small">约定 physical = A × raw − bias。陀螺矩阵须有限、非奇异、保向；加速度计矩阵须为正对角下三角。</p><h3>陀螺 A</h3>${matrixInputs('gyro_axis', imu.axis_calibration.gyro_calibrated_from_raw)}<h3>加速度计 A</h3>${matrixInputs('accelerometer_axis', imu.axis_calibration.accelerometer_calibrated_from_raw)}</details>`;
  }


  _handleInput(event) {
    const target = event.target;
    if (!(target instanceof HTMLInputElement || target instanceof HTMLSelectElement || target instanceof HTMLTextAreaElement)) return;
    if (target.matches('[data-board-dimension]')) {
      if (event.type !== 'change') return;
      this._readForm();
      const rows = Math.max(1, Math.min(64, integer(this._find('[data-board-dimension="rows"]').value) || 1));
      const columns = Math.max(1, Math.min(64, integer(this._find('[data-board-dimension="columns"]').value) || 1));
      this._resizeBoard(rows, columns);
      this.render();
      this._notify();
      return;
    }
    const structural = target.matches('[data-camera-enabled],[data-imu-choice],[data-camera-advanced-select],[data-calibration="method"],[data-calibration="intrinsics_strategy"],[data-time-offset="type"],[data-solver="rejection_mode"]');
    if (structural && event.type !== 'change') return;
    this._readForm();
    if (target.matches('[data-calibration="intrinsics_strategy"]') && target.value === 'file' && !this.config.calibration.initial_intrinsics?.path) {
      this.config.calibration.initial_intrinsics = initialIntrinsics(this.auxiliaryFiles[0] || null);
    }
    if (target.matches('[data-camera-enabled]')) this._syncCameraSelection(target.dataset.cameraEnabled, target.checked);
    if (target.matches('[data-imu-choice]')) this.config.source.imu_topic = target.dataset.imuChoice || null;
    if (target.matches('[data-camera-enabled],[data-imu-choice]')) this._deriveCalibrationMode();
    if (target.matches('[data-camera-advanced-select]')) this.advancedCameraTopic = target.value;
    if (target.matches('[data-camera-field="id"]')) {
      const option = this.containers.sensors.querySelector(`[data-camera-advanced-select] option[value="${CSS.escape(target.dataset.cameraTopic)}"]`);
      if (option) option.textContent = `${target.value} · ${target.dataset.cameraTopic}`;
    }
    if (target.matches('[data-camera-field="duration_source"]')) {
      const fixed = this.containers.sensors.querySelector('[data-camera-field="fixed_exposure_duration_s"]')?.closest('label');
      const metadata = this.containers.sensors.querySelector('[data-camera-field="exposure_metadata_topic"]')?.closest('label');
      if (fixed) fixed.hidden = target.value !== 'fixed';
      if (metadata) metadata.hidden = target.value !== 'metadata';
    }
    if (target.matches('[data-calibration="method"]')) this._setInertialMethod(target.value);
    if (structural) this.render();
    else this._renderIssuesOnly();
    this._notify();
  }

  _readForm() {
    const get = (selector) => this._find(selector);
    const source = this.config.source;
    const imu = get('[data-imu-choice]:checked');
    if (imu) source.imu_topic = imu.dataset.imuChoice || null;
    const delta = get('[data-field="max_delta_ns"]');
    if (delta) source.synchronization.max_delta_ns = integer(delta.value);
    const requireAll = get('[data-field="require_all_cameras"]');
    if (requireAll) source.synchronization.require_all_cameras = requireAll.checked;
    for (const camera of this.config.cameras) {
      const selector = `[data-camera-topic="${CSS.escape(camera.topic)}"]`;
      const field = (name) => get(`${selector}[data-camera-field="${name}"]`);
      const id = field('id');
      const model = field('model');
      const pixelSigma = field('pixel_sigma');
      const timestampReference = field('timestamp_reference');
      const durationSource = field('duration_source');
      if (id) camera.id = id.value;
      if (model) camera.model = model.value;
      if (pixelSigma) camera.pixel_sigma = number(pixelSigma.value);
      if (timestampReference) camera.timing.timestamp_reference = timestampReference.value;
      if (durationSource) {
        camera.timing.fixed_exposure_duration_s = durationSource.value === 'fixed' ? number(field('fixed_exposure_duration_s')?.value) : null;
        camera.timing.exposure_metadata_topic = durationSource.value === 'metadata' ? (field('exposure_metadata_topic')?.value.trim() || null) : null;
      }
    }
    this._readTarget();
    this._readDetector();
    this._readCalibration();
    this._readSolver();
  }

  _readTarget() {
    const field = (name) => this._find(`[data-target="${name}"]`);
    if (!field('family')) return;
    const target = this.config.target;
    target.family = field('family').value;
    target.border_bits = integer(field('border_bits').value);
    target.tag_size_m = number(field('tag_size_m').value);
    target.tag_gap_m = number(field('tag_gap_m').value);
    const start = integer(field('corner_start').value);
    target.decoded_corner_to_front_corner = [start, (start + 1) % 4, (start + 2) % 4, (start + 3) % 4];
    for (const input of this.containers.target.querySelectorAll('[data-board-id]')) {
      target.id_rows_front_view_left_to_right[Number(input.dataset.row)][Number(input.dataset.column)] = integer(input.value);
    }
  }

  _readDetector() {
    const field = (name) => this._find(`[data-detector="${name}"]`);
    if (!field('max_hamming')) return;
    const detector = this.config.detector;
    detector.max_hamming = integer(field('max_hamming').value);
    detector.refinement.method = field('refinement_method').value;
    detector.refinement.max_iterations = integer(field('refinement_iterations').value);
    detector.budgets.max_pixels = integer(field('max_pixels').value);
    detector.budgets.max_clusters = integer(field('max_clusters').value);
    detector.budgets.max_candidates = integer(field('max_candidates').value);
  }

  _readCalibration() {
    const referenceField = this._find('[data-calibration="reference"]');
    if (!referenceField) return;
    const calibration = this.config.calibration;
    const reference = referenceField.value || '';
    if (calibration.type === 'single_camera') calibration.camera = reference;
    if (calibration.type === 'multi_camera') {
      calibration.reference_camera = reference;
      calibration.refine_intrinsics = this._find('[data-calibration="refine_intrinsics"]')?.checked ?? calibration.refine_intrinsics;
      calibration.max_closure_rotation_deg = number(this._find('[data-calibration="max_closure_rotation_deg"]')?.value);
      calibration.max_closure_translation_m = number(this._find('[data-calibration="max_closure_translation_m"]')?.value);
    }
    if (calibration.type === 'multi_camera_imu') {
      calibration.visual_reference_camera = reference;
      const method = this._find('[data-calibration="method"]');
      if (method) calibration.method = method.value;
      this._readTimeOffset(calibration);
      this._readImu(calibration.imu);
    }
    const strategy = this._find('[data-calibration="intrinsics_strategy"]')?.value;
    calibration.initial_intrinsics = strategy === 'file' ? initialIntrinsics(this._find('[data-calibration="intrinsics_path"]')?.value || null) : {};
  }

  _readTimeOffset(calibration) {
    const type = this._find('[data-time-offset="type"]');
    if (!type) return;
    calibration.time_offset = type.value === 'fixed' ? {
      type: 'fixed', value_s: number(this._find('[data-time-offset="value_s"]')?.value),
    } : {
      type: 'estimate', initial_s: number(this._find('[data-time-offset="initial_s"]')?.value),
      minimum_s: number(this._find('[data-time-offset="minimum_s"]')?.value),
      maximum_s: number(this._find('[data-time-offset="maximum_s"]')?.value),
      initialization_steps: integer(this._find('[data-time-offset="initialization_steps"]')?.value),
    };
  }

  _readImu(imu) {
    const value = (name) => this._find(`[data-imu="${name}"]`);
    imu.gravity_magnitude_m_s2 = number(value('gravity')?.value);
    imu.integration.method = value('integration_method')?.value || 'midpoint';
    imu.integration.max_gap_s = number(value('max_gap_s')?.value);
    imu.integration.max_samples = integer(value('max_samples')?.value);
    imu.noise.gyro_bias_walk_rad_s2_sqrt_hz = number(value('gyro_bias_walk')?.value);
    imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz = number(value('accelerometer_bias_walk')?.value);
    this._readMatrix('gyro_covariance', imu.noise.gyro_covariance_rad2_s_hz);
    this._readMatrix('accelerometer_covariance', imu.noise.accelerometer_covariance_m2_s4_hz);
    const continuous = imu.continuous_time;
    const continuousValue = (name) => this._find(`[data-continuous="${name}"]`);
    if (continuousValue('knot_interval_s')) {
      continuous.knot_interval_s = number(continuousValue('knot_interval_s').value);
      continuous.camera_time_offsets.refine = continuousValue('camera_offsets_refine').checked;
      continuous.camera_time_offsets.minimum_s = number(continuousValue('camera_offsets_minimum_s').value);
      continuous.camera_time_offsets.maximum_s = number(continuousValue('camera_offsets_maximum_s').value);
      continuous.clock_skews.refine_global = continuousValue('refine_global_skew').checked;
      continuous.clock_skews.refine_cameras = continuousValue('refine_camera_skews').checked;
      continuous.clock_skews.minimum_ppm = number(continuousValue('skew_minimum_ppm').value);
      continuous.clock_skews.maximum_ppm = number(continuousValue('skew_maximum_ppm').value);
      continuous.visual_motion_anisotropy.enabled = continuousValue('motion_anisotropy_enabled').checked;
      continuous.visual_motion_anisotropy.variance_ratio_per_motion_squared = number(continuousValue('motion_variance_ratio').value);
    }
    const axisRefine = this._find('[data-axis="refine"]');
    if (axisRefine) {
      imu.axis_calibration.refine = axisRefine.checked;
      this._readMatrix('gyro_axis', imu.axis_calibration.gyro_calibrated_from_raw);
      this._readMatrix('accelerometer_axis', imu.axis_calibration.accelerometer_calibrated_from_raw);
    }
  }

  _readMatrix(name, matrix) {
    for (const input of this.containers.solver.querySelectorAll(`[data-matrix="${name}"]`)) matrix[Number(input.dataset.row)][Number(input.dataset.column)] = number(input.value);
  }

  _readSolver() {
    const field = (name) => this._find(`[data-solver="${name}"]`);
    if (!field('max_iterations')) return;
    this.config.solver.max_iterations = integer(field('max_iterations').value);
    this.config.solver.huber_delta = number(field('huber_delta').value);
    this.config.solver.point_rejection_px = field('rejection_mode').value === 'huber_only' ? null : number(field('point_rejection_px')?.value);
  }

  _syncCameraSelection(topicName, enabled) {
    const index = this.config.cameras.findIndex((camera) => camera.topic === topicName);
    if (!enabled && index >= 0) this.config.cameras.splice(index, 1);
    if (enabled && index < 0) {
      const used = new Set(this.config.cameras.map((camera) => camera.id));
      let next = Math.max(0, sortedTopics(this.inspection).filter((topic) => IMAGE_TYPES.has(topic.type_name) && topicUsable(topic)).findIndex((topic) => topic.name === topicName));
      while (used.has(cameraId(next))) next += 1;
      this.config.cameras.push({ id: cameraId(next), topic: topicName, model: 'double-sphere', pixel_sigma: 1, timing: { timestamp_reference: 'exposure_midpoint', fixed_exposure_duration_s: null, exposure_metadata_topic: null } });
      this.config.cameras.sort((a, b) => compareTopicPaths(a.topic, b.topic));
    }
    const ids = new Set(this.config.cameras.map((camera) => camera.id));
    const first = this.config.cameras[0]?.id || '';
    if (this.config.calibration.type === 'single_camera' && !ids.has(this.config.calibration.camera)) this.config.calibration.camera = first;
    if (this.config.calibration.type === 'multi_camera' && !ids.has(this.config.calibration.reference_camera)) this.config.calibration.reference_camera = first;
    if (this.config.calibration.type === 'multi_camera_imu' && !ids.has(this.config.calibration.visual_reference_camera)) this.config.calibration.visual_reference_camera = first;
  }

  _deriveCalibrationMode() {
    const desired = this.config.source.imu_topic ? 'multi_camera_imu' : this.config.cameras.length >= 2 ? 'multi_camera' : 'single_camera';
    if (this.config.calibration.type !== desired) this._setCalibrationType(desired);
    const ids = new Set(this.config.cameras.map((camera) => camera.id));
    const first = this.config.cameras[0]?.id || '';
    if (desired === 'single_camera' && !ids.has(this.config.calibration.camera)) this.config.calibration.camera = first;
    if (desired === 'multi_camera' && !ids.has(this.config.calibration.reference_camera)) this.config.calibration.reference_camera = first;
    if (desired === 'multi_camera_imu' && !ids.has(this.config.calibration.visual_reference_camera)) this.config.calibration.visual_reference_camera = first;
  }

  _setCalibrationType(type) {
    const current = this.config.calibration;
    this._calibrationDrafts.set(current.type, clone(current));
    const path = current.initial_intrinsics?.path || null;
    const reference = this.config.cameras[0]?.id || '';
    const saved = this._calibrationDrafts.get(type);
    if (saved) {
      this.config.calibration = normalizeCalibration(saved, this.config.cameras);
    } else if (type === 'single_camera') {
      this.config.calibration = { type, camera: reference, initial_intrinsics: initialIntrinsics(path) };
    } else if (type === 'multi_camera') {
      this.config.calibration = { type, reference_camera: reference, refine_intrinsics: true, initial_intrinsics: initialIntrinsics(path), max_closure_rotation_deg: 2, max_closure_translation_m: 0.05 };
    } else {
      this.config.calibration = { type: 'multi_camera_imu', method: 'continuous_time', visual_reference_camera: reference, initial_intrinsics: initialIntrinsics(path), time_offset: { type: 'estimate', initial_s: 0, minimum_s: -0.1, maximum_s: 0.1, initialization_steps: 11 }, imu: defaultImu() };
    }
    if (type !== 'multi_camera_imu' && this.config.solver.point_rejection_px == null) this.config.solver.point_rejection_px = 6;
  }

  _setInertialMethod(method) {
    const calibration = this.config.calibration;
    if (calibration.type !== 'multi_camera_imu') return;
    calibration.method = method;
    if (method === 'preintegration') {
      calibration.imu.axis_calibration = { refine: false, gyro_calibrated_from_raw: identity(), accelerometer_calibrated_from_raw: identity() };
      calibration.imu.continuous_time.camera_time_offsets.refine = false;
      calibration.imu.continuous_time.clock_skews.refine_global = false;
      calibration.imu.continuous_time.clock_skews.refine_cameras = false;
      calibration.imu.continuous_time.visual_motion_anisotropy.enabled = false;
      if (this.config.solver.point_rejection_px == null) this.config.solver.point_rejection_px = 6;
    }
  }

  _resizeBoard(rows, columns) {
    const current = this.config.target.id_rows_front_view_left_to_right || [];
    const used = new Set(current.flat().filter((value) => Number.isInteger(Number(value))).map(Number));
    let next = 0;
    const nextId = () => { while (used.has(next)) next += 1; const value = next; used.add(value); next += 1; return value; };
    this.config.target.id_rows_front_view_left_to_right = Array.from({ length: rows }, (_, row) => Array.from({ length: columns }, (_, column) => current[row]?.[column] ?? nextId()));
  }

  _validateTarget(target, add) {
    const rows = target?.id_rows_front_view_left_to_right;
    if (!Array.isArray(rows) || rows.length < 1 || rows.length > 64) { add('target.id_rows_front_view_left_to_right', '标定板行数必须在 1..64。'); return; }
    const columns = rows[0]?.length || 0;
    if (columns < 1 || columns > 64 || rows.some((row) => !Array.isArray(row) || row.length !== columns)) add('target.id_rows_front_view_left_to_right', 'ID 布局必须是 1..64 列的矩形。');
    const values = rows.flat();
    if (values.some((value) => !Number.isInteger(Number(value)) || Number(value) < 0 || Number(value) > 4294967295)) add('target.id_rows_front_view_left_to_right', 'Tag ID 必须是非负 u32 整数。');
    if (new Set(values.map(Number)).size !== values.length) add('target.id_rows_front_view_left_to_right', 'Tag ID 不能重复；页面不会自动重排。');
    if (!TAG_FAMILIES.includes(target.family)) add('target.family', 'Tag family 不受支持。');
    if (![1, 2].includes(Number(target.border_bits))) add('target.border_bits', '边框宽度只能为 1 或 2 bit。');
    if (!finite(target.tag_size_m) || Number(target.tag_size_m) <= 0) add('target.tag_size_m', 'Tag 边长必须为正数。');
    if (!finite(target.tag_gap_m) || Number(target.tag_gap_m) < 0) add('target.tag_gap_m', 'Tag 间隙必须为非负数。');
    const mapping = target.decoded_corner_to_front_corner;
    const start = mapping?.[0];
    if (!Array.isArray(mapping) || mapping.length !== 4 || !Number.isInteger(start) || start < 0 || start > 3 || mapping.some((value, index) => value !== (start + index) % 4)) add('target.decoded_corner_to_front_corner', '角点映射必须是 TL/TR/BR/BL 的循环排列。');
  }

  _validateDetector(detector, add) {
    if (!Number.isInteger(Number(detector?.refinement?.max_iterations)) || Number(detector.refinement.max_iterations) <= 0) add('detector.refinement.max_iterations', '角点细化迭代数必须为正整数。');
    for (const key of ['max_pixels', 'max_clusters', 'max_candidates']) if (!Number.isInteger(Number(detector?.budgets?.[key])) || Number(detector.budgets[key]) <= 0) add(`detector.budgets.${key}`, '检测资源预算必须为正整数。');
  }

  _validateInertial(calibration, add) {
    const imu = calibration.imu;
    const time = calibration.time_offset;
    if (!['continuous_time', 'preintegration'].includes(calibration.method)) add('calibration.method', '惯性方法不受支持。');
    if (time.type === 'fixed') {
      if (!finite(time.value_s)) add('calibration.time_offset.value_s', '固定时差必须是有限数。');
    } else if (time.type === 'estimate') {
      if (![time.initial_s, time.minimum_s, time.maximum_s].every(finite) || Number(time.minimum_s) >= Number(time.maximum_s) || Number(time.initial_s) < Number(time.minimum_s) || Number(time.initial_s) > Number(time.maximum_s)) add('calibration.time_offset', '时差初值必须位于有限且递增的上下界内。');
      if (!Number.isInteger(Number(time.initialization_steps)) || Number(time.initialization_steps) < 2 || Number(time.initialization_steps) > 101) add('calibration.time_offset.initialization_steps', '时差搜索步数必须在 2..101。');
    } else add('calibration.time_offset.type', '时差策略不受支持。');
    for (const [field, value] of [['gravity_magnitude_m_s2', imu.gravity_magnitude_m_s2], ['noise.gyro_bias_walk_rad_s2_sqrt_hz', imu.noise.gyro_bias_walk_rad_s2_sqrt_hz], ['noise.accelerometer_bias_walk_m_s3_sqrt_hz', imu.noise.accelerometer_bias_walk_m_s3_sqrt_hz], ['integration.max_gap_s', imu.integration.max_gap_s]]) if (!finite(value) || Number(value) <= 0) add(`calibration.imu.${field}`, 'IMU 数值必须为有限正数。');
    if (!Number.isInteger(Number(imu.integration.max_samples)) || Number(imu.integration.max_samples) < 2) add('calibration.imu.integration.max_samples', 'IMU 最大样本数至少为 2。');
    this._validateCovariance(imu.noise.gyro_covariance_rad2_s_hz, 'calibration.imu.noise.gyro_covariance_rad2_s_hz', add);
    this._validateCovariance(imu.noise.accelerometer_covariance_m2_s4_hz, 'calibration.imu.noise.accelerometer_covariance_m2_s4_hz', add);
    const continuous = imu.continuous_time;
    if (!finite(continuous.knot_interval_s) || Number(continuous.knot_interval_s) <= 0) add('calibration.imu.continuous_time.knot_interval_s', '样条 knot 间隔必须为正数。');
    if (!finite(continuous.camera_time_offsets.minimum_s) || !finite(continuous.camera_time_offsets.maximum_s) || Number(continuous.camera_time_offsets.minimum_s) >= Number(continuous.camera_time_offsets.maximum_s)) add('calibration.imu.continuous_time.camera_time_offsets', '相机时间偏移上下界必须有限且递增。');
    if (!finite(continuous.clock_skews.minimum_ppm) || !finite(continuous.clock_skews.maximum_ppm) || Number(continuous.clock_skews.minimum_ppm) >= Number(continuous.clock_skews.maximum_ppm) || Number(continuous.clock_skews.minimum_ppm) <= -500000 || Number(continuous.clock_skews.maximum_ppm) >= 500000) add('calibration.imu.continuous_time.clock_skews', '时钟 skew 上下界必须递增且位于 (-500000, 500000) ppm。');
    if (!finite(continuous.visual_motion_anisotropy.variance_ratio_per_motion_squared) || Number(continuous.visual_motion_anisotropy.variance_ratio_per_motion_squared) <= 0) add('calibration.imu.continuous_time.visual_motion_anisotropy', '曝光运动方差比必须为正数。');
    if (calibration.method === 'preintegration') {
      if (imu.axis_calibration.refine || !this._matrixEquals(imu.axis_calibration.gyro_calibrated_from_raw, IDENTITY) || !this._matrixEquals(imu.axis_calibration.accelerometer_calibrated_from_raw, IDENTITY)) add('calibration.imu.axis_calibration', '预积分只支持单位轴矩阵且 refine=false。', 'UNSUPPORTED_COMBINATION');
      if (continuous.camera_time_offsets.refine || continuous.clock_skews.refine_global || continuous.clock_skews.refine_cameras || continuous.visual_motion_anisotropy.enabled) add('calibration.imu.continuous_time', '预积分不支持残余相机时差、时钟 skew 或曝光运动各向异性。', 'UNSUPPORTED_COMBINATION');
      if (!this._isIsotropicDiagonal(imu.noise.gyro_covariance_rad2_s_hz) || !this._isIsotropicDiagonal(imu.noise.accelerometer_covariance_m2_s4_hz)) add('calibration.imu.noise', '预积分只支持各向同性对角白噪声 PSD。', 'UNSUPPORTED_COMBINATION');
    }
  }

  _validateCovariance(matrix, field, add) {
    if (!Array.isArray(matrix) || matrix.length !== 3 || matrix.some((row) => !Array.isArray(row) || row.length !== 3 || row.some((value) => !finite(value)))) { add(field, '协方差必须是有限的 3×3 数字矩阵。'); return; }
    if ([0, 1, 2].some((row) => [0, 1, 2].some((column) => Math.abs(Number(matrix[row][column]) - Number(matrix[column][row])) > 1e-12))) { add(field, '协方差矩阵必须对称。'); return; }
    const a = matrix.map((row) => row.map(Number));
    const leadingTwo = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    const determinant = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (!(a[0][0] > 0 && leadingTwo > 0 && determinant > 0)) add(field, '协方差矩阵必须正定。');
  }

  _isIsotropicDiagonal(matrix) {
    if (!Array.isArray(matrix) || matrix.length !== 3) return false;
    const diagonalValue = Number(matrix[0]?.[0]);
    return finite(diagonalValue) && diagonalValue > 0 && matrix.every((row, i) => row.every((value, j) => i === j ? Math.abs(Number(value) - diagonalValue) <= 1e-12 : Math.abs(Number(value)) <= 1e-12));
  }

  _matrixEquals(left, right) {
    return Array.isArray(left) && left.length === 3 && left.every((row, i) => Array.isArray(row) && row.length === 3 && row.every((value, j) => Number(value) === Number(right[i][j])));
  }

  _find(selector) {
    for (const container of Object.values(this.containers)) {
      const found = container.querySelector(selector);
      if (found) return found;
    }
    return null;
  }

  _renderIssuesOnly() {
    const panel = this.containers.solver.querySelector('.checks');
    if (!panel) return;
    const issues = this.getIssues();
    panel.innerHTML = issues.length ? issues.map((issue) => `<li class="blocking"><strong>阻断</strong>${escapeHtml(issue.message)}</li>`).join('') : '<li class="ok"><strong>表单完整</strong>仍需 Rust 权威验证与数据预检。</li>';
    this._notifyIssues(issues);
  }

  _notify() {
    const config = this.getConfig();
    const issues = this.getIssues();
    this._safeCallback(this.onChange, config);
    this._safeCallback(this.onIssues, clone(issues));
  }

  _notifyIssues(issues = this.getIssues()) {
    this._safeCallback(this.onIssues, clone(issues));
  }

  _safeCallback(callback, value) {
    if (!callback) return;
    try { callback(value); }
    catch (error) { console.error('ConfigurationForms callback failed', error); }
  }
}
