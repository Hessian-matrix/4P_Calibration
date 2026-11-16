import { extrinsicsView } from './extrinsics-view.js';

const ACCEPTANCE_LABELS = {
  pending_human_review: '待人工审核',
  accepted: '已接受',
  rejected: '已拒绝',
};

const MODEL_LABELS = {
  pinhole: '针孔',
  'kannala-brandt4': 'Kannala–Brandt 4',
  'extended-unified': '扩展统一模型',
  'double-sphere': 'Double Sphere',
};

function isObject(value) {
  return value !== null && typeof value === 'object' && !Array.isArray(value);
}

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

function finite(value) {
  return typeof value === 'number' && Number.isFinite(value);
}

function present(value) {
  return value !== undefined && value !== null && value !== '';
}

function number(value, digits = 6) {
  if (!finite(value)) return '未提供';
  const absolute = Math.abs(value);
  if (absolute !== 0 && (absolute >= 1e6 || absolute < 1e-4)) return value.toExponential(6);
  return value.toLocaleString('zh-CN', { maximumFractionDigits: digits, useGrouping: false });
}

function count(value) {
  return Number.isSafeInteger(value) && value >= 0 ? value.toLocaleString('zh-CN') : '未提供';
}

function valueText(value) {
  if (finite(value)) return number(value);
  if (typeof value === 'string' || typeof value === 'boolean') return String(value);
  if (Array.isArray(value) && value.every(finite)) return value.map((item) => number(item)).join(', ');
  return '未提供';
}

function heading(title, description) {
  const header = element('div', 'result-section-heading');
  header.append(element('h2', '', title));
  if (description) header.append(element('p', 'small', description));
  return header;
}

function metric(label, value, note) {
  const card = element('div', 'metric-card');
  card.append(element('span', 'metric-label', label), element('strong', 'metric-value', value));
  if (note) card.append(element('span', 'metric-note', note));
  return card;
}

function table(headers, rows, label) {
  const wrap = element('div', 'table-wrap');
  const node = element('table', 'result-table');
  if (label) node.setAttribute('aria-label', label);
  const head = document.createElement('thead');
  const headRow = document.createElement('tr');
  for (const header of headers) headRow.append(element('th', '', header));
  head.append(headRow);
  const body = document.createElement('tbody');
  for (const row of rows) {
    const tr = document.createElement('tr');
    for (const cell of row) tr.append(element('td', '', cell));
    body.append(tr);
  }
  node.append(head, body);
  wrap.append(node);
  return wrap;
}

function statusClass(value) {
  if (value === 'accepted' || value === true) return 'status-chip status-positive';
  if (value === 'rejected' || value === false) return 'status-chip status-caution';
  return 'status-chip status-pending';
}

function cameraError(summary, index) {
  if (!Array.isArray(summary?.camera_errors)) return null;
  return summary.camera_errors.find((item) => isObject(item) && item.camera_index === index) ?? null;
}

function cameraModel(camera) {
  const wrapper = isObject(camera?.parameters) ? camera.parameters : null;
  if (!wrapper) return '未提供';
  const top = MODEL_LABELS[wrapper.model] ?? wrapper.model;
  const subtype = wrapper.model === 'pinhole' && present(wrapper.parameters?.model) ? ` · ${wrapper.parameters.model}` : '';
  return present(top) ? `${top}${subtype}` : '未提供';
}

const PARAMETER_LABELS = {
  model: '模型', fx: '水平焦距', fy: '垂直焦距', cx: '主点 X', cy: '主点 Y',
  k1: '畸变 k1', k2: '畸变 k2', k3: '畸变 k3', k4: '畸变 k4',
  alpha: '模型参数 α', beta: '模型参数 β', xi: '模型参数 ξ',
};

function parameter(label, key, value, unit = '', aliases = []) {
  return { label, key, value: valueText(value), unit, aliases };
}

function parameterRows(rows) {
  const list = element('div', 'parameter-rows');
  for (const row of rows) {
    const node = element('div', 'parameter-row');
    node.dataset.search = [row.label, row.key, row.value, row.unit, ...row.aliases].join(' ').toLocaleLowerCase();
    const identity = element('div', 'parameter-identity');
    identity.append(element('strong', '', row.label), element('code', '', row.key));
    const renderedValue = element('span', 'parameter-value', row.value);
    if (row.unit) renderedValue.append(element('span', 'parameter-unit', ` ${row.unit}`));
    node.append(identity, renderedValue);
    list.append(node);
  }
  return list;
}

function parameterGroup(title, description, rows, aliases = []) {
  const group = element('details', 'parameter-group');
  group.dataset.search = [title, description, ...aliases].filter(Boolean).join(' ').toLocaleLowerCase();
  const summary = element('summary');
  const text = element('span');
  text.append(element('strong', '', title));
  if (description) text.append(element('small', '', description));
  summary.append(text, element('span', 'parameter-count', `${rows.length} 项`));
  group.append(summary, parameterRows(rows));
  return group;
}

function intrinsicGroups(result) {
  const cameras = Array.isArray(result.cameras) ? result.cameras : [];
  return cameras.map((camera, index) => {
    const id = present(camera?.id) ? String(camera.id) : `相机 ${index + 1}`;
    const wrapper = isObject(camera?.parameters) ? camera.parameters : {};
    const values = isObject(wrapper.parameters) ? wrapper.parameters : {};
    const base = `cameras[${index}]`;
    const rawModel = present(wrapper.model) ? String(wrapper.model) : undefined;
    const rows = [
      parameter('相机 ID', `${base}.id`, camera?.id, '', [id]),
      parameter('相机模型', `${base}.parameters.model`, rawModel, '', [cameraModel(camera)]),
      parameter('图像宽度', `${base}.image_width`, camera?.image_width, 'px'),
      parameter('图像高度', `${base}.image_height`, camera?.image_height, 'px'),
    ];
    for (const [key, value] of Object.entries(values)) {
      if (key === 'coefficients' && Array.isArray(value)) {
        value.forEach((coefficient, coefficientIndex) => rows.push(parameter(
          `畸变系数 ${coefficientIndex + 1}`,
          `${base}.parameters.parameters.coefficients[${coefficientIndex}]`,
          coefficient,
          '',
          ['coefficients', `coefficient ${coefficientIndex + 1}`],
        )));
      } else {
        rows.push(parameter(PARAMETER_LABELS[key] ?? key, `${base}.parameters.parameters.${key}`, value));
      }
    }
    return parameterGroup(`相机 · ${id}`, cameraModel(camera), rows, [id, rawModel, '相机 内参 camera intrinsic']);
  });
}

function extrinsicGroups(result) {
  const inertial = isObject(result.inertial) ? result.inertial : null;
  const items = inertial ? inertial.camera_from_imu : result.camera_from_reference;
  if (!Array.isArray(items)) return [];
  const base = inertial ? 'inertial.camera_from_imu' : 'camera_from_reference';
  return items.map((item, index) => {
    const direction = present(item?.direction) ? String(item.direction) : '方向未提供';
    const path = `${base}[${index}]`;
    return parameterGroup(`外参 · ${direction}`, '原始变换方向；平移与四元数不做换向', [
      parameter('变换方向', `${path}.direction`, item?.direction, '', [direction, 'transform']),
      parameter('平移 [x, y, z]', `${path}.pose.translation_m`, item?.pose?.translation_m, 'm', [direction]),
      parameter('四元数 [w, x, y, z]', `${path}.pose.quaternion_wxyz`, item?.pose?.quaternion_wxyz, '', [direction]),
    ], [direction, 'transform', 'extrinsic']);
  });
}

function matrixParameters(label, key, value) {
  const matrix = Array.isArray(value) ? value : [];
  return [0, 1, 2].map((index) => parameter(
    `${label} · 第 ${index + 1} 行`, `${key}[${index}]`, matrix[index], '', [label, 'axis matrix'],
  ));
}

function flattenParameters(value, path, rows) {
  if (Array.isArray(value) && value.every((item) => !isObject(item) && !Array.isArray(item))) {
    rows.push(parameter(path.split('.').at(-1), path, value));
  } else if (isObject(value) || Array.isArray(value)) {
    for (const [key, child] of Object.entries(value)) {
      const childPath = Array.isArray(value) ? `${path}[${key}]` : `${path}.${key}`;
      flattenParameters(child, childPath, rows);
    }
  } else {
    rows.push(parameter(path.split('.').at(-1), path, value));
  }
}

export class ResultsView {
  constructor(root) {
    this.root = typeof root === 'string' ? document.querySelector(root) : root;
    if (!(this.root instanceof Element)) throw new TypeError('ResultsView 需要有效的根元素');
    this.root.classList.add('results-view');
  }

  render(payload = {}) {
    const result = isObject(payload.result) ? payload.result : null;
    const summary = isObject(payload.summary) ? payload.summary : isObject(payload.solveSummary) ? payload.solveSummary : null;
    const previousVisualization = this.root.querySelector('.extrinsics-view');
    previousVisualization?.dispose?.();
    this.root.replaceChildren();

    if (!result) {
      const empty = element('section', 'result-empty');
      empty.append(element('h2', '', '尚无完整标定结果'), element('p', '', '运行完整结束并取得权威结果后，才会在这里显示参数与诊断。'));
      this.root.append(empty);
      return;
    }

    this.root.append(this.renderStyles(), this.renderOverview(result, summary), this.renderCameraMetrics(result, summary));
    this.root.append(this.renderExtrinsicsVisualization(result), this.renderParameterBrowser(result));
  }

  renderStyles() {
    const style = element('style');
    style.textContent = `
      .results-view .parameter-toolbar{display:flex;align-items:end;gap:8px;flex-wrap:wrap;margin:12px 0}
      .results-view .parameter-search{display:grid;gap:4px;flex:1 1 260px;font-size:11px;color:var(--muted)}
      .results-view .parameter-search input{width:100%;box-sizing:border-box}
      .results-view .parameter-category{margin-top:14px}.results-view .parameter-category>h3{margin:0 0 7px;color:var(--green);font-size:12px;letter-spacing:.04em}
      .results-view .parameter-group{margin:6px 0;border:1px solid var(--line);border-radius:7px;background:var(--surface)}
      .results-view .parameter-group>summary{display:flex;justify-content:space-between;align-items:center;gap:12px;padding:9px 11px;cursor:pointer}
      .results-view .parameter-group>summary span:first-child{display:grid;gap:2px}.results-view .parameter-group>summary small{color:var(--muted);font-weight:400}
      .results-view .parameter-count{color:var(--muted);font-size:10px;white-space:nowrap}
      .results-view .parameter-rows{border-top:1px solid var(--line)}
      .results-view .parameter-row{display:grid;grid-template-columns:minmax(180px,1fr) minmax(160px,1fr);gap:16px;align-items:center;padding:8px 11px;border-top:1px solid color-mix(in srgb,var(--line) 60%,transparent)}
      .results-view .parameter-row:first-child{border-top:0}.results-view .parameter-identity{display:grid;gap:2px;min-width:0}
      .results-view .parameter-identity strong{font-size:11px}.results-view .parameter-identity code{overflow-wrap:anywhere;color:var(--muted);font-size:10px}
      .results-view .parameter-value{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;text-align:right;overflow-wrap:anywhere}
      .results-view .parameter-unit{color:var(--muted)}.results-view .parameter-search-status{margin:0;color:var(--muted)}
      .results-view .parameter-no-match{padding:12px;border:1px dashed var(--line);border-radius:7px;text-align:center}
      @media(max-width:620px){.results-view .parameter-row{grid-template-columns:1fr;gap:5px}.results-view .parameter-value{text-align:left}}
    `;
    return style;
  }

  renderOverview(result, summary) {
    const section = element('section', 'result-section result-overview');
    section.setAttribute('aria-labelledby', 'result-overview-title');
    const head = heading('结果概览', '数值收敛与人工验收是两个独立状态。');
    head.querySelector('h2').id = 'result-overview-title';
    section.append(head);

    const statuses = element('div', 'result-statuses');
    const acceptance = present(result.acceptance) ? result.acceptance : null;
    const acceptanceText = acceptance ? (ACCEPTANCE_LABELS[acceptance] ?? String(acceptance)) : '未提供';
    const acceptanceBlock = element('div', 'result-status-block');
    acceptanceBlock.append(element('span', 'metric-label', '人工验收'), element('strong', statusClass(acceptance), acceptanceText));

    const converged = summary?.converged;
    const convergenceText = converged === true ? '已数值收敛' : converged === false ? '未数值收敛' : '未提供';
    const convergenceBlock = element('div', 'result-status-block');
    convergenceBlock.append(element('span', 'metric-label', '求解状态'), element('strong', statusClass(converged), convergenceText));
    statuses.append(acceptanceBlock, convergenceBlock);
    section.append(statuses);

    const grid = element('div', 'metric-grid');
    const cost = finite(summary?.initial_cost) && finite(summary?.final_cost)
      ? `${number(summary.initial_cost)} → ${number(summary.final_cost)}`
      : '未提供';
    const covariance = summary?.covariance_is_pseudoinverse === true
      ? '伪逆'
      : summary?.covariance_is_pseudoinverse === false ? '全秩逆' : '未提供';
    grid.append(
      metric('标定模式', ({ single_camera: '单相机', multi_camera: '多相机', camera_imu: '相机与 IMU', multi_camera_imu: '相机与 IMU' })[result.mode] || String(result.mode || '未提供')),
      metric('终止原因', present(summary?.termination) ? String(summary.termination) : '未提供'),
      metric('迭代次数', count(summary?.iterations)),
      metric('初始 → 最终 cost', cost),
      metric('接受 / 拒绝步', `${count(summary?.accepted_steps)} / ${count(summary?.rejected_steps)}`),
      metric('最终阻尼', number(summary?.final_damping)),
      metric('缩放梯度 ∞ 范数', number(summary?.scaled_gradient_inf_norm)),
      metric('信息秩 / 参数数', `${count(summary?.information_rank)} / ${count(summary?.information_parameters)}`),
      metric('条件数', number(summary?.condition_number)),
      metric('协方差', covariance),
    );
    const diagnostics = element('details', 'advanced');
    diagnostics.append(element('summary', '', '查看求解诊断'), grid);
    section.append(diagnostics);
    return section;
  }

  renderCameraMetrics(result, summary) {
    const section = element('section', 'result-section');
    section.append(heading('逐相机重投影误差', '误差单位为像素；点数来自本次求解摘要。'));
    const cameras = Array.isArray(result.cameras) ? result.cameras : [];
    if (!cameras.length) {
      section.append(element('p', 'result-unavailable', '结果未提供相机数据。'));
      return section;
    }

    section.append(table(['相机', '模型', 'RMS / px', 'P95 / px', '有效点', '拒绝点'], cameras.map((camera, index) => {
      const error = cameraError(summary, index);
      return [camera.id ?? `相机 ${index + 1}`, cameraModel(camera), number(error?.rms_px), number(error?.p95_px), count(error?.used_points), count(error?.rejected_points)];
    }), '逐相机重投影误差'));
    return section;
  }

  renderExtrinsicsVisualization(result) {
    const section = element('section', 'result-section result-extrinsics-visualization');
    section.append(heading('外参三维视图', '左键旋转、右键平移、滚轮缩放；参数原值在下方分组中保留。'));
    const visualization = extrinsicsView(result);
    if (visualization) section.append(visualization);
    else section.append(element('p', 'result-unavailable', '当前模式没有可显示的外参，或结果未提供有效外参。'));
    return section;
  }

  renderParameterBrowser(result) {
    const section = element('section', 'result-section result-parameters');
    section.append(heading('标定参数', '按语义分组；值保持结果文件的方向、顺序和单位，原始键名显示在每一项下方。'));

    const toolbar = element('div', 'parameter-toolbar');
    const label = element('label', 'parameter-search');
    label.append(element('span', '', '搜索参数名、相机 ID、原始键或变换方向'));
    const input = element('input');
    input.type = 'search';
    input.placeholder = '例如 fx、camera_1、T_cam_from_imu、clock';
    input.setAttribute('aria-label', '搜索标定参数');
    const clear = element('button', '', '清除');
    clear.type = 'button';
    clear.disabled = true;
    label.append(input);
    toolbar.append(label, clear);
    section.append(toolbar);

    const status = element('p', 'parameter-search-status');
    status.setAttribute('role', 'status');
    const groupsRoot = element('div', 'parameter-groups');
    const categories = [];
    const addCategory = (title, groups) => {
      if (!groups.length) return;
      const category = element('div', 'parameter-category');
      category.append(element('h3', '', title), ...groups);
      groups[0].open = true;
      groupsRoot.append(category);
      categories.push(category);
    };

    addCategory('结果标识', [parameterGroup('结果与坐标系', '结果文件顶层标识', [
      parameter('Schema 版本', 'schema_version', result.schema_version),
      parameter('标定模式', 'mode', result.mode),
      parameter('人工验收状态', 'acceptance', result.acceptance),
      parameter('视觉参考相机', 'reference_camera', result.reference_camera),
    ], ['schema', 'mode', 'reference camera'])]);

    const cameras = intrinsicGroups(result);
    if (cameras.length) addCategory('相机内参', cameras);
    else addCategory('相机内参', [parameterGroup('相机 · 未提供', '结果中没有相机条目', [
      parameter('相机数据', 'cameras', undefined),
    ], ['camera', 'intrinsic'])]);

    const transforms = extrinsicGroups(result);
    if (transforms.length) addCategory('外参变换', transforms);
    else addCategory('外参变换', [parameterGroup('外参 · 不适用', '单相机模式无需相对外参，其他模式则表示结果未提供', [
      parameter('外参列表', isObject(result.inertial) ? 'inertial.camera_from_imu' : 'camera_from_reference', '不适用或未提供'),
    ], ['transform', 'extrinsic'])]);

    const inertial = isObject(result.inertial) ? result.inertial : null;
    if (inertial) {
      addCategory('IMU 物理量', [parameterGroup('IMU 状态', 'physical = A × raw − bias；向量按 x, y, z 排列', [
        parameter('求解方法', 'inertial.method', inertial.method),
        parameter('陀螺常量偏置 [x, y, z]', 'inertial.gyro_bias_rad_s', inertial.gyro_bias_rad_s, 'rad/s'),
        parameter('加速度计常量偏置 [x, y, z]', 'inertial.accelerometer_bias_m_s2', inertial.accelerometer_bias_m_s2, 'm/s²'),
        parameter('世界系重力 [x, y, z]', 'inertial.gravity_world_m_s2', inertial.gravity_world_m_s2, 'm/s²'),
      ], ['imu', 'bias', 'gravity'])]);

      const axes = isObject(inertial.axis_calibration) ? inertial.axis_calibration : {};
      addCategory('IMU 轴矩阵', [
        parameterGroup('陀螺轴矩阵 A', '行主序 · physical_gyro = A × raw_gyro − bias', matrixParameters(
          '陀螺轴矩阵', 'inertial.axis_calibration.gyro_calibrated_from_raw', axes.gyro_calibrated_from_raw,
        ), ['gyro', 'axis calibration']),
        parameterGroup('加速度计轴矩阵 A', '行主序 · physical_specific_force = A × raw_accelerometer − bias', matrixParameters(
          '加速度计轴矩阵', 'inertial.axis_calibration.accelerometer_calibrated_from_raw', axes.accelerometer_calibrated_from_raw,
        ), ['accelerometer', 'axis calibration']),
      ]);

      const clockRows = [
        parameter('相机到 IMU 全局时差', 'inertial.time_offset_s', inertial.time_offset_s, 's'),
        parameter('仿射时钟参考时刻', 'inertial.clock_reference_time_s', inertial.clock_reference_time_s, 's'),
        parameter('全局时钟漂移', 'inertial.clock_skew_ppm', inertial.clock_skew_ppm, 'ppm'),
      ];
      const offsets = Array.isArray(inertial.camera_time_offsets_s) ? inertial.camera_time_offsets_s : [];
      offsets.forEach((item, index) => {
        clockRows.push(parameter(`相机 ${item?.id ?? index + 1} 残余时差`, `inertial.camera_time_offsets_s[${index}].value_s`, item?.value_s, 's', [item?.id]));
        clockRows.push(parameter(`残余时差相机 ID`, `inertial.camera_time_offsets_s[${index}].id`, item?.id, '', [item?.id]));
      });
      const skews = Array.isArray(inertial.camera_clock_skews_ppm) ? inertial.camera_clock_skews_ppm : [];
      skews.forEach((item, index) => {
        clockRows.push(parameter(`相机 ${item?.id ?? index + 1} 时钟漂移`, `inertial.camera_clock_skews_ppm[${index}].value_ppm`, item?.value_ppm, 'ppm', [item?.id]));
        clockRows.push(parameter('时钟漂移相机 ID', `inertial.camera_clock_skews_ppm[${index}].id`, item?.id, '', [item?.id]));
      });
      addCategory('时钟参数', [parameterGroup('相机 / IMU 时钟', '时差为参考时刻的截距；漂移单位 ppm', clockRows, ['clock', 'time offset', 'skew'])]);

      const known = new Set(['method', 'camera_from_imu', 'gyro_bias_rad_s', 'accelerometer_bias_m_s2', 'gravity_world_m_s2', 'time_offset_s', 'clock_reference_time_s', 'clock_skew_ppm', 'camera_time_offsets_s', 'camera_clock_skews_ppm', 'axis_calibration']);
      const additional = [];
      for (const [key, value] of Object.entries(inertial)) if (!known.has(key)) flattenParameters(value, `inertial.${key}`, additional);
      if (additional.length) addCategory('其他 IMU 字段', [parameterGroup('未分类字段', '按结果文件原始路径保留', additional)]);
    }

    const noMatch = element('div', 'parameter-no-match');
    noMatch.hidden = true;
    noMatch.append(element('p', '', '没有匹配的参数。'));
    const noMatchClear = element('button', '', '清除搜索');
    noMatchClear.type = 'button';
    noMatch.append(noMatchClear);
    section.append(status, groupsRoot, noMatch);

    const applySearch = () => {
      const query = input.value.trim().toLocaleLowerCase();
      const terms = query.split(/\s+/).filter(Boolean);
      let visibleGroups = 0;
      let visibleRows = 0;
      for (const category of categories) {
        const groups = [...category.querySelectorAll('.parameter-group')];
        for (const group of groups) {
          let matches = 0;
          for (const row of group.querySelectorAll('.parameter-row')) {
            const haystack = `${group.dataset.search} ${row.dataset.search}`;
            const show = terms.every(term => haystack.includes(term));
            row.hidden = !show;
            if (show) matches += 1;
          }
          group.hidden = matches === 0;
          if (matches) {
            visibleGroups += 1;
            visibleRows += matches;
            if (query) group.open = true;
          }
        }
        category.hidden = groups.every((group) => group.hidden);
      }
      clear.disabled = !query;
      noMatch.hidden = visibleGroups !== 0;
      groupsRoot.hidden = visibleGroups === 0;
      status.textContent = query ? `找到 ${visibleGroups} 个分组、${visibleRows} 项参数` : `共 ${visibleGroups} 个分组、${visibleRows} 项参数`;
    };
    const clearSearch = () => { input.value = ''; applySearch(); input.focus(); };
    input.addEventListener('input', applySearch);
    clear.addEventListener('click', clearSearch);
    noMatchClear.addEventListener('click', clearSearch);
    applySearch();
    return section;
  }
}

export default ResultsView;
