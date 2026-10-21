use crate::input::TopicInspection;

use super::App;
use super::draft::{
    CalibrationMode, CameraDraft, IntrinsicStrategy, RejectionPolicy, TimeOffsetKind,
    bag_format_label, camera_model_label, inertial_method_label, integration_method_label,
    refinement_label, rejection_policy_label, tag_family_label, time_offset_kind_label,
};
use super::preflight::imu_selection_note;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FormSection {
    Source,
    Sensors,
    Board,
    Solver,
    Preflight,
}

impl FormSection {
    pub const ALL: [Self; 5] = [
        Self::Source,
        Self::Sensors,
        Self::Board,
        Self::Solver,
        Self::Preflight,
    ];

    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Source => 0,
            Self::Sensors => 1,
            Self::Board => 2,
            Self::Solver => 3,
            Self::Preflight => 4,
        }
    }

    #[must_use]
    pub const fn tab(self) -> &'static str {
        match self {
            Self::Source => "Source",
            Self::Sensors => "Sensors",
            Self::Board => "Board",
            Self::Solver => "Solver",
            Self::Preflight => "Preflight",
        }
    }

    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Source => "数据源",
            Self::Sensors => "传感器",
            Self::Board => "标定板",
            Self::Solver => "求解器与惯性",
            Self::Preflight => "预检与启动",
        }
    }

    #[must_use]
    pub fn next(self, delta: isize) -> Self {
        let index = self.index();
        let count = Self::ALL.len();
        let target = if delta < 0 {
            (index + count - 1) % count
        } else {
            (index + 1) % count
        };
        Self::ALL[target]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FieldKind {
    Text,
    Choice,
    Toggle,
    Button,
    ReadOnly,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FieldId {
    SourcePath,
    SourceFormat,
    SourceMaxDelta,
    SourceRequireAll,
    SourceScan,
    Mode,
    Reference,
    IntrinsicStrategy,
    IntrinsicPath,
    ScanTopics,
    CameraToggle(String),
    CameraReference(usize),
    CameraId(usize),
    CameraModel(usize),
    ImuChoice(Option<String>),
    BoardFamily,
    BoardBorderBits,
    BoardTagSize,
    BoardGap,
    BoardRows,
    BoardCornerMapping,
    BoardGrid,
    DetectorMaxHamming,
    DetectorRefinement,
    DetectorMaxIterations,
    DetectorMaxPixels,
    DetectorMaxClusters,
    DetectorMaxCandidates,
    ClosureRotation,
    ClosureTranslation,
    SolverMaxIterations,
    SolverPixelSigma,
    SolverHuberDelta,
    SolverPointRejection,
    SolverRejection,
    InertialMethod,
    TimeOffsetKind,
    TimeOffsetFixed,
    TimeOffsetInitial,
    TimeOffsetMinimum,
    TimeOffsetMaximum,
    TimeOffsetSteps,
    ImuGravity,
    ImuGyroDensity,
    ImuAccelDensity,
    ImuGyroBiasWalk,
    ImuAccelBiasWalk,
    ImuMaxGap,
    ImuMaxSamples,
    ImuIntegrationMethod,
    ImuKnotInterval,
    ImuRefineAxes,
    ImuGyroMatrix,
    ImuAccelMatrix,
    GroupToggle(GroupId),
    PreflightStart,
    PreflightExpert,
}

/// Logical, collapsible group inside a form section. Advanced groups start
/// collapsed so the common path stays visible first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GroupId {
    SourceInput,
    SourceSync,
    BoardGeometry,
    BoardCorner,
    BoardDetector,
    SolverCommon,
    SolverNumeric,
    SolverClosure,
    SolverInertial,
    SolverImuNoise,
    SolverImuAxes,
    SensorCameras,
    SensorImu,
    SensorActions,
}

impl GroupId {
    const fn section(self) -> FormSection {
        match self {
            Self::SourceInput | Self::SourceSync => FormSection::Source,
            Self::BoardGeometry | Self::BoardCorner | Self::BoardDetector => FormSection::Board,
            Self::SolverCommon
            | Self::SolverNumeric
            | Self::SolverClosure
            | Self::SolverInertial
            | Self::SolverImuNoise
            | Self::SolverImuAxes => FormSection::Solver,
            Self::SensorCameras | Self::SensorImu | Self::SensorActions => FormSection::Sensors,
        }
    }

    pub const fn bit(self) -> u8 {
        match self {
            Self::SourceInput | Self::BoardGeometry | Self::SolverCommon | Self::SensorCameras => 0,
            Self::SourceSync | Self::BoardCorner | Self::SolverNumeric | Self::SensorImu => 1,
            Self::BoardDetector | Self::SolverClosure | Self::SensorActions => 2,
            Self::SolverInertial => 3,
            Self::SolverImuNoise => 4,
            Self::SolverImuAxes => 5,
        }
    }

    const fn title(self) -> &'static str {
        match self {
            Self::SourceInput => "数据源",
            Self::SourceSync => "同步（高级）",
            Self::BoardGeometry => "标定板几何",
            Self::BoardCorner => "角点映射（高级）",
            Self::BoardDetector => "检测器与预算（高级）",
            Self::SolverCommon => "模式与内参",
            Self::SolverNumeric => "数值求解（高级）",
            Self::SolverClosure => "闭环约束（高级）",
            Self::SolverInertial => "惯性方法（高级）",
            Self::SolverImuNoise => "IMU 噪声与积分（高级）",
            Self::SolverImuAxes => "IMU 轴参数（高级）",
            Self::SensorCameras => "相机",
            Self::SensorImu => "IMU",
            Self::SensorActions => "操作",
        }
    }

    const fn advanced(self) -> bool {
        matches!(
            self,
            Self::SourceSync
                | Self::BoardCorner
                | Self::BoardDetector
                | Self::SolverNumeric
                | Self::SolverClosure
                | Self::SolverInertial
                | Self::SolverImuNoise
                | Self::SolverImuAxes
        )
    }

    const fn table(self) -> TableId {
        match self {
            Self::SensorCameras => TableId::Camera,
            Self::SensorImu => TableId::Imu,
            Self::SensorActions => TableId::Actions,
            _ => TableId::Params,
        }
    }
}

const SOURCE_GROUPS: [GroupId; 2] = [GroupId::SourceInput, GroupId::SourceSync];
const BOARD_GROUPS: [GroupId; 3] = [
    GroupId::BoardGeometry,
    GroupId::BoardCorner,
    GroupId::BoardDetector,
];
const SOLVER_GROUPS: [GroupId; 6] = [
    GroupId::SolverCommon,
    GroupId::SolverNumeric,
    GroupId::SolverClosure,
    GroupId::SolverInertial,
    GroupId::SolverImuNoise,
    GroupId::SolverImuAxes,
];
const SENSOR_GROUPS: [GroupId; 3] = [
    GroupId::SensorCameras,
    GroupId::SensorImu,
    GroupId::SensorActions,
];

fn groups(section: FormSection) -> &'static [GroupId] {
    match section {
        FormSection::Source => &SOURCE_GROUPS[..],
        FormSection::Sensors => &SENSOR_GROUPS[..],
        FormSection::Board => &BOARD_GROUPS[..],
        FormSection::Solver => &SOLVER_GROUPS[..],
        FormSection::Preflight => &[],
    }
}

/// Bit mask of advanced groups, which start collapsed.
#[must_use]
pub fn default_collapsed(section: FormSection) -> u32 {
    groups(section)
        .iter()
        .filter(|group| group.advanced())
        .fold(0, |mask, group| mask | (1 << group.bit()))
}

#[must_use]
pub fn is_collapsed(app: &App, group: GroupId) -> bool {
    app.collapsed_groups[group.section().index()] & (1 << group.bit()) != 0
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TableId {
    Params,
    Camera,
    Imu,
    Actions,
}

#[derive(Clone, Copy, Debug)]
pub struct Column {
    pub title: &'static str,
    pub min_width: u16,
    pub weight: u16,
}

const PARAM_COLUMNS: [Column; 3] = [
    Column {
        title: "参数",
        min_width: 10,
        weight: 0,
    },
    Column {
        title: "值",
        min_width: 16,
        weight: 2,
    },
    Column {
        title: "说明",
        min_width: 16,
        weight: 1,
    },
];

const CAMERA_COLUMNS: [Column; 7] = [
    Column {
        title: "启用",
        min_width: 4,
        weight: 0,
    },
    Column {
        title: "ID",
        min_width: 8,
        weight: 1,
    },
    Column {
        title: "Topic",
        min_width: 16,
        weight: 3,
    },
    Column {
        title: "模型",
        min_width: 10,
        weight: 1,
    },
    Column {
        title: "数量",
        min_width: 8,
        weight: 0,
    },
    Column {
        title: "Hz",
        min_width: 7,
        weight: 0,
    },
    Column {
        title: "参考",
        min_width: 4,
        weight: 0,
    },
];

const IMU_COLUMNS: [Column; 6] = [
    Column {
        title: "选择",
        min_width: 4,
        weight: 0,
    },
    Column {
        title: "Topic",
        min_width: 16,
        weight: 3,
    },
    Column {
        title: "类型",
        min_width: 6,
        weight: 0,
    },
    Column {
        title: "数量",
        min_width: 8,
        weight: 0,
    },
    Column {
        title: "频率",
        min_width: 7,
        weight: 0,
    },
    Column {
        title: "时间",
        min_width: 6,
        weight: 0,
    },
];

#[must_use]
pub fn columns(table: TableId) -> &'static [Column] {
    match table {
        TableId::Params => &PARAM_COLUMNS,
        TableId::Camera => &CAMERA_COLUMNS,
        TableId::Imu => &IMU_COLUMNS,
        TableId::Actions => &[],
    }
}

#[derive(Clone, Debug)]
pub struct Cell {
    pub id: Option<FieldId>,
    pub kind: FieldKind,
    pub text: String,
}

#[derive(Clone, Debug)]
pub struct Row {
    pub group: GroupId,
    pub table: TableId,
    pub selectable: bool,
    pub header: bool,
    pub columns: bool,
    pub cells: Vec<Cell>,
    pub note: String,
    pub detail: String,
}

impl Row {
    fn selectable_cell(&self, cell: usize) -> Option<&Cell> {
        self.cells.get(cell)
    }
}

fn cell(id: Option<FieldId>, kind: FieldKind, text: impl Into<String>) -> Cell {
    Cell {
        id,
        kind,
        text: text.into(),
    }
}

fn param_row(
    group: GroupId,
    id: FieldId,
    label: impl Into<String>,
    value: impl Into<String>,
    kind: FieldKind,
    note: impl Into<String>,
) -> Row {
    let note = note.into();
    Row {
        group,
        table: TableId::Params,
        selectable: true,
        header: false,
        columns: false,
        cells: vec![
            cell(None, FieldKind::ReadOnly, label),
            cell(Some(id), kind, value),
        ],
        detail: note.clone(),
        note,
    }
}

fn action_row(
    group: GroupId,
    id: FieldId,
    label: impl Into<String>,
    note: impl Into<String>,
) -> Row {
    let note = note.into();
    Row {
        group,
        table: TableId::Actions,
        selectable: true,
        header: false,
        columns: false,
        cells: vec![cell(Some(id), FieldKind::Button, label)],
        detail: note.clone(),
        note,
    }
}

fn group_header_row(app: &App, group: GroupId, count: usize) -> Row {
    let collapsed = is_collapsed(app, group);
    let marker = if collapsed { "▸" } else { "▾" };
    let body = if collapsed {
        format!("{count} 项 · 已收起 · Enter/Space 展开")
    } else if group.advanced() {
        format!("{count} 项 · 高级 · Enter/Space 收起")
    } else {
        format!("{count} 项 · Enter/Space 收起")
    };
    let note = match group {
        GroupId::SensorCameras => format!(
            "{}  · 已启用 {} · 扫描到图像 topic {}",
            body,
            app.draft.cameras.len(),
            image_topics(app).len()
        ),
        GroupId::SensorImu => format!(
            "{}  · 当前 {}",
            body,
            app.draft.imu_topic.as_deref().unwrap_or("（不使用）")
        ),
        _ => body,
    };
    Row {
        group,
        table: group.table(),
        selectable: true,
        header: true,
        columns: false,
        cells: vec![cell(
            Some(FieldId::GroupToggle(group)),
            FieldKind::Button,
            format!("{marker} {}", group.title()),
        )],
        detail: note.clone(),
        note,
    }
}

fn column_header_row(group: GroupId, table: TableId) -> Row {
    Row {
        group,
        table,
        selectable: false,
        header: false,
        columns: true,
        cells: columns(table)
            .iter()
            .map(|column| cell(None, FieldKind::ReadOnly, column.title))
            .collect(),
        note: String::new(),
        detail: String::new(),
    }
}

/// Visible rows for a section, including collapsible group headers and
/// non-focusable column headers.
#[must_use]
pub fn rows(app: &App, section: FormSection) -> Vec<Row> {
    let raw = match section {
        FormSection::Source => source_rows(app),
        FormSection::Sensors => sensor_rows(app),
        FormSection::Board => board_rows(app),
        FormSection::Solver => solver_rows(app),
        FormSection::Preflight => Vec::new(),
    };
    let mut visible = Vec::with_capacity(raw.len() + 8);
    for group in groups(section) {
        let members = raw
            .iter()
            .filter(|row| row.group == *group)
            .cloned()
            .collect::<Vec<_>>();
        if members.is_empty() {
            continue;
        }
        if group.table() == TableId::Actions {
            visible.extend(members);
            continue;
        }
        visible.push(group_header_row(app, *group, members.len()));
        if is_collapsed(app, *group) {
            continue;
        }
        visible.push(column_header_row(*group, group.table()));
        visible.extend(members);
    }
    visible
}

#[must_use]
pub fn normalize_row(rows: &[Row], current: usize) -> usize {
    if rows.is_empty() {
        return 0;
    }
    if rows.get(current).is_some_and(|row| row.selectable) {
        return current;
    }
    rows.iter()
        .enumerate()
        .skip(current)
        .find_map(|(index, row)| row.selectable.then_some(index))
        .or_else(|| rows.iter().position(|row| row.selectable))
        .unwrap_or(current.min(rows.len() - 1))
}

#[must_use]
pub fn last_selectable_row(rows: &[Row]) -> usize {
    rows.iter().rposition(|row| row.selectable).unwrap_or(0)
}

/// Move by `delta` selectable rows. Wrapping is used for Tab/BackTab.
#[must_use]
pub fn navigate_row(rows: &[Row], current: usize, delta: isize, wrap: bool) -> usize {
    if rows.is_empty() || delta == 0 {
        return current;
    }
    let count = isize::try_from(rows.len()).unwrap_or(isize::MAX);
    let mut index = current.min(rows.len() - 1);
    let step = delta.signum();
    for _ in 0..delta.unsigned_abs() {
        let mut candidate = isize::try_from(index).unwrap_or(0);
        loop {
            let next = candidate + step;
            if wrap {
                candidate = next.rem_euclid(count);
            } else {
                if next < 0 || next >= count {
                    break;
                }
                candidate = next;
            }
            let position = usize::try_from(candidate).unwrap_or(0);
            if rows[position].selectable {
                index = position;
                break;
            }
            if candidate == isize::try_from(index).unwrap_or(0) {
                break;
            }
        }
    }
    index
}

#[must_use]
pub fn find_topic_row(rows: &[Row], topic: &str) -> Option<usize> {
    rows.iter().position(|row| {
        row.table == TableId::Camera
            && row
                .cells
                .get(2)
                .is_some_and(|cel| cel.text.as_str() == topic)
    })
}

#[must_use]
pub fn focused_cell(rows: &[Row], row: usize, index: usize) -> Option<Cell> {
    rows.get(row)
        .and_then(|row| row.selectable_cell(index))
        .cloned()
}

fn source_rows(app: &App) -> Vec<Row> {
    let mut rows = vec![
        param_row(
            GroupId::SourceInput,
            FieldId::SourcePath,
            "bag 路径",
            app.draft.source_path.as_str(),
            FieldKind::Text,
            "Enter 编辑；改变路径会清空已扫描的 topic 元数据",
        ),
        param_row(
            GroupId::SourceInput,
            FieldId::SourceFormat,
            "bag 格式",
            bag_format_label(app.draft.format),
            FieldKind::Choice,
            "Enter 打开下拉；方向键只移动焦点，不改值",
        ),
        param_row(
            GroupId::SourceInput,
            FieldId::SourceScan,
            "扫描 bag 元数据",
            if app.inspect_worker.is_some() {
                "扫描中…"
            } else {
                "开始扫描"
            },
            FieldKind::Button,
            "只读取 topic/消息数/时间戳，不解码图像；扫描完成不会切换页面或重置焦点",
        ),
    ];
    rows.push(param_row(
        GroupId::SourceSync,
        FieldId::SourceMaxDelta,
        "同步最大时间差",
        app.draft.max_delta_ns.as_str(),
        FieldKind::Text,
        "单位 ns；非负整数",
    ));
    rows.push(param_row(
        GroupId::SourceSync,
        FieldId::SourceRequireAll,
        "要求所有相机同步",
        if app.draft.require_all_cameras {
            "x"
        } else {
            " "
        },
        FieldKind::Toggle,
        "Space 切换；关闭后缺失部分相机的视图仍会保留",
    ));
    rows
}

fn board_rows(app: &App) -> Vec<Row> {
    let mut rows = board_geometry_rows(app);
    rows.push(param_row(
        GroupId::BoardCorner,
        FieldId::BoardCornerMapping,
        "decoded→front 角点",
        app.draft.target.corner_mapping_text.as_str(),
        FieldKind::Text,
        "4 个整数，必须是 TL/TR/BR/BL 的循环排列；默认 2, 3, 0, 1",
    ));
    rows.extend(board_detector_rows(app));
    rows
}

fn board_geometry_rows(app: &App) -> Vec<Row> {
    let target = &app.draft.target;
    vec![
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardFamily,
            "tag 家族",
            tag_family_label(target.family),
            FieldKind::Choice,
            "AprilGrid 编码家族；Enter 打开下拉",
        ),
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardBorderBits,
            "border bits",
            target.border_bits.as_str(),
            FieldKind::Text,
            "整数；配置文件要求 1 或 2",
        ),
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardTagSize,
            "tag 边长 [m]",
            target.tag_size_m.as_str(),
            FieldKind::Text,
            "单位米；正值",
        ),
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardGap,
            "tag 间距 [m]",
            target.tag_gap_m.as_str(),
            FieldKind::Text,
            "单位米；非负",
        ),
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardRows,
            "tag ID 行（; 分行，, 分列）",
            target.id_rows_text.as_str(),
            FieldKind::Text,
            "Enter 打开完整文本编辑器；完整保留行数与每行 ID 顺序，不会按尺寸重新生成",
        ),
        param_row(
            GroupId::BoardGeometry,
            FieldId::BoardGrid,
            "网格",
            target.grid_summary(),
            FieldKind::ReadOnly,
            "由上面的 ID 行精确计算，仅用于显示",
        ),
    ]
}

fn board_detector_rows(app: &App) -> Vec<Row> {
    let detector = &app.draft.detector;
    vec![
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorMaxHamming,
            "max_hamming",
            detector.max_hamming.as_str(),
            FieldKind::Text,
            "0..=255；0 表示严格匹配",
        ),
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorRefinement,
            "角点细化",
            refinement_label(detector.refinement),
            FieldKind::Choice,
            "disabled / saddle / outer_contour",
        ),
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorMaxIterations,
            "细化最大迭代",
            detector.max_iterations.as_str(),
            FieldKind::Text,
            "正整数",
        ),
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorMaxPixels,
            "max_pixels",
            detector.max_pixels.as_str(),
            FieldKind::Text,
            "检测器预算；正整数",
        ),
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorMaxClusters,
            "max_clusters",
            detector.max_clusters.as_str(),
            FieldKind::Text,
            "检测器预算；正整数",
        ),
        param_row(
            GroupId::BoardDetector,
            FieldId::DetectorMaxCandidates,
            "max_candidates",
            detector.max_candidates.as_str(),
            FieldKind::Text,
            "检测器预算；正整数",
        ),
    ]
}

#[allow(
    clippy::too_many_lines,
    reason = "one declarative list keeps mode-dependent controls and row order explicit"
)]
fn solver_rows(app: &App) -> Vec<Row> {
    let draft = &app.draft;
    let solver = &draft.solver;
    let mut rows = vec![
        param_row(
            GroupId::SolverCommon,
            FieldId::Mode,
            "标定模式（显式）",
            draft.mode.title(),
            FieldKind::Choice,
            "Enter 打开下拉；勾选 IMU 不会自动改变模式，方向键不改值",
        ),
        param_row(
            GroupId::SolverCommon,
            FieldId::Reference,
            "参考相机",
            draft.reference_label(),
            FieldKind::Choice,
            "当前模式使用的参考/单目相机；也可在 Sensors 相机表的参考列选择",
        ),
        param_row(
            GroupId::SolverCommon,
            FieldId::IntrinsicStrategy,
            "内参策略",
            draft.intrinsic.strategy.label(),
            FieldKind::Choice,
            format!(
                "当前模式支持：{}",
                draft
                    .strategy_options()
                    .iter()
                    .map(|strategy| strategy.short_label())
                    .collect::<Vec<_>>()
                    .join(" / ")
            ),
        ),
    ];
    if !matches!(draft.intrinsic.strategy, IntrinsicStrategy::AutoOptimize) {
        rows.push(param_row(
            GroupId::SolverCommon,
            FieldId::IntrinsicPath,
            "内参初值文件",
            if draft.intrinsic.path.is_empty() {
                "（空）"
            } else {
                draft.intrinsic.path.as_str()
            },
            FieldKind::Text,
            "Enter 编辑；只检查存在性，内容在求解阶段读取",
        ));
    }
    rows.push(param_row(
        GroupId::SolverNumeric,
        FieldId::SolverMaxIterations,
        "最大迭代",
        solver.max_iterations.as_str(),
        FieldKind::Text,
        "1..=1000",
    ));
    rows.push(param_row(
        GroupId::SolverNumeric,
        FieldId::SolverPixelSigma,
        "pixel_sigma",
        solver.pixel_sigma.as_str(),
        FieldKind::Text,
        "正值；像素噪声尺度",
    ));
    rows.push(param_row(
        GroupId::SolverNumeric,
        FieldId::SolverHuberDelta,
        "huber_delta",
        solver.huber_delta.as_str(),
        FieldKind::Text,
        "白化单位；像素阈值 = pixel_sigma × huber_delta",
    ));
    if matches!(draft.method, crate::config::InertialMethod::ContinuousTime)
        && matches!(draft.mode, CalibrationMode::MultiCameraImu)
    {
        rows.push(param_row(
            GroupId::SolverNumeric,
            FieldId::SolverRejection,
            "截断策略",
            rejection_policy_label(solver.rejection),
            FieldKind::Choice,
            "连续时间可选仅 Huber；像素阈值截断仅用于视觉与预积分",
        ));
    }
    if solver.rejection == RejectionPolicy::PixelThreshold {
        rows.push(param_row(
            GroupId::SolverNumeric,
            FieldId::SolverPointRejection,
            "point_rejection_px",
            solver.point_rejection_px.as_str(),
            FieldKind::Text,
            "正值；像素",
        ));
    }
    if matches!(draft.mode, CalibrationMode::MultiCamera) {
        rows.push(param_row(
            GroupId::SolverClosure,
            FieldId::ClosureRotation,
            "闭环旋转上限 [deg]",
            draft.closure_rotation_deg.as_str(),
            FieldKind::Text,
            "正值；多相机模式使用",
        ));
        rows.push(param_row(
            GroupId::SolverClosure,
            FieldId::ClosureTranslation,
            "闭环平移上限 [m]",
            draft.closure_translation_m.as_str(),
            FieldKind::Text,
            "正值；多相机模式使用",
        ));
    }
    if matches!(draft.mode, CalibrationMode::MultiCameraImu) {
        rows.push(param_row(
            GroupId::SolverInertial,
            FieldId::InertialMethod,
            "惯性方法",
            inertial_method_label(draft.method),
            FieldKind::Choice,
            "预积分或连续时间 B 样条；只在相机-IMU 模式显示",
        ));
        rows.push(param_row(
            GroupId::SolverInertial,
            FieldId::TimeOffsetKind,
            "时间偏移",
            time_offset_kind_label(draft.time_offset.kind),
            FieldKind::Choice,
            "固定或估计",
        ));
        match draft.time_offset.kind {
            TimeOffsetKind::Fixed => rows.push(param_row(
                GroupId::SolverInertial,
                FieldId::TimeOffsetFixed,
                "固定时间偏移 [s]",
                draft.time_offset.fixed_s.as_str(),
                FieldKind::Text,
                "有限数值",
            )),
            TimeOffsetKind::Estimate => {
                rows.push(param_row(
                    GroupId::SolverInertial,
                    FieldId::TimeOffsetInitial,
                    "偏移初值 [s]",
                    draft.time_offset.initial_s.as_str(),
                    FieldKind::Text,
                    "必须位于最小/最大范围内",
                ));
                rows.push(param_row(
                    GroupId::SolverInertial,
                    FieldId::TimeOffsetMinimum,
                    "偏移最小 [s]",
                    draft.time_offset.minimum_s.as_str(),
                    FieldKind::Text,
                    "min < max",
                ));
                rows.push(param_row(
                    GroupId::SolverInertial,
                    FieldId::TimeOffsetMaximum,
                    "偏移最大 [s]",
                    draft.time_offset.maximum_s.as_str(),
                    FieldKind::Text,
                    "min < max",
                ));
                rows.push(param_row(
                    GroupId::SolverInertial,
                    FieldId::TimeOffsetSteps,
                    "初始化步数",
                    draft.time_offset.initialization_steps.as_str(),
                    FieldKind::Text,
                    "2..=101",
                ));
            }
        }
        let imu = &draft.imu;
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuGravity,
            "重力模长 [m/s²]",
            imu.gravity_magnitude_m_s2.as_str(),
            FieldKind::Text,
            "正值",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuGyroDensity,
            "gyro 噪声密度 [rad/s/√Hz]",
            imu.gyro_density_rad_s_sqrt_hz.as_str(),
            FieldKind::Text,
            "正值；标定物理测量帧",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuAccelDensity,
            "accel 噪声密度 [m/s²/√Hz]",
            imu.accelerometer_density_m_s2_sqrt_hz.as_str(),
            FieldKind::Text,
            "正值；标定物理测量帧",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuGyroBiasWalk,
            "gyro bias walk [rad/s²/√Hz]",
            imu.gyro_bias_walk_rad_s2_sqrt_hz.as_str(),
            FieldKind::Text,
            "正值",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuAccelBiasWalk,
            "accel bias walk [m/s³/√Hz]",
            imu.accelerometer_bias_walk_m_s3_sqrt_hz.as_str(),
            FieldKind::Text,
            "正值",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuMaxGap,
            "积分最大间隔 [s]",
            imu.max_gap_s.as_str(),
            FieldKind::Text,
            "正值",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuMaxSamples,
            "积分最大样本数",
            imu.max_samples.as_str(),
            FieldKind::Text,
            ">= 2",
        ));
        rows.push(param_row(
            GroupId::SolverImuNoise,
            FieldId::ImuKnotInterval,
            "样条节点间隔 [s]",
            imu.knot_interval_s.as_str(),
            FieldKind::Text,
            "仅连续时间使用；正值",
        ));
        rows.push(param_row(
            GroupId::SolverImuAxes,
            FieldId::ImuRefineAxes,
            "优化 IMU 轴参数",
            if imu.refine_axes { "x" } else { " " },
            FieldKind::Toggle,
            if matches!(draft.method, crate::config::InertialMethod::ContinuousTime) {
                "连续时间第二遍优化轴矩阵；预积分要求关闭"
            } else {
                "预积分要求关闭且矩阵为单位阵"
            },
        ));
        rows.push(param_row(
            GroupId::SolverImuAxes,
            FieldId::ImuGyroMatrix,
            "gyro 轴矩阵 3×3（行优先）",
            imu.gyro_calibrated_from_raw.as_str(),
            FieldKind::Text,
            "physical_gyro = A * raw_gyro - bias；9 个数值",
        ));
        rows.push(param_row(
            GroupId::SolverImuAxes,
            FieldId::ImuAccelMatrix,
            "accel 轴矩阵 3×3（行优先）",
            imu.accelerometer_calibrated_from_raw.as_str(),
            FieldKind::Text,
            "physical_specific_force = A * raw_accel - bias；9 个数值",
        ));
        rows.push(param_row(
            GroupId::SolverImuAxes,
            FieldId::ImuIntegrationMethod,
            "积分方法",
            integration_method_label(imu.integration_method),
            FieldKind::ReadOnly,
            "配置只支持 midpoint",
        ));
    }
    rows
}

fn image_topics(app: &App) -> Vec<&TopicInspection> {
    visible_topics(app)
        .into_iter()
        .filter(|topic| topic.type_name != "sensor_msgs/Imu")
        .collect()
}

fn camera_row(
    app: &App,
    index: Option<usize>,
    camera: Option<&CameraDraft>,
    meta: Option<&TopicInspection>,
    topic_name: &str,
) -> Row {
    let topic = topic_name.to_owned();
    let selected = camera.is_some();
    let reference = camera.is_some_and(|camera| app.draft.reference_id == camera.id);
    let count = meta.map_or_else(
        || "未知".to_owned(),
        |topic| topic.message_count.to_string(),
    );
    let note = camera.map_or_else(
        || "未启用".to_owned(),
        |camera| {
            format!(
                "{} · {} · {}",
                camera.id,
                camera_model_label(camera.model),
                if reference { "参考" } else { "非参考" }
            )
        },
    );
    Row {
        group: GroupId::SensorCameras,
        table: TableId::Camera,
        selectable: true,
        header: false,
        columns: false,
        cells: vec![
            cell(
                Some(FieldId::CameraToggle(topic.clone())),
                FieldKind::Toggle,
                if selected { "x" } else { " " },
            ),
            cell(
                index.map(FieldId::CameraId),
                if selected {
                    FieldKind::Text
                } else {
                    FieldKind::ReadOnly
                },
                camera.map_or_else(|| "—".to_owned(), |camera| camera.id.clone()),
            ),
            cell(None, FieldKind::ReadOnly, topic),
            cell(
                index.map(FieldId::CameraModel),
                if selected {
                    FieldKind::Choice
                } else {
                    FieldKind::ReadOnly
                },
                camera.map_or_else(
                    || "—".to_owned(),
                    |camera| camera_model_label(camera.model).to_owned(),
                ),
            ),
            cell(None, FieldKind::ReadOnly, count),
            cell(
                None,
                FieldKind::ReadOnly,
                meta.and_then(TopicInspection::rate_hz)
                    .map_or_else(|| "—".to_owned(), |rate| format!("{rate:.2}")),
            ),
            cell(
                index.map(FieldId::CameraReference),
                if selected {
                    FieldKind::Toggle
                } else {
                    FieldKind::ReadOnly
                },
                if camera.is_some() {
                    if reference { "●" } else { "○" }
                } else {
                    "—"
                },
            ),
        ],
        detail: camera_detail(app, camera, meta),
        note,
    }
}

fn camera_detail(
    app: &App,
    camera: Option<&CameraDraft>,
    meta: Option<&TopicInspection>,
) -> String {
    let mut lines = Vec::new();
    if let Some(camera) = camera {
        lines.push(format!(
            "ID {} · 模型 {} · {}",
            camera.id,
            camera_model_label(camera.model),
            if app.draft.reference_id == camera.id {
                "参考相机"
            } else {
                "非参考"
            }
        ));
    } else {
        lines.push("未启用：启用后可编辑 ID/模型并选择参考".to_owned());
    }
    let topic = camera
        .map(|camera| camera.topic.as_str())
        .or_else(|| meta.map(|topic| topic.name.as_str()))
        .unwrap_or("未知");
    lines.push(format!("Topic {topic}"));
    lines.push(format!(
        "类型 {} · 消息数 {} · 平均消息频率（bag 记录时间） {}",
        meta.map_or("未知", |topic| topic.type_name.as_str()),
        meta.map_or_else(
            || "未知".to_owned(),
            |topic| topic.message_count.to_string()
        ),
        meta.and_then(TopicInspection::rate_hz)
            .map_or_else(|| "未知".to_owned(), |rate| format!("{rate:.2} Hz"))
    ));
    lines.push(format!(
        "header 时间 {} · 记录 {}",
        meta.map_or("未知", |topic| if topic.header_time_valid {
            "OK"
        } else {
            "ERR"
        }),
        time_range(meta)
    ));
    lines.push(format!(
        "状态 {} · 图像尺寸 未知（扫描只读元数据，不解码图像）",
        meta.map_or("未知", |topic| topic.status.as_str())
    ));
    lines.join("\n")
}

fn imu_detail(app: &App, option: Option<&String>, meta: Option<&TopicInspection>) -> String {
    let Some(topic) = option else {
        return format!(
            "不使用 IMU（当前模式仍为 {}）。选择 IMU 不会自动改变标定模式。",
            app.draft.mode.title()
        );
    };
    format!(
        "Topic {topic} · 类型 {} · 消息数 {}\n频率 {} · header 时间 {} · 记录 {}\n状态 {} · IMU 选择不会自动改变标定模式（当前 {}）",
        meta.map_or("未知", |topic| topic.type_name.as_str()),
        meta.map_or_else(
            || "未知".to_owned(),
            |topic| topic.message_count.to_string()
        ),
        meta.and_then(TopicInspection::rate_hz)
            .map_or_else(|| "未知".to_owned(), |rate| format!("{rate:.2} Hz")),
        meta.map_or("未知", |topic| if topic.header_time_valid {
            "OK"
        } else {
            "ERR"
        }),
        time_range(meta),
        meta.map_or("未知", |topic| topic.status.as_str()),
        app.draft.mode.title()
    )
}

fn time_range(meta: Option<&TopicInspection>) -> String {
    let Some(topic) = meta else {
        return "未知".to_owned();
    };
    match (topic.first_record_time_ns, topic.last_record_time_ns) {
        (Some(first), Some(last)) => format!("{}..{}", format_time_ns(first), format_time_ns(last)),
        _ => "未知".to_owned(),
    }
}

fn format_time_ns(ns: i64) -> String {
    let sign = if ns < 0 { "-" } else { "" };
    let value = ns.unsigned_abs();
    format!(
        "{sign}{}.{:09} s",
        value / 1_000_000_000,
        value % 1_000_000_000
    )
}

fn sensor_rows(app: &App) -> Vec<Row> {
    let mut rows = camera_rows(app);
    rows.extend(imu_rows(app));
    rows.push(action_row(
        GroupId::SensorActions,
        FieldId::ScanTopics,
        if app.inspect_worker.is_some() {
            "扫描中…"
        } else {
            "扫描 / 刷新 bag 元数据"
        },
        "只读取 topic/消息数/时间戳，不解码图像；扫描完成不会切换页面或重置焦点",
    ));
    rows
}

fn camera_rows(app: &App) -> Vec<Row> {
    let mut rows = Vec::new();
    let topics = image_topics(app);
    let mut known = app.seen_image_topics.clone();
    for topic in &topics {
        if !known.contains(&topic.name) {
            known.push(topic.name.clone());
        }
    }
    for (index, camera) in app.draft.cameras.iter().enumerate() {
        let meta = topics
            .iter()
            .find(|topic| topic.name == camera.topic)
            .copied();
        rows.push(camera_row(
            app,
            Some(index),
            Some(camera),
            meta,
            &camera.topic,
        ));
    }
    for name in &known {
        if !app.draft.supports(name) {
            let meta = topics.iter().find(|topic| topic.name == *name).copied();
            rows.push(camera_row(app, None, None, meta, name));
        }
    }
    if rows.is_empty() {
        rows.push(Row {
            group: GroupId::SensorCameras,
            table: TableId::Camera,
            selectable: true,
            header: false,
            columns: false,
            cells: vec![
                cell(None, FieldKind::ReadOnly, "—"),
                cell(None, FieldKind::ReadOnly, "无相机"),
                cell(None, FieldKind::ReadOnly, "未扫描或 bag 中无图像 topic"),
                cell(None, FieldKind::ReadOnly, "—"),
                cell(None, FieldKind::ReadOnly, "未知"),
                cell(None, FieldKind::ReadOnly, "—"),
            ],
            note: "先扫描 bag 元数据，或使用 YAML 专家编辑器添加相机".to_owned(),
            detail: "没有可显示的图像 topic。扫描只读取元数据，不会解码图像或切换页面。".to_owned(),
        });
    }
    rows
}

fn imu_rows(app: &App) -> Vec<Row> {
    let mut rows = Vec::new();
    for option in app.imu_topic_options() {
        let meta = option.as_ref().and_then(|name| {
            visible_topics(app)
                .into_iter()
                .find(|topic| topic.name.as_str() == name.as_str())
        });
        let selected = app.draft.imu_topic == option;
        let is_none = option.is_none();
        rows.push(Row {
            group: GroupId::SensorImu,
            table: TableId::Imu,
            selectable: true,
            header: false,
            columns: false,
            cells: vec![
                cell(
                    Some(FieldId::ImuChoice(option.clone())),
                    FieldKind::Toggle,
                    if selected { "x" } else { " " },
                ),
                cell(
                    None,
                    FieldKind::ReadOnly,
                    option.as_deref().unwrap_or("（不使用）"),
                ),
                cell(
                    None,
                    FieldKind::ReadOnly,
                    if is_none {
                        "—".to_owned()
                    } else {
                        meta.map_or_else(|| "未知".to_owned(), |topic| topic.type_name.clone())
                    },
                ),
                cell(
                    None,
                    FieldKind::ReadOnly,
                    if is_none {
                        "—".to_owned()
                    } else {
                        meta.map_or_else(
                            || "未知".to_owned(),
                            |topic| topic.message_count.to_string(),
                        )
                    },
                ),
                cell(
                    None,
                    FieldKind::ReadOnly,
                    if is_none {
                        "—".to_owned()
                    } else {
                        meta.and_then(TopicInspection::rate_hz)
                            .map_or_else(|| "未知".to_owned(), |rate| format!("{rate:.2}"))
                    },
                ),
                cell(
                    None,
                    FieldKind::ReadOnly,
                    if is_none {
                        "—".to_owned()
                    } else {
                        meta.map_or(
                            "未知",
                            |topic| if topic.header_time_valid { "OK" } else { "ERR" },
                        )
                        .to_owned()
                    },
                ),
            ],
            note: if is_none {
                "不使用 IMU".to_owned()
            } else {
                imu_selection_note(&app.draft)
            },
            detail: imu_detail(app, option.as_ref(), meta),
        });
    }
    rows
}

#[must_use]
pub fn visible_topics(app: &App) -> Vec<&TopicInspection> {
    app.inspection.as_ref().map_or_else(Vec::new, |inspection| {
        inspection
            .topics
            .iter()
            .filter(|topic| {
                matches!(
                    topic.type_name.as_str(),
                    "sensor_msgs/Image" | "sensor_msgs/CompressedImage" | "sensor_msgs/Imu"
                )
            })
            .collect()
    })
}

#[derive(Clone, Debug)]
pub struct Field {
    pub id: FieldId,
    pub label: String,
    pub value: String,
    pub kind: FieldKind,
    pub note: String,
}

fn field(
    id: FieldId,
    label: impl Into<String>,
    value: impl Into<String>,
    kind: FieldKind,
    note: impl Into<String>,
) -> Field {
    Field {
        id,
        label: label.into(),
        value: value.into(),
        kind,
        note: note.into(),
    }
}

/// Preflight keeps its existing action-field layout (checks + two actions).
/// Table sections use [`rows`] instead.
#[must_use]
pub fn fields(app: &App, section: FormSection) -> Vec<Field> {
    if section == FormSection::Preflight {
        preflight_fields(app)
    } else {
        Vec::new()
    }
}

fn preflight_fields(app: &App) -> Vec<Field> {
    let preflight = app.preflight();
    let start_label = if preflight.blockers() == 0 {
        "开始标定（唯一启动入口）".to_owned()
    } else {
        format!("开始标定（被 {} 个阻断项阻止）", preflight.blockers())
    };
    vec![
        field(
            FieldId::PreflightStart,
            "启动",
            start_label,
            FieldKind::Button,
            "Enter/Space 或鼠标点击；仅在无阻断项时真正启动",
        ),
        field(
            FieldId::PreflightExpert,
            "YAML 专家编辑器",
            "编辑完整配置",
            FieldKind::Button,
            "同一份 RunSettings；非法 YAML 不会覆盖表单草稿，离开需确认丢弃",
        ),
    ]
}
