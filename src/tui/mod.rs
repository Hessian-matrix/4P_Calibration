#[cfg(test)]
mod source_tests;

mod draft;
mod form;
mod preflight;
mod result;
mod task;
mod text;
mod views;

use std::io::{self, Stdout};
use std::path::Path;
use std::sync::mpsc;
use std::thread::{self, JoinHandle};
use std::time::Duration;

use anyhow::{Context, Result};
use crossterm::event::{
    self, DisableBracketedPaste, DisableMouseCapture, EnableBracketedPaste, EnableMouseCapture,
    Event as TerminalEvent, KeyCode, KeyEventKind, KeyModifiers, MouseButton, MouseEvent,
    MouseEventKind,
};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::Rect;

use crate::config::{BagSourceConfig, InertialMethod, RunSettings};
use crate::input::BagInspection;
use crate::run_record::{RunContext, RunStatus};

use self::draft::{
    BAG_FORMATS, CAMERA_MODELS, CalibrationMode, Draft, INERTIAL_METHODS, REFINEMENTS,
    RejectionPolicy, TAG_FAMILIES, TimeOffsetKind,
};
use self::form::{FieldId, FieldKind, FormSection};
use self::preflight::Preflight;
use self::result::{RawSource, ResultData};
use self::task::Task;
use self::text::TextBuffer;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Screen {
    Form(FormSection),
    Expert,
    Run,
    Result,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RunTab {
    Overview,
    Details,
    Log,
}

impl RunTab {
    const ALL: [Self; 3] = [Self::Overview, Self::Details, Self::Log];

    const fn index(self) -> usize {
        match self {
            Self::Overview => 0,
            Self::Details => 1,
            Self::Log => 2,
        }
    }

    const fn title(self) -> &'static str {
        match self {
            Self::Overview => "Overview",
            Self::Details => "Details",
            Self::Log => "Log",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ResultTab {
    Overview,
    Parameters,
    Extrinsics,
    Diagnostics,
    Raw,
}

impl ResultTab {
    const ALL: [Self; 5] = [
        Self::Overview,
        Self::Parameters,
        Self::Extrinsics,
        Self::Diagnostics,
        Self::Raw,
    ];

    const fn index(self) -> usize {
        match self {
            Self::Overview => 0,
            Self::Parameters => 1,
            Self::Extrinsics => 2,
            Self::Diagnostics => 3,
            Self::Raw => 4,
        }
    }

    const fn title(self) -> &'static str {
        match self {
            Self::Overview => "Overview",
            Self::Parameters => "Parameters",
            Self::Extrinsics => "Extrinsics",
            Self::Diagnostics => "Diagnostics",
            Self::Raw => "Raw",
        }
    }

    fn next(self, delta: isize) -> Self {
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
enum DropdownKind {
    Format,
    Mode,
    Reference,
    IntrinsicStrategy,
    CameraModel(usize),
    BoardFamily,
    Refinement,
    InertialMethod,
    TimeOffsetKind,
    Rejection,
}

#[derive(Clone, Debug)]
struct Dropdown {
    kind: DropdownKind,
    selected: usize,
}

#[derive(Clone, Debug)]
struct InputModal {
    title: String,
    target: FieldId,
    buffer: TextBuffer,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ConfirmAction {
    CancelRun,
    Quit,
    DiscardExpert,
}

#[derive(Clone, Debug)]
struct Confirmation {
    action: ConfirmAction,
    title: String,
    message: String,
    yes: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct TableFocus {
    row: usize,
    cell: usize,
}

#[derive(Clone, Debug)]
struct ExpertEditor {
    buffer: TextBuffer,
    saved_text: String,
    return_to: FormSection,
    error: Option<String>,
}

impl ExpertEditor {
    fn is_dirty(&self) -> bool {
        self.buffer.text() != self.saved_text
    }
}

#[derive(Clone, Debug)]
struct HitTarget {
    area: Rect,
    action: MouseAction,
}

impl HitTarget {
    fn contains(&self, column: u16, row: u16) -> bool {
        column >= self.area.x
            && column < self.area.x.saturating_add(self.area.width)
            && row >= self.area.y
            && row < self.area.y.saturating_add(self.area.height)
    }
}

#[derive(Clone, Debug)]
enum MouseAction {
    Section(FormSection),
    PrevSection,
    NextSection,
    Field(usize),
    Cell(usize, usize),
    RunTab(usize),
    ResultTab(usize),
    DropdownOption(usize),
    InputApply,
    InputCancel,
    ConfirmYes,
    ConfirmNo,
    RunCancel,
    DashboardGraph(usize),
    RunOpenResult,
    ResultDashboard,
    ResultOpenDir,
    ResultOpenHtml,
    ResultModify,
    ResultRerun,
    ResultRawCycle,
    ExpertApply,
    ExpertClose,
    Quit,
}

struct InspectWorker {
    receiver: mpsc::Receiver<Result<BagInspection, String>>,
    handle: Option<JoinHandle<()>>,
    source_generation: u64,
}

impl InspectWorker {
    fn start(source: BagSourceConfig, source_generation: u64) -> Self {
        let (sender, receiver) = mpsc::sync_channel(1);
        let handle = thread::spawn(move || {
            let result = crate::input::inspect(&source).map_err(|error| format!("{error:#}"));
            let _ = sender.send(result);
        });
        Self {
            receiver,
            handle: Some(handle),
            source_generation,
        }
    }

    fn join(&mut self) -> Result<()> {
        if let Some(handle) = self.handle.take() {
            handle
                .join()
                .map_err(|_| anyhow::anyhow!("bag inspection worker panicked"))?;
        }
        Ok(())
    }
}

#[allow(
    clippy::struct_excessive_bools,
    reason = "independent UI mode flags: scan/run lifecycle, help, mouse capture, quit"
)]
struct App {
    base_run: RunContext,
    started_once: bool,
    draft: Draft,
    inspection: Option<BagInspection>,
    inspect_worker: Option<InspectWorker>,
    source_generation: u64,
    seen_image_topics: Vec<String>,
    screen: Screen,
    section_focus: [TableFocus; 5],
    section_scroll: [usize; 5],
    collapsed_groups: [u32; 5],
    preflight_scroll: usize,
    dropdown: Option<Dropdown>,
    input: Option<InputModal>,
    confirmation: Option<Confirmation>,
    expert: Option<ExpertEditor>,
    help: bool,
    help_scroll: usize,
    state: String,
    mouse_capture: bool,
    task: Option<Task>,
    run_tab: RunTab,
    run_scroll: [usize; 3],
    result_tab: ResultTab,
    result_scroll: [usize; 5],
    raw_source: RawSource,
    result_data: Option<ResultData>,
    quit_requested: bool,
    hit_targets: Vec<HitTarget>,
}

impl App {
    fn new(initial: Option<&RunSettings>, base_run: RunContext) -> Self {
        let draft = initial.map_or_else(Draft::default, Draft::from_settings);
        let mut collapsed_groups = [0; 5];
        let mut section_focus = [TableFocus::default(); 5];
        let seen_image_topics = draft
            .cameras
            .iter()
            .map(|camera| camera.topic.clone())
            .collect();
        for section in FormSection::ALL {
            collapsed_groups[section.index()] = form::default_collapsed(section);
            if section != FormSection::Preflight {
                // Row 0 is a group header; start on the first data row after
                // the column header so Enter edits a value instead of folding.
                section_focus[section.index()].row = 1;
            }
        }
        Self {
            base_run,
            started_once: false,
            draft,
            inspection: None,
            inspect_worker: None,
            source_generation: 0,
            seen_image_topics,
            screen: Screen::Form(FormSection::Source),
            section_focus,
            section_scroll: [0; 5],
            collapsed_groups,
            preflight_scroll: 0,
            dropdown: None,
            input: None,
            confirmation: None,
            expert: None,
            help: false,
            help_scroll: 0,
            state: "ready".to_owned(),
            mouse_capture: true,
            task: None,
            run_tab: RunTab::Overview,
            run_scroll: [0; 3],
            result_tab: ResultTab::Overview,
            result_scroll: [0; 5],
            raw_source: RawSource::CalibrationYaml,
            result_data: None,
            quit_requested: false,
            hit_targets: Vec::new(),
        }
    }

    fn preflight(&self) -> Preflight {
        let root = self.base_run.root();
        let parent = root
            .parent()
            .map_or_else(|| std::path::PathBuf::from("runs"), Path::to_path_buf);
        Preflight::evaluate(&self.draft, self.inspection.as_ref(), &parent)
    }

    fn section(&self) -> Option<FormSection> {
        match self.screen {
            Screen::Form(section) => Some(section),
            _ => None,
        }
    }

    fn focus(&self, section: FormSection) -> usize {
        self.section_focus[section.index()].row
    }

    fn focus_cell(&self, section: FormSection) -> usize {
        self.section_focus[section.index()].cell
    }

    fn set_focus(&mut self, section: FormSection, row: usize, cell: usize) {
        self.section_focus[section.index()] = TableFocus { row, cell };
    }

    fn field_count(&self, section: FormSection) -> usize {
        if section == FormSection::Preflight {
            form::fields(self, section).len()
        } else {
            form::rows(self, section).len()
        }
    }

    fn clamp_focus(&mut self) {
        let Some(section) = self.section() else {
            return;
        };
        let focus = self.section_focus[section.index()];
        if section == FormSection::Preflight {
            let count = form::fields(self, section).len();
            let row = if count == 0 {
                0
            } else {
                focus.row.min(count - 1)
            };
            self.set_focus(section, row, 0);
            return;
        }
        let rows = form::rows(self, section);
        if rows.is_empty() {
            self.set_focus(section, 0, 0);
            return;
        }
        let row = form::normalize_row(&rows, focus.row);
        let cells = rows[row].cells.len();
        let cell = if cells == 0 {
            0
        } else {
            focus.cell.min(cells - 1)
        };
        self.set_focus(section, row, cell);
    }

    fn move_focus(&mut self, delta: isize) {
        self.move_focus_with(delta, false);
    }

    fn move_focus_wrapped(&mut self, delta: isize) {
        self.move_focus_with(delta, true);
    }

    fn move_focus_with(&mut self, delta: isize, wrap: bool) {
        let Some(section) = self.section() else {
            return;
        };
        if section == FormSection::Preflight {
            let count = form::fields(self, section).len();
            if count == 0 {
                return;
            }
            let current = isize::try_from(self.focus(section)).unwrap_or(0);
            let count_isize = isize::try_from(count).unwrap_or(1);
            let next = if wrap {
                (current + delta).rem_euclid(count_isize)
            } else {
                (current + delta).clamp(0, count_isize - 1)
            };
            self.set_focus(section, usize::try_from(next).unwrap_or(0), 0);
            return;
        }
        let rows = form::rows(self, section);
        if rows.is_empty() {
            return;
        }
        let row = form::navigate_row(&rows, self.focus(section), delta, wrap);
        let cell = self.focus_cell(section);
        let cells = rows[row].cells.len();
        self.set_focus(
            section,
            row,
            if cells == 0 { 0 } else { cell.min(cells - 1) },
        );
    }

    fn move_cell(&mut self, delta: isize) {
        let Some(section) = self.section() else {
            return;
        };
        if section == FormSection::Preflight {
            self.move_focus_with(delta, false);
            return;
        }
        let rows = form::rows(self, section);
        let row = form::normalize_row(&rows, self.focus(section));
        let Some(current_row) = rows.get(row) else {
            return;
        };
        let count = current_row.cells.len();
        if count == 0 {
            return;
        }
        let cell = isize::try_from(self.focus_cell(section)).unwrap_or(0);
        let count = isize::try_from(count).unwrap_or(1);
        let next = (cell + delta).clamp(0, count - 1);
        self.set_focus(section, row, usize::try_from(next).unwrap_or(0));
    }

    fn toggle_group(&mut self, section: FormSection, group: form::GroupId) {
        self.collapsed_groups[section.index()] ^= 1 << group.bit();
        self.clamp_focus();
    }

    fn retain_topic_row(&mut self, topic: &str) {
        let rows = form::rows(self, FormSection::Sensors);
        if let Some(row) = form::find_topic_row(&rows, topic) {
            let cell = self.focus_cell(FormSection::Sensors);
            let cells = rows[row].cells.len();
            self.set_focus(
                FormSection::Sensors,
                row,
                if cells == 0 { 0 } else { cell.min(cells - 1) },
            );
        }
    }

    fn switch_section(&mut self, delta: isize) {
        if let Some(section) = self.section() {
            self.screen = Screen::Form(section.next(delta));
            self.clamp_focus();
        }
    }

    fn invalidate_source(&mut self) {
        self.source_generation = self.source_generation.wrapping_add(1);
        self.inspection = None;
        self.seen_image_topics.clear();
        self.draft.cameras.clear();
        self.draft.imu_topic = None;
        self.draft.reference_id.clear();
        self.section_focus[FormSection::Sensors.index()] = TableFocus::default();
        self.section_scroll[FormSection::Sensors.index()] = 0;
    }

    fn start_inspection(&mut self) {
        if self.inspect_worker.is_some() {
            "扫描已在进行中".clone_into(&mut self.state);
            return;
        }
        let source = match self.draft.source_config() {
            Ok(source) => source,
            Err(error) => {
                self.state = format!("无法扫描: {error:#}");
                self.screen = Screen::Form(FormSection::Source);
                return;
            }
        };
        self.inspection = None;
        self.inspect_worker = Some(InspectWorker::start(source, self.source_generation));
        "正在扫描 bag 元数据…".clone_into(&mut self.state);
    }

    fn poll_inspect(&mut self) {
        let result = {
            let Some(worker) = &mut self.inspect_worker else {
                return;
            };
            match worker.receiver.try_recv() {
                Ok(result) => Some((result, worker.join(), worker.source_generation)),
                Err(_) => None,
            }
        };
        let Some((result, join_result, source_generation)) = result else {
            return;
        };
        self.inspect_worker = None;
        if source_generation != self.source_generation {
            "数据源已更改，旧扫描结果已丢弃；请重新扫描".clone_into(&mut self.state);
            return;
        }
        if let Err(error) = join_result {
            self.state = format!("扫描线程错误: {error:#}");
            return;
        }
        match result {
            Ok(inspection) => {
                let current_path = self.draft.source_path.trim().to_owned();
                if inspection.source.as_path() == Path::new(&current_path) {
                    self.seen_image_topics = inspection
                        .topics
                        .iter()
                        .filter(|topic| is_image_topic(&topic.type_name))
                        .map(|topic| topic.name.clone())
                        .collect();
                    self.inspection = Some(inspection);
                    // Data-only refresh: never steal the section or reset focus.
                    "bag 元数据已就绪".clone_into(&mut self.state);
                } else {
                    "扫描完成，但 bag 路径已更改；结果未应用，请重新扫描"
                        .clone_into(&mut self.state);
                }
            }
            Err(error) => {
                self.state = format!("扫描失败: {error}");
            }
        }
    }

    fn poll_task(&mut self) {
        let Some(task) = &mut self.task else {
            return;
        };
        if !task.poll() {
            return;
        }
        let root = task.run_root.clone();
        let state = task.state.clone();
        if self
            .confirmation
            .as_ref()
            .is_some_and(|confirmation| confirmation.action == ConfirmAction::CancelRun)
        {
            self.confirmation = None;
        }
        self.result_data = Some(ResultData::load(&root));
        self.result_tab = ResultTab::Overview;
        self.result_scroll = [0; 5];
        self.run_tab = RunTab::Overview;
        self.run_scroll = [0; 3];
        self.screen = Screen::Run;
        self.state = state;
    }

    fn start_two_phase_confirm(&mut self, action: ConfirmAction, title: &str, message: String) {
        self.confirmation = Some(Confirmation {
            action,
            title: title.to_owned(),
            message,
            yes: false,
        });
    }

    fn request_cancel(&mut self) {
        self.start_two_phase_confirm(
            ConfirmAction::CancelRun,
            "确认取消标定？",
            "取消会请求 worker 停止；不可中断的数值内核必须返回后才能退出。数据与已生成产物会保留。".to_owned(),
        );
    }

    fn request_quit(&mut self) {
        let running = self.task.as_ref().is_some_and(Task::is_running);
        let dirty_expert = self.expert.as_ref().is_some_and(ExpertEditor::is_dirty);
        let mut message = if running {
            "正在标定：确认将请求取消，并在当前 worker 退出后关闭界面。".to_owned()
        } else if self.started_once {
            "确认关闭界面？已完成的运行记录不会被修改。".to_owned()
        } else {
            "确认关闭界面？本次尚未启动标定，运行记录将标记为未执行。".to_owned()
        };
        if dirty_expert {
            message.push_str(" YAML 专家编辑器中的未保存修改会被丢弃。");
        }
        self.start_two_phase_confirm(ConfirmAction::Quit, "确认退出？", message);
    }

    fn confirm(&mut self) {
        let Some(confirmation) = self.confirmation.take() else {
            return;
        };
        if !confirmation.yes {
            return;
        }
        match confirmation.action {
            ConfirmAction::CancelRun => {
                if let Some(task) = &mut self.task {
                    task.request_cancel();
                    "取消中：等待 worker 退出".clone_into(&mut self.state);
                    self.screen = Screen::Run;
                    self.run_tab = RunTab::Overview;
                }
            }
            ConfirmAction::Quit => {
                if let Some(task) = &mut self.task {
                    task.request_cancel();
                }
                self.quit_requested = true;
            }
            ConfirmAction::DiscardExpert => {
                let return_to = self
                    .expert
                    .as_ref()
                    .map_or(FormSection::Preflight, |expert| expert.return_to);
                self.expert = None;
                self.screen = Screen::Form(return_to);
                "已放弃 YAML 编辑".clone_into(&mut self.state);
            }
        }
    }

    fn start_from_preflight(&mut self) {
        let preflight = self.preflight();
        if !preflight.can_start() {
            self.state = format!(
                "无法启动：{} 个阻断项（{} 个警告，{} 项未检查）",
                preflight.blockers(),
                preflight.warnings(),
                preflight.not_checked()
            );
            return;
        }
        let Some(settings) = preflight.settings else {
            "无法启动：配置未通过解析".clone_into(&mut self.state);
            return;
        };
        self.start_task(settings);
    }

    fn start_task(&mut self, settings: RunSettings) {
        if self.task.as_ref().is_some_and(Task::is_running) {
            "已有标定正在运行；请等待或取消".clone_into(&mut self.state);
            return;
        }
        let context = if self.started_once {
            match self.base_run.next_run() {
                Ok(run) => run,
                Err(error) => {
                    self.state = format!("创建下一次运行记录失败: {error:#}；旧运行记录未被修改");
                    return;
                }
            }
        } else {
            self.base_run.clone()
        };
        self.started_once = true;
        self.result_data = None;
        self.task = Some(Task::start(settings, self.inspection.clone(), context));
        self.run_tab = RunTab::Overview;
        self.run_scroll = [0; 3];
        self.screen = Screen::Run;
        "starting".clone_into(&mut self.state);
    }

    fn rerun_same_settings(&mut self) {
        let Some(task) = &self.task else {
            "没有可重跑的运行".clone_into(&mut self.state);
            return;
        };
        if task.is_running() {
            "当前运行尚未结束".clone_into(&mut self.state);
            return;
        }
        let settings = task.settings.clone();
        self.start_task(settings);
    }

    fn modify_after_result(&mut self) {
        self.screen = Screen::Form(FormSection::Preflight);
        "草稿已保留；修改后从预检启动（会创建独立运行记录）".clone_into(&mut self.state);
    }

    fn open_result_from_run(&mut self) {
        let Some(task) = &self.task else {
            "没有可查看的结果".clone_into(&mut self.state);
            return;
        };
        if task.is_running() {
            "运行尚未结束；完成后按 r 查看结果".clone_into(&mut self.state);
            return;
        }
        let root = task.run_root.clone();
        if self.result_data.is_none() {
            self.result_data = Some(ResultData::load(&root));
        }
        self.screen = Screen::Result;
        self.result_tab = ResultTab::Overview;
        self.result_scroll = [0; 5];
        "结果页；Esc 返回运行仪表盘".clone_into(&mut self.state);
    }

    fn open_expert(&mut self) {
        let settings = match self.draft.to_settings() {
            Ok(settings) => settings,
            Err(error) => {
                self.state = format!("先修复表单字段再打开 YAML 编辑器: {error:#}");
                return;
            }
        };
        let text = match serde_yaml::to_string(&settings) {
            Ok(text) => text,
            Err(error) => {
                self.state = format!("无法序列化配置: {error:#}");
                return;
            }
        };
        let return_to = self.section().unwrap_or(FormSection::Preflight);
        self.expert = Some(ExpertEditor {
            buffer: TextBuffer::new(text.clone()),
            saved_text: text,
            return_to,
            error: None,
        });
        self.screen = Screen::Expert;
        "YAML 专家编辑：Ctrl-S 应用，Esc 关闭（有修改时需确认）".clone_into(&mut self.state);
    }

    fn save_expert(&mut self) {
        let Some(expert) = &self.expert else {
            return;
        };
        let text = expert.buffer.text().to_owned();
        let return_to = expert.return_to;
        let parsed: Result<RunSettings> = (|| {
            let settings: RunSettings = serde_yaml::from_str(&text).context("parse YAML")?;
            settings.validate()?;
            Ok(settings)
        })();
        match parsed {
            Ok(settings) => {
                if self.draft.source_path.trim() != settings.source.path.to_string_lossy()
                    || draft::bag_format_index(self.draft.format)
                        != draft::bag_format_index(settings.source.format)
                {
                    self.invalidate_source();
                }
                self.draft = Draft::from_settings(&settings);
                for camera in &self.draft.cameras {
                    if !self.seen_image_topics.contains(&camera.topic) {
                        self.seen_image_topics.push(camera.topic.clone());
                    }
                }
                self.draft.touch();
                if let Some(expert) = &mut self.expert {
                    expert.saved_text = text;
                    expert.error = None;
                }
                self.screen = Screen::Form(return_to);
                "YAML 已应用；表单草稿已同步".clone_into(&mut self.state);
            }
            Err(error) => {
                if let Some(expert) = &mut self.expert {
                    expert.error = Some(format!("{error:#}"));
                }
                "YAML 未应用：解析或校验失败，文本保留".clone_into(&mut self.state);
            }
        }
    }

    fn request_close_expert(&mut self) {
        let Some(expert) = &self.expert else {
            return;
        };
        if expert.is_dirty() {
            self.start_two_phase_confirm(
                ConfirmAction::DiscardExpert,
                "放弃 YAML 修改？",
                "专家编辑器中存在未应用的无效或未保存文本。确认后丢弃，表单草稿保持不变。"
                    .to_owned(),
            );
        } else {
            let return_to = expert.return_to;
            self.expert = None;
            self.screen = Screen::Form(return_to);
            "已关闭 YAML 编辑器".clone_into(&mut self.state);
        }
    }

    fn open_dir(&mut self) {
        let Some(task) = &self.task else {
            "没有运行目录".clone_into(&mut self.state);
            return;
        };
        match open_path(&task.run_root) {
            Ok(()) => self.state = format!("已用系统程序打开 {}", task.run_root.display()),
            Err(error) => self.state = format!("打开运行目录失败: {error:#}"),
        }
    }

    fn open_html(&mut self) {
        let Some(task) = &self.task else {
            "没有运行结果".clone_into(&mut self.state);
            return;
        };
        let path = task.run_root.join("result/extrinsics_3d.html");
        if !path.is_file() {
            self.state = format!("{} 未生成", path.display());
            return;
        }
        match open_path(&path) {
            Ok(()) => self.state = format!("已用系统程序打开 {}", path.display()),
            Err(error) => self.state = format!("打开 HTML 失败: {error:#}"),
        }
    }

    fn toggle_mouse(&mut self, terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> Result<()> {
        self.mouse_capture = !self.mouse_capture;
        if self.mouse_capture {
            execute!(terminal.backend_mut(), EnableMouseCapture).context("enable mouse capture")?;
            "mouse input enabled".clone_into(&mut self.state);
        } else {
            execute!(terminal.backend_mut(), DisableMouseCapture)
                .context("disable mouse capture for text selection")?;
            "copy mode: select text with mouse; Ctrl-Y restores mouse input"
                .clone_into(&mut self.state);
        }
        Ok(())
    }

    fn finalize_exit(&mut self) {
        if !self.started_once {
            let _ = self
                .base_run
                .finish(RunStatus::ClosedWithoutCalibration, None);
            return;
        }
        if let Some(task) = &self.task
            && !task.is_running()
            && task.final_status.is_none()
        {
            let _ = task.run.finish(RunStatus::ClosedWithoutCalibration, None);
        }
    }

    // ------------------------------------------------------------------
    // Field manipulation
    // ------------------------------------------------------------------

    fn current_text(&self, id: &FieldId) -> String {
        match id {
            FieldId::SourcePath => self.draft.source_path.clone(),
            FieldId::SourceMaxDelta => self.draft.max_delta_ns.clone(),
            FieldId::IntrinsicPath => self.draft.intrinsic.path.clone(),
            FieldId::CameraId(index) => self
                .draft
                .cameras
                .get(*index)
                .map_or_else(String::new, |camera| camera.id.clone()),
            FieldId::BoardBorderBits => self.draft.target.border_bits.clone(),
            FieldId::BoardTagSize => self.draft.target.tag_size_m.clone(),
            FieldId::BoardGap => self.draft.target.tag_gap_m.clone(),
            FieldId::BoardRows => self.draft.target.id_rows_text.clone(),
            FieldId::BoardCornerMapping => self.draft.target.corner_mapping_text.clone(),
            FieldId::DetectorMaxHamming => self.draft.detector.max_hamming.clone(),
            FieldId::DetectorMaxIterations => self.draft.detector.max_iterations.clone(),
            FieldId::DetectorMaxPixels => self.draft.detector.max_pixels.clone(),
            FieldId::DetectorMaxClusters => self.draft.detector.max_clusters.clone(),
            FieldId::DetectorMaxCandidates => self.draft.detector.max_candidates.clone(),
            FieldId::ClosureRotation => self.draft.closure_rotation_deg.clone(),
            FieldId::ClosureTranslation => self.draft.closure_translation_m.clone(),
            FieldId::SolverMaxIterations => self.draft.solver.max_iterations.clone(),
            FieldId::SolverPixelSigma => self.draft.solver.pixel_sigma.clone(),
            FieldId::SolverHuberDelta => self.draft.solver.huber_delta.clone(),
            FieldId::SolverPointRejection => self.draft.solver.point_rejection_px.clone(),
            FieldId::TimeOffsetFixed => self.draft.time_offset.fixed_s.clone(),
            FieldId::TimeOffsetInitial => self.draft.time_offset.initial_s.clone(),
            FieldId::TimeOffsetMinimum => self.draft.time_offset.minimum_s.clone(),
            FieldId::TimeOffsetMaximum => self.draft.time_offset.maximum_s.clone(),
            FieldId::TimeOffsetSteps => self.draft.time_offset.initialization_steps.clone(),
            FieldId::ImuGravity => self.draft.imu.gravity_magnitude_m_s2.clone(),
            FieldId::ImuGyroDensity => self.draft.imu.gyro_density_rad_s_sqrt_hz.clone(),
            FieldId::ImuAccelDensity => self.draft.imu.accelerometer_density_m_s2_sqrt_hz.clone(),
            FieldId::ImuGyroBiasWalk => self.draft.imu.gyro_bias_walk_rad_s2_sqrt_hz.clone(),
            FieldId::ImuAccelBiasWalk => {
                self.draft.imu.accelerometer_bias_walk_m_s3_sqrt_hz.clone()
            }
            FieldId::ImuMaxGap => self.draft.imu.max_gap_s.clone(),
            FieldId::ImuMaxSamples => self.draft.imu.max_samples.clone(),
            FieldId::ImuKnotInterval => self.draft.imu.knot_interval_s.clone(),
            FieldId::ImuGyroMatrix => self.draft.imu.gyro_calibrated_from_raw.clone(),
            FieldId::ImuAccelMatrix => self.draft.imu.accelerometer_calibrated_from_raw.clone(),
            _ => String::new(),
        }
    }

    fn open_input(&mut self, id: &FieldId, title: impl Into<String>) {
        self.input = Some(InputModal {
            title: title.into(),
            target: id.clone(),
            buffer: TextBuffer::new(self.current_text(id)),
        });
    }

    fn apply_input(&mut self, target: &FieldId, text: &str) {
        let trimmed = text.trim().to_owned();
        let mut clear_inspection = false;
        match target {
            FieldId::SourcePath => {
                if self.draft.source_path != trimmed {
                    self.draft.source_path.clone_from(&trimmed);
                    clear_inspection = true;
                }
            }
            FieldId::SourceMaxDelta => self.draft.max_delta_ns = trimmed,
            FieldId::IntrinsicPath => self.draft.intrinsic.path = trimmed,
            FieldId::CameraId(index) => self.draft.set_camera_id(*index, &trimmed),
            FieldId::BoardBorderBits => self.draft.target.border_bits = trimmed,
            FieldId::BoardTagSize => self.draft.target.tag_size_m = trimmed,
            FieldId::BoardGap => self.draft.target.tag_gap_m = trimmed,
            FieldId::BoardRows => text.clone_into(&mut self.draft.target.id_rows_text),
            FieldId::BoardCornerMapping => self.draft.target.corner_mapping_text = trimmed,
            FieldId::DetectorMaxHamming => self.draft.detector.max_hamming = trimmed,
            FieldId::DetectorMaxIterations => self.draft.detector.max_iterations = trimmed,
            FieldId::DetectorMaxPixels => self.draft.detector.max_pixels = trimmed,
            FieldId::DetectorMaxClusters => self.draft.detector.max_clusters = trimmed,
            FieldId::DetectorMaxCandidates => self.draft.detector.max_candidates = trimmed,
            FieldId::ClosureRotation => self.draft.closure_rotation_deg = trimmed,
            FieldId::ClosureTranslation => self.draft.closure_translation_m = trimmed,
            FieldId::SolverMaxIterations => self.draft.solver.max_iterations = trimmed,
            FieldId::SolverPixelSigma => self.draft.solver.pixel_sigma = trimmed,
            FieldId::SolverHuberDelta => self.draft.solver.huber_delta = trimmed,
            FieldId::SolverPointRejection => self.draft.solver.point_rejection_px = trimmed,
            FieldId::TimeOffsetFixed => self.draft.time_offset.fixed_s = trimmed,
            FieldId::TimeOffsetInitial => self.draft.time_offset.initial_s = trimmed,
            FieldId::TimeOffsetMinimum => self.draft.time_offset.minimum_s = trimmed,
            FieldId::TimeOffsetMaximum => self.draft.time_offset.maximum_s = trimmed,
            FieldId::TimeOffsetSteps => {
                self.draft.time_offset.initialization_steps = trimmed;
            }
            FieldId::ImuGravity => self.draft.imu.gravity_magnitude_m_s2 = trimmed,
            FieldId::ImuGyroDensity => {
                self.draft.imu.gyro_density_rad_s_sqrt_hz = trimmed;
            }
            FieldId::ImuAccelDensity => {
                self.draft.imu.accelerometer_density_m_s2_sqrt_hz = trimmed;
            }
            FieldId::ImuGyroBiasWalk => {
                self.draft.imu.gyro_bias_walk_rad_s2_sqrt_hz = trimmed;
            }
            FieldId::ImuAccelBiasWalk => {
                self.draft.imu.accelerometer_bias_walk_m_s3_sqrt_hz = trimmed;
            }
            FieldId::ImuMaxGap => self.draft.imu.max_gap_s = trimmed,
            FieldId::ImuMaxSamples => self.draft.imu.max_samples = trimmed,
            FieldId::ImuKnotInterval => self.draft.imu.knot_interval_s = trimmed,
            FieldId::ImuGyroMatrix => {
                text.clone_into(&mut self.draft.imu.gyro_calibrated_from_raw);
            }
            FieldId::ImuAccelMatrix => {
                text.clone_into(&mut self.draft.imu.accelerometer_calibrated_from_raw);
            }
            _ => {}
        }
        if clear_inspection {
            self.invalidate_source();
        }
        self.draft.touch();
        "字段已更新".clone_into(&mut self.state);
    }

    fn commit_input(&mut self) {
        let Some(input) = self.input.take() else {
            return;
        };
        let text = input.buffer.text().to_owned();
        self.apply_input(&input.target, &text);
    }

    fn open_dropdown(&mut self, kind: DropdownKind) {
        let selected = self.dropdown_selected(&kind);
        self.dropdown = Some(Dropdown { kind, selected });
    }

    fn dropdown_options(&self, kind: DropdownKind) -> Vec<String> {
        match kind {
            DropdownKind::Format => BAG_FORMATS
                .iter()
                .map(|format| draft::bag_format_label(*format).to_owned())
                .collect(),
            DropdownKind::Mode => CalibrationMode::ALL
                .iter()
                .map(|mode| format!("{}  [{}]", mode.title(), mode.label()))
                .collect(),
            DropdownKind::Reference => self
                .draft
                .cameras
                .iter()
                .map(|camera| format!("{}  ({})", camera.id, camera.topic))
                .collect(),
            DropdownKind::IntrinsicStrategy => self
                .draft
                .strategy_options()
                .into_iter()
                .map(|strategy| strategy.label().to_owned())
                .collect(),
            DropdownKind::CameraModel(_) => CAMERA_MODELS
                .iter()
                .map(|model| draft::camera_model_label(*model).to_owned())
                .collect(),
            DropdownKind::BoardFamily => TAG_FAMILIES
                .iter()
                .map(|family| draft::tag_family_label(*family).to_owned())
                .collect(),
            DropdownKind::Refinement => REFINEMENTS
                .iter()
                .map(|method| draft::refinement_label(*method).to_owned())
                .collect(),
            DropdownKind::InertialMethod => INERTIAL_METHODS
                .iter()
                .map(|method| draft::inertial_method_label(*method).to_owned())
                .collect(),
            DropdownKind::TimeOffsetKind => vec![
                draft::time_offset_kind_label(TimeOffsetKind::Fixed).to_owned(),
                draft::time_offset_kind_label(TimeOffsetKind::Estimate).to_owned(),
            ],
            DropdownKind::Rejection => draft::REJECTION_POLICIES
                .iter()
                .map(|policy| draft::rejection_policy_label(*policy).to_owned())
                .collect(),
        }
    }

    fn imu_topic_options(&self) -> Vec<Option<String>> {
        let mut options = vec![None];
        if let Some(inspection) = &self.inspection {
            for topic in &inspection.topics {
                if topic.type_name == "sensor_msgs/Imu"
                    && !options
                        .iter()
                        .any(|option| option.as_deref() == Some(topic.name.as_str()))
                {
                    options.push(Some(topic.name.clone()));
                }
            }
        }
        if let Some(current) = &self.draft.imu_topic
            && !options
                .iter()
                .any(|option| option.as_deref() == Some(current.as_str()))
        {
            options.push(Some(current.clone()));
        }
        options
    }

    fn dropdown_selected(&self, kind: &DropdownKind) -> usize {
        match kind {
            DropdownKind::Format => draft::bag_format_index(self.draft.format),
            DropdownKind::Mode => CalibrationMode::ALL
                .iter()
                .position(|mode| *mode == self.draft.mode)
                .unwrap_or(0),
            DropdownKind::Reference => self.draft.reference_index(),
            DropdownKind::IntrinsicStrategy => self
                .draft
                .strategy_options()
                .iter()
                .position(|strategy| *strategy == self.draft.intrinsic.strategy)
                .unwrap_or(0),
            DropdownKind::CameraModel(index) => self
                .draft
                .cameras
                .get(*index)
                .map_or(0, |camera| draft::camera_model_index(camera.model)),
            DropdownKind::BoardFamily => draft::tag_family_index(self.draft.target.family),
            DropdownKind::Refinement => draft::refinement_index(self.draft.detector.refinement),
            DropdownKind::InertialMethod => draft::inertial_method_index(self.draft.method),
            DropdownKind::TimeOffsetKind => {
                usize::from(self.draft.time_offset.kind == TimeOffsetKind::Estimate)
            }
            DropdownKind::Rejection => draft::rejection_policy_index(self.draft.solver.rejection),
        }
    }

    fn apply_dropdown(&mut self, kind: DropdownKind, index: usize) {
        match kind {
            DropdownKind::Format => {
                if let Some(format) = BAG_FORMATS.get(index).copied()
                    && draft::bag_format_index(format) != draft::bag_format_index(self.draft.format)
                {
                    self.draft.format = format;
                    self.invalidate_source();
                    self.draft.touch();
                    "已切换 bag 格式；请重新扫描".clone_into(&mut self.state);
                }
            }
            DropdownKind::Mode => {
                if let Some(mode) = CalibrationMode::ALL.get(index).copied() {
                    self.draft.set_mode(mode);
                    self.state = format!("模式已显式切换为 {}", mode.label());
                }
            }
            DropdownKind::Reference => {
                if let Some(camera) = self.draft.cameras.get(index) {
                    let id = camera.id.clone();
                    self.draft.set_reference_to_id(id);
                    "参考相机已更新".clone_into(&mut self.state);
                }
            }
            DropdownKind::IntrinsicStrategy => {
                let options = self.draft.strategy_options();
                if let Some(strategy) = options.get(index).copied() {
                    self.draft.intrinsic.strategy = strategy;
                    self.draft.touch();
                    self.state = format!("内参策略: {}", strategy.label());
                }
            }
            DropdownKind::CameraModel(camera_index) => {
                if let Some(model) = CAMERA_MODELS.get(index).copied() {
                    self.draft.set_camera_model(camera_index, model);
                    "相机模型已更新".clone_into(&mut self.state);
                }
            }
            DropdownKind::BoardFamily => {
                if let Some(family) = TAG_FAMILIES.get(index).copied() {
                    self.draft.target.family = family;
                    self.draft.touch();
                }
            }
            DropdownKind::Refinement => {
                if let Some(method) = REFINEMENTS.get(index).copied() {
                    self.draft.detector.refinement = method;
                    self.draft.touch();
                }
            }
            DropdownKind::InertialMethod => {
                if let Some(method) = INERTIAL_METHODS.get(index).copied() {
                    self.draft.method = method;
                    // Method changes reset the robust policy to its matching workflow
                    // default; an explicitly loaded configuration is never rewritten.
                    if matches!(method, InertialMethod::ContinuousTime) {
                        self.draft.solver.rejection = RejectionPolicy::HuberOnly;
                    } else if self.draft.solver.rejection == RejectionPolicy::HuberOnly {
                        self.draft.solver.rejection = RejectionPolicy::PixelThreshold;
                    }
                    self.draft.touch();
                }
            }
            DropdownKind::TimeOffsetKind => {
                self.draft.time_offset.kind = if index == 0 {
                    TimeOffsetKind::Fixed
                } else {
                    TimeOffsetKind::Estimate
                };
                self.draft.touch();
            }
            DropdownKind::Rejection => {
                if let Some(policy) = draft::REJECTION_POLICIES.get(index).copied() {
                    self.draft.solver.rejection = policy;
                    if policy == RejectionPolicy::PixelThreshold
                        && self.draft.solver.point_rejection_px.trim().is_empty()
                    {
                        "6".clone_into(&mut self.draft.solver.point_rejection_px);
                    }
                    self.draft.touch();
                }
            }
        }
        self.clamp_focus();
    }

    fn toggle_field(&mut self, id: &FieldId) {
        match id {
            FieldId::SourceRequireAll => {
                self.draft.require_all_cameras = !self.draft.require_all_cameras;
                self.draft.touch();
            }
            FieldId::ImuRefineAxes => {
                self.draft.imu.refine_axes = !self.draft.imu.refine_axes;
                self.draft.touch();
            }
            FieldId::CameraToggle(topic) => {
                let topic = topic.clone();
                if !self.seen_image_topics.contains(&topic) {
                    self.seen_image_topics.push(topic.clone());
                }
                self.draft.toggle_image_topic(&topic);
                self.retain_topic_row(&topic);
                "相机 topic 已切换".clone_into(&mut self.state);
            }
            FieldId::CameraReference(index) => {
                if let Some(camera) = self.draft.cameras.get(*index) {
                    let camera_id = camera.id.clone();
                    self.draft.set_reference_to_id(camera_id);
                    "参考相机已更新".clone_into(&mut self.state);
                }
            }
            FieldId::ImuChoice(option) => {
                let next = if self.draft.imu_topic.as_ref() == option.as_ref() {
                    None
                } else {
                    option.clone()
                };
                self.draft.set_imu_topic(next);
                "IMU topic 已更新（模式保持不变）".clone_into(&mut self.state);
            }
            _ => {}
        }
        self.clamp_focus();
    }

    fn focus_field(&mut self, section: FormSection, target: &FieldId) {
        if section == FormSection::Preflight {
            if let Some(index) = form::fields(self, section)
                .iter()
                .position(|field| &field.id == target)
            {
                self.set_focus(section, index, 0);
            }
            return;
        }
        let rows = form::rows(self, section);
        if let Some((row, cell)) = rows.iter().enumerate().find_map(|(row, data)| {
            data.cells
                .iter()
                .position(|cell| cell.id.as_ref() == Some(target))
                .map(|cell| (row, cell))
        }) {
            self.set_focus(section, row, cell);
        }
    }

    fn open_dropdown_for(&mut self, id: &FieldId) {
        let kind = match id {
            FieldId::SourceFormat => Some(DropdownKind::Format),
            FieldId::Mode => Some(DropdownKind::Mode),
            FieldId::Reference => Some(DropdownKind::Reference),
            FieldId::IntrinsicStrategy => Some(DropdownKind::IntrinsicStrategy),
            FieldId::CameraModel(index) => Some(DropdownKind::CameraModel(*index)),
            FieldId::BoardFamily => Some(DropdownKind::BoardFamily),
            FieldId::DetectorRefinement => Some(DropdownKind::Refinement),
            FieldId::InertialMethod => Some(DropdownKind::InertialMethod),
            FieldId::TimeOffsetKind => Some(DropdownKind::TimeOffsetKind),
            FieldId::SolverRejection => Some(DropdownKind::Rejection),
            _ => None,
        };
        if let Some(kind) = kind {
            if self.dropdown_options(kind).is_empty() {
                "没有可选项".clone_into(&mut self.state);
            } else {
                self.open_dropdown(kind);
            }
        }
    }

    fn activate_button(&mut self, id: &FieldId) {
        match id {
            FieldId::SourceScan | FieldId::ScanTopics => {
                if self.draft.source_path.trim().is_empty() {
                    self.screen = Screen::Form(FormSection::Source);
                    self.focus_field(FormSection::Source, &FieldId::SourcePath);
                    "请先输入 bag 路径".clone_into(&mut self.state);
                } else {
                    self.start_inspection();
                }
            }
            FieldId::GroupToggle(group) => {
                let section = self.section().unwrap_or(FormSection::Source);
                self.toggle_group(section, *group);
            }
            FieldId::PreflightStart => self.start_from_preflight(),
            FieldId::PreflightExpert => self.open_expert(),
            _ => {}
        }
    }

    fn activate_focused(&mut self, edit: bool) {
        let Some(section) = self.section() else {
            return;
        };
        let (id, kind, title) = if section == FormSection::Preflight {
            let fields = form::fields(self, section);
            if fields.is_empty() {
                return;
            }
            let index = self.focus(section).min(fields.len() - 1);
            (
                fields[index].id.clone(),
                fields[index].kind,
                fields[index].label.clone(),
            )
        } else {
            let rows = form::rows(self, section);
            let row = form::normalize_row(&rows, self.focus(section));
            let Some(cell) = form::focused_cell(&rows, row, self.focus_cell(section)) else {
                return;
            };
            let Some(id) = cell.id else {
                "当前单元格只读；方向键移动焦点，Enter 编辑值列".clone_into(&mut self.state);
                return;
            };
            let cell_index = self.focus_cell(section);
            let title = rows.get(row).map_or_else(
                || "编辑".to_owned(),
                |data| {
                    if data.table == form::TableId::Params {
                        data.cells
                            .first()
                            .map_or_else(|| "编辑".to_owned(), |label| label.text.clone())
                    } else {
                        form::columns(data.table)
                            .get(cell_index)
                            .map_or_else(|| "编辑".to_owned(), |column| column.title.to_owned())
                    }
                },
            );
            (id, cell.kind, title)
        };
        match kind {
            FieldKind::Text if edit => self.open_input(&id, title),
            FieldKind::Text => "Enter 编辑文本".clone_into(&mut self.state),
            FieldKind::Choice if edit => self.open_dropdown_for(&id),
            FieldKind::Choice => "Enter 打开下拉".clone_into(&mut self.state),
            FieldKind::Toggle => self.toggle_field(&id),
            FieldKind::Button => self.activate_button(&id),
            FieldKind::ReadOnly => "只读字段".clone_into(&mut self.state),
        }
    }

    fn scroll_run(&mut self, delta: isize) {
        self.run_scroll[self.run_tab.index()] =
            adjust_scroll(self.run_scroll[self.run_tab.index()], delta);
    }

    fn scroll_result(&mut self, delta: isize) {
        self.result_scroll[self.result_tab.index()] =
            adjust_scroll(self.result_scroll[self.result_tab.index()], delta);
    }

    // ------------------------------------------------------------------
    // Input dispatch
    // ------------------------------------------------------------------

    fn handle_key(&mut self, code: KeyCode, modifiers: KeyModifiers) {
        let control = modifiers.contains(KeyModifiers::CONTROL);
        if self.help {
            match code {
                KeyCode::F(1) | KeyCode::Esc => self.help = false,
                KeyCode::Up => self.help_scroll = self.help_scroll.saturating_sub(1),
                KeyCode::Down => self.help_scroll = self.help_scroll.saturating_add(1),
                KeyCode::PageUp => self.help_scroll = self.help_scroll.saturating_sub(8),
                KeyCode::PageDown => self.help_scroll = self.help_scroll.saturating_add(8),
                KeyCode::Home => self.help_scroll = 0,
                _ => {}
            }
            return;
        }
        if code == KeyCode::F(1) {
            self.help = true;
            self.help_scroll = 0;
            return;
        }
        if self.confirmation.is_some() {
            self.handle_confirmation_key(code, control);
            return;
        }

        if control && matches!(code, KeyCode::Char('q')) {
            self.request_quit();
            return;
        }
        if control && matches!(code, KeyCode::Char('c')) {
            if self.task.as_ref().is_some_and(Task::is_running) {
                self.request_cancel();
            } else {
                self.request_quit();
            }
            return;
        }
        if control && matches!(code, KeyCode::Enter | KeyCode::Tab | KeyCode::BackTab) {
            // Ctrl-M / Ctrl-I are not shortcuts; neither reaches text buffers.
            return;
        }
        if self.input.is_some() {
            self.handle_input_key(code, control);
            return;
        }
        if self.dropdown.is_some() {
            self.handle_dropdown_key(code, control);
            return;
        }

        match code {
            KeyCode::F(6) if self.screen == Screen::Result => {
                self.open_dir();
                return;
            }
            KeyCode::F(7) if self.screen == Screen::Result => {
                self.open_html();
                return;
            }
            KeyCode::F(9) if self.screen == Screen::Result => {
                self.modify_after_result();
                return;
            }
            KeyCode::F(10) if self.screen == Screen::Result => {
                self.rerun_same_settings();
                return;
            }
            KeyCode::F(8) => {
                if self.task.as_ref().is_some_and(Task::is_running) {
                    self.request_cancel();
                }
                return;
            }
            _ => {}
        }

        // Ctrl-arrow can be consumed by desktop shortcuts before reaching the terminal.
        // Map ASCII controls only after modal dispatch, never inside text editors.
        let code = if self.screen == Screen::Expert {
            code
        } else {
            match (code, control) {
                (KeyCode::Char('p' | 'P'), true) => KeyCode::Left,
                (KeyCode::Char('n' | 'N'), true) => KeyCode::Right,
                _ => code,
            }
        };
        match self.screen {
            Screen::Form(section) => self.handle_form_key(section, code, control),
            Screen::Expert => self.handle_expert_key(code, control),
            Screen::Run => self.handle_run_key(code, control),
            Screen::Result => self.handle_result_key(code, control),
        }
    }

    fn handle_confirmation_key(&mut self, code: KeyCode, control: bool) {
        if control && matches!(code, KeyCode::Enter | KeyCode::Tab | KeyCode::BackTab) {
            return;
        }
        match code {
            KeyCode::Esc | KeyCode::Char('n' | 'N') => {
                self.confirmation = None;
            }
            KeyCode::Char('y' | 'Y') => {
                if let Some(confirmation) = &mut self.confirmation {
                    confirmation.yes = true;
                }
                self.confirm();
            }
            KeyCode::Left
            | KeyCode::Right
            | KeyCode::Tab
            | KeyCode::BackTab
            | KeyCode::Char(' ') => {
                if let Some(confirmation) = &mut self.confirmation {
                    confirmation.yes = !confirmation.yes;
                }
            }
            KeyCode::Enter => self.confirm(),
            _ => {}
        }
    }

    fn handle_input_key(&mut self, code: KeyCode, control: bool) {
        match code {
            KeyCode::Esc => self.input = None,
            KeyCode::Enter if !control => self.commit_input(),
            _ => {
                let Some(input) = &mut self.input else {
                    return;
                };
                match code {
                    KeyCode::Left => input.buffer.move_left(),
                    KeyCode::Right => input.buffer.move_right(),
                    KeyCode::Up => input.buffer.move_vertical(-1),
                    KeyCode::Down => input.buffer.move_vertical(1),
                    KeyCode::Home => input.buffer.move_home(),
                    KeyCode::End => input.buffer.move_end(),
                    KeyCode::Backspace => input.buffer.backspace(),
                    KeyCode::Delete => input.buffer.delete(),
                    KeyCode::Tab | KeyCode::BackTab => input.buffer.insert_text("  "),
                    KeyCode::Char(value) if !control => input.buffer.insert_char(value),
                    _ => {}
                }
            }
        }
    }

    fn handle_dropdown_key(&mut self, code: KeyCode, control: bool) {
        let Some(mut dropdown) = self.dropdown.take() else {
            return;
        };
        let count = self.dropdown_options(dropdown.kind).len();
        match code {
            KeyCode::Esc => {
                // closed
            }
            KeyCode::Up => {
                if count > 0 {
                    dropdown.selected = (dropdown.selected + count - 1) % count;
                }
                self.dropdown = Some(dropdown);
            }
            KeyCode::Down => {
                if count > 0 {
                    dropdown.selected = (dropdown.selected + 1) % count;
                }
                self.dropdown = Some(dropdown);
            }
            KeyCode::Home => {
                dropdown.selected = 0;
                self.dropdown = Some(dropdown);
            }
            KeyCode::End => {
                dropdown.selected = count.saturating_sub(1);
                self.dropdown = Some(dropdown);
            }
            KeyCode::Enter | KeyCode::Char(' ') if !control => {
                self.apply_dropdown(dropdown.kind, dropdown.selected);
            }
            _ => {
                self.dropdown = Some(dropdown);
            }
        }
    }

    fn handle_expert_key(&mut self, code: KeyCode, control: bool) {
        if control && matches!(code, KeyCode::Char('s')) {
            self.save_expert();
            return;
        }
        if code == KeyCode::Esc {
            self.request_close_expert();
            return;
        }
        let Some(expert) = &mut self.expert else {
            return;
        };
        match code {
            KeyCode::Left => expert.buffer.move_left(),
            KeyCode::Right => expert.buffer.move_right(),
            KeyCode::Up => expert.buffer.move_vertical(-1),
            KeyCode::Down => expert.buffer.move_vertical(1),
            KeyCode::PageUp => expert.buffer.move_vertical(-12),
            KeyCode::PageDown => expert.buffer.move_vertical(12),
            KeyCode::Home => expert.buffer.move_home(),
            KeyCode::End => expert.buffer.move_end(),
            KeyCode::Backspace => expert.buffer.backspace(),
            KeyCode::Delete => expert.buffer.delete(),
            KeyCode::Enter => expert.buffer.insert_char('\n'),
            KeyCode::Tab | KeyCode::BackTab => expert.buffer.insert_text("  "),
            KeyCode::Char(value) if !control => expert.buffer.insert_char(value),
            _ => {}
        }
    }

    fn handle_form_key(&mut self, section: FormSection, code: KeyCode, control: bool) {
        if control && matches!(code, KeyCode::Enter | KeyCode::Tab | KeyCode::BackTab) {
            // Ctrl-M / Ctrl-I are deliberately not shortcuts.
            return;
        }
        if control && matches!(code, KeyCode::Left) {
            self.switch_section(-1);
            return;
        }
        if control && matches!(code, KeyCode::Right) {
            self.switch_section(1);
            return;
        }
        if section == FormSection::Preflight {
            match code {
                KeyCode::Tab | KeyCode::Right => self.move_focus_wrapped(1),
                KeyCode::BackTab | KeyCode::Left => self.move_focus_wrapped(-1),
                KeyCode::Up => self.preflight_scroll = self.preflight_scroll.saturating_sub(1),
                KeyCode::Down => {
                    self.preflight_scroll = self.preflight_scroll.saturating_add(1);
                }
                KeyCode::PageUp => self.preflight_scroll = self.preflight_scroll.saturating_sub(8),
                KeyCode::PageDown => {
                    self.preflight_scroll = self.preflight_scroll.saturating_add(8);
                }
                KeyCode::Home => self.preflight_scroll = 0,
                KeyCode::End => self.preflight_scroll = usize::MAX,

                KeyCode::Enter if !control => self.activate_focused(true),
                KeyCode::Char(' ') if !control => self.activate_focused(false),
                KeyCode::Esc => {
                    "Esc 不退出、不取消；Ctrl-P/N 切换分区，Ctrl-Q 退出"
                        .clone_into(&mut self.state);
                }
                _ => {}
            }
            return;
        }
        match code {
            KeyCode::Tab => self.move_focus_wrapped(1),
            KeyCode::BackTab => self.move_focus_wrapped(-1),
            KeyCode::Up => self.move_focus(-1),
            KeyCode::Down => self.move_focus(1),
            KeyCode::PageUp => self.move_focus(-8),
            KeyCode::PageDown => self.move_focus(8),
            KeyCode::Home => self.set_focus(section, 0, 0),
            KeyCode::End => {
                let rows = form::rows(self, section);
                let row = form::last_selectable_row(&rows);
                self.set_focus(section, row, 0);
            }
            KeyCode::Left if !control => self.move_cell(-1),
            KeyCode::Right if !control => self.move_cell(1),
            KeyCode::Enter if !control => self.activate_focused(true),
            KeyCode::Char(' ') if !control => self.activate_focused(false),
            KeyCode::Esc => {
                "Esc 不退出、不取消；Ctrl-P/N 切换分区，Ctrl-Q 退出".clone_into(&mut self.state);
            }
            _ => {}
        }
    }

    fn handle_run_key(&mut self, code: KeyCode, control: bool) {
        if control && matches!(code, KeyCode::Enter | KeyCode::Tab | KeyCode::BackTab) {
            return;
        }
        match code {
            KeyCode::Tab | KeyCode::Right => {
                self.run_tab = RunTab::ALL[(self.run_tab.index() + 1) % 3];
            }
            KeyCode::BackTab | KeyCode::Left => {
                self.run_tab = RunTab::ALL[(self.run_tab.index() + 2) % 3];
            }
            KeyCode::Up => self.scroll_run(-1),
            KeyCode::Down => self.scroll_run(1),
            KeyCode::PageUp => self.scroll_run(-8),
            KeyCode::PageDown => self.scroll_run(8),
            KeyCode::Home => self.run_scroll[self.run_tab.index()] = 0,
            KeyCode::End => self.run_scroll[self.run_tab.index()] = usize::MAX,
            KeyCode::Char('g') if !control => {
                let label = self
                    .task
                    .as_mut()
                    .map(|task| task.cycle_dashboard_graph().label());
                if let Some(label) = label {
                    self.state = format!("图表: {label}");
                }
            }
            KeyCode::Char('r') if !control => self.open_result_from_run(),
            KeyCode::Esc => {
                "Esc 不取消、不退出；F8 或 Ctrl-C 打开取消确认".clone_into(&mut self.state);
            }
            _ => {}
        }
    }

    fn handle_result_key(&mut self, code: KeyCode, control: bool) {
        if control && matches!(code, KeyCode::Enter | KeyCode::Tab | KeyCode::BackTab) {
            return;
        }
        match code {
            KeyCode::Tab | KeyCode::Right => self.result_tab = self.result_tab.next(1),
            KeyCode::BackTab | KeyCode::Left => self.result_tab = self.result_tab.next(-1),
            KeyCode::Up => self.scroll_result(-1),
            KeyCode::Down => self.scroll_result(1),
            KeyCode::PageUp => self.scroll_result(-8),
            KeyCode::PageDown => self.scroll_result(8),
            KeyCode::Home => self.result_scroll[self.result_tab.index()] = 0,
            KeyCode::End => self.result_scroll[self.result_tab.index()] = usize::MAX,
            KeyCode::Enter | KeyCode::Char(' ') if self.result_tab == ResultTab::Raw => {
                self.raw_source = self.raw_source.next(1);
                self.result_scroll[ResultTab::Raw.index()] = 0;
            }
            KeyCode::Esc => {
                if self.task.is_some() {
                    self.screen = Screen::Run;
                    self.run_tab = RunTab::Overview;
                    "已返回运行仪表盘；g 切换图表，r 查看结果".clone_into(&mut self.state);
                } else {
                    "Esc 不退出；Tab 切换页，F6 打开目录，Ctrl-Q 退出".clone_into(&mut self.state);
                }
            }
            _ => {}
        }
    }

    fn handle_paste(&mut self, text: &str) {
        if self.help || self.confirmation.is_some() {
            return;
        }
        let text = text.replace('\r', "");
        if let Some(input) = &mut self.input {
            input.buffer.insert_text(&text);
        } else if self.screen == Screen::Expert
            && let Some(expert) = &mut self.expert
        {
            expert.buffer.insert_text(&text);
        }
    }

    fn handle_mouse(&mut self, mouse: MouseEvent) {
        match mouse.kind {
            MouseEventKind::ScrollUp => {
                self.handle_scroll(-3);
                return;
            }
            MouseEventKind::ScrollDown => {
                self.handle_scroll(3);
                return;
            }
            MouseEventKind::Down(MouseButton::Left) => {}
            _ => return,
        }
        let action = self
            .hit_targets
            .iter()
            .rev()
            .find(|target| target.contains(mouse.column, mouse.row))
            .map(|target| target.action.clone());
        if self.help {
            self.help = false;
            return;
        }
        if self.confirmation.is_some() {
            if let Some(action) = action {
                self.handle_modal_mouse(&action);
            }
            return;
        }
        if self.input.is_some() {
            if let Some(action) = action {
                match action {
                    MouseAction::InputApply => self.commit_input(),
                    MouseAction::InputCancel => self.input = None,
                    _ => {}
                }
            }
            return;
        }
        if self.dropdown.is_some() {
            match action {
                Some(MouseAction::DropdownOption(index)) => {
                    if let Some(dropdown) = self.dropdown.take() {
                        self.apply_dropdown(dropdown.kind, index);
                    }
                }
                _ => self.dropdown = None,
            }
            return;
        }
        let Some(action) = action else {
            self.dropdown = None;
            return;
        };
        self.handle_mouse_action(&action);
    }

    fn handle_mouse_action(&mut self, action: &MouseAction) {
        if matches!(
            action,
            MouseAction::Section(_)
                | MouseAction::PrevSection
                | MouseAction::NextSection
                | MouseAction::Field(_)
                | MouseAction::Cell(_, _)
        ) {
            self.handle_form_mouse_action(action);
            return;
        }
        match action {
            MouseAction::DashboardGraph(index) => {
                if let Some(dashboard) = &mut self.task {
                    let graph = if *index == 0 {
                        task::DashboardGraph::Detection
                    } else {
                        task::DashboardGraph::Convergence
                    };
                    dashboard.select_dashboard_graph(graph);
                    self.state = format!("图表: {}", graph.label());
                }
            }
            MouseAction::RunOpenResult => self.open_result_from_run(),
            MouseAction::ResultDashboard => {
                if self.task.is_some() {
                    self.screen = Screen::Run;
                    self.run_tab = RunTab::Overview;
                    "已返回运行仪表盘".clone_into(&mut self.state);
                }
            }
            MouseAction::RunTab(index) => {
                if let Some(tab) = RunTab::ALL.get(*index).copied() {
                    self.run_tab = tab;
                }
            }
            MouseAction::ResultTab(index) => {
                if let Some(tab) = ResultTab::ALL.get(*index).copied() {
                    self.result_tab = tab;
                }
            }
            MouseAction::RunCancel => {
                if self.task.as_ref().is_some_and(Task::is_running) {
                    self.request_cancel();
                }
            }
            MouseAction::ResultOpenDir => self.open_dir(),
            MouseAction::ResultOpenHtml => self.open_html(),
            MouseAction::ResultModify => self.modify_after_result(),
            MouseAction::ResultRerun => self.rerun_same_settings(),
            MouseAction::ResultRawCycle => {
                self.raw_source = self.raw_source.next(1);
                self.result_scroll[ResultTab::Raw.index()] = 0;
            }
            MouseAction::Quit => self.request_quit(),
            MouseAction::ExpertApply => self.save_expert(),
            MouseAction::ExpertClose => self.request_close_expert(),
            MouseAction::Section(_)
            | MouseAction::PrevSection
            | MouseAction::NextSection
            | MouseAction::Field(_)
            | MouseAction::Cell(_, _)
            | MouseAction::DropdownOption(_)
            | MouseAction::InputApply
            | MouseAction::InputCancel
            | MouseAction::ConfirmYes
            | MouseAction::ConfirmNo => {}
        }
    }

    fn handle_form_mouse_action(&mut self, action: &MouseAction) {
        match action {
            MouseAction::Section(section) => {
                self.screen = Screen::Form(*section);
                self.clamp_focus();
            }
            MouseAction::PrevSection => self.switch_section(-1),
            MouseAction::NextSection => self.switch_section(1),
            MouseAction::Field(index) => {
                let Some(section) = self.section() else {
                    return;
                };
                let count = self.field_count(section);
                if *index < count {
                    self.set_focus(section, *index, 0);
                    self.activate_focused(true);
                }
            }
            MouseAction::Cell(row, cell) => {
                let Some(section) = self.section() else {
                    return;
                };
                self.set_focus(section, *row, *cell);
                self.clamp_focus();
                let has_action = if section == FormSection::Preflight {
                    true
                } else {
                    let rows = form::rows(self, section);
                    form::focused_cell(&rows, self.focus(section), self.focus_cell(section))
                        .is_some_and(|cell| cell.id.is_some())
                };
                if has_action {
                    self.activate_focused(true);
                }
            }
            _ => {}
        }
    }

    fn handle_modal_mouse(&mut self, action: &MouseAction) {
        match action {
            MouseAction::ConfirmYes => {
                if let Some(confirmation) = &mut self.confirmation {
                    confirmation.yes = true;
                }
                self.confirm();
            }
            MouseAction::ConfirmNo => {
                self.confirmation = None;
            }
            _ => {}
        }
    }

    fn handle_scroll(&mut self, delta: isize) {
        if self.help {
            self.help_scroll = adjust_scroll(self.help_scroll, delta);
            return;
        }
        if self.confirmation.is_some() {
            return;
        }
        if self.input.is_some() {
            return;
        }
        if let Some(kind) = self.dropdown.as_ref().map(|dropdown| dropdown.kind) {
            let count = self.dropdown_options(kind).len();
            if let Some(dropdown) = &mut self.dropdown
                && count > 0
            {
                let selected = isize::try_from(dropdown.selected).unwrap_or(isize::MAX);
                let count = isize::try_from(count).unwrap_or(isize::MAX);
                let next = (selected + delta).clamp(0, count - 1);
                dropdown.selected = usize::try_from(next).unwrap_or(0);
            }
            return;
        }
        match self.screen {
            Screen::Form(FormSection::Preflight) => {
                self.preflight_scroll = adjust_scroll(self.preflight_scroll, delta);
            }
            Screen::Form(_) => {
                self.move_focus(delta.signum());
            }
            Screen::Expert => {
                if let Some(expert) = &mut self.expert {
                    expert.buffer.move_vertical(delta.signum() * 3);
                }
            }
            Screen::Run => self.scroll_run(delta),
            Screen::Result => self.scroll_result(delta),
        }
    }
}

fn is_image_topic(type_name: &str) -> bool {
    matches!(
        type_name,
        "sensor_msgs/Image" | "sensor_msgs/CompressedImage"
    )
}

fn adjust_scroll(value: usize, delta: isize) -> usize {
    if delta < 0 {
        value.saturating_sub(delta.unsigned_abs())
    } else {
        value.saturating_add(delta.unsigned_abs())
    }
}

fn open_path(path: &Path) -> Result<()> {
    #[cfg(target_os = "macos")]
    let program = "open";
    #[cfg(target_os = "windows")]
    let program = "explorer";
    #[cfg(all(unix, not(target_os = "macos")))]
    let program = "xdg-open";
    std::process::Command::new(program)
        .arg(path)
        .spawn()
        .with_context(|| format!("spawn {program} for {}", path.display()))?;
    Ok(())
}

#[allow(
    clippy::needless_pass_by_value,
    reason = "public TUI entry point keeps owned settings for CLI compatibility"
)]
pub fn run(initial: Option<RunSettings>, run: &RunContext) -> Result<()> {
    let mut app = App::new(initial.as_ref(), run.clone());
    if !app.draft.source_path.trim().is_empty() {
        app.start_inspection();
    }
    let mut terminal = open_terminal()?;
    let loop_result = run_loop(&mut terminal, &mut app);
    let close_result = close_terminal(&mut terminal);
    loop_result?;
    close_result?;
    Ok(())
}

fn run_loop(terminal: &mut Terminal<CrosstermBackend<Stdout>>, app: &mut App) -> Result<()> {
    loop {
        app.poll_inspect();
        app.poll_task();
        views::draw(terminal, app)?;
        if app.quit_requested && !app.task.as_ref().is_some_and(Task::is_running) {
            app.finalize_exit();
            return Ok(());
        }
        let busy = app.inspect_worker.is_some() || app.task.as_ref().is_some_and(Task::is_running);
        let poll_timeout = if busy {
            Duration::from_millis(50)
        } else {
            Duration::from_secs(3_600)
        };
        if !event::poll(poll_timeout).context("poll terminal event")? {
            continue;
        }
        match event::read().context("read terminal event")? {
            TerminalEvent::Paste(text) => app.handle_paste(&text),
            TerminalEvent::Mouse(mouse) => app.handle_mouse(mouse),
            TerminalEvent::Key(key) if key.kind == KeyEventKind::Press => {
                if key.modifiers.contains(KeyModifiers::CONTROL) && key.code == KeyCode::Char('y') {
                    app.toggle_mouse(terminal)?;
                    continue;
                }
                app.handle_key(key.code, key.modifiers);
            }
            _ => {}
        }
    }
}

fn open_terminal() -> Result<Terminal<CrosstermBackend<Stdout>>> {
    enable_raw_mode().context("enable terminal raw mode")?;
    let mut stdout = io::stdout();
    execute!(
        stdout,
        EnterAlternateScreen,
        EnableBracketedPaste,
        EnableMouseCapture
    )
    .context("enter alternate screen")?;
    Terminal::new(CrosstermBackend::new(stdout)).context("create terminal")
}

fn close_terminal(terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> Result<()> {
    disable_raw_mode().context("disable terminal raw mode")?;
    execute!(
        terminal.backend_mut(),
        DisableMouseCapture,
        DisableBracketedPaste,
        LeaveAlternateScreen
    )
    .context("leave alternate screen")?;
    terminal.show_cursor().context("show cursor")
}
