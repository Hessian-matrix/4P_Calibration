use std::io::Stdout;
use std::time::Instant;

use anyhow::{Context, Result};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Axis, Block, Borders, Chart, Clear, Dataset, GraphType, LegendPosition, Paragraph, Wrap,
};

use crate::run_record::RunStatus;

use super::form::{self, Field, FieldKind, FormSection, Row, TableId};
use super::preflight::{CheckLevel, Preflight};
use super::result::ResultData;
use super::task::{self, Task};
use super::{App, ConfirmAction, DropdownKind, HitTarget, MouseAction, ResultTab, RunTab, Screen};

pub(super) fn draw(terminal: &mut Terminal<CrosstermBackend<Stdout>>, app: &mut App) -> Result<()> {
    terminal
        .draw(|frame| draw_frame(frame, app))
        .context("draw TUI")?;
    Ok(())
}

fn draw_frame(frame: &mut ratatui::Frame<'_>, app: &mut App) {
    app.hit_targets.clear();
    let area = frame.area();
    let compact = area.height < 20;
    let header_height = if compact { 1 } else { 3 };
    let footer_height = if compact { 1 } else { 3 };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(header_height),
            Constraint::Min(1),
            Constraint::Length(footer_height),
        ])
        .split(area);
    render_header(frame, app, chunks[0], !compact);
    match app.screen {
        Screen::Form(section) => render_form(frame, app, section, chunks[1]),
        Screen::Expert => render_expert(frame, app, chunks[1]),
        Screen::Run => render_run(frame, app, chunks[1]),
        Screen::Result => render_result(frame, app, chunks[1]),
    }
    render_footer(frame, app, chunks[2], !compact);
    if app.dropdown.is_some() {
        render_dropdown(frame, app, chunks[1]);
    }
    if app.input.is_some() {
        render_input(frame, app, chunks[1]);
    }
    if app.confirmation.is_some() {
        render_confirmation(frame, app, chunks[1]);
    }
    if app.help {
        render_help(frame, app, chunks[1]);
    }
}

fn render_header(frame: &mut ratatui::Frame<'_>, app: &App, area: Rect, bordered: bool) {
    let title = match app.screen {
        Screen::Form(section) => format!("数据配置 · {}", section.title()),
        Screen::Expert => "YAML 专家编辑".to_owned(),
        Screen::Run => format!("标定运行 · {}", run_status_label(app)),
        Screen::Result => format!("标定结果 · {}", run_status_label(app)),
    };
    let line = Line::from(vec![
        Span::styled(
            "4P Offline Calibration  ",
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(title, Style::default().add_modifier(Modifier::BOLD)),
        Span::styled(
            format!("  rev{}", app.draft.revision),
            Style::default().fg(Color::DarkGray),
        ),
    ]);
    if bordered {
        frame.render_widget(
            Paragraph::new(line).block(Block::default().borders(Borders::ALL)),
            area,
        );
    } else {
        frame.render_widget(Paragraph::new(line), area);
    }
}

fn run_status_label(app: &App) -> &'static str {
    match app.task.as_ref().map(|task| task.final_status) {
        Some(Some(RunStatus::Complete)) => "成功",
        Some(Some(RunStatus::Failed)) => "失败",
        Some(Some(RunStatus::Cancelled)) => "已取消",
        Some(Some(RunStatus::ClosedWithoutCalibration)) => "未执行",
        Some(None | Some(RunStatus::InProgress)) => "运行中",
        None => "未开始",
    }
}

fn render_footer(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect, bordered: bool) {
    let hint = screen_hint(app);
    let copy = if app.mouse_capture {
        "  Ctrl-Y 复制模式"
    } else {
        "  复制模式（Ctrl-Y 恢复鼠标）"
    };
    let line = Line::from(vec![
        Span::styled(
            format!("状态: {}  ", app.state),
            Style::default().fg(Color::Yellow),
        ),
        Span::raw(hint),
        Span::styled(
            copy,
            Style::default().fg(if app.mouse_capture {
                Color::DarkGray
            } else {
                Color::Green
            }),
        ),
    ]);
    if bordered {
        frame.render_widget(
            Paragraph::new(line).block(Block::default().borders(Borders::ALL)),
            area,
        );
    } else {
        frame.render_widget(Paragraph::new(line), area);
    }
    if area.width >= 32 {
        let label = "[ 退出 Ctrl-Q ]";
        let width = u16::try_from(display_width(label)).unwrap_or(u16::MAX);
        let inner_right = if bordered {
            area.right().saturating_sub(1)
        } else {
            area.right()
        };
        let x = inner_right.saturating_sub(width);
        let y = if bordered {
            area.y.saturating_add(1)
        } else {
            area.y
        };
        frame.render_widget(
            Paragraph::new(Line::from(Span::styled(
                label,
                Style::default().fg(Color::Red),
            ))),
            Rect::new(x, y, width.min(inner_right.saturating_sub(x)), 1),
        );
        app.hit_targets.push(HitTarget {
            area: Rect::new(x, y, width.min(inner_right.saturating_sub(x)), 1),
            action: MouseAction::Quit,
        });
    }
}

fn screen_hint(app: &App) -> &'static str {
    match app.screen {
        Screen::Form(FormSection::Source) => {
            "↑/↓ 行  ←/→ 单元格  Tab 下一项  Ctrl-P/N 分区  Enter 编辑/下拉  F1 帮助"
        }
        Screen::Form(FormSection::Sensors) => {
            "↑/↓ 行  ←/→ 单元格  Space 勾选/选择  Enter 编辑/下拉  Ctrl-P/N 分区  F1 帮助"
        }
        Screen::Form(FormSection::Board) => {
            "↑/↓ 行  ←/→ 单元格  Ctrl-P/N 分区  Enter 编辑（ID 行完整保留）  F1 帮助"
        }
        Screen::Form(FormSection::Solver) => {
            "↑/↓ 行  ←/→ 单元格  Ctrl-P/N 分区  Enter 打开下拉/编辑  Space 切换  F1 帮助"
        }
        Screen::Form(FormSection::Preflight) => {
            "Tab/BackTab 操作  ←/→ 操作  ↑/↓/PgUp/PgDn 滚动检查  Ctrl-P/N 分区  Enter 启动"
        }
        Screen::Expert => "Ctrl-S 应用  Esc 关闭（有修改需确认）  Tab 缩进  F1 帮助",
        Screen::Run => {
            "Tab 切换页  ↑/↓ 滚动  g 切换图表  r 查看结果（完成后）  F8/Ctrl-C 取消  F1 帮助"
        }
        Screen::Result => {
            "Tab 切换页  ↑/↓ 滚动  Esc 返回仪表盘  F6 目录  F7 HTML  F9 修改  F10 重跑  F1 帮助"
        }
    }
}

fn render_tabs(
    frame: &mut ratatui::Frame<'_>,
    app: &mut App,
    area: Rect,
    labels: &[&str],
    active: usize,
    actions: &[MouseAction],
    suffix: Option<String>,
) {
    let mut spans = Vec::new();
    let mut x = area.x;
    for (index, label) in labels.iter().enumerate() {
        let text = format!(" {label} ");
        let width = u16::try_from(display_width(&text)).unwrap_or(u16::MAX);
        if x >= area.right() {
            break;
        }
        let style = if index == active {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default().fg(Color::Gray)
        };
        spans.push(Span::styled(text, style));
        if let Some(action) = actions.get(index) {
            app.hit_targets.push(HitTarget {
                area: Rect::new(x, area.y, width.min(area.right().saturating_sub(x)), 1),
                action: action.clone(),
            });
        }
        x = x.saturating_add(width);
    }
    if let Some(suffix) = suffix {
        spans.push(Span::styled(suffix, Style::default().fg(Color::DarkGray)));
    }
    frame.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn section_tab_actions() -> Vec<MouseAction> {
    FormSection::ALL
        .iter()
        .map(|section| MouseAction::Section(*section))
        .collect()
}

fn run_tab_actions() -> Vec<MouseAction> {
    (0..RunTab::ALL.len()).map(MouseAction::RunTab).collect()
}

fn result_tab_actions() -> Vec<MouseAction> {
    (0..ResultTab::ALL.len())
        .map(MouseAction::ResultTab)
        .collect()
}

fn render_form(frame: &mut ratatui::Frame<'_>, app: &mut App, section: FormSection, area: Rect) {
    app.clamp_focus();
    let action_height = u16::from(area.height >= 3);
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Min(1),
            Constraint::Length(action_height),
        ])
        .split(area);
    let focus = app.focus(section);
    let count = app.field_count(section);
    render_tabs(
        frame,
        app,
        chunks[0],
        &FormSection::ALL.map(FormSection::tab),
        section.index(),
        &section_tab_actions(),
        Some(format!(
            "  行 {}/{}",
            if count == 0 {
                0
            } else {
                (focus.min(count - 1)) + 1
            },
            count
        )),
    );
    if section == FormSection::Preflight {
        render_preflight(frame, app, chunks[1]);
    } else {
        render_fields(frame, app, section, chunks[1]);
    }
    if action_height > 0 {
        render_form_actions(frame, app, chunks[2]);
    }
}

fn render_form_actions(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let labels = [
        ("[ ‹ 上一分区 ]", MouseAction::PrevSection, Color::Cyan),
        ("[ 下一分区 › ]", MouseAction::NextSection, Color::Cyan),
    ];
    let mut spans = Vec::new();
    let mut x = area.x;
    for (label, action, color) in labels {
        let width = u16::try_from(display_width(label)).unwrap_or(u16::MAX);
        if x >= area.right() {
            break;
        }
        let width = width.min(area.right().saturating_sub(x));
        spans.push(Span::styled(
            label,
            Style::default().fg(color).add_modifier(Modifier::BOLD),
        ));
        app.hit_targets.push(HitTarget {
            area: Rect::new(x, area.y, width, 1),
            action,
        });
        x = x.saturating_add(width).saturating_add(2);
    }
    if area.width >= 48 {
        spans.push(Span::styled(
            "  Ctrl-P 上一分区 / Ctrl-N 下一分区 · 方向键移动焦点 · Enter 编辑 · Space 切换",
            Style::default().fg(Color::DarkGray),
        ));
    }
    frame.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn render_fields(frame: &mut ratatui::Frame<'_>, app: &mut App, section: FormSection, area: Rect) {
    if section == FormSection::Sensors {
        render_sensor_fields(frame, app, area);
        return;
    }
    let rows = form::rows(app, section);
    if rows.is_empty() {
        render_message(frame, area, false, "没有可编辑字段");
        return;
    }
    let show_note = area.width >= 100;
    let detail_height = if !show_note && area.height >= 8 {
        (area.height / 4).clamp(2, 4)
    } else {
        0
    };
    let parts = if detail_height > 0 {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(1), Constraint::Length(detail_height)])
            .split(area)
    } else {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(1)])
            .split(area)
    };
    let table_area = parts[0];
    let widths = param_widths(&rows, usize::from(table_area.width), show_note);
    render_param_rows(frame, app, section, &rows, table_area, show_note, &widths);
    if detail_height > 0 {
        render_selected_detail(frame, &rows, app.focus(section), parts[1]);
    }
}

#[derive(Clone, Debug)]
struct ParamWidths {
    marker: usize,
    param: usize,
    value: usize,
    note: usize,
    gap: usize,
}

fn param_widths(rows: &[Row], available: usize, show_note: bool) -> ParamWidths {
    let marker = 2;
    let gap = 2;
    let max_label = rows
        .iter()
        .filter(|row| !row.header && !row.columns)
        .filter_map(|row| row.cells.first())
        .map(|cell| display_width(&cell.text))
        .max()
        .unwrap_or(10);
    let param = max_label.clamp(8, 26);
    let remaining = available.saturating_sub(marker + param + gap);
    let note = if show_note && remaining >= 40 {
        (remaining / 3).clamp(16, 48)
    } else {
        0
    };
    let value = remaining.saturating_sub(if note > 0 { note + gap } else { 0 });
    ParamWidths {
        marker,
        param,
        value,
        note,
        gap,
    }
}

fn render_param_rows(
    frame: &mut ratatui::Frame<'_>,
    app: &mut App,
    section: FormSection,
    rows: &[Row],
    area: Rect,
    show_note: bool,
    widths: &ParamWidths,
) {
    let viewport = usize::from(area.height);
    if viewport == 0 {
        return;
    }
    let focus = app.focus(section);
    let scroll = adjust_row_scroll(app, section, focus, rows.len(), viewport);
    let column_widths = [
        u16::try_from(widths.param).unwrap_or(0),
        u16::try_from(widths.value).unwrap_or(0),
        u16::try_from(widths.note).unwrap_or(0),
    ];
    let mut lines = Vec::with_capacity(viewport);
    for (index, row) in rows.iter().enumerate().skip(scroll).take(viewport) {
        let offset = u16::try_from(index - scroll).unwrap_or(u16::MAX);
        if offset >= area.height {
            break;
        }
        let y = area.y.saturating_add(offset);
        let focused = index == focus;
        if row.header {
            lines.push(group_line(row, focused, usize::from(area.width)));
            app.hit_targets.push(HitTarget {
                area: Rect::new(area.x, y, area.width, 1),
                action: MouseAction::Cell(index, 0),
            });
            continue;
        }
        if row.columns {
            lines.push(column_line(row, &column_widths));
            continue;
        }
        let line = param_line(row, focused, app.focus_cell(section), widths, show_note);
        lines.push(line);
        let param_x = area
            .x
            .saturating_add(u16::try_from(widths.marker).unwrap_or(0));
        let param_width = u16::try_from(widths.param).unwrap_or(0);
        let value_x = param_x
            .saturating_add(param_width)
            .saturating_add(u16::try_from(widths.gap).unwrap_or(0));
        let value_width = u16::try_from(widths.value).unwrap_or(0);
        app.hit_targets.push(HitTarget {
            area: Rect::new(
                param_x,
                y,
                param_width.min(area.right().saturating_sub(param_x)),
                1,
            ),
            action: MouseAction::Cell(index, 0),
        });
        app.hit_targets.push(HitTarget {
            area: Rect::new(
                value_x,
                y,
                value_width.min(area.right().saturating_sub(value_x)),
                1,
            ),
            action: MouseAction::Cell(index, 1),
        });
    }
    frame.render_widget(Paragraph::new(lines), area);
}

fn render_sensor_fields(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let rows = form::rows(app, FormSection::Sensors);
    if rows.is_empty() {
        render_message(frame, area, false, "没有可编辑传感器字段");
        return;
    }
    let detail_height = if area.height >= 10 {
        (area.height / 3).clamp(3, 6)
    } else {
        0
    };
    let parts = if detail_height > 0 {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(1), Constraint::Length(detail_height)])
            .split(area)
    } else {
        Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Min(1)])
            .split(area)
    };
    let table_area = parts[0];
    let viewport = usize::from(table_area.height);
    if viewport == 0 {
        return;
    }
    let focus = app.focus(FormSection::Sensors);
    let scroll = adjust_row_scroll(app, FormSection::Sensors, focus, rows.len(), viewport);
    let camera_widths = table_widths(TableId::Camera, usize::from(table_area.width));
    let imu_widths = table_widths(TableId::Imu, usize::from(table_area.width));
    let mut lines = Vec::with_capacity(viewport);
    for (index, row) in rows.iter().enumerate().skip(scroll).take(viewport) {
        let offset = u16::try_from(index - scroll).unwrap_or(u16::MAX);
        if offset >= table_area.height {
            break;
        }
        let y = table_area.y.saturating_add(offset);
        let focused = index == focus;
        if row.header {
            lines.push(group_line(row, focused, usize::from(table_area.width)));
            app.hit_targets.push(HitTarget {
                area: Rect::new(table_area.x, y, table_area.width, 1),
                action: MouseAction::Cell(index, 0),
            });
            continue;
        }
        if row.columns {
            let widths = if row.table == TableId::Imu {
                &imu_widths
            } else {
                &camera_widths
            };
            lines.push(column_line(row, widths));
            continue;
        }
        if row.table == TableId::Actions {
            lines.push(action_line(row, focused, usize::from(table_area.width)));
            app.hit_targets.push(HitTarget {
                area: Rect::new(table_area.x, y, table_area.width, 1),
                action: MouseAction::Cell(index, 0),
            });
            continue;
        }
        let widths = if row.table == TableId::Imu {
            &imu_widths
        } else {
            &camera_widths
        };
        let (line, starts) =
            sensor_line(row, focused, app.focus_cell(FormSection::Sensors), widths);
        lines.push(line);
        for (cell_index, (x, width)) in starts.iter().enumerate() {
            let x = table_area.x.saturating_add(*x);
            app.hit_targets.push(HitTarget {
                area: Rect::new(x, y, (*width).min(table_area.right().saturating_sub(x)), 1),
                action: MouseAction::Cell(index, cell_index),
            });
        }
    }
    frame.render_widget(Paragraph::new(lines), table_area);
    if detail_height > 0 {
        render_selected_detail(frame, &rows, focus, parts[1]);
    }
}

fn adjust_row_scroll(
    app: &mut App,
    section: FormSection,
    focus: usize,
    len: usize,
    viewport: usize,
) -> usize {
    let mut scroll = app.section_scroll[section.index()];
    if focus < scroll {
        scroll = focus;
    } else if focus >= scroll.saturating_add(viewport) {
        scroll = focus + 1 - viewport;
    }
    let max_scroll = len.saturating_sub(viewport);
    scroll = scroll.min(max_scroll);
    app.section_scroll[section.index()] = scroll;
    scroll
}

fn table_widths(table: TableId, available: usize) -> Vec<u16> {
    let columns = form::columns(table);
    let spacing = columns.len().saturating_sub(1) * 2;
    let available = available.saturating_sub(2 + spacing);
    let total_min = columns
        .iter()
        .map(|column| usize::from(column.min_width))
        .sum::<usize>();
    if available == 0 {
        return vec![0; columns.len()];
    }
    let mut widths = vec![0usize; columns.len()];
    if available < total_min {
        let mut used: usize = 0;
        for (index, column) in columns.iter().enumerate() {
            let width = if index + 1 == columns.len() {
                available.saturating_sub(used).max(1)
            } else {
                (usize::from(column.min_width) * available / total_min).max(1)
            };
            widths[index] = width;
            used = used.saturating_add(width);
            if used >= available {
                break;
            }
        }
    } else {
        let extra = available - total_min;
        let total_weight = columns
            .iter()
            .map(|column| usize::from(column.weight))
            .sum::<usize>();
        let mut used: usize = 0;
        for (index, column) in columns.iter().enumerate() {
            let share = (extra * usize::from(column.weight))
                .checked_div(total_weight)
                .unwrap_or(0);
            widths[index] = usize::from(column.min_width) + share;
            used = used.saturating_add(widths[index]);
        }
        if let Some(last) = widths.last_mut() {
            *last = last.saturating_add(available.saturating_sub(used));
        }
    }
    widths
        .into_iter()
        .map(|width| u16::try_from(width).unwrap_or(u16::MAX))
        .collect()
}

fn group_line(row: &Row, focused: bool, width: usize) -> Line<'static> {
    let title = row
        .cells
        .first()
        .map_or_else(String::new, |cell| cell.text.clone());
    let title = truncate_display(&title, width);
    let remaining = width.saturating_sub(display_width(&title));
    let note = if remaining > 4 {
        format!("  {}", truncate_display(&row.note, remaining - 2))
    } else {
        String::new()
    };
    let style = if focused {
        Style::default()
            .fg(Color::Black)
            .bg(Color::Cyan)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD)
    };
    Line::from(vec![
        Span::styled(title, style),
        Span::styled(note, Style::default().fg(Color::DarkGray)),
    ])
}

fn column_line(row: &Row, widths: &[u16]) -> Line<'static> {
    let mut spans = vec![Span::raw("  ")];
    for (index, cell) in row.cells.iter().enumerate() {
        let width = widths.get(index).copied().unwrap_or(0);
        if width == 0 {
            break;
        }
        let text = pad_display(&truncate_display(&cell.text, usize::from(width)), width);
        spans.push(Span::styled(
            text,
            Style::default()
                .fg(Color::Gray)
                .add_modifier(Modifier::BOLD),
        ));
        if widths.get(index + 1).is_some_and(|width| *width > 0) {
            spans.push(Span::raw("  "));
        }
    }
    Line::from(spans)
}

fn param_line(
    row: &Row,
    focused: bool,
    focus_cell: usize,
    widths: &ParamWidths,
    show_note: bool,
) -> Line<'static> {
    let param = row.cells.first();
    let value = row.cells.get(1);
    let marker = if focused { "▶ " } else { "  " };
    let param_text = pad_display(
        &truncate_display(param.map_or("", |cell| cell.text.as_str()), widths.param),
        u16::try_from(widths.param).unwrap_or(0),
    );
    let value_text = value.map_or_else(String::new, summarize_cell);
    let value_text = pad_display(
        &truncate_display(&value_text, widths.value),
        u16::try_from(widths.value).unwrap_or(0),
    );
    let mut spans = vec![Span::styled(
        marker,
        Style::default().fg(if focused {
            Color::Cyan
        } else {
            Color::DarkGray
        }),
    )];
    let param_style = if focused && focus_cell == 0 {
        Style::default()
            .fg(Color::White)
            .bg(Color::Rgb(52, 68, 80))
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(if focused { Color::White } else { Color::Gray })
    };
    spans.push(Span::styled(param_text, param_style));
    spans.push(Span::raw(" ".repeat(widths.gap)));
    let value_style = kind_style(value.map_or(FieldKind::ReadOnly, |cell| cell.kind));
    let value_style = if focused && focus_cell == 1 {
        value_style
            .bg(Color::Rgb(52, 68, 80))
            .add_modifier(Modifier::BOLD)
    } else {
        value_style
    };
    spans.push(Span::styled(value_text, value_style));
    if show_note && widths.note > 0 {
        spans.push(Span::raw(" ".repeat(widths.gap)));
        spans.push(Span::styled(
            truncate_display(&row.note, widths.note),
            Style::default().fg(Color::DarkGray),
        ));
    }
    let line = Line::from(spans);
    if focused {
        line.style(Style::default().bg(Color::Rgb(36, 44, 52)))
    } else {
        line
    }
}

fn action_line(row: &Row, focused: bool, width: usize) -> Line<'static> {
    let marker = if focused { "▶ " } else { "  " };
    let label = row.cells.first().map_or("", |cell| cell.text.as_str());
    let label = format!("[ {label} ]");
    let label = truncate_display(&label, width.saturating_sub(2));
    let mut spans = vec![
        Span::styled(
            marker,
            Style::default().fg(if focused {
                Color::Cyan
            } else {
                Color::DarkGray
            }),
        ),
        Span::styled(
            label,
            Style::default()
                .fg(Color::LightGreen)
                .add_modifier(Modifier::BOLD),
        ),
    ];
    let used = 2 + display_width(row.cells.first().map_or("", |cell| cell.text.as_str()));
    if width > used + 4 {
        spans.push(Span::styled(
            format!("  {}", truncate_display(&row.note, width - used - 2)),
            Style::default().fg(Color::DarkGray),
        ));
    }
    let line = Line::from(spans);
    if focused {
        line.style(Style::default().bg(Color::Rgb(36, 44, 52)))
    } else {
        line
    }
}

fn sensor_line(
    row: &Row,
    focused: bool,
    focus_cell: usize,
    widths: &[u16],
) -> (Line<'static>, Vec<(u16, u16)>) {
    let marker = if focused { "▶ " } else { "  " };
    let mut spans = vec![Span::styled(
        marker,
        Style::default().fg(if focused {
            Color::Cyan
        } else {
            Color::DarkGray
        }),
    )];
    let mut starts = Vec::with_capacity(row.cells.len());
    let mut x = 2u16;
    for (index, cell) in row.cells.iter().enumerate() {
        let width = widths.get(index).copied().unwrap_or(0);
        if width == 0 {
            break;
        }
        let text = summarize_cell(cell);
        let text = pad_display(&truncate_display(&text, usize::from(width)), width);
        let mut style = kind_style(cell.kind);
        if focused && focus_cell == index {
            style = style
                .bg(Color::Rgb(52, 68, 80))
                .add_modifier(Modifier::BOLD);
        }
        spans.push(Span::styled(text, style));
        if index + 1 < row.cells.len() {
            spans.push(Span::raw("  "));
        }
        starts.push((x, width));
        x = x.saturating_add(width).saturating_add(2);
    }
    let line = Line::from(spans);
    let line = if focused {
        line.style(Style::default().bg(Color::Rgb(36, 44, 52)))
    } else {
        line
    };
    (line, starts)
}

fn render_selected_detail(frame: &mut ratatui::Frame<'_>, rows: &[Row], focus: usize, area: Rect) {
    if area.height == 0 || area.width == 0 {
        return;
    }
    let text = rows.get(focus).map_or_else(
        || "当前行没有详情".to_owned(),
        |row| {
            if row.detail.is_empty() {
                row.note.clone()
            } else {
                row.detail.clone()
            }
        },
    );
    let width = usize::from(area.width).max(1);
    let lines = wrap_text(&text, width)
        .into_iter()
        .take(usize::from(area.height))
        .map(|line| Line::from(Span::styled(line, Style::default().fg(Color::Gray))))
        .collect::<Vec<_>>();
    frame.render_widget(Paragraph::new(lines), area);
}

fn kind_style(kind: FieldKind) -> Style {
    match kind {
        FieldKind::Toggle => Style::default().fg(Color::Green),
        FieldKind::Choice => Style::default().fg(Color::Cyan),
        FieldKind::Text => Style::default().fg(Color::Yellow),
        FieldKind::Button => Style::default()
            .fg(Color::LightGreen)
            .add_modifier(Modifier::BOLD),
        FieldKind::ReadOnly => Style::default().fg(Color::Gray),
    }
}

fn summarize_cell(cell: &form::Cell) -> String {
    let text = cell
        .text
        .replace('\n', " ⏎ ")
        .replace('\r', "")
        .trim()
        .to_owned();
    match cell.kind {
        FieldKind::Toggle => format!("[{}]", cell.text),
        FieldKind::Choice => format!("‹{text}›"),
        FieldKind::Text | FieldKind::ReadOnly => text,
        FieldKind::Button => format!("[ {text} ]"),
    }
}

fn truncate_display(text: &str, width: usize) -> String {
    if width == 0 {
        return String::new();
    }
    if display_width(text) <= width {
        return text.to_owned();
    }
    let limit = width.saturating_sub(1);
    let mut out = String::new();
    let mut used: usize = 0;
    for character in text.chars() {
        let character_width = if character.is_ascii() { 1 } else { 2 };
        if used + character_width > limit {
            break;
        }
        out.push(character);
        used += character_width;
    }
    out.push('…');
    out
}

fn pad_display(text: &str, width: u16) -> String {
    let width = usize::from(width);
    let used = display_width(text);
    if used >= width {
        text.to_owned()
    } else {
        format!("{text}{}", " ".repeat(width - used))
    }
}

fn field_line(field: &Field, focused: bool) -> Line<'static> {
    let marker = if focused { "▶ " } else { "  " };
    let mut spans = vec![Span::styled(
        marker,
        Style::default().fg(if focused {
            Color::Cyan
        } else {
            Color::DarkGray
        }),
    )];
    let label = format!("{}: ", field.label);
    spans.push(Span::styled(
        label,
        Style::default()
            .fg(if focused { Color::White } else { Color::Gray })
            .add_modifier(if focused {
                Modifier::BOLD
            } else {
                Modifier::empty()
            }),
    ));
    let value_style = match field.kind {
        FieldKind::Toggle => Style::default().fg(if field.value == "x" {
            Color::Green
        } else {
            Color::DarkGray
        }),
        FieldKind::Choice => Style::default().fg(Color::Cyan),
        FieldKind::Text => Style::default().fg(Color::Yellow),
        FieldKind::Button => Style::default()
            .fg(Color::LightGreen)
            .add_modifier(Modifier::BOLD),
        FieldKind::ReadOnly => Style::default().fg(Color::DarkGray),
    };
    match field.kind {
        FieldKind::Toggle => spans.push(Span::styled(format!("[{}]", field.value), value_style)),
        FieldKind::Choice => spans.push(Span::styled(format!("‹{}›", field.value), value_style)),
        FieldKind::Text => spans.push(Span::styled(format!("{} ✎", field.value), value_style)),
        FieldKind::Button => spans.push(Span::styled(format!("[ {} ]", field.value), value_style)),
        FieldKind::ReadOnly => spans.push(Span::styled(field.value.clone(), value_style)),
    }
    if !field.note.is_empty() {
        spans.push(Span::styled(
            format!("   · {}", field.note),
            Style::default().fg(Color::DarkGray),
        ));
    }
    let line = Line::from(spans);
    if focused {
        line.style(Style::default().bg(Color::Rgb(36, 44, 52)))
    } else {
        line
    }
}

fn render_preflight(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let preflight = app.preflight();
    let fields = super::form::fields(app, FormSection::Preflight);
    let focus = app
        .focus(FormSection::Preflight)
        .min(fields.len().saturating_sub(1));
    let action_height = u16::try_from(fields.len())
        .unwrap_or(u16::MAX)
        .min(area.height.saturating_sub(1))
        .max(1);
    let parts = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(1), Constraint::Length(action_height)])
        .split(area);
    let checks_bordered = parts[0].height >= 3 && parts[0].width >= 20;
    let checks_inner = inner_area(parts[0], checks_bordered);
    let summary = format!(
        "预检 — 阻断 {} / 警告 {} / 未检查 {}",
        preflight.blockers(),
        preflight.warnings(),
        preflight.not_checked()
    );
    let check_lines = preflight_lines(&preflight, usize::from(checks_inner.width).max(10));
    let viewport = usize::from(checks_inner.height).max(1);
    let max_scroll = check_lines.len().saturating_sub(viewport);
    app.preflight_scroll = app.preflight_scroll.min(max_scroll);
    let visible = check_lines
        .iter()
        .skip(app.preflight_scroll)
        .take(viewport)
        .map(|(line, _level)| line.clone())
        .collect::<Vec<_>>();
    if checks_bordered {
        frame.render_widget(
            Paragraph::new(visible).block(Block::default().borders(Borders::ALL).title(summary)),
            parts[0],
        );
    } else {
        frame.render_widget(Paragraph::new(visible), parts[0]);
    }

    let action_bordered = parts[1].height >= 3 && parts[1].width >= 20;
    let action_inner = inner_area(parts[1], action_bordered);
    let mut action_lines = Vec::new();
    for (index, field) in fields.iter().enumerate() {
        let row = u16::try_from(index).unwrap_or(u16::MAX);
        if row >= action_inner.height {
            break;
        }
        action_lines.push(field_line(field, index == focus));
        app.hit_targets.push(HitTarget {
            area: Rect::new(
                action_inner.x,
                action_inner.y.saturating_add(row),
                action_inner.width,
                1,
            ),
            action: MouseAction::Field(index),
        });
    }
    if action_bordered {
        frame.render_widget(
            Paragraph::new(action_lines)
                .block(Block::default().borders(Borders::ALL).title("操作")),
            parts[1],
        );
    } else {
        frame.render_widget(Paragraph::new(action_lines), parts[1]);
    }
}

fn preflight_lines(preflight: &Preflight, width: usize) -> Vec<(Line<'static>, CheckLevel)> {
    let mut lines = Vec::new();
    for check in &preflight.checks {
        let color = match check.level {
            CheckLevel::Blocker => Color::Red,
            CheckLevel::Warning => Color::Yellow,
            CheckLevel::Ok => Color::Green,
            CheckLevel::NotChecked => Color::DarkGray,
        };
        let text = format!(
            "[{}] {} — {}",
            check.level.badge(),
            check.title,
            check.detail
        );
        for (index, piece) in wrap_text(&text, width).into_iter().enumerate() {
            let style = if index == 0 {
                Style::default()
                    .fg(color)
                    .add_modifier(if check.level == CheckLevel::Blocker {
                        Modifier::BOLD
                    } else {
                        Modifier::empty()
                    })
            } else {
                Style::default().fg(color)
            };
            lines.push((Line::from(Span::styled(piece, style)), check.level));
        }
    }
    lines
}

fn render_expert(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let Some(expert) = &app.expert else {
        render_message(frame, area, true, "编辑器未打开");
        return;
    };
    let action_height = u16::from(area.height >= 3);
    let parts = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Min(1), Constraint::Length(action_height)])
        .split(area);
    let editor_area = parts[0];
    let viewport = usize::from(inner_area(editor_area, true).height);
    let cursor_line = expert.buffer.cursor_line();
    let scroll = cursor_line.saturating_sub(viewport.saturating_sub(1));
    let title = if expert.is_dirty() {
        "YAML 专家编辑 *未保存*（Ctrl-S 应用，Esc 关闭）"
    } else {
        "YAML 专家编辑（Ctrl-S 应用，Esc 关闭）"
    };
    let text = expert.buffer.display_text();
    let error = expert.error.clone();
    frame.render_widget(
        Paragraph::new(text)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(title.to_owned()),
            )
            .wrap(Wrap { trim: false })
            .scroll((u16::try_from(scroll).unwrap_or(u16::MAX), 0)),
        editor_area,
    );
    if let Some(error) = error {
        let height = u16::try_from(error.lines().count().saturating_add(2))
            .unwrap_or(u16::MAX)
            .min(editor_area.height / 2);
        if height > 0 {
            let popup = Rect::new(
                editor_area.x.saturating_add(2),
                editor_area
                    .y
                    .saturating_add(editor_area.height.saturating_sub(height + 2)),
                editor_area.width.saturating_sub(4),
                height,
            );
            frame.render_widget(Clear, popup);
            frame.render_widget(
                Paragraph::new(error)
                    .block(
                        Block::default()
                            .borders(Borders::ALL)
                            .title("YAML 错误（未应用）"),
                    )
                    .wrap(Wrap { trim: false }),
                popup,
            );
        }
    }
    if action_height == 0 {
        return;
    }
    // Mouse access to save/close without keyboard chords.
    let apply = "[ 应用 Ctrl-S ]";
    let close = "[ 关闭 Esc ]";
    let apply_width = u16::try_from(display_width(apply)).unwrap_or(u16::MAX);
    let close_width = u16::try_from(display_width(close)).unwrap_or(u16::MAX);
    let apply_x = parts[1].x;
    let close_x = apply_x.saturating_add(apply_width).saturating_add(2);
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(
                apply,
                Style::default()
                    .fg(Color::LightGreen)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(close, Style::default().fg(Color::Red)),
        ])),
        parts[1],
    );
    app.hit_targets.push(HitTarget {
        area: Rect::new(
            apply_x,
            parts[1].y,
            apply_width.min(parts[1].right().saturating_sub(apply_x)),
            1,
        ),
        action: MouseAction::ExpertApply,
    });
    app.hit_targets.push(HitTarget {
        area: Rect::new(
            close_x,
            parts[1].y,
            close_width.min(parts[1].right().saturating_sub(close_x)),
            1,
        ),
        action: MouseAction::ExpertClose,
    });
}

fn render_run(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Min(1),
            Constraint::Length(1),
        ])
        .split(area);
    render_tabs(
        frame,
        app,
        chunks[0],
        &RunTab::ALL.map(RunTab::title),
        app.run_tab.index(),
        &run_tab_actions(),
        None,
    );
    let Some(task) = &app.task else {
        render_message(frame, chunks[1], true, "尚未启动标定");
        return;
    };
    match app.run_tab {
        RunTab::Overview => render_dashboard(frame, task, &mut app.hit_targets, chunks[1]),
        RunTab::Details => {
            let text = format!(
                "{}\n\n{}\n\n{}\n\n求解活动:\n{}\n\n最近事件:\n{}",
                task::settings_snapshot_text(&task.settings),
                task::run_timing_text(task),
                task::dataset_summary_text(task.dataset_summary.as_ref()),
                task::solver_activity_text(task, Instant::now()),
                task::messages_text(task, 8)
            );
            if chunks[1].height < 14 {
                render_text_pane(
                    frame,
                    chunks[1],
                    "运行详情（终端过小，图表隐藏）",
                    &text,
                    &mut app.run_scroll[RunTab::Details.index()],
                );
            } else {
                let parts = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([Constraint::Length(8), Constraint::Min(6)])
                    .split(chunks[1]);
                render_chart(frame, task, parts[1]);
                render_text_pane(
                    frame,
                    parts[0],
                    "求解活动",
                    &text,
                    &mut app.run_scroll[RunTab::Details.index()],
                );
            }
        }
        RunTab::Log => {
            let log_path = task.run_root.join("run.log");
            let log = std::fs::read_to_string(&log_path)
                .unwrap_or_else(|error| format!("无法读取 {}: {error}", log_path.display()));
            let text = format!(
                "run.log: {}\n\n{}\n\n内存事件流:\n{}",
                log_path.display(),
                log,
                task.messages.join("\n")
            );
            render_text_pane(
                frame,
                chunks[1],
                "运行日志（内存事件流；完整文件见 run.log）",
                &text,
                &mut app.run_scroll[RunTab::Log.index()],
            );
        }
    }
    render_run_actions(frame, app, chunks[2]);
}

fn render_run_actions(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let running = app.task.as_ref().is_some_and(Task::is_running);
    let finished = !running && app.task.is_some();
    let finished_prefix = "运行已结束 · g 切换图表 · Tab 切换页 · Ctrl-Q 退出  ";
    let text = if running {
        "F8 / Ctrl-C 取消运行（需确认）   Esc 不取消、不退出"
    } else if finished {
        finished_prefix
    } else {
        "尚无运行 · Ctrl-Q 退出 · F1 帮助"
    };
    let mut spans = vec![Span::styled(text, Style::default().fg(Color::DarkGray))];
    if running {
        let prefix = "F8 / Ctrl-C 取消运行（需确认）   Esc 不取消、不退出  ";
        let button = "[ 取消运行 ]";
        let x = area
            .x
            .saturating_add(u16::try_from(display_width(prefix)).unwrap_or(u16::MAX));
        let width = u16::try_from(display_width(button)).unwrap_or(u16::MAX);
        spans.push(Span::styled(
            button,
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ));
        app.hit_targets.push(HitTarget {
            area: Rect::new(x, area.y, width.min(area.right().saturating_sub(x)), 1),
            action: MouseAction::RunCancel,
        });
    } else if finished {
        let button = "[ 查看结果表 (r) ]";
        let x = area
            .x
            .saturating_add(u16::try_from(display_width(finished_prefix)).unwrap_or(u16::MAX));
        let width = u16::try_from(display_width(button)).unwrap_or(u16::MAX);
        spans.push(Span::styled(
            button,
            Style::default()
                .fg(Color::Green)
                .add_modifier(Modifier::BOLD),
        ));
        app.hit_targets.push(HitTarget {
            area: Rect::new(x, area.y, width.min(area.right().saturating_sub(x)), 1),
            action: MouseAction::RunOpenResult,
        });
    }
    frame.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn render_dashboard(
    frame: &mut ratatui::Frame<'_>,
    task: &Task,
    hit_targets: &mut Vec<HitTarget>,
    area: Rect,
) {
    if area.height == 0 {
        return;
    }
    let compact = area.height < 24 || area.width < 100;
    let rows = task::camera_detection_rows(task);
    let status_height = if compact { 3 } else { 4 };
    let desired_table = if compact {
        rows.len().saturating_add(1)
    } else {
        rows.len().saturating_add(3)
    };
    let minimum_graph = 6_u16;
    let status_height =
        status_height.min(area.height.saturating_sub(minimum_graph.saturating_add(2)));
    let table_height = u16::try_from(desired_table)
        .unwrap_or(u16::MAX)
        .min(area.height.saturating_sub(status_height + minimum_graph));
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(status_height),
            Constraint::Length(table_height),
            Constraint::Min(1),
        ])
        .split(area);
    if chunks[0].height > 0 {
        let lines = dashboard_status_lines(task);
        frame.render_widget(Paragraph::new(lines).wrap(Wrap { trim: true }), chunks[0]);
    }
    if chunks[1].height > 0 {
        render_detection_table(frame, task, &rows, chunks[1], !compact);
    }
    render_dashboard_graphs(frame, task, hit_targets, chunks[2]);
}

fn dashboard_status_lines(task: &Task) -> Vec<Line<'static>> {
    let now = Instant::now();
    let at = task.effective_now(now);
    vec![
        dashboard_stage_line(task, now, at),
        Line::from(Span::styled(
            dashboard_timing_text(task),
            Style::default().fg(Color::Gray),
        )),
        dashboard_activity_line(task, at),
        dashboard_lm_line(task),
    ]
}

fn dashboard_stage_line(task: &Task, now: Instant, at: Instant) -> Line<'static> {
    let stage_elapsed = task.stage_started_at.map_or_else(
        || "--".to_owned(),
        |started| task::format_short_duration(at.saturating_duration_since(started)),
    );
    let run_state = if task.finished_at.is_some() {
        "已结束"
    } else if task.cancel_requested_at.is_some() {
        "取消中"
    } else {
        "运行中"
    };
    Line::from(vec![
        Span::styled("阶段 ", Style::default().fg(Color::DarkGray)),
        Span::styled(
            task::stage_label(task.current_stage()).to_owned(),
            Style::default()
                .fg(Color::Cyan)
                .add_modifier(Modifier::BOLD),
        ),
        Span::styled(
            format!(
                "  {stage_elapsed}  · 总用时 {}  · {run_state}",
                task::format_duration(task.run_elapsed_at(now))
            ),
            Style::default().fg(Color::Gray),
        ),
    ])
}

fn dashboard_timing_text(task: &Task) -> String {
    let mut timing = task::run_timing_text(task);
    if task.current_stage() == "scan_detect"
        && let Some((processed, total)) = task::detection_frame_progress(task)
    {
        timing.push_str("  检测帧 ");
        timing.push_str(&processed.to_string());
        timing.push('/');
        timing.push_str(&total.to_string());
    }
    timing
}

fn dashboard_activity_line(task: &Task, at: Instant) -> Line<'static> {
    let activity = task.solver_activity.activity.map_or_else(
        || {
            if task.current_stage() == "scan_detect" {
                dashboard_detection_activity(task)
            } else {
                "活动 尚未收到求解器活动上报（不显示推测进度）".to_owned()
            }
        },
        |activity| dashboard_solver_activity(task, at, activity),
    );
    Line::from(Span::styled(activity, Style::default()))
}

fn dashboard_detection_activity(task: &Task) -> String {
    task.detection_progress.last().map_or_else(
        || "检测已启动，等待首批进度…".to_owned(),
        |progress| {
            format!(
                "检测吞吐 {:.1} fps（平均 {:.1}）  workers={}",
                progress.frames_per_second, progress.average_frames_per_second, progress.workers
            )
        },
    )
}

fn dashboard_solver_activity(
    task: &Task,
    at: Instant,
    activity: ccl_calib::camera::SolveActivity,
) -> String {
    let phase_elapsed = task
        .solver_activity
        .phase_elapsed(at)
        .map_or_else(|| "--".to_owned(), task::format_short_duration);
    let update_age = task
        .solver_activity
        .update_age(at)
        .map_or_else(|| "--".to_owned(), task::format_short_duration);
    let work = task::known_work_text(activity).unwrap_or_else(|| "工作量未上报".to_owned());
    let interruption = if activity.cancellable {
        "可取消"
    } else {
        "不可中断（取消需等待返回）"
    };
    format!(
        "活动 {}  本阶段 {phase_elapsed}  更新 {update_age} 前  工作量 {work}  {interruption}",
        task::solve_phase_label(activity.phase)
    )
}

fn dashboard_lm_line(task: &Task) -> Line<'static> {
    let (label, _) = task::lm_snapshot_state(task);
    Line::from(match task.solve_progress.last() {
        Some(last) => Span::styled(
            format!(
                "LM {label} iter={}/{}  cost={:.3e}  λ={:.2e}  |g|∞={:.2e}  接受/拒绝={}/{}",
                last.iteration,
                last.max_iterations,
                last.cost,
                last.damping,
                last.scaled_gradient_inf_norm,
                last.accepted_steps,
                last.rejected_steps
            ),
            Style::default().fg(Color::Green),
        ),
        None => Span::styled(
            "LM 尚无迭代快照（初始化/装配/线性求解中）".to_owned(),
            Style::default().fg(Color::DarkGray),
        ),
    })
}

fn render_detection_table(
    frame: &mut ratatui::Frame<'_>,
    task: &Task,
    rows: &[task::CameraDetectionRow],
    area: Rect,
    bordered: bool,
) {
    let cumulative = rows.first().is_some_and(|row| row.corners_are_cumulative);
    let header = ratatui::widgets::Row::new(vec![
        ratatui::widgets::Cell::from("相机"),
        ratatui::widgets::Cell::from("已处理"),
        ratatui::widgets::Cell::from("检出帧"),
        ratatui::widgets::Cell::from(if cumulative {
            "累计角点"
        } else {
            "最近角点"
        }),
        ratatui::widgets::Cell::from("fps"),
        ratatui::widgets::Cell::from("平均 fps"),
        ratatui::widgets::Cell::from("workers"),
    ])
    .style(
        Style::default()
            .fg(Color::Cyan)
            .add_modifier(Modifier::BOLD),
    );
    let mut table_rows = rows
        .iter()
        .map(|row| {
            ratatui::widgets::Row::new(vec![
                ratatui::widgets::Cell::from(row.camera.clone()),
                ratatui::widgets::Cell::from(optional_count(row.processed)),
                ratatui::widgets::Cell::from(optional_count(row.detected_frames)),
                ratatui::widgets::Cell::from(optional_count(row.corners)),
                ratatui::widgets::Cell::from(optional_rate(row.frames_per_second)),
                ratatui::widgets::Cell::from(optional_rate(row.average_frames_per_second)),
                ratatui::widgets::Cell::from(optional_count(row.workers)),
            ])
        })
        .collect::<Vec<_>>();
    table_rows.push(
        ratatui::widgets::Row::new(vec![
            ratatui::widgets::Cell::from("合计"),
            ratatui::widgets::Cell::from(total_if_complete(rows.iter().map(|row| row.processed))),
            ratatui::widgets::Cell::from(total_if_complete(
                rows.iter().map(|row| row.detected_frames),
            )),
            ratatui::widgets::Cell::from(total_if_complete(rows.iter().map(|row| row.corners))),
            ratatui::widgets::Cell::from("—"),
            ratatui::widgets::Cell::from("—"),
            ratatui::widgets::Cell::from("—"),
        ])
        .style(Style::default().fg(Color::Gray)),
    );
    let widths = [
        Constraint::Min(8),
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Length(10),
        Constraint::Length(8),
        Constraint::Length(9),
        Constraint::Length(7),
    ];
    let mut table = ratatui::widgets::Table::new(table_rows, widths)
        .header(header)
        .column_spacing(1);
    if bordered && area.height >= 4 {
        let workers = task
            .detection_progress
            .last()
            .map_or_else(|| "-".to_owned(), |progress| progress.workers.to_string());
        let note = if cumulative {
            "检测汇总 · 线程数来自检测进度"
        } else {
            "检测进度 · 检出帧与累计角点在汇总后显示"
        };
        table = table.block(
            Block::default()
                .borders(Borders::ALL)
                .title(format!("检测统计  workers={workers}  ·  {note}")),
        );
    }
    frame.render_widget(table, area);
}

fn optional_count(value: Option<usize>) -> String {
    value.map_or_else(|| "—".to_owned(), |value| value.to_string())
}

fn optional_rate(value: Option<f64>) -> String {
    value.map_or_else(|| "—".to_owned(), |value| format!("{value:.1}"))
}

fn total_if_complete(values: impl Iterator<Item = Option<usize>>) -> String {
    let mut total = 0_usize;
    let mut any = false;
    for value in values {
        let Some(value) = value else {
            return "—".to_owned();
        };
        let Some(next) = total.checked_add(value) else {
            return "—".to_owned();
        };
        total = next;
        any = true;
    }
    if any {
        total.to_string()
    } else {
        "—".to_owned()
    }
}

fn render_dashboard_graphs(
    frame: &mut ratatui::Frame<'_>,
    task: &Task,
    hit_targets: &mut Vec<HitTarget>,
    area: Rect,
) {
    if area.height == 0 {
        return;
    }
    let parts = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(1), Constraint::Min(1)])
        .split(area);
    render_graph_selector(frame, task, hit_targets, parts[0]);
    let graph_area = parts[1];
    if graph_area.height == 0 {
        return;
    }
    let selected = task.dashboard_graph();
    if graph_area.width >= 120 && graph_area.height >= 10 {
        let columns = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(graph_area);
        render_detection_curve(
            frame,
            task,
            columns[0],
            selected == task::DashboardGraph::Detection,
        );
        render_convergence(
            frame,
            task,
            columns[1],
            selected == task::DashboardGraph::Convergence,
        );
    } else if graph_area.width >= 100 && graph_area.height >= 12 {
        let rows = Layout::default()
            .direction(Direction::Vertical)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(graph_area);
        render_detection_curve(
            frame,
            task,
            rows[0],
            selected == task::DashboardGraph::Detection,
        );
        render_convergence(
            frame,
            task,
            rows[1],
            selected == task::DashboardGraph::Convergence,
        );
    } else {
        match selected {
            task::DashboardGraph::Detection => {
                render_detection_curve(frame, task, graph_area, true);
            }
            task::DashboardGraph::Convergence => {
                render_convergence(frame, task, graph_area, true);
            }
        }
    }
}

fn render_graph_selector(
    frame: &mut ratatui::Frame<'_>,
    task: &Task,
    hit_targets: &mut Vec<HitTarget>,
    area: Rect,
) {
    let selected = task.dashboard_graph();
    let prefix = "图表 (g / 点击): ";
    let mut spans = vec![Span::styled(prefix, Style::default().fg(Color::DarkGray))];
    let mut x = area
        .x
        .saturating_add(u16::try_from(display_width(prefix)).unwrap_or(u16::MAX));
    for (index, graph) in [
        (0_usize, task::DashboardGraph::Detection),
        (1, task::DashboardGraph::Convergence),
    ] {
        let active = graph == selected;
        let label = format!("[{}]", graph.label());
        let width = u16::try_from(display_width(&label)).unwrap_or(u16::MAX);
        if x < area.right() {
            hit_targets.push(HitTarget {
                area: Rect::new(x, area.y, width.min(area.right().saturating_sub(x)), 1),
                action: MouseAction::DashboardGraph(index),
            });
        }
        spans.push(Span::styled(
            label,
            if active {
                Style::default()
                    .fg(Color::Yellow)
                    .add_modifier(Modifier::BOLD)
            } else {
                Style::default().fg(Color::DarkGray)
            },
        ));
        spans.push(Span::raw(" "));
        x = x.saturating_add(width).saturating_add(1);
    }
    spans.push(Span::styled(
        "宽屏并排 · 中屏上下 · 小屏显示所选图表",
        Style::default().fg(Color::DarkGray),
    ));
    frame.render_widget(Paragraph::new(Line::from(spans)), area);
}

fn chart_block(title: String, focused: bool) -> Block<'static> {
    Block::default()
        .borders(Borders::ALL)
        .border_style(Style::default().fg(if focused { Color::Yellow } else { Color::Cyan }))
        .title(title)
}

fn render_chart(frame: &mut ratatui::Frame<'_>, task: &Task, area: Rect) {
    match task.dashboard_graph() {
        task::DashboardGraph::Detection => render_detection_curve(frame, task, area, false),
        task::DashboardGraph::Convergence => render_convergence(frame, task, area, false),
    }
}

fn render_detection_curve(frame: &mut ratatui::Frame<'_>, task: &Task, area: Rect, focused: bool) {
    let Some(last) = task.detection_progress.last() else {
        frame.render_widget(
            Paragraph::new("等待检测结果…").block(chart_block("检测历史".to_owned(), focused)),
            area,
        );
        return;
    };
    let chart_data = task::detection_chart_series(
        &task.detection_progress,
        task.detection_time_range_ns.unwrap_or_default(),
    );
    let y_max = task::board_corner_limit(&task.settings)
        .max(chart_data.y_max)
        .max(1.0);
    let colors = [
        Color::Cyan,
        Color::Yellow,
        Color::Magenta,
        Color::Green,
        Color::Blue,
        Color::Red,
        Color::LightCyan,
        Color::LightYellow,
    ];
    let datasets = chart_data
        .series
        .iter()
        .enumerate()
        .map(|(index, (camera, points))| {
            Dataset::default()
                .name(camera.as_str())
                .marker(Marker::Braille)
                .graph_type(GraphType::Line)
                .style(Style::default().fg(colors[index % colors.len()]))
                .data(points)
        })
        .collect::<Vec<_>>();
    let x_max = chart_data.x_max.max(1.0);
    let title = format!(
        "检测历史 · AprilGrid 角点 · {:.1} fps（平均 {:.1}）workers={}",
        last.frames_per_second, last.average_frames_per_second, last.workers,
    );
    let chart = Chart::new(datasets)
        .block(chart_block(title, focused))
        .legend_position(Some(LegendPosition::TopLeft))
        .x_axis(
            Axis::default()
                .title("检测开始后时间 (s)")
                .bounds([0.0, x_max])
                .labels(["0".to_owned(), format!("{x_max:.1}")]),
        )
        .y_axis(
            Axis::default()
                .title("角点")
                .bounds([0.0, y_max])
                .labels(["0".to_owned(), format!("{y_max:.0}")]),
        );
    frame.render_widget(chart, area);
}

fn render_convergence(frame: &mut ratatui::Frame<'_>, task: &Task, area: Rect, focused: bool) {
    let Some(last) = task.solve_progress.last() else {
        let body = task.solver_activity.activity.map_or_else(
            || "等待求解器活动…".to_owned(),
            |activity| {
                let elapsed = task
                    .solver_activity
                    .phase_elapsed(task.effective_now(Instant::now()))
                    .map_or_else(|| "--".to_owned(), task::format_short_duration);
                task::known_work_text(activity).map_or_else(
                    || {
                        format!(
                            "当前活动: {}  本阶段 {elapsed}  当前工作量不可计数，不推算完成百分比",
                            task::solve_phase_label(activity.phase)
                        )
                    },
                    |counts| {
                        format!(
                            "当前活动: {}  本阶段 {elapsed}  {counts}",
                            task::solve_phase_label(activity.phase)
                        )
                    },
                )
            },
        );
        frame.render_widget(
            Paragraph::new(body)
                .wrap(Wrap { trim: false })
                .block(chart_block("LM 收敛（等待迭代快照）".to_owned(), focused)),
            area,
        );
        return;
    };
    let Some(data) = task::convergence_chart_data(&task.solve_progress) else {
        frame.render_widget(
            Paragraph::new("LM 快照没有可绘制的有限 cost")
                .block(chart_block("LM 收敛".to_owned(), focused)),
            area,
        );
        return;
    };
    let (label, _) = task::lm_snapshot_state(task);
    let title = format!(
        "LM 收敛 · {label} · iter={}/{} · cost={:.3e} · λ={:.2e} · |g|∞={:.2e}",
        last.iteration, last.max_iterations, last.cost, last.damping, last.scaled_gradient_inf_norm,
    );
    let x_max = data
        .points
        .last()
        .map_or(1.0, |&(iteration, _)| iteration.max(1.0));
    let dataset = Dataset::default()
        .name("cost")
        .marker(Marker::Braille)
        .graph_type(GraphType::Line)
        .style(Style::default().fg(Color::Green))
        .data(&data.points);
    let chart = Chart::new(vec![dataset])
        .block(chart_block(title, focused))
        .x_axis(
            Axis::default()
                .title("iteration")
                .bounds([0.0, x_max])
                .labels(["0".to_owned(), format!("{x_max:.0}")]),
        )
        .y_axis(
            Axis::default()
                .title("log10(cost)")
                .bounds([data.y_min, data.y_max])
                .labels([format!("{:.1}", data.y_min), format!("{:.1}", data.y_max)]),
        );
    frame.render_widget(chart, area);
}

fn render_result(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect) {
    let narrow = area.width < 130;
    let action_count = if app.result_tab == ResultTab::Raw {
        6
    } else {
        5
    };
    let action_height = if narrow {
        action_count.min(area.height.saturating_sub(2)).max(1)
    } else {
        1
    };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),
            Constraint::Min(1),
            Constraint::Length(action_height),
        ])
        .split(area);
    render_tabs(
        frame,
        app,
        chunks[0],
        &ResultTab::ALL.map(ResultTab::title),
        app.result_tab.index(),
        &result_tab_actions(),
        None,
    );
    let Some(task) = &app.task else {
        render_message(frame, chunks[1], true, "没有运行结果");
        return;
    };
    let Some(data) = &app.result_data else {
        render_message(frame, chunks[1], true, "结果尚未加载");
        return;
    };
    let (title, text, scroll_slot) = match app.result_tab {
        ResultTab::Overview => (
            "结果概览".to_owned(),
            data.overview_text(task),
            ResultTab::Overview.index(),
        ),
        ResultTab::Parameters => (
            "参数（result/calibration.yaml）".to_owned(),
            data.parameters_text(),
            ResultTab::Parameters.index(),
        ),
        ResultTab::Extrinsics => (
            "外参与 IMU".to_owned(),
            data.extrinsics_text(),
            ResultTab::Extrinsics.index(),
        ),
        ResultTab::Diagnostics => (
            "诊断".to_owned(),
            data.diagnostics_text(task),
            ResultTab::Diagnostics.index(),
        ),
        ResultTab::Raw => {
            let path = ResultData::raw_path(&task.run_root, app.raw_source);
            let exists = path.is_file();
            let text = if exists {
                data.raw_text(app.raw_source)
            } else {
                format!("{} 未生成", path.display())
            };
            (
                format!(
                    "原始文件: {}（Enter/Space 切换，F6 打开目录）",
                    app.raw_source.label()
                ),
                text,
                ResultTab::Raw.index(),
            )
        }
    };
    render_text_pane(
        frame,
        chunks[1],
        &title,
        &text,
        &mut app.result_scroll[scroll_slot],
    );
    render_result_actions(frame, app, chunks[2], narrow);
}

fn render_result_actions(frame: &mut ratatui::Frame<'_>, app: &mut App, area: Rect, narrow: bool) {
    let html_exists = app
        .task
        .as_ref()
        .is_some_and(|task| ResultData::has_html(&task.run_root));
    let raw_tab = app.result_tab == ResultTab::Raw;
    let mut actions: Vec<(&str, MouseAction, Color)> = vec![
        ("[ 仪表盘 Esc ]", MouseAction::ResultDashboard, Color::Cyan),
        (
            "[ 打开运行目录 F6 ]",
            MouseAction::ResultOpenDir,
            Color::Green,
        ),
        (
            "[ 打开 3D 结果 HTML F7 ]",
            MouseAction::ResultOpenHtml,
            if html_exists {
                Color::Green
            } else {
                Color::DarkGray
            },
        ),
        ("[ 修改配置 ]", MouseAction::ResultModify, Color::Cyan),
        (
            "[ 用相同配置重跑 ]",
            MouseAction::ResultRerun,
            Color::LightGreen,
        ),
    ];
    if raw_tab {
        actions.push((
            "[ 切换原始文件 Enter ]",
            MouseAction::ResultRawCycle,
            Color::Yellow,
        ));
    }
    let mut lines: Vec<Line<'static>> = Vec::new();
    if narrow {
        for (index, (label, action, color)) in actions.iter().enumerate() {
            if index >= usize::from(area.height) {
                break;
            }
            let y = area
                .y
                .saturating_add(u16::try_from(index).unwrap_or(u16::MAX));
            let width = u16::try_from(display_width(label)).unwrap_or(u16::MAX);
            app.hit_targets.push(HitTarget {
                area: Rect::new(area.x, y, width.min(area.width), 1),
                action: action.clone(),
            });
            lines.push(Line::from(Span::styled(
                (*label).to_owned(),
                Style::default().fg(*color).add_modifier(Modifier::BOLD),
            )));
        }
    } else {
        let mut spans = Vec::new();
        let mut x = area.x;
        for (label, action, color) in &actions {
            let label = format!("{label}  ");
            let width = u16::try_from(display_width(&label)).unwrap_or(u16::MAX);
            if x >= area.right() {
                break;
            }
            spans.push(Span::styled(
                label,
                Style::default().fg(*color).add_modifier(Modifier::BOLD),
            ));
            app.hit_targets.push(HitTarget {
                area: Rect::new(x, area.y, width.min(area.right().saturating_sub(x)), 1),
                action: action.clone(),
            });
            x = x.saturating_add(width);
        }
        lines.push(Line::from(spans));
    }
    frame.render_widget(Paragraph::new(lines), area);
}

fn render_text_pane(
    frame: &mut ratatui::Frame<'_>,
    area: Rect,
    title: &str,
    text: &str,
    scroll: &mut usize,
) {
    let bordered = area.height >= 3 && area.width >= 12;
    let inner = inner_area(area, bordered);
    let viewport = usize::from(inner.height);
    if viewport == 0 {
        return;
    }
    let width = usize::from(inner.width).max(10);
    let wrapped = wrap_text(text, width);
    let max_scroll = wrapped.len().saturating_sub(viewport);
    *scroll = (*scroll).min(max_scroll);
    let content = wrapped.join("\n");
    let paragraph = Paragraph::new(content).scroll((u16::try_from(*scroll).unwrap_or(u16::MAX), 0));
    if bordered {
        frame.render_widget(
            paragraph.block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(title.to_owned()),
            ),
            area,
        );
    } else {
        frame.render_widget(paragraph, area);
    }
}

fn render_message(frame: &mut ratatui::Frame<'_>, area: Rect, bordered: bool, text: &str) {
    if bordered && area.height >= 3 {
        frame.render_widget(
            Paragraph::new(text.to_owned()).block(Block::default().borders(Borders::ALL)),
            area,
        );
    } else {
        frame.render_widget(Paragraph::new(text.to_owned()), area);
    }
}

fn render_dropdown(frame: &mut ratatui::Frame<'_>, app: &mut App, body: Rect) {
    let Some(kind) = app.dropdown.as_ref().map(|dropdown| dropdown.kind) else {
        return;
    };
    let selected = app
        .dropdown
        .as_ref()
        .map_or(0, |dropdown| dropdown.selected);
    let labels = app.dropdown_options(kind);
    if labels.is_empty() {
        return;
    }
    let label_width = labels
        .iter()
        .map(|label| display_width(label))
        .max()
        .unwrap_or(1)
        .saturating_add(4);
    let width = u16::try_from(label_width)
        .unwrap_or(u16::MAX)
        .min(body.width.saturating_sub(2))
        .max(1);
    let height = u16::try_from(labels.len().saturating_add(2))
        .unwrap_or(u16::MAX)
        .min(body.height.saturating_sub(2))
        .max(1);
    let popup = Rect::new(
        body.x.saturating_add(body.width.saturating_sub(width) / 2),
        body.y
            .saturating_add(body.height.saturating_sub(height) / 2),
        width,
        height,
    );
    let visible = usize::from(popup.height.saturating_sub(2)).max(1);
    let start = if selected >= visible {
        selected + 1 - visible
    } else {
        0
    };
    let mut lines = Vec::new();
    for (index, label) in labels.iter().enumerate().skip(start).take(visible) {
        let style = if index == selected {
            Style::default()
                .fg(Color::Black)
                .bg(Color::Cyan)
                .add_modifier(Modifier::BOLD)
        } else {
            Style::default()
        };
        lines.push(Line::styled(
            format!("{} {label}", if index == selected { ">" } else { " " }),
            style,
        ));
        let row = u16::try_from(index - start).unwrap_or(u16::MAX);
        app.hit_targets.push(HitTarget {
            area: Rect::new(
                popup.x.saturating_add(1),
                popup.y.saturating_add(1).saturating_add(row),
                popup.width.saturating_sub(2),
                1,
            ),
            action: MouseAction::DropdownOption(index),
        });
    }
    frame.render_widget(Clear, popup);
    frame.render_widget(
        Paragraph::new(lines).block(Block::default().borders(Borders::ALL).title("选择")),
        popup,
    );
}

fn render_input(frame: &mut ratatui::Frame<'_>, app: &mut App, body: Rect) {
    let Some(input) = &app.input else {
        return;
    };
    let title = format!("编辑: {}", input.title);
    let display = input.buffer.display_text();
    let cursor = input.buffer.cursor();
    let text = input.buffer.text();
    let width = body.width.saturating_sub(2).clamp(1, 90);
    let max_height = body.height.saturating_sub(2).max(1);
    let height = max_height.clamp(3, 12).min(body.height.max(1));
    let popup = Rect::new(
        body.x.saturating_add(body.width.saturating_sub(width) / 2),
        body.y
            .saturating_add(body.height.saturating_sub(height) / 2),
        width,
        height,
    );
    frame.render_widget(Clear, popup);
    frame.render_widget(
        Paragraph::new("").block(Block::default().borders(Borders::ALL).title(title)),
        popup,
    );
    let inner = inner_area(popup, true);
    let inner_width = usize::from(inner.width).max(1);
    let text_height = usize::from(inner.height.saturating_sub(1)).max(1);
    // Wrap the full field text so a narrow terminal still exposes every
    // character; scroll just enough to keep the cursor visible.
    let wrapped = wrap_text(&display, inner_width);
    let prefix = format!("{}▏", &text[..cursor]);
    let cursor_line = wrap_text(&prefix, inner_width).len().saturating_sub(1);
    let scroll = cursor_line.saturating_sub(text_height.saturating_sub(1));
    let visible = wrapped
        .iter()
        .skip(scroll)
        .take(text_height)
        .cloned()
        .collect::<Vec<_>>()
        .join("\n");
    frame.render_widget(
        Paragraph::new(visible),
        Rect::new(
            inner.x,
            inner.y,
            inner.width,
            u16::try_from(text_height).unwrap_or(inner.height.saturating_sub(1)),
        ),
    );
    let y = inner.y.saturating_add(inner.height.saturating_sub(1));
    let apply = "[ 应用 Enter ]";
    let cancel = "[ 取消 Esc ]";
    let apply_width = u16::try_from(display_width(apply)).unwrap_or(u16::MAX);
    let cancel_width = u16::try_from(display_width(cancel)).unwrap_or(u16::MAX);
    let apply_x = inner.x;
    let cancel_x = apply_x
        .saturating_add(apply_width)
        .saturating_add(2)
        .min(inner.right().saturating_sub(cancel_width));
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(
                apply,
                Style::default()
                    .fg(Color::LightGreen)
                    .add_modifier(Modifier::BOLD),
            ),
            Span::raw("  "),
            Span::styled(cancel, Style::default().fg(Color::Red)),
        ])),
        Rect::new(apply_x, y, inner.right().saturating_sub(apply_x), 1),
    );
    app.hit_targets.push(HitTarget {
        area: Rect::new(
            apply_x,
            y,
            apply_width.min(inner.right().saturating_sub(apply_x)),
            1,
        ),
        action: MouseAction::InputApply,
    });
    app.hit_targets.push(HitTarget {
        area: Rect::new(
            cancel_x,
            y,
            cancel_width.min(inner.right().saturating_sub(cancel_x)),
            1,
        ),
        action: MouseAction::InputCancel,
    });
}

fn render_confirmation(frame: &mut ratatui::Frame<'_>, app: &mut App, body: Rect) {
    let Some(confirmation) = &app.confirmation else {
        return;
    };
    let title = confirmation.title.clone();
    let message = confirmation.message.clone();
    let yes = confirmation.yes;
    let action = confirmation.action;
    let width = body.width.saturating_sub(2).clamp(1, 80);
    let inner_width = usize::from(width.saturating_sub(4)).max(10);
    let message_lines = wrap_text(&message, inner_width);
    let height = u16::try_from(message_lines.len().saturating_add(6))
        .unwrap_or(u16::MAX)
        .min(body.height.saturating_sub(2))
        .max(1);
    let popup = Rect::new(
        body.x.saturating_add(body.width.saturating_sub(width) / 2),
        body.y
            .saturating_add(body.height.saturating_sub(height) / 2),
        width,
        height,
    );
    frame.render_widget(Clear, popup);
    frame.render_widget(
        Paragraph::new(message_lines.join("\n"))
            .block(Block::default().borders(Borders::ALL).title(title))
            .wrap(Wrap { trim: false }),
        popup,
    );
    let y = popup.y.saturating_add(popup.height.saturating_sub(2));
    let yes_label = " 是 (Y) ";
    let no_label = " 否 (N) ";
    let yes_style = if yes {
        Style::default()
            .fg(Color::Black)
            .bg(Color::Green)
            .add_modifier(Modifier::BOLD)
    } else {
        Style::default().fg(Color::Green)
    };
    let no_style = if yes {
        Style::default().fg(Color::Red)
    } else {
        Style::default()
            .fg(Color::Black)
            .bg(Color::Red)
            .add_modifier(Modifier::BOLD)
    };
    let yes_width = u16::try_from(display_width(yes_label)).unwrap_or(u16::MAX);
    let no_width = u16::try_from(display_width(no_label)).unwrap_or(u16::MAX);
    let yes_x = popup.x.saturating_add(3);
    let no_x = yes_x.saturating_add(yes_width).saturating_add(2);
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(yes_label, yes_style),
            Span::raw("  "),
            Span::styled(no_label, no_style),
            Span::styled(
                format!("   {}", confirm_action_hint(action)),
                Style::default().fg(Color::DarkGray),
            ),
        ])),
        Rect::new(yes_x, y, popup.right().saturating_sub(yes_x), 1),
    );
    app.hit_targets.push(HitTarget {
        area: Rect::new(
            yes_x,
            y,
            yes_width.min(popup.right().saturating_sub(yes_x)),
            1,
        ),
        action: MouseAction::ConfirmYes,
    });
    app.hit_targets.push(HitTarget {
        area: Rect::new(no_x, y, no_width.min(popup.right().saturating_sub(no_x)), 1),
        action: MouseAction::ConfirmNo,
    });
}

fn confirm_action_hint(action: ConfirmAction) -> &'static str {
    match action {
        ConfirmAction::CancelRun | ConfirmAction::Quit => "Enter 确认当前选择 / Esc 返回",
        ConfirmAction::DiscardExpert => "Enter 确认当前选择 / Esc 返回编辑",
    }
}

fn render_help(frame: &mut ratatui::Frame<'_>, app: &mut App, body: Rect) {
    let lines = help_lines(app);
    let width = body.width.saturating_sub(2).max(1);
    let height = body.height.saturating_sub(2).max(1);
    let popup = Rect::new(
        body.x.saturating_add(1),
        body.y.saturating_add(1),
        width,
        height,
    );
    frame.render_widget(Clear, popup);
    let inner = inner_area(popup, true);
    let viewport = usize::from(inner.height).max(1);
    let max_scroll = lines.len().saturating_sub(viewport);
    app.help_scroll = app.help_scroll.min(max_scroll);
    let visible = lines
        .iter()
        .skip(app.help_scroll)
        .take(viewport)
        .cloned()
        .collect::<Vec<_>>()
        .join("\n");
    frame.render_widget(
        Paragraph::new(visible)
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title("快捷键/行为（F1 或 Esc 关闭）"),
            )
            .wrap(Wrap { trim: false }),
        popup,
    );
}

fn help_lines(app: &App) -> Vec<String> {
    let mut lines = vec![
        "全局".to_owned(),
        "  F1: 帮助（再按 F1/Esc 关闭）".to_owned(),
        "  Ctrl-Q: 退出确认（永不直接退出）".to_owned(),
        "  Ctrl-C: 运行中打开取消确认；未运行打开退出确认".to_owned(),
        "  Ctrl-Y: 切换鼠标捕获/复制模式".to_owned(),
        "  Tab / BackTab: 配置表在行/操作间移动；运行/结果页切换只读页签".to_owned(),
        "  Ctrl-Left/Right 或点击分区标签/上一分区/下一分区: 配置页前后切换".to_owned(),
        "  ↑/↓: 浏览表格行或滚动；←/→: 在同一行内移动单元格，绝不改变取值".to_owned(),
        "  Enter: 编辑文本/打开下拉/激活按钮；Space: 切换勾选、选择或激活按钮".to_owned(),
        "  Esc: 关闭当前弹窗/返回一层；永不取消、永不退出".to_owned(),
        "  鼠标: 点击单元格等同聚焦并 Enter；滚轮滚动；文本弹窗支持应用/取消按钮".to_owned(),
        String::new(),
        "配置".to_owned(),
        "  三段对齐表：参数列固定、值列优先、说明列占余量；窄屏隐藏说明并在选中行详情中显示"
            .to_owned(),
        "  Source → Sensors → Board → Solver → Preflight；分区标签/上一分区/下一分区可直接点击"
            .to_owned(),
        "  Ctrl-P / Ctrl-N：上一页 / 下一页；Ctrl-←/→ 仍可用，但可能被系统桌面快捷键截获"
            .to_owned(),
        "  Sensors: 每相机一行（启用/ID/Topic/模型/数量/Hz/参考）；Hz 按记录时间估算；IMU 独立表"
            .to_owned(),
        "  Solvers: 模式、参考相机、内参策略/初值文件；高级组可折叠，常用项在前".to_owned(),
        "  Board: ID 行与角点映射按当前文本精确保留；Enter 打开完整文本编辑器".to_owned(),
        "  IMU 控件仅在相机-IMU 模式显示；预积分要求单位轴矩阵且 refine=false".to_owned(),
        "  连续时间可选仅 Huber 鲁棒权重（不截断）或像素阈值截断；预积分/视觉固定像素阈值"
            .to_owned(),
        "  YAML 专家编辑: Ctrl-S 应用并校验；非法文本保留且不覆盖表单草稿；Esc 需确认丢弃"
            .to_owned(),
        String::new(),
        "运行".to_owned(),
        "  唯一启动入口在 Preflight；阻断项非零时不会启动".to_owned(),
        "  运行设置是启动时的不可变快照；运行中编辑草稿不影响当前任务".to_owned(),
        "  默认保留运行仪表盘；g 切换检测历史/LM 收敛，r 在完成后查看结果".to_owned(),
        "  F8 或 Ctrl-C: 打开取消确认；确认后等待 worker 在检查点退出".to_owned(),
        String::new(),
        "结果".to_owned(),
        "  Overview / Parameters / Extrinsics / Diagnostics / Raw，↑/↓ 滚动；Esc 返回仪表盘"
            .to_owned(),
        "  F6 打开运行目录；F7 打开 result/extrinsics_3d.html（仅完成后存在）".to_owned(),
        "  F9 [修改配置] 返回预检并保留草稿；F10 [用相同配置重跑] 创建独立 next_run 记录"
            .to_owned(),
        "  Raw 页 Enter/Space 在 calibration.yaml / report.txt / solve.yaml / run.log 间切换"
            .to_owned(),
    ];
    match app.screen {
        Screen::Form(FormSection::Preflight) => {
            lines.push(String::new());
            lines.push("Preflight: Tab/BackTab 在启动/专家按钮间移动；↑/↓/PgUp/PgDn 滚动检查；Ctrl-P/N 切换分区".to_owned());
        }
        Screen::Run => {
            lines.push(String::new());
            lines.push(
                "运行中 Ctrl-Q 会请求取消并在 worker 退出后关闭；g 只切换图表，不改配置".to_owned(),
            );
        }
        _ => {}
    }
    lines
}

fn inner_area(area: Rect, bordered: bool) -> Rect {
    if bordered {
        Rect::new(
            area.x.saturating_add(1),
            area.y.saturating_add(1),
            area.width.saturating_sub(2),
            area.height.saturating_sub(2),
        )
    } else {
        area
    }
}

fn display_width(text: &str) -> usize {
    text.chars()
        .map(|character| if character.is_ascii() { 1 } else { 2 })
        .sum()
}

fn wrap_text(text: &str, width: usize) -> Vec<String> {
    if width == 0 {
        return vec![String::new()];
    }
    let mut lines = Vec::new();
    for paragraph in text.split('\n') {
        if paragraph.is_empty() {
            lines.push(String::new());
            continue;
        }
        let mut current = String::new();
        let mut current_width = 0;
        for character in paragraph.chars() {
            let character_width = if character.is_ascii() { 1 } else { 2 };
            if current_width + character_width > width && !current.is_empty() {
                lines.push(std::mem::take(&mut current));
                current_width = 0;
            }
            current.push(character);
            current_width += character_width;
        }
        if !current.is_empty() {
            lines.push(current);
        }
    }
    if lines.is_empty() {
        lines.push(String::new());
    }
    lines
}

#[allow(
    dead_code,
    reason = "kept as the single source for dropdown metadata in views"
)]
fn dropdown_kind_title(kind: DropdownKind) -> &'static str {
    match kind {
        DropdownKind::Format => "bag 格式",
        DropdownKind::Mode => "标定模式",
        DropdownKind::Reference => "参考相机",
        DropdownKind::IntrinsicStrategy => "内参策略",
        DropdownKind::CameraModel(_) => "相机模型",
        DropdownKind::BoardFamily => "tag 家族",
        DropdownKind::Refinement => "角点细化",
        DropdownKind::InertialMethod => "惯性方法",
        DropdownKind::TimeOffsetKind => "时间偏移",
        DropdownKind::Rejection => "截断策略",
    }
}

#[allow(
    dead_code,
    reason = "result status helper kept next to result rendering"
)]
fn status_text(status: Option<RunStatus>) -> &'static str {
    match status {
        Some(RunStatus::Complete) => "成功",
        Some(RunStatus::Failed) => "失败",
        Some(RunStatus::Cancelled) => "已取消",
        Some(RunStatus::ClosedWithoutCalibration) => "未执行",
        Some(RunStatus::InProgress) | None => "进行中",
    }
}
