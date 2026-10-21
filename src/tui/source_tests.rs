use super::*;

struct Fixture(App);
impl Fixture {
    fn new() -> Self {
        let settings = RunSettings::load(Path::new("config/presets/baton-bag-ds.yaml")).unwrap();
        Self(App::new(Some(&settings), RunContext::create().unwrap()))
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        std::fs::remove_dir_all(self.0.base_run.root()).unwrap();
    }
}

fn delivered_scan(app: &mut App, generation: u64, source: &str, topic: &str) {
    let (sender, receiver) = mpsc::sync_channel(1);
    sender
        .send(Ok(BagInspection {
            source: source.into(),
            topics: vec![crate::input::TopicInspection {
                name: topic.to_owned(),
                type_name: "sensor_msgs/Image".to_owned(),
                message_count: 1,
                first_record_time_ns: Some(0),
                last_record_time_ns: Some(0),
                header_time_valid: true,
                status: "ok".to_owned(),
            }],
        }))
        .unwrap();
    app.inspect_worker = Some(InspectWorker {
        receiver,
        handle: None,
        source_generation: generation,
    });
}

#[test]
fn changing_bag_clears_old_bindings_and_only_shows_new_topics() {
    let mut fixture = Fixture::new();
    let app = &mut fixture.0;
    app.started_once = true;
    let board = app.draft.target.id_rows_text.clone();
    app.apply_input(&FieldId::SourcePath, "/new.bag");
    assert!(app.draft.cameras.is_empty());
    assert!(app.draft.imu_topic.is_none());
    assert!(app.draft.reference_id.is_empty());
    assert!(app.seen_image_topics.is_empty());
    assert_eq!(app.draft.target.id_rows_text, board);
    delivered_scan(app, app.source_generation, "/new.bag", "/new/image");
    app.poll_inspect();
    assert_eq!(app.seen_image_topics, ["/new/image"]);
}

#[test]
fn stale_scan_cannot_return_after_source_changes_away_and_back() {
    let mut fixture = Fixture::new();
    let app = &mut fixture.0;
    let original = app.draft.source_path.clone();
    let generation = app.source_generation;
    app.apply_input(&FieldId::SourcePath, "/other.bag");
    app.apply_input(&FieldId::SourcePath, &original);
    delivered_scan(app, generation, &original, "/obsolete/image");
    app.poll_inspect();
    assert!(app.inspection.is_none());
    assert!(app.seen_image_topics.is_empty());
}

#[test]
fn format_change_rejects_pending_scan_for_same_path() {
    let mut fixture = Fixture::new();
    let app = &mut fixture.0;
    let original = app.draft.source_path.clone();
    let generation = app.source_generation;
    let other = (draft::bag_format_index(app.draft.format) + 1) % BAG_FORMATS.len();
    app.apply_dropdown(DropdownKind::Format, other);
    delivered_scan(app, generation, &original, "/obsolete/image");
    app.poll_inspect();
    assert!(app.inspection.is_none());
    assert!(app.draft.cameras.is_empty());
    assert!(app.draft.imu_topic.is_none());
}

#[test]
fn yaml_source_change_keeps_only_explicit_new_bindings() {
    let mut fixture = Fixture::new();
    let app = &mut fixture.0;
    let mut settings = app.draft.to_settings().unwrap();
    settings.source.path = "/replacement.bag".into();
    for camera in &mut settings.cameras {
        camera.topic = format!("/new/{}", camera.id);
    }
    settings.source.imu_topic = Some("/new/imu".to_owned());
    app.open_expert();
    app.expert.as_mut().unwrap().buffer =
        TextBuffer::new(serde_yaml::to_string(&settings).unwrap());
    app.save_expert();
    assert_eq!(app.seen_image_topics, ["/new/cam0", "/new/cam1"]);
    assert_eq!(app.draft.imu_topic.as_deref(), Some("/new/imu"));
    assert!(app.inspection.is_none());
}

#[test]
fn ascii_page_shortcuts_navigate_without_leaving_source_editor() {
    let mut fixture = Fixture::new();
    let app = &mut fixture.0;
    app.handle_key(KeyCode::Char('n'), KeyModifiers::CONTROL);
    assert_eq!(app.screen, Screen::Form(FormSection::Sensors));
    app.handle_key(KeyCode::Char('p'), KeyModifiers::CONTROL);
    assert_eq!(app.screen, Screen::Form(FormSection::Source));
    let source = app.draft.source_path.clone();
    app.open_input(&FieldId::SourcePath, "bag 路径".to_owned());
    app.handle_key(KeyCode::Char('n'), KeyModifiers::CONTROL);
    app.handle_key(KeyCode::Char('p'), KeyModifiers::CONTROL);
    assert_eq!(app.screen, Screen::Form(FormSection::Source));
    assert!(app.input.is_some());
    app.handle_key(KeyCode::Enter, KeyModifiers::NONE);
    assert_eq!(app.draft.source_path, source);
}
