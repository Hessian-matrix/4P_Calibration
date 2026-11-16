//! Elapsed-time attribution. Parallel worker durations overlap; none are OS CPU time.
use ccl_image::apriltag::AprilTagWorkspaceStats;
use serde::Serialize;

#[derive(Clone, Debug, Default, Serialize)]
pub struct PipelinePerformance {
    pub workers: usize,
    pub open_ms: f64,
    pub visit_ms: f64,
    pub finish_ms: f64,
    pub total_ms: f64,
    pub input_prepare_ms: f64,
    pub input_payload_bytes: u64,
    /// Blocking submission, including normal channel send overhead; not pure lock wait.
    pub submit_ms: f64,
    /// Includes waiting to acquire the receiver mutex and waiting for a task.
    pub worker_wait_ms: f64,
    pub completion_lag_ms: f64,
    pub completion_lag_max_ms: f64,
    pub decode_grayscale_worker_ms: f64,
    pub detection_worker_ms: f64,
    pub preview_worker_ms: f64,
    pub preview_count: usize,
    pub preview_bytes: u64,
    /// Maximum retained detector + decoder capacity observed for one worker, not RSS.
    pub max_worker_retained_bytes: usize,
    pub core_worker_ms: CoreStageTimes,
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct CoreStageTimes {
    pub input: f64,
    pub segmentation: f64,
    pub blur: f64,
    pub gradients: f64,
    pub edges_sort: f64,
    pub merge: f64,
    pub fit: f64,
    pub link: f64,
    pub search: f64,
    pub decode: f64,
    pub refinement: f64,
    pub total: f64,
}

impl CoreStageTimes {
    pub(super) fn record(&mut self, stats: &AprilTagWorkspaceStats) {
        self.input += stats.input.as_secs_f64() * 1000.0;
        self.segmentation += stats.segmentation.as_secs_f64() * 1000.0;
        self.blur += stats.segmentation_blur.as_secs_f64() * 1000.0;
        self.gradients += stats.segmentation_gradients.as_secs_f64() * 1000.0;
        self.edges_sort += stats.segmentation_edges.as_secs_f64() * 1000.0;
        self.merge += stats.segmentation_merge.as_secs_f64() * 1000.0;
        self.fit += stats.segmentation_fit.as_secs_f64() * 1000.0;
        self.link += stats.segmentation_link.as_secs_f64() * 1000.0;
        self.search += stats.segmentation_search.as_secs_f64() * 1000.0;
        self.decode += stats.decode.as_secs_f64() * 1000.0;
        self.refinement += stats.refinement.as_secs_f64() * 1000.0;
        self.total += stats.total.as_secs_f64() * 1000.0;
    }
}
