use anyhow::{Context, Result};
use clap::Parser;
use four_p_calibration::cli::Cli;
use four_p_calibration::config::RunSettings;
use four_p_calibration::run_record::{RunContext, RunStatus};
use four_p_calibration::{runner, tui};

fn main() -> Result<()> {
    let cli = Cli::parse();
    let settings = cli
        .config
        .as_deref()
        .map(RunSettings::load)
        .transpose()
        .context("load run settings")?;
    let run = RunContext::create().context("create run record")?;

    if cli.no_tui {
        let settings = settings.expect("clap requires --config with --no-tui");
        let cancelled = || false;
        let mut emit = |event| match event {
            runner::Event::Stage(stage) => println!("stage={stage}"),
            runner::Event::Message(message) => println!("{message}"),
            runner::Event::DetectionProgress(progress) if progress.frames_processed % 25 == 0 => {
                println!(
                    "camera={} frames={} timestamp_ns={} tags={} points={} fps={:.1} avg_fps={:.1}",
                    progress.camera_id,
                    progress.frames_processed,
                    progress.timestamp_ns,
                    progress.tags,
                    progress.points,
                    progress.frames_per_second,
                    progress.average_frames_per_second,
                );
            }
            runner::Event::SolveProgress(progress) => println!(
                "iteration={}/{} cost={:.9e} damping={:.3e} gradient={:.3e} accepted={} rejected={}",
                progress.iteration,
                progress.max_iterations,
                progress.cost,
                progress.damping,
                progress.scaled_gradient_inf_norm,
                progress.accepted_steps,
                progress.rejected_steps,
            ),
            runner::Event::SolveActivity(activity) => println!(
                "activity={:?} completed={:?} total={:?} iteration={:?} cancellable={}",
                activity.phase,
                activity.completed,
                activity.total,
                activity.iteration,
                activity.cancellable,
            ),
            runner::Event::DetectionProgress(_)
            | runner::Event::Dataset(_)
            | runner::Event::SolveSummary(_) => {}
        };
        match runner::execute(&settings, &run, &cancelled, &mut emit) {
            Ok(_) => {
                run.finish(RunStatus::Complete, None)?;
                println!(
                    "result={}",
                    run.root().join("result/calibration.yaml").display()
                );
                Ok(())
            }
            Err(error) => {
                run.log("ERROR", &format!("{error:#}"))?;
                run.finish(RunStatus::Failed, Some("calibration"))?;
                Err(error)
            }
        }
    } else if let Err(error) = tui::run(settings, &run) {
        let _ = run.log("ERROR", &format!("TUI failed: {error:#}"));
        let _ = run.finish(RunStatus::Failed, Some("tui"));
        Err(error)
    } else {
        Ok(())
    }
}
