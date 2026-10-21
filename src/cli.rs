use std::path::PathBuf;

use clap::Parser;

#[derive(Clone, Debug, Parser)]
#[command(
    name = "four-p-calibration",
    version,
    about = "Offline camera calibration TUI"
)]
pub struct Cli {
    /// Run without the TUI. Requires --config.
    #[arg(long, requires = "config")]
    pub no_tui: bool,

    /// Load run settings from YAML. Optional for the TUI.
    #[arg(long, value_name = "FILE")]
    pub config: Option<PathBuf>,
}
