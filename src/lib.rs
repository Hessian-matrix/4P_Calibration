#![allow(
    clippy::missing_errors_doc,
    reason = "application-internal modules are not a published library API"
)]

pub mod cli;
pub mod config;
pub mod detection;
pub mod export;
pub mod input;
pub mod run_record;
pub mod runner;
pub mod trajectory;
pub mod tui;
