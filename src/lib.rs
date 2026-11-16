#![allow(
    clippy::missing_errors_doc,
    reason = "application-internal modules are not a published library API"
)]

#[cfg(not(target_arch = "wasm32"))]
pub mod cli;
pub mod config;
pub mod detection;
pub mod export;
pub mod input;
pub mod run_record;
pub mod runner;
pub mod trajectory;
#[cfg(not(target_arch = "wasm32"))]
pub mod tui;

#[cfg(target_arch = "wasm32")]
mod browser;
