use std::{env, path::PathBuf, process::Command};

const CRATES: &[&str] = &["ccl_calib", "ccl_geometry", "ccl_image", "ccl_rosbag"];

fn main() {
    let mut args = env::args_os().skip(1);
    let rustc = args.next().expect("rustc wrapper requires the real rustc path");
    let mut rustc_args: Vec<_> = args.collect();
    let bundle = PathBuf::from(env::var_os("FOUR_P_CORELIB_BUNDLE").expect(
        "FOUR_P_CORELIB_BUNDLE is required; run tools/build.py",
    ));
    if env::var_os("CARGO_PRIMARY_PACKAGE").as_deref() == Some("1".as_ref()) {
        rustc_args.push("-L".into());
        rustc_args.push(format!("dependency={}", bundle.join("deps").display()).into());
        rustc_args.push("-L".into());
        rustc_args.push(format!("dependency={}", bundle.join("host-deps").display()).into());
        for crate_name in CRATES {
            rustc_args.push("--extern".into());
            rustc_args.push(
                format!(
                    "{crate_name}={}",
                    bundle.join("lib").join(format!("lib{crate_name}.rlib")).display()
                )
                .into(),
            );
        }
    }
    let status = Command::new(rustc)
        .args(rustc_args)
        .status()
        .expect("failed to execute rustc");
    std::process::exit(status.code().unwrap_or(1));
}
