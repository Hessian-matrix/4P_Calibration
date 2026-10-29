use std::{env, path::PathBuf, process::Command};

const CRATES: &[&str] = &["ccl_calib", "ccl_geometry", "ccl_image", "ccl_rosbag"];
const SHARED_CRATES: &[&str] = &["anyhow", "image", "serde", "serde_json", "serde_yaml"];


fn main() {
    let mut args = env::args_os().skip(1);
    let rustc = args.next().expect("rustc wrapper requires the real rustc path");
    let mut rustc_args: Vec<_> = args.collect();
    let bundle = PathBuf::from(env::var_os("FOUR_P_CORELIB_BUNDLE").expect(
        "FOUR_P_CORELIB_BUNDLE is required; run tools/build.py",
    ));
    for crate_name in SHARED_CRATES {
        let prefix = format!("lib{crate_name}-");
        let matches: Vec<_> = std::fs::read_dir(bundle.join("deps"))
            .expect("read bundle deps")
            .filter_map(Result::ok)
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name().is_some_and(|name| name.to_string_lossy().starts_with(&prefix))
                    && path.extension().is_some_and(|extension| extension == "rlib")
            })
            .collect();
        assert_eq!(matches.len(), 1, "bundle must contain exactly one {crate_name} rlib");
        for index in 0..rustc_args.len().saturating_sub(1) {
            if rustc_args[index] == "--extern"
                && rustc_args[index + 1].to_string_lossy().starts_with(&format!("{crate_name}="))
            {
                rustc_args[index + 1] = format!("{crate_name}={}", matches[0].display()).into();
            }
        }
    }
    rustc_args.splice(
        0..0,
        [
            "-L".into(),
            format!("dependency={}", bundle.join("deps").display()).into(),
            "-L".into(),
            format!("dependency={}", bundle.join("host-deps").display()).into(),
        ],
    );
    if env::var_os("CARGO_PRIMARY_PACKAGE").as_deref() == Some("1".as_ref()) {
        for crate_name in CRATES {
            rustc_args.push("--extern".into());
            rustc_args.push(
                format!("{crate_name}={}", bundle.join("lib").join(format!("lib{crate_name}.rlib")).display()).into(),
            );
        }
    }
    let status = Command::new(rustc)
        .args(rustc_args)
        .status()
        .expect("failed to execute rustc");
    std::process::exit(status.code().unwrap_or(1));
}
