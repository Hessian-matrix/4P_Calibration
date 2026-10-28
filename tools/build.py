#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

REQUIRED_MANIFEST = {
    "format": "2",
    "package": "camera_corelib",
    "version": "0.6.0",
    "git_commit": "d279273e2ae6bd7a21738d108837ded12fc6c64a",
    "workspace_features": "all-features",
}
CORELIB_CRATES = ("ccl_calib", "ccl_geometry", "ccl_image", "ccl_rosbag")


def parse_manifest(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if not separator or not key or key in values:
            raise SystemExit(f"invalid bundle manifest line: {line!r}")
        values[key] = value
    return values


def rustc_info() -> dict[str, str]:
    output = subprocess.check_output(["rustc", "-vV"], text=True)
    values: dict[str, str] = {}
    for line in output.splitlines():
        key, separator, value = line.partition(": ")
        if separator:
            values[key.lower().replace("-", "_").replace(" ", "_")] = value
    return values

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

def resolve_bundle(path: Path) -> tuple[Path, Path | None]:
    if path.is_dir():
        return path.resolve(), None
    if not path.name.endswith(".tar.gz"):
        raise SystemExit("bundle must be an extracted directory or .tar.gz archive")
    work = Path(tempfile.mkdtemp(prefix="four-p-corelib."))
    with tarfile.open(path, "r:gz") as archive:
        members = archive.getmembers()
        roots = {member.name.split("/", 1)[0] for member in members if member.name}
        if len(roots) != 1 or any(
            ".." in Path(member.name).parts or member.issym() or member.islnk()
            for member in members
        ):
            raise SystemExit("unsafe corelib archive layout")
        archive.extractall(work, filter="data")
    return (work / roots.pop()).resolve(), work


def validate_bundle(bundle: Path, target: str) -> dict[str, str]:
    manifest = parse_manifest(bundle / "MANIFEST.txt")
    for key, expected in REQUIRED_MANIFEST.items():
        if manifest.get(key) != expected:
            raise SystemExit(f"bundle {key} must be {expected!r}, got {manifest.get(key)!r}")
    current = rustc_info()
    for key in ("release", "commit_hash", "commit_date", "llvm_version"):
        if manifest.get(f"rustc_{key}") != current.get(key):
            raise SystemExit(f"rustc {key} does not match bundle manifest")
    if manifest.get("target") != target:
        raise SystemExit(f"bundle target {manifest.get('target')!r} does not match requested target {target!r}")
    for crate in CORELIB_CRATES:
        if not (bundle / "lib" / f"lib{crate}.rlib").is_file():
            raise SystemExit(f"bundle is missing lib{crate}.rlib")
    if not (bundle / "deps").is_dir() or not (bundle / "host-deps").is_dir():
        raise SystemExit("bundle is missing dependency directories")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Build 4P with a verified camera_corelib rlib bundle.")
    parser.add_argument("--bundle", type=Path, default=os.environ.get("FOUR_P_CORELIB_BUNDLE"))
    parser.add_argument("--release", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--target")
    parser.add_argument("cargo_args", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.bundle is None:
        raise SystemExit("pass --bundle or set FOUR_P_CORELIB_BUNDLE")
    requested_bundle = args.bundle.resolve()
    bundle, temporary_root = resolve_bundle(requested_bundle)
    target = args.target or rustc_info()["host"]
    manifest = validate_bundle(bundle, target)
    env = os.environ.copy()
    env["FOUR_P_CORELIB_BUNDLE"] = str(bundle)
    wrapper_name = "corelib-rustc-wrapper.exe" if os.name == "nt" else "corelib-rustc-wrapper.py"
    env["RUSTC_WRAPPER"] = str((Path(__file__).parent / wrapper_name).resolve())
    for key in ("version", "git_commit", "target", "rustc_release", "workspace_features"):
        env[f"FOUR_P_CORELIB_{key.upper()}"] = manifest[key]
    env["FOUR_P_CORELIB_ARTIFACT_SHA256"] = (
        sha256(requested_bundle) if requested_bundle.is_file() else sha256(bundle / "MANIFEST.txt")
    )
    rustdoc_flags = [
        "-L", f"dependency={bundle / 'deps'}",
        "-L", f"dependency={bundle / 'host-deps'}",
    ]
    for crate in CORELIB_CRATES:
        rustdoc_flags.extend(["--extern", f"{crate}={bundle / 'lib' / f'lib{crate}.rlib'}"])
    existing_rustdoc_flags = env.get("RUSTDOCFLAGS")
    env["RUSTDOCFLAGS"] = " ".join(
        ([existing_rustdoc_flags] if existing_rustdoc_flags else []) + rustdoc_flags
    )
    command = ["cargo", "test" if args.test else "build", "--locked"]
    if args.release:
        command.append("--release")
    if args.target is not None:
        command.extend(["--target", args.target])
    command.extend(args.cargo_args)
    try:
        return subprocess.run(command, env=env, check=False).returncode
    finally:
        if temporary_root is not None:
            shutil.rmtree(temporary_root, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())
