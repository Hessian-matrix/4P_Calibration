#!/usr/bin/env python3
"""Build the real browser engine and publish a content-addressed local manifest.

Run from any directory. Does not deploy, upload, or change the registry source.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import tomllib
from package_web import verify_generated_output


def generated_files(staging: Path, profile: str) -> list[Path]:
    files: dict[str, Path] = {}
    for candidate in staging.rglob("*"):
        if candidate.is_symlink():
            raise SystemExit(f"Generated engine contains a symlink: {candidate}")
        if candidate.is_file():
            relative = candidate.relative_to(staging).as_posix()
            files[relative] = candidate
    verify_generated_output(files, profile, staging)
    return [files[name] for name in sorted(files)]


def verify_immutable_build(output: Path, entries: list[dict[str, str]]) -> None:
    expected = {entry["url"]: entry for entry in entries}
    actual: dict[str, Path] = {}
    actual_dirs: set[str] = set()
    for candidate in output.rglob("*"):
        if candidate.is_symlink():
            raise SystemExit(f"Immutable build directory contains a symlink: {candidate}")
        relative = candidate.relative_to(output).as_posix()
        if candidate.is_file():
            actual[relative] = candidate
        elif candidate.is_dir():
            actual_dirs.add(relative)
        else:
            raise SystemExit(f"Immutable build directory contains an unsupported entry: {candidate}")
    expected_dirs = {
        parent.as_posix()
        for relative in expected
        for parent in Path(relative).parents
        if parent.as_posix() != "."
    }
    if set(actual) != set(expected) or actual_dirs != expected_dirs:
        raise SystemExit(
            f"Immutable build directory has missing or extra entries: {output}; "
            f"missing={sorted(set(expected) - set(actual))}, extra={sorted(set(actual) - set(expected))}"
        )
    for relative, existing in actual.items():
        content = existing.read_bytes()
        entry = expected[relative]
        if len(content) != int(entry["bytes"]) or hashlib.sha256(content).hexdigest() != entry["sha256"]:
            raise SystemExit(f"Immutable build directory differs: {existing}")


ROOT = Path(__file__).resolve().parents[1]


def run(*args: str) -> None:
    subprocess.run(args, cwd=ROOT, check=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--debug", action="store_true", help="Build without release optimization")
    parser.add_argument("--threads", action="store_true", help="Build the explicit shared-memory CPU profile")
    parser.add_argument("--toolchain", default="nightly-2026-11-01", help="Pinned threaded build toolchain")
    args = parser.parse_args()
    lock = tomllib.loads((ROOT / "Cargo.lock").read_text())
    version = next(p["version"] for p in lock["package"] if p["name"] == "wasm-bindgen")
    installed = subprocess.check_output(["wasm-bindgen", "--version"], text=True).strip()
    if installed != f"wasm-bindgen {version}":
        raise SystemExit(f"wasm-bindgen CLI mismatch: need {version}; found {installed}")
    command = ["cargo"]
    flags = "-C target-feature=+simd128"
    if args.threads:
        command.append(f"+{args.toolchain}")
        flags = "-C target-feature=+simd128,+atomics,+bulk-memory -C link-arg=--shared-memory -C link-arg=--max-memory=1073741824 -C link-arg=--import-memory -C link-arg=--export=__wasm_init_tls -C link-arg=--export=__tls_size -C link-arg=--export=__tls_align -C link-arg=--export=__tls_base"
    os.environ["RUSTFLAGS"] = f"{os.environ.get('RUSTFLAGS', '')} {flags}".strip()
    command.extend(["build", "--locked", "--lib", "--target", "wasm32-unknown-unknown"])
    if args.threads:
        command.extend(["--features", "wasm-threads", "-Z", "build-std=panic_abort,std"])
    if not args.debug:
        command.append("--release")
    run(*command)
    target = Path(os.environ.get("CARGO_TARGET_DIR", ROOT / "target"))
    if not target.is_absolute():
        target = ROOT / target
    wasm = target / "wasm32-unknown-unknown" / ("debug" if args.debug else "release") / "four_p_calibration.wasm"
    destination = ROOT / "web" / "app" / "engine"
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="4p-engine-", dir=destination) as temporary:
        staging = Path(temporary)
        run("wasm-bindgen", str(wasm), "--target", "web", "--out-dir", str(staging), "--out-name", "four_p_calibration", "--no-typescript")
        profile = "cpu-rayon" if args.threads else "cpu-single"
        files = generated_files(staging, profile)
        digest = hashlib.sha256()
        entries = []
        for file in files:
            relative = file.relative_to(staging).as_posix()
            content = file.read_bytes()
            sha256 = hashlib.sha256(content).hexdigest()
            digest.update(relative.encode() + b"\0" + bytes.fromhex(sha256))
            role = "glue" if relative == "four_p_calibration.js" else "wasm" if relative == "four_p_calibration_bg.wasm" else "module"
            entries.append({"role": role, "url": relative, "bytes": str(len(content)), "sha256": sha256})
        build_id = digest.hexdigest()
        release = tomllib.loads((ROOT / "Cargo.toml").read_text())["package"]["version"]
        output = destination / build_id
        if output.exists():
            if not output.is_dir() or output.is_symlink():
                raise SystemExit(f"Immutable build path is not a directory: {output}")
            verify_immutable_build(output, entries)
        else:
            # Same-filesystem rename makes every engine generation visible as one directory.
            staging.rename(output)
        for entry in entries:
            entry["url"] = f"{build_id}/{entry['url']}"
        manifest = {"manifestVersion": 1, "releaseId": release, "buildId": build_id, "engineApi": "fourp.engine.v1", "executionProfile": profile, "resources": entries}
        manifest_name = "manifest-threads.json" if args.threads else "manifest.json"
        pending = destination / f"{manifest_name}.tmp"
        pending.write_text(json.dumps(manifest, indent=2) + "\n")
        pending.replace(destination / manifest_name)
        print(f"Built engine {build_id}; {sum(int(item['bytes']) for item in entries)} bytes")


if __name__ == "__main__":
    main()
