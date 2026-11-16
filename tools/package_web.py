#!/usr/bin/env python3
"""Verify the CPU browser engines and assemble an allowlisted static site.

This command only copies files. It does not upload, publish, or invoke a build.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from html.parser import HTMLParser
from pathlib import Path
import re
import shutil
import tempfile
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "web" / "app"
ENGINE_API = "fourp.engine.v1"
PROFILES = {
    "manifest.json": "cpu-single",
    "manifest-threads.json": "cpu-rayon",
}
SHELL_FILES = (
    "index.html",
    "style.css",
    "app.js",
    "browser-api.js",
    "coi-serviceworker.js",
    "configuration-forms.js",
    "controller.js",
    "convergence-view.js",
    "engine-worker.js",
    "extrinsics-view.js",
    "frame-history.js",
    "isolation.js",
    "loader.js",
    "results-view.js",
    "stage-timing.js",
)
VENDOR_FILES = (
    "vendor/three-0.160.0/LICENSE",
    "vendor/three-0.160.0/OrbitControls.js",
    "vendor/three-0.160.0/three.module.js",
)
HEX_64 = re.compile(r"[0-9a-f]{64}")
DECIMAL = re.compile(r"0|[1-9][0-9]*")
HELPER_PATH = re.compile(
    r"snippets/wasm-bindgen-rayon-[0-9a-f]+/src/workerHelpers\.no-bundler\.js"
)


class _CspMetaParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.policies: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        if tag.lower() != "meta":
            return
        values = {name.lower(): value for name, value in attrs}
        if (values.get("http-equiv") or "").lower() == "content-security-policy":
            content = values.get("content")
            if content is not None:
                self.policies.append(content)


def verify_csp(app: Path) -> None:
    index = app / "index.html"
    worker = app / "coi-serviceworker.js"
    try:
        parser = _CspMetaParser()
        parser.feed(index.read_text(encoding="utf-8"))
        worker_lines = worker.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        abort(f"cannot read CSP sources: {error}")
    if len(parser.policies) != 1:
        abort(f"index must contain exactly one CSP meta policy: {index}")
    prefix = "const CONTENT_SECURITY_POLICY = "
    declarations = [line[len(prefix):-1] for line in worker_lines if line.startswith(prefix) and line.endswith(";")]
    if len(declarations) != 1:
        abort(f"service worker must contain exactly one canonical CSP declaration: {worker}")
    try:
        worker_policy = json.loads(declarations[0])
    except json.JSONDecodeError as error:
        abort(f"service worker CSP must be a JSON string literal: {worker}: {error}")
    if not isinstance(worker_policy, str) or worker_policy != parser.policies[0]:
        abort(f"HTML and service worker CSP policies differ: {index}, {worker}")


def abort(message: str) -> None:
    raise SystemExit(message)


def load_manifest(path: Path) -> dict:
    if not path.is_file() or path.is_symlink():
        abort(f"missing engine manifest: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        abort(f"invalid engine manifest {path}: {error}")
    if not isinstance(value, dict):
        abort(f"engine manifest must be an object: {path}")
    return value


def resource_path(engine: Path, build_id: str, url: object, manifest: Path) -> tuple[str, Path]:
    if not isinstance(url, str) or not url or "\\" in url or "%" in url:
        abort(f"invalid resource URL in {manifest}: {url!r}")
    parsed = urlsplit(url)
    parts = parsed.path.split("/")
    if parsed.scheme or parsed.netloc or parsed.query or parsed.fragment or parsed.path.startswith("/"):
        abort(f"resource URL is not a plain relative path in {manifest}: {url}")
    if len(parts) < 2 or parts[0] != build_id or any(part in {"", ".", ".."} for part in parts):
        abort(f"resource URL escapes or does not match buildId in {manifest}: {url}")
    relative = "/".join(parts[1:])
    path = engine.joinpath(*parts)
    if path.is_symlink() or not path.is_file():
        abort(f"missing engine resource declared by {manifest}: {path}")
    try:
        path.resolve().relative_to(engine.resolve())
    except ValueError:
        abort(f"engine resource escapes its directory in {manifest}: {url}")
    return relative, path


def verify_generated_output(files: dict[str, Path], profile: str, source: Path) -> None:
    """Accept only the known wasm-bindgen no-bundler dependency shape.

    This deliberately is not a general JavaScript parser. Any generated file set or
    dependency-bearing construct outside the pinned shape is rejected.
    """
    glue_name = "four_p_calibration.js"
    wasm_name = "four_p_calibration_bg.wasm"
    helpers = [name for name in files if HELPER_PATH.fullmatch(name)]
    expected = {glue_name, wasm_name}
    if profile == "cpu-rayon":
        if len(helpers) != 1:
            abort(f"threaded engine must contain exactly one supported worker helper: {source}")
        expected.add(helpers[0])
    elif helpers:
        abort(f"single-threaded engine unexpectedly contains a worker helper: {source}")
    if set(files) != expected:
        extra = sorted(set(files) - expected)
        missing = sorted(expected - set(files))
        abort(f"unsupported generated engine file set in {source}; missing={missing}, extra={extra}")

    try:
        glue = files[glue_name].read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        abort(f"invalid generated glue {files[glue_name]}: {error}")
    wasm_reference = "new URL('four_p_calibration_bg.wasm', import.meta.url)"
    if glue.count(wasm_reference) != 1:
        abort(f"generated glue lacks its one canonical WASM dependency: {files[glue_name]}")

    allowed_import = None
    if helpers:
        allowed_import = f"import {{ startWorkers }} from './{helpers[0]}';"
        if glue.splitlines()[:1] != [allowed_import]:
            abort(f"unsupported threaded glue import shape: {files[glue_name]}")
    dependency_tokens = re.findall(r"\bimport\s*(?:\(|\{|\*|['\"])|\bfetch\s*\(|\bnew\s+Worker\s*\(", glue)
    expected_tokens = ["import {"] if helpers else []
    expected_tokens.append("fetch(")  # wasm-bindgen fetches the canonical URL supplied above.
    if dependency_tokens != expected_tokens:
        abort(f"unsupported dependency-bearing construct in generated glue: {files[glue_name]}")

    if helpers:
        helper_path = files[helpers[0]]
        try:
            helper = helper_path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as error:
            abort(f"invalid generated worker helper {helper_path}: {error}")
        required = (
            "const pkg = await import(data.mainJS);",
            "mainJS: builder.mainJS()",
            "fetch(import.meta.url)",
            "new Worker(url, {",
        )
        if any(helper.count(fragment) != 1 for fragment in required):
            abort(f"unsupported worker helper dependency shape: {helper_path}")
        tokens = re.findall(r"\bimport\s*(?:\(|\{|\*|['\"])|\bfetch\s*\(|\bnew\s+Worker\s*\(", helper)
        if tokens != ["import(", "fetch(", "new Worker("]:
            abort(f"unsupported dependency-bearing construct in worker helper: {helper_path}")


def verify_manifest(engine: Path, name: str, expected_profile: str) -> tuple[str, tuple[str, ...]]:
    path = engine / name
    manifest = load_manifest(path)
    if set(manifest) != {"manifestVersion", "releaseId", "buildId", "engineApi", "executionProfile", "resources"}:
        abort(f"unsupported engine manifest fields: {path}")
    if manifest.get("manifestVersion") != 1 or manifest.get("engineApi") != ENGINE_API:
        abort(f"unsupported engine manifest schema/API: {path}")
    if manifest.get("executionProfile") != expected_profile:
        abort(f"wrong executionProfile in {path}: expected {expected_profile!r}")
    if not isinstance(manifest.get("releaseId"), str) or not manifest["releaseId"]:
        abort(f"missing releaseId in {path}")
    build_id = manifest.get("buildId")
    if not isinstance(build_id, str) or HEX_64.fullmatch(build_id) is None:
        abort(f"invalid buildId in {path}")
    resources = manifest.get("resources")
    if not isinstance(resources, list) or not resources:
        abort(f"resources must be a non-empty array in {path}")

    files: dict[str, Path] = {}
    roles: dict[str, str] = {}
    digest = hashlib.sha256()
    for index, resource in enumerate(resources):
        if not isinstance(resource, dict) or set(resource) != {"role", "url", "bytes", "sha256"}:
            abort(f"resource {index} has unsupported fields in {path}")
        url = resource.get("url")
        relative, file = resource_path(engine, build_id, url, path)
        if relative in files:
            abort(f"duplicate resource URL in {path}: {url}")
        files[relative] = file

        declared_bytes = resource.get("bytes")
        if not isinstance(declared_bytes, str) or DECIMAL.fullmatch(declared_bytes) is None:
            abort(f"invalid byte length for {url} in {path}")
        actual = file.read_bytes()
        if len(actual) != int(declared_bytes):
            abort(f"resource length mismatch for {file}: expected {declared_bytes}, got {len(actual)}")
        declared_hash = resource.get("sha256")
        if not isinstance(declared_hash, str) or HEX_64.fullmatch(declared_hash) is None:
            abort(f"invalid SHA-256 for {url} in {path}")
        actual_hash = hashlib.sha256(actual).hexdigest()
        if actual_hash != declared_hash:
            abort(f"resource SHA-256 mismatch for {file}: expected {declared_hash}, got {actual_hash}")
        digest.update(relative.encode("utf-8") + b"\0" + bytes.fromhex(actual_hash))
        roles[relative] = resource.get("role")

    if list(files) != sorted(files):
        abort(f"resources are not in canonical path order: {path}")
    if digest.hexdigest() != build_id:
        abort(f"buildId does not match canonical resource digest: {path}")
    expected_roles = {
        relative: "glue" if relative == "four_p_calibration.js" else "wasm" if relative == "four_p_calibration_bg.wasm" else "module"
        for relative in files
    }
    if roles != expected_roles:
        abort(f"manifest contains non-canonical resource roles: {path}")

    build = engine / build_id
    actual_files: set[str] = set()
    actual_dirs: set[str] = set()
    for candidate in build.rglob("*"):
        if candidate.is_symlink():
            abort(f"symlink is not allowed in immutable build: {candidate}")
        relative = candidate.relative_to(build).as_posix()
        if candidate.is_file():
            actual_files.add(relative)
        elif candidate.is_dir():
            actual_dirs.add(relative)
        else:
            abort(f"unsupported filesystem entry in immutable build: {candidate}")
    expected_dirs = {
        parent.as_posix()
        for relative in files
        for parent in Path(relative).parents
        if parent.as_posix() != "."
    }
    if actual_files != set(files) or actual_dirs != expected_dirs:
        abort(f"immutable build contains missing or unlisted entries: {build}")
    verify_generated_output(files, expected_profile, path)
    return build_id, tuple(files)


def package(destination: Path, app: Path = APP) -> None:
    verify_csp(app)
    engine = app / "engine"
    builds: list[str] = []
    for name, profile in PROFILES.items():
        build_id, _ = verify_manifest(engine, name, profile)
        if build_id not in builds:
            builds.append(build_id)

    destination = destination.expanduser().resolve()
    source = app.resolve()
    if destination == source or source in destination.parents or destination in source.parents:
        abort(f"destination must be outside the source tree: {destination}")
    if destination.exists():
        abort(f"destination already exists: {destination}")
    destination.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix=f".{destination.name}-", dir=destination.parent) as temporary:
        staged = Path(temporary) / "site"
        staged.mkdir()
        for relative in (*SHELL_FILES, *VENDOR_FILES):
            source_file = app / relative
            if source_file.is_symlink() or not source_file.is_file():
                abort(f"missing allowlisted web asset: {source_file}")
            target = staged / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_file, target)
        target_engine = staged / "engine"
        target_engine.mkdir()
        for manifest_name in PROFILES:
            shutil.copy2(engine / manifest_name, target_engine / manifest_name)
        for build_id in builds:
            shutil.copytree(engine / build_id, target_engine / build_id, symlinks=False)
        staged.replace(destination)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path, help="New directory that will become the static site root")
    args = parser.parse_args()
    package(args.destination)
    print(f"Packaged verified CPU single/Rayon site at {args.destination.expanduser().resolve()}")


if __name__ == "__main__":
    main()
