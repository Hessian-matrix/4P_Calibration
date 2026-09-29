#!/usr/bin/env python3
"""Release orchestration; native loader closure lives in the two platform bundlers."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import tomllib

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TARGETS = [
    {"runner": "windows-2022", "target": "x86_64-pc-windows-msvc", "triplet": "x64-windows-release", "label": "windows-x86_64"},
    {"runner": "ubuntu-22.04", "target": "x86_64-unknown-linux-gnu", "triplet": "x64-linux-release", "label": "linux-x86_64"},
    {"runner": "ubuntu-22.04-arm", "target": "aarch64-unknown-linux-gnu", "triplet": "arm64-linux-release", "label": "linux-aarch64"},
]
BINARIES = ("rigcal-camera", "rigcal-gui")


def run(*args, cwd=ROOT, env=None):
    command = [str(arg) for arg in args]
    print("+", subprocess.list2cmdline(command), flush=True)
    subprocess.run(command, cwd=cwd, env=env, check=True)


def capture(*args, cwd=ROOT):
    return subprocess.check_output([str(arg) for arg in args], cwd=cwd, text=True, encoding="utf-8").strip()


def workspace():
    with (ROOT / "Cargo.toml").open("rb") as stream:
        return tomllib.load(stream)["workspace"]["package"]


def metadata():
    package = workspace()
    version = package["version"]
    if not re.fullmatch(r"\d+\.\d+\.\d+(?:-[0-9A-Za-z.-]+)?", version):
        raise ValueError(f"unsupported release version: {version}")
    revision = capture("git", "rev-parse", "HEAD")
    tag = os.environ.get("GITHUB_REF", "")
    if tag.startswith("refs/tags/") and tag != f"refs/tags/v{version}":
        raise ValueError(f"tag {tag} must equal workspace version v{version}")
    release_id = version if tag.startswith("refs/tags/") else f"{version}-preview-{revision[:12]}"
    native = json.loads((HERE / "vcpkg.json").read_text(encoding="utf-8"))
    rust = package["rust-version"]
    if rust.count(".") == 1:
        rust += ".0"
    values = {
        "prefix": f"rigcal-{release_id}",
        "rust": rust,
        "vcpkg": native["builtin-baseline"],
        "matrix": json.dumps({"include": TARGETS}, separators=(",", ":")),
    }
    print(json.dumps(values, indent=2))
    if output := os.environ.get("GITHUB_OUTPUT"):
        with open(output, "a", encoding="utf-8") as stream:
            for key, value in values.items():
                stream.write(f"{key}={value}\n")


def enable_apt_sources():
    """Retain the runner's mirrors, suites and signing keys for exact source versions."""
    directory = Path("/etc/apt/sources.list.d")
    lists = [Path("/etc/apt/sources.list"), *directory.glob("*.list")]
    source_lines = []
    for path in lists:
        if path.name.startswith("rigcal-") or not path.exists():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            if re.match(r"^\s*deb\s", line):
                source_lines.append(re.sub(r"^(\s*)deb\s", r"\1deb-src ", line))
    blocks = []
    for path in directory.glob("*.sources"):
        if path.name.startswith("rigcal-"):
            continue
        for block in re.split(r"\n\s*\n", path.read_text(encoding="utf-8")):
            if re.search(r"^Enabled:\s*no\s*$", block, re.M | re.I):
                continue
            if re.search(r"^Types:.*\bdeb\b", block, re.M):
                blocks.append(re.sub(r"^Types:.*$", "Types: deb-src", block, flags=re.M))
    if not source_lines and not blocks:
        raise RuntimeError("no enabled APT repositories found")
    if source_lines:
        (directory / "rigcal-release.list").write_text("\n".join(source_lines) + "\n", encoding="utf-8")
    if blocks:
        (directory / "rigcal-release.sources").write_text("\n\n".join(blocks) + "\n", encoding="utf-8")


def build_native(vcpkg, triplet):
    executable = vcpkg / ("vcpkg.exe" if os.name == "nt" else "vcpkg")
    bootstrap = vcpkg / ("bootstrap-vcpkg.bat" if os.name == "nt" else "bootstrap-vcpkg.sh")
    if os.name == "nt":
        run("cmd.exe", "/c", bootstrap, "-disableMetrics")
    else:
        run("bash", bootstrap, "-disableMetrics")
    run(executable, "install", "--triplet", triplet, "--host-triplet", triplet,
        f"--x-manifest-root={HERE}", f"--x-install-root={vcpkg / 'installed'}",
        f"--overlay-triplets={HERE / 'triplets'}", "--disable-metrics")
    prefix = vcpkg / "installed" / triplet
    environment = {
        "VCPKG_ROOT": str(vcpkg),
        "VCPKGRS_TRIPLET": triplet,
        "VCPKG_TARGET_TRIPLET": triplet,
        "VCPKGRS_DYNAMIC": "1",
    }
    if os.name == "nt":
        clang = Path(os.environ["ProgramFiles"]) / "LLVM" / "bin"
        if not (clang / "libclang.dll").is_file():
            raise FileNotFoundError(f"libclang.dll is missing from {clang}")
        environment.update({"LIBCLANG_PATH": str(clang), "OPENCV_DISABLE_PROBES": "environment,pkg_config,cmake,vcpkg_cmake"})
        with open(os.environ["GITHUB_PATH"], "a", encoding="utf-8") as stream:
            stream.write(f"{prefix / 'bin'}\n")
    else:
        environment.update({
            "PKG_CONFIG_PATH": str(prefix / "lib" / "pkgconfig"),
            "FFMPEG_DIR": str(prefix),
            "OpenCV_DIR": str(prefix / "share" / "opencv4"),
            "CMAKE_PREFIX_PATH": str(prefix),
            "LD_LIBRARY_PATH": str(prefix / "lib"),
            "OPENCV_DISABLE_PROBES": "environment,pkg_config,vcpkg_cmake,vcpkg",
        })
    with open(os.environ["GITHUB_ENV"], "a", encoding="utf-8") as stream:
        for key, value in environment.items():
            stream.write(f"{key}={value}\n")


def configs(destination):
    import yaml

    destination.mkdir()
    rig = yaml.safe_load((ROOT / "crates/rigcal-gui/example.rig.yaml").read_text(encoding="utf-8"))
    for index, camera in enumerate(rig["rig"]["cameras"]):
        camera["guidance"] = {"type": "rtsp", "url": f"rtsp://10.21.12.162:{554 + index}/PRR"}
    rig["rig"]["evidence"] = {"host": "10.21.12.162", "port": 30432}
    camera = yaml.safe_load((ROOT / "crates/rigcal-core/tests/fixtures/session_single.yaml").read_text(encoding="utf-8"))
    camera["capture"]["evidence"]["port"] = 30432
    camera["solver"]["models"] = ["kb4"]
    for name, value in (("rig.example.yaml", rig), ("camera.example.yaml", camera)):
        (destination / name).write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False), encoding="utf-8")


def rust_licenses(destination, target):
    graph = json.loads(capture("cargo", "metadata", "--locked", "--format-version", "1", "--filter-platform", target))
    selected = {node["id"] for node in graph["resolve"]["nodes"]}
    packages = [item for item in graph["packages"] if item["id"] in selected and item["source"] is not None]
    catalog = []
    for package in packages:
        source = Path(package["manifest_path"]).parent
        name = f"{package['name']}-{package['version']}"
        output = destination / "rust" / name
        output.mkdir(parents=True)
        notices = [path for path in source.iterdir() if path.name.lower().startswith(("license", "copying", "notice", "copyright"))]
        if package.get("license_file"):
            path = source / package["license_file"]
            if not path.is_file():
                raise FileNotFoundError(path)
            if path not in notices:
                notices.append(path)
        for notice in notices:
            if notice.is_dir():
                shutil.copytree(notice, output / notice.name)
            else:
                shutil.copy2(notice, output / notice.name)
        catalog.append({"name": package["name"], "version": package["version"], "license": package["license"], "source": package["source"]})
    (destination / "rust-packages.json").write_text(json.dumps(catalog, indent=2) + "\n", encoding="utf-8")
    return packages


def system_licenses_and_sources(bundle, prefix, downloads):
    sources = set()
    records = json.loads((bundle / "native-dependencies.json").read_text(encoding="utf-8"))
    for record in records:
        if not record["bundled"]:
            continue
        source = Path(record["source_path"]).resolve()
        if source.is_relative_to(prefix):
            continue
        candidates = [str(source)]
        if source.is_relative_to("/usr"):
            candidates.append("/" + str(source.relative_to("/usr")))
        owner = None
        for candidate in candidates:
            found = subprocess.run(["dpkg-query", "-S", candidate], capture_output=True, text=True)
            if found.returncode == 0:
                owner = found.stdout.splitlines()[0].rsplit(": ", 1)[0]
                break
        if not owner or ", " in owner:
            raise RuntimeError(f"cannot identify Debian source package for {source}")
        name, version = capture("dpkg-query", "-W", "-f=${source:Package}\t${source:Version}", owner).split("\t")
        binary_name = owner.split(":", 1)[0]
        copyright_file = Path("/usr/share/doc") / binary_name / "copyright"
        output = bundle / "LICENSES" / "system" / binary_name
        output.mkdir(parents=True, exist_ok=True)
        shutil.copy2(copyright_file, output / "copyright")
        sources.add((name, version))
    for name, version in sorted(sources):
        directory = downloads / name
        directory.mkdir(parents=True, exist_ok=True)
        run("apt-get", "source", "--download-only", "--only-source", f"{name}={version}", cwd=directory)
        if not list(directory.glob("*.dsc")):
            raise RuntimeError(f"missing source control file for {name}={version}")
    return [{"name": name, "version": version} for name, version in sorted(sources)]


def make_sources(output, vcpkg, packages, system_sources):
    native_trees = sorted((vcpkg / "buildtrees").glob("*/src"))
    for required in ("ffmpeg", "opencv4"):
        if not any(path.parent.name == required and any(path.iterdir()) for path in native_trees):
            raise RuntimeError(f"{required} corresponding source missing; rebuild the native cache")
    with tarfile.open(output, "w:gz") as archive:
        for filename in capture("git", "ls-files", "-z").split("\0"):
            if filename:
                archive.add(ROOT / filename, arcname=f"project/{filename}", recursive=False)
        for tree in native_trees:
            archive.add(tree, arcname=f"native/patched/{tree.parent.name}")
        for name in ("ports", "scripts", "triplets"):
            archive.add(vcpkg / name, arcname=f"native/vcpkg/{name}")
        for package in packages:
            archive.add(Path(package["manifest_path"]).parent, arcname=f"rust/{package['name']}-{package['version']}")
        if system_sources.exists():
            archive.add(system_sources, arcname="native/system-source-archives")


def checksum(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def package_release(args):
    target = next(item for item in TARGETS if item["label"] == args.label)
    name = f"{args.prefix}-{args.label}"
    if not re.fullmatch(r"[A-Za-z0-9.+_-]+", name):
        raise ValueError("unsafe package name")
    output = ROOT / ".release" / "stage" / name
    output.mkdir(parents=True, exist_ok=False)
    prefix = args.vcpkg / "installed" / target["triplet"]
    bundler = HERE / ("bundle_windows.py" if os.name == "nt" else "bundle_linux.py")
    run(sys.executable, bundler, "--binaries", ROOT / "target" / target["target"] / "release", "--prefix", prefix, "--output", output)
    configs(output / "config")
    shutil.copytree(ROOT / "docs", output / "docs")
    shutil.copy2(ROOT / "README.md", output / "README.md")
    licenses = output / "LICENSES"
    licenses.mkdir()
    for directory in sorted((prefix / "share").iterdir()):
        copyright_file = directory / "copyright"
        if copyright_file.is_file():
            destination = licenses / "native" / directory.name
            destination.mkdir(parents=True)
            shutil.copy2(copyright_file, destination / "copyright")
            spdx = directory / "vcpkg.spdx.json"
            if spdx.exists():
                shutil.copy2(spdx, destination / spdx.name)
    for required in ("ffmpeg", "opencv4"):
        if not (licenses / "native" / required / "copyright").is_file():
            raise RuntimeError(f"missing {required} redistribution notice")
    packages = rust_licenses(licenses, target["target"])
    downloads = ROOT / ".release" / "system-sources"
    system_packages = system_licenses_and_sources(output, prefix, downloads) if os.name != "nt" else []
    build = {
        "version": workspace()["version"], "commit": capture("git", "rev-parse", "HEAD"),
        "target": target["target"], "vcpkg": capture("git", "rev-parse", "HEAD", cwd=args.vcpkg),
        "rustc": capture("rustc", "--version"), "system_sources": system_packages,
        "minimum_os": "Windows 10 x64; OpenGL driver for GUI" if os.name == "nt" else "Linux glibc >= 2.35; desktop and OpenGL driver for GUI",
    }
    (output / "build-info.json").write_text(json.dumps(build, indent=2) + "\n", encoding="utf-8")
    shutil.copy2(ROOT / "Cargo.lock", output / "Cargo.lock")
    (output / "RUN.txt").write_text(
        "Keep the complete extracted directory together; do not copy only the executable.\n"
        "Edit config/*.example.yaml: device IP, raw port, image size and measured board geometry.\n"
        "GUI: rigcal-gui --config config/rig.example.yaml\n"
        "CLI: rigcal-camera --config config/camera.example.yaml --live\n"
        "Linux: prefix executable names with ./ ; desktop/OpenGL required for GUI.\n"
        "Windows: use .\\rigcal-gui.exe or .\\rigcal-camera.exe in PowerShell; OpenGL driver required for GUI.\n"
        "--help needs no camera. --check-deps additionally needs ldd (Linux) or dumpbin (Windows).\n"
        "Rust KB4/DS need no Python. The optional SciPy reference backend is not included.\n"
        "See docs/operations.md, LICENSES/ and build-info.json.\n"
        "Distribution permission is not granted by this package: project license is UNLICENSED.\n"
        "Review Slint's selected license and third-party redistribution conditions before publishing.\n",
        encoding="utf-8",
    )
    dist = ROOT / "dist"
    dist.mkdir(exist_ok=True)
    source_archive = dist / f"{name}-sources.tar.gz"
    make_sources(source_archive, args.vcpkg, packages, downloads)
    if os.name == "nt":
        archive = Path(shutil.make_archive(str(dist / name), "zip", output.parent, output.name))
    else:
        archive = Path(shutil.make_archive(str(dist / name), "gztar", output.parent, output.name))
    checksums = "".join(f"{checksum(path)}  {path.name}\n" for path in (archive, source_archive))
    (dist / f"SHA256SUMS-{args.label}.txt").write_text(checksums, encoding="utf-8")


def smoke(args):
    """Run extracted artifacts after hiding the SDK; nothing may resolve from the build prefix."""
    suffix = ".zip" if os.name == "nt" else ".tar.gz"
    name = f"{args.prefix}-{args.label}"
    archive = ROOT / "dist" / (name + suffix)
    installed = args.vcpkg / "installed"
    hidden = args.vcpkg / "installed-smoke-hidden"
    if hidden.exists():
        raise FileExistsError(hidden)
    environment = os.environ.copy()
    for key in ("LD_LIBRARY_PATH", "LD_PRELOAD", "LD_AUDIT", "LIBRARY_PATH", "LIB", "LIBPATH", "INCLUDE", "VCPKG_ROOT", "VCPKGRS_TRIPLET", "VCPKG_TARGET_TRIPLET", "VCPKGRS_DYNAMIC", "FFMPEG_DIR", "OpenCV_DIR", "CMAKE_PREFIX_PATH", "PKG_CONFIG_PATH", "PKG_CONFIG_LIBDIR"):
        environment.pop(key, None)
    excluded = (str(args.vcpkg).lower(), str(ROOT / "target").lower())
    environment["PATH"] = os.pathsep.join(part for part in environment.get("PATH", "").split(os.pathsep) if not any(item in part.lower() for item in excluded))
    installed.rename(hidden)
    try:
        with tempfile.TemporaryDirectory(prefix="rigcal relocated ") as temporary:
            shutil.unpack_archive(archive, temporary)
            bundle = Path(temporary) / name
            for executable in BINARIES:
                program = bundle / (executable + (".exe" if os.name == "nt" else ""))
                for argument in ("--help", "--check-deps"):
                    result = subprocess.run([str(program), argument], cwd=temporary, env=environment,
                                            text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=60)
                    print(result.stdout, result.stderr)
                    if result.returncode != 0:
                        raise RuntimeError(f"relocated {program.name} {argument} failed: {result.returncode}")
                    if any(path in (result.stdout + result.stderr).lower() for path in excluded):
                        raise RuntimeError(f"{program.name} still references a build directory")
    finally:
        hidden.rename(installed)


def publish(args):
    assets = []
    lines = []
    for target in TARGETS:
        label = target["label"]
        name = f"{args.prefix}-{label}"
        extension = ".zip" if label.startswith("windows") else ".tar.gz"
        expected = {name + extension, name + "-sources.tar.gz"}
        seen = set()
        for line in (args.directory / f"SHA256SUMS-{label}.txt").read_text(encoding="utf-8").splitlines():
            digest, filename = line.split("  ", 1)
            if filename not in expected or filename in seen or not re.fullmatch(r"[0-9a-f]{64}", digest):
                raise ValueError(f"unexpected checksum entry: {line}")
            path = args.directory / filename
            if checksum(path) != digest:
                raise ValueError(f"checksum mismatch: {filename}")
            seen.add(filename)
            assets.append(path)
            lines.append(line)
        if seen != expected:
            raise ValueError(f"incomplete {label} release assets")
    sums = args.directory / "SHA256SUMS.txt"
    sums.write_text("\n".join(sorted(lines)) + "\n", encoding="utf-8")
    notes = args.directory / "release-notes.txt"
    notes.write_text(
        "Portable Windows x86_64 and Linux x86_64 / ARM64 packages.\n\n"
        "Extract the complete runtime archive, edit config/*.example.yaml, and run the GUI or CLI. "
        "Linux requires glibc >= 2.35; both platforms need a desktop and OpenGL driver for the GUI. "
        "No Rust, Python, OpenCV or FFmpeg development installation is required.\n\n"
        "SHA256SUMS.txt covers all runtime and corresponding-source archives. "
        "Build provenance and third-party notices are included in each runtime package.\n\n"
        "**Draft review required:** this repository currently declares UNLICENSED. "
        "Confirm project distribution rights and the applicable Slint license before publishing. "
        "Slint's royalty-free desktop license does not cover embedded systems. "
        "The badge below supplies attribution when that license is applicable; it is not a license grant.\n\n"
        "![Made with Slint](https://raw.githubusercontent.com/slint-ui/slint/master/logo/MadeWithSlint-logo-whitebg.png)\n",
        encoding="utf-8",
    )
    run("gh", "release", "create", args.tag, *assets, sums, "--verify-tag", "--draft", "--title", args.tag, "--notes-file", notes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("metadata")
    commands.add_parser("apt-sources")
    native = commands.add_parser("build-native")
    native.add_argument("--vcpkg", type=Path, required=True)
    native.add_argument("--triplet", required=True)
    for name in ("package", "smoke"):
        command = commands.add_parser(name)
        command.add_argument("--vcpkg", type=Path, required=True)
        command.add_argument("--prefix", required=True)
        command.add_argument("--label", choices=[target["label"] for target in TARGETS], required=True)
    command = commands.add_parser("publish")
    command.add_argument("--directory", type=Path, required=True)
    command.add_argument("--prefix", required=True)
    command.add_argument("--tag", required=True)
    args = parser.parse_args()
    if hasattr(args, "vcpkg"):
        args.vcpkg = args.vcpkg.resolve()
    if args.command == "metadata":
        metadata()
    elif args.command == "apt-sources":
        enable_apt_sources()
    elif args.command == "build-native":
        build_native(args.vcpkg, args.triplet)
    elif args.command == "package":
        package_release(args)
    elif args.command == "smoke":
        smoke(args)
    else:
        publish(args)


if __name__ == "__main__":
    main()
