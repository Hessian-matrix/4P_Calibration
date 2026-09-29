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
import time
import tomllib
import xml.etree.ElementTree as ET

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
    example = yaml.safe_load((ROOT / "crates/rigcal-gui/example.yaml").read_text(encoding="utf-8"))
    for index, camera in enumerate(example["cameras"]):
        camera["guidance"] = {"type": "rtsp", "url": f"rtsp://10.21.12.162:{554 + index}/PRR"}
    # 证据端口以板端 `~/demo/config/sensor_config.yaml` 的 `raw_server.port` 为准。
    example["evidence"] = {"host": "10.21.12.162", "port": 30432}
    (destination / "example.yaml").write_text(
        yaml.safe_dump(example, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )


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


BURN_NAMESPACE = "{http://schemas.microsoft.com/wix/2008/Burn}"


def read_burn_license(extracted, installer):
    """Return ``(license.rtf bytes, redistributable version)`` of an extracted Burn bundle.

    ``0`` is the Burn manifest and ``u<N>`` the embedded payloads.  A bundle also
    carries one translated copy per language (``1028\\license.rtf`` and friends);
    only the untranslated payload is the text the bootstrapper shows by default, so
    it is the notice that accompanies the copied runtime files.  Size and SHA-1 come
    from the manifest and are re-checked against the payload bytes.
    """
    try:
        manifest = ET.parse(extracted / "0").getroot()
    except (OSError, ET.ParseError) as error:
        raise RuntimeError(f"cannot read the Burn manifest of {installer}: {error}") from error
    payloads = [
        payload
        for payload in manifest.findall(f"{BURN_NAMESPACE}UX/{BURN_NAMESPACE}Payload")
        if payload.get("FilePath", "").lower() == "license.rtf"
    ]
    if len(payloads) != 1:
        raise RuntimeError(
            f"expected one original MSVC license.rtf in {installer}, found {len(payloads)}"
        )
    payload = payloads[0]
    member = payload.get("SourcePath", "")
    if not re.fullmatch(r"u[0-9]+", member):
        raise RuntimeError(f"unexpected MSVC license payload path {member!r} in {installer}")
    try:
        data = (extracted / member).read_bytes()
    except OSError as error:
        raise RuntimeError(
            f"cannot read the MSVC license payload {member} of {installer}: {error}"
        ) from error
    if not data.startswith(b"{\\rtf"):
        raise RuntimeError(f"the MSVC license payload of {installer} is not RTF")
    if (str(len(data)) != payload.get("FileSize")
            or hashlib.sha1(data).hexdigest().lower() != payload.get("Hash", "").lower()):
        raise RuntimeError(f"the MSVC license payload of {installer} failed its integrity checks")
    registration = manifest.find(f"{BURN_NAMESPACE}Registration")
    if registration is None or not registration.get("Version"):
        raise RuntimeError(f"the MSVC redistributable version is missing from {installer}")
    return data, registration.get("Version")


def extract_msvc_license(installer, destination):
    extractor = shutil.which("7z") or shutil.which("7zz")
    if not extractor:
        raise RuntimeError("7-Zip is required to extract the MSVC redistribution notice")
    with tempfile.TemporaryDirectory(prefix="rigcal msvc notice ") as temporary:
        extracted = Path(temporary)
        run(extractor, "x", "-y", f"-o{extracted}", installer)
        data, version = read_burn_license(extracted, installer)
        destination.mkdir(parents=True, exist_ok=True)
        license_file = destination / "license.rtf"
        license_file.write_bytes(data)
        return {
            "installer": str(installer.resolve()),
            "installer_sha256": checksum(installer),
            "installer_version": version,
            "license_sha256": checksum(license_file),
        }


def redistributable_installer(root):
    """The ``vc_redist.x64.exe`` whose ``license.rtf`` is the runtime's notice.

    Visual Studio keeps the versioned redistributable tree under
    ``VC\\Redist\\MSVC\\<version>`` with the toolset alias (``v143``) beside it, and
    which of the two holds the standalone installer differs between releases and
    SKUs.  The fixed locations are tried first, then one bounded recursive sweep of
    the versioned tree and its parent.  A miss reports every candidate it tried plus
    what those two directories actually contain, so a layout change is diagnosable
    from the packaging log alone instead of guessing at another path.
    """
    candidates = [root / "vc_redist.x64.exe"]
    for directory in sorted((root / "x64").glob("Microsoft.VC*.CRT")):
        toolset = re.fullmatch(r"Microsoft\.VC([0-9]+)\.CRT", directory.name, re.IGNORECASE)
        if toolset:
            candidates.append(root.parent / f"v{toolset[1]}" / "vc_redist.x64.exe")
    candidates.append(root.parent / "vc_redist.x64.exe")
    if len(root.parents) > 3:
        # <VS root>\Common7\IDE\VC\vc_redist is the other place VS keeps it.
        candidates.append(root.parents[3] / "Common7" / "IDE" / "VC" / "vc_redist" / "vc_redist.x64.exe")
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    swept = (root, root.parent)
    for directory in swept:
        found = sorted(directory.glob("**/vc_redist.x64.exe"))
        if found:
            return found[0]
    listing = []
    for directory in (*swept, root / "x64", *candidates):
        try:
            listing.append(f"{directory}: {sorted(entry.name for entry in directory.iterdir())}")
        except OSError as error:
            listing.append(f"{directory}: {error}")
    raise FileNotFoundError(
        "the MSVC redistributable installer (vc_redist.x64.exe) is missing; its license.rtf is "
        "the redistribution notice of the bundled runtime. Looked for "
        + ", ".join(str(candidate) for candidate in candidates)
        + " and recursively under "
        + ", ".join(str(directory) for directory in swept)
        + ". Found there: "
        + " | ".join(listing)
    )


def msvc_licenses(bundle):
    value = os.environ.get("VCToolsRedistDir")
    if not value:
        raise RuntimeError("VCToolsRedistDir is required to collect MSVC notices")
    root = Path(value).resolve()
    records = json.loads((bundle / "native-dependencies.json").read_text(encoding="utf-8"))
    libraries = [record for record in records
                 if record["bundled"] and record["source"].startswith("vctools-redist:")]
    if not libraries:
        raise RuntimeError("no bundled MSVC redistributables recorded; cannot account for their notices")
    for record in libraries:
        source = Path(record["source_path"]).resolve()
        if not source.is_relative_to(root) or not source.is_file():
            raise RuntimeError(f"MSVC runtime source is outside VCToolsRedistDir or missing: {source}")
    # VS may keep the installer under the toolset alias (v143), beside the versioned DLL tree.
    installer = redistributable_installer(root)
    destination = bundle / "LICENSES" / "msvc"
    provenance = extract_msvc_license(installer, destination)
    provenance["redistributable_root"] = str(root)
    provenance["libraries"] = libraries
    provenance["redistribution_terms"] = "https://learn.microsoft.com/en-us/visualstudio/releases/2022/redistribution"
    (destination / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return provenance


def _split_debian_version(version):
    """``version`` as ``(epoch, upstream, revision)`` (Debian policy §5.6.12).

    The revision is what follows the *last* hyphen, so an upstream part that
    contains one keeps it; the epoch is the leading integer before the first
    colon and defaults to 0.
    """
    epoch = re.match(r"^(\d+):", version)
    body = version[epoch.end():] if epoch else version
    upstream, separator, revision = body.rpartition("-")
    return (int(epoch[1]) if epoch else 0), (upstream if separator else body), (revision if separator else "")


def debian_upstream_version(version):
    """The upstream part of a Debian version.

    Two source packages that share it unpack the same ``.orig.tar``, which is what
    makes it the identity to compare a superseded packaging against the version a
    bundled binary was built from.
    """
    return _split_debian_version(version)[1]


def _version_character_order(character):
    """dpkg's ``order()`` for one version character (``None`` is the end of a part)."""
    if character is None or "0" <= character <= "9":
        return 0
    if "a" <= character <= "z" or "A" <= character <= "Z":
        return ord(character)
    if character == "~":
        return -1
    return ord(character) + 256


def _compare_version_parts(left, right):
    """dpkg's ``verrevcmp()`` over one version part: ``~`` first, digit runs numerically."""
    left_index = right_index = 0
    while left_index < len(left) or right_index < len(right):
        while (
            (left_index < len(left) and not "0" <= left[left_index] <= "9")
            or (right_index < len(right) and not "0" <= right[right_index] <= "9")
        ):
            left_order = _version_character_order(left[left_index] if left_index < len(left) else None)
            right_order = _version_character_order(right[right_index] if right_index < len(right) else None)
            if left_order != right_order:
                return -1 if left_order < right_order else 1
            left_index += 1
            right_index += 1
        while left_index < len(left) and left[left_index] == "0":
            left_index += 1
        while right_index < len(right) and right[right_index] == "0":
            right_index += 1
        difference = 0
        while (
            left_index < len(left) and right_index < len(right)
            and "0" <= left[left_index] <= "9" and "0" <= right[right_index] <= "9"
        ):
            if difference == 0:
                difference = ord(left[left_index]) - ord(right[right_index])
            left_index += 1
            right_index += 1
        if left_index < len(left) and "0" <= left[left_index] <= "9":
            return 1
        if right_index < len(right) and "0" <= right[right_index] <= "9":
            return -1
        if difference:
            return -1 if difference < 0 else 1
    return 0


def debian_version_compare(left, right):
    """Order two Debian versions: ``-1``, ``0`` or ``1``, as ``dpkg --compare-versions``.

    Only what the source choice needs is covered, following Debian policy §5.6.12
    and dpkg's ``verrevcmp()``: the numeric epoch wins first, then the upstream
    part, then the revision, with ``~`` sorting before everything (including the
    end of a part).
    """
    left_epoch, left_upstream, left_revision = _split_debian_version(left)
    right_epoch, right_upstream, right_revision = _split_debian_version(right)
    if left_epoch != right_epoch:
        return -1 if left_epoch < right_epoch else 1
    for left_part, right_part in ((left_upstream, right_upstream), (left_revision, right_revision)):
        ordered = _compare_version_parts(left_part, right_part)
        if ordered:
            return ordered
    return 0


def choose_source_version(installed, available):
    """Pick which published packaging of a source package a bundled binary corresponds to.

    ``installed`` is the source version ``dpkg-query`` reports for the binary and
    ``available`` what the enabled ``deb-src`` pockets publish.  The installed
    version is used whenever the archive still has it.  Ubuntu keeps only the
    newest version per pocket, so a runner image that predates a security update
    reports a version the archive has already replaced; the newest packaging of
    the same upstream version then stands in -- it unpacks the same ``.orig.tar``
    and its patch series is the cumulative one -- and is returned as a
    substitution for the caller to record.

    ``(None, None)`` means nothing published corresponds: either no packaging is
    built from the same upstream version, or all of them are older than the
    installed binary, which carries patches they do not have.  The caller must
    then abort rather than ship a source tree that is not the corresponding one.
    """
    if installed in available:
        return installed, None
    upstream = debian_upstream_version(installed)
    newest = None
    for candidate in available:
        if debian_upstream_version(candidate) != upstream:
            continue
        if newest is None or debian_version_compare(candidate, newest) > 0:
            newest = candidate
    if newest is None or debian_version_compare(newest, installed) < 0:
        return None, None
    return newest, (
        f"{installed} is no longer published for this release; shipping {newest}, "
        f"the newest packaging of the same upstream version {upstream}"
    )


def source_unavailable_message(name, binaries, installed, available, policy=None):
    """The fail-closed message for a bundled system library whose source is unobtainable.

    It names the bundled libraries (what has to be accounted for), their source
    package, the exact version the installed binary was built from and every
    version the pockets still publish, and quotes the APT version table so the
    packaging log itself shows which pockets carry which version.
    """
    return (
        f"cannot obtain the corresponding source of {'/'.join(binaries)} (source package {name}): "
        f"the installed version {installed} is no longer published by the enabled deb-src pockets, "
        f"and no version they publish ({', '.join(available) if available else 'none'}) is a "
        f"replacement built from the same upstream version {debian_upstream_version(installed)} at "
        "least as new as the installed binary. Refresh the runner image or repin the bundled "
        "library so its version is still in the archive; a release must not be packaged without "
        "the corresponding source."
        + (f"\napt-cache policy {' '.join(binaries)}:\n{policy}" if policy else "")
    )


def apt_source_versions(name):
    """Every source version the enabled ``deb-src`` pockets publish for ``name``.

    ``apt-cache showsrc`` answers from the same indexes ``apt-get source`` uses,
    so this is the list the download is chosen from.
    """
    found = subprocess.run(["apt-cache", "showsrc", name], capture_output=True, text=True)
    if found.returncode != 0:
        raise RuntimeError(
            f"apt-cache showsrc {name} failed ({found.returncode}): {found.stderr.strip()}. "
            "The deb-src indexes are required to download the corresponding sources."
        )
    return sorted(set(re.findall(r"^Version: (.+)$", found.stdout, re.M)))


def apt_version_table(binaries):
    """``apt-cache policy`` for the bundled binaries; diagnostics for a failure, never fatal."""
    try:
        return capture("apt-cache", "policy", *binaries)
    except (OSError, subprocess.SubprocessError):
        return ""


def fetch_system_source(name, version, directory):
    """Download exactly ``version`` of source package ``name`` into ``directory``."""
    try:
        run("apt-get", "source", "--download-only", "--only-source", f"{name}={version}", cwd=directory)
    except subprocess.CalledProcessError as error:
        raise RuntimeError(
            f"downloading the corresponding source {name}={version} failed ({error.returncode}) "
            "although the deb-src index lists it; the archive or mirror changed under the run."
        ) from error


def system_licenses_and_sources(bundle, prefix, downloads):
    sources = {}
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
        sources.setdefault((name, version), set()).add(binary_name)
    packages = []
    for (name, version), binaries in sorted(sources.items()):
        binaries = sorted(binaries)
        directory = downloads / name
        directory.mkdir(parents=True, exist_ok=True)
        available = apt_source_versions(name)
        chosen, substitution = choose_source_version(version, available)
        if chosen is None:
            raise RuntimeError(
                source_unavailable_message(name, binaries, version, available, apt_version_table(binaries))
            )
        if substitution:
            print(f"note: {substitution}", flush=True)
        fetch_system_source(name, chosen, directory)
        if not list(directory.glob("*.dsc")):
            raise RuntimeError(f"missing source control file for {name}={chosen}")
        packages.append({"name": name, "version": chosen, "installed_version": version, "binaries": binaries})
    return packages


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


#: ZIP 能存储的最早时间：1980-01-01T00:00:00Z（MS-DOS 时间戳的起点）。
ARCHIVE_EPOCH_FLOOR = 315_532_800


def normalize_archive_timestamps(directory):
    """Raise mtimes that predate the ZIP epoch so archiving cannot fail on a vendored file.

    ``zipfile`` refuses any entry dated before 1980 outright, and one upstream file with a
    bogus date (some tarballs carry 1970/1979 timestamps) would otherwise abort packaging
    after the whole bundle is already staged.  Tar has no such limit, so this is a no-op on
    Linux; the re-dated paths are printed so the offender is visible in the packaging log.
    """
    bumped = []
    for path in sorted(directory.rglob("*")):
        if not path.is_file():
            continue
        mtime = path.stat().st_mtime
        if mtime < ARCHIVE_EPOCH_FLOOR:
            os.utime(path, (ARCHIVE_EPOCH_FLOOR, ARCHIVE_EPOCH_FLOOR))
            stamped = time.strftime("%Y-%m-%d", time.gmtime(mtime))
            bumped.append(f"{path.relative_to(directory)} ({stamped})")
    if bumped:
        print(
            f"note: {len(bumped)} archived file(s) predate 1980 and were re-dated: "
            + ", ".join(bumped),
            flush=True,
        )


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
    msvc = msvc_licenses(output) if os.name == "nt" else None
    build = {
        "version": workspace()["version"], "commit": capture("git", "rev-parse", "HEAD"),
        "target": target["target"], "vcpkg": capture("git", "rev-parse", "HEAD", cwd=args.vcpkg),
        "rustc": capture("rustc", "--version"), "system_sources": system_packages,
        "msvc_runtime": msvc,
        "minimum_os": "Windows 10 x64; OpenGL driver for GUI" if os.name == "nt" else "Linux glibc >= 2.35; desktop and OpenGL driver for GUI",
    }
    (output / "build-info.json").write_text(json.dumps(build, indent=2) + "\n", encoding="utf-8")
    shutil.copy2(ROOT / "Cargo.lock", output / "Cargo.lock")
    (output / "RUN.txt").write_text(
        "Keep the complete extracted directory together; do not copy only the executable.\n"
        "Edit config/example.yaml: device IP, raw port, image size and measured board geometry.\n"
        "GUI: rigcal-gui --config config/example.yaml\n"
        "CLI: rigcal-camera --config config/example.yaml --live (keep a single cameras entry)\n"
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
    normalize_archive_timestamps(output)
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
