#!/usr/bin/env python3
"""Stage the Windows release payload for the ``rigcal-camera`` / ``rigcal-gui`` entry points.

Usage
=====

    python tools/release/bundle_windows.py \
        --binaries target/release \
        --prefix   path/to/vcpkg/installed/x64-windows \
        --output   dist/rigcal-windows-x86_64

``--binaries`` is the directory produced by ``cargo build --locked --release``; it
must hold ``rigcal-camera.exe`` and ``rigcal-gui.exe``.  ``--prefix`` is the vcpkg
``installed/<triplet>`` tree whose ``bin/`` directory holds the release copies of
OpenCV, FFmpeg and their transitive DLLs.  ``--output`` is the staged bundle
directory; it may already contain the shared config/docs/LICENSES assets and is
never wiped.

What the script does
====================

* copies both executables into the bundle root and every required DLL next to
  them (app-local), so the Windows loader finds the closure without ``PATH`` help;
* reads the PE import directory and the delay-load import directory of every
  staged module with a built-in parser (no dumpbin dependency, see below) and
  recurses through the whole app-local closure;
* resolves each import in loader order: the importing module's directory, the
  vcpkg ``bin/`` directory, the MSVC app-local redistributable directories under
  ``%VCToolsRedistDir%\\x64`` (``Microsoft.VC*.CRT`` / ``.OpenMP`` / ``.CXXAMP``),
  then the bundle directory;
* treats ``api-ms-win-*`` / ``ext-ms-win-*`` API sets and DLLs that only exist in
  ``%SystemRoot%\\System32`` as Windows-supplied (recorded, never copied);
* refuses to let the build machine's ``System32``/``PATH`` satisfy a dependency
  that must ship with the bundle (VC++ runtime, OpenCV, FFmpeg, libclang, common
  third-party runtimes) -- see ``_NEVER_SYSTEM``;
* rejects non-AMD64 modules, debug artifacts, compiler/runtime tooling and files
  that collide with different bytes;
* asserts the whole closure is loadable from the bundle directory alone and that
  both OpenCV and FFmpeg (``avcodec``/``avformat``/``avutil``/``swscale``) made it in;
* writes ``native-dependencies.json`` (see below).

Manifest
========

``<output>/native-dependencies.json`` is a JSON list of
``{"name": str, "source": str, "bundled": bool, "source_path": str | null}``
sorted by ``name``.  ``name`` is the lowercased module file name (module lookup on
Windows is case-insensitive, and the staged file keeps whatever spelling the
vcpkg/redist tree ships).  ``bundled`` is true when the file was copied into the
bundle root.  ``source_path`` is the absolute path the bytes were read from for
bundled modules and ``null`` for Windows-supplied ones (this key matches the
Linux packager's manifest so both feed one consumer).  ``source`` is a categorized
locator:

* ``prefix:<path relative to --prefix>``  staged from the vcpkg prefix (e.g.
  ``prefix:bin/opencv_world4120.dll``);
* ``binaries:<path relative to --binaries>``  already app-local next to the EXEs;
* ``vctools-redist:<path relative to %VCToolsRedistDir%>``  MSVC app-local
  redistributable (e.g. ``vctools-redist:x64/Microsoft.VC143.CRT/msvcp140.dll``);
* ``bundle:<name>``  already present in ``--output`` before this run;
* ``system:windows``  supplied by Windows 10/11, not copied.

Dynamically loaded libraries
============================

winit/femtovg/glutin and the media runtimes reach some libraries through
``LoadLibrary`` rather than the import table.  Those names are listed in
``_DYNAMIC_LOADS``; any candidate that the vcpkg prefix or the MSVC
redistributables actually provide is staged and recorded like a normal import,
and the diagnostic summary reports where every other candidate is expected to
come from (Windows itself, or an optional host backend).  ``_DYNAMIC_LOAD_GLOBS``
covers the same case for version-stamped names, currently OpenCV's videoio
FFmpeg wrapper.

The other direction is deliberate: ``System32`` on a developer or CI machine
contains the VC++/OpenCV/FFmpeg DLLs, which is exactly why it is never accepted
as the source of a bundleable dependency.

``--check-deps`` / dumpbin
==========================

The product's ``--check-deps`` diagnostic shells out to ``dumpbin /DEPENDENTS``
and fails when the tool is absent, which is always the case on end-user machines
(dumpbin ships with the MSVC toolchain and is intentionally not bundled).  The
unpacked/relocated smoke test must therefore run on a runner with the MSVC
developer environment active (``VCToolsRedistDir`` is required by this script
anyway) and must not treat a dumpbin failure as a bundle defect -- the version
records printed before that step are the part that proves the shipped libraries
load.  This script itself never calls dumpbin; its loadability assertion is done
in-process with the PE parser above.
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import os
import re
import shutil
import struct
import sys
from pathlib import Path

if sys.version_info < (3, 11):  # pragma: no cover - environment guard
    raise SystemExit("error: Python 3.11 or newer is required")

AMD64 = 0x8664

_MACHINE_NAMES = {
    0x014C: "I386",
    0x01C0: "ARM",
    0x01C4: "ARMNT",
    0x0200: "IA64",
    0x5032: "RISCV32",
    0x5064: "RISCV64",
    0x8664: "AMD64",
    0xAA64: "ARM64",
}

#: Production entry points; both must exist in ``--binaries`` and both go to the bundle root.
_ENTRY_POINTS = ("rigcal-camera", "rigcal-gui")

_IMPORT_DIRECTORY = 1
_DELAY_IMPORT_DIRECTORY = 13

_API_SET = re.compile(r"^(?:api-ms-win|ext-ms-win)-", re.IGNORECASE)

#: Dependencies that must be shipped with the bundle.  A build machine routinely
#: has these in System32/PATH (VC++ redistributable installed system-wide, a
#: stray OpenCV/FFmpeg install); accepting that copy would produce a bundle that
#: only runs on this machine.  Names are matched case-insensitively as prefixes.
_NEVER_SYSTEM = re.compile(
    r"^(?:"
    r"msvcp\d|vcruntime\d|concrt\d|vccorlib\d|vcomp\d|ucrtbased|"
    r"opencv|avcodec|avformat|avutil|avfilter|avdevice|postproc|swscale|swresample|"
    r"libpng|png|zlib|libzstd|zstd|liblzma|lzma|libbz2|bz2|"
    r"libjpeg|jpeg|libtiff|tiff|libwebp|webp|openjp2|libopenjp2|jxl|libjxl|openexr|"
    r"libcrypto|libssl|openblas|libopenblas|lapack|blas|libgfortran|libgcc|libwinpthread|libstdc\+\+|"
    r"x264|libx264|x265|libx265|vpx|libvpx|aom|libaom|dav1d|libdav1d|"
    r"opus|libopus|vorbis|libvorbis|ogg|libogg|mp3lame|libmp3lame|"
    r"freetype|libfreetype|harfbuzz|libharfbuzz|fribidi|libfribidi|ass|libass|zimg|libzimg|"
    r"snappy|libsnappy|protobuf|libprotobuf|pcre2|libpcre2|tbb|libtbb|libiomp5md|libomp|omp"
    r")",
    re.IGNORECASE,
)

#: Compiler/runtime tooling that must never end up in a product bundle.
_FORBIDDEN = re.compile(r"^(?:libclang|clang|llvm|lld)", re.IGNORECASE)

#: Libraries the Slint/winit/femtovg stack and the media runtimes load by name at
#: runtime.  Only entries provided by the vcpkg prefix or the MSVC redistributables
#: are staged; the rest are supplied by Windows or are optional host backends
#: (e.g. ANGLE's libEGL/libGLESv2, which glutin only needs when it picks EGL).
_DYNAMIC_LOADS = (
    # GL / ANGLE entry points used by glutin + femtovg
    "opengl32.dll",
    "libEGL.dll",
    "libGLESv2.dll",
    # winit window management / composition
    "user32.dll",
    "gdi32.dll",
    "shell32.dll",
    "dwmapi.dll",
    "shcore.dll",
    "dcomp.dll",
    # Direct3D, probed by winit's dxgi/d3d12 paths
    "d3d11.dll",
    "d3d12.dll",
    "dxgi.dll",
    "d3dcompiler_47.dll",
    # media foundation, probed by the video stack
    "mfplat.dll",
    "mf.dll",
    "mfreadwrite.dll",
    "evr.dll",
)

#: Pattern-based dynamic loads.  OpenCV's videoio backend hands decoded frames to
#: a separately loaded FFmpeg wrapper (``LoadLibrary("opencv_videoio_ffmpeg<ver>_<bits>.dll")``)
#: whose name tracks the OpenCV version, so it cannot be listed literally.
_DYNAMIC_LOAD_GLOBS = ("opencv_videoio_ffmpeg*.dll",)

_MANIFEST_NAME = "native-dependencies.json"


class BundleError(RuntimeError):
    """Any condition that must abort the staging run."""


# --------------------------------------------------------------------------- #
# PE parsing
# --------------------------------------------------------------------------- #


def _u16(data: bytes, offset: int) -> int:
    return struct.unpack_from("<H", data, offset)[0]


def _u32(data: bytes, offset: int) -> int:
    return struct.unpack_from("<I", data, offset)[0]


def _u64(data: bytes, offset: int) -> int:
    return struct.unpack_from("<Q", data, offset)[0]


class PeFile:
    """Minimal PE32/PE32+ reader: machine type and imported module names."""

    def __init__(self, path: Path) -> None:
        self.path = path
        try:
            self._data = path.read_bytes()
        except OSError as error:
            raise BundleError(f"cannot read {path}: {error}") from error
        try:
            self._parse()
        except BundleError:
            raise
        except (struct.error, IndexError, ValueError) as error:
            raise BundleError(f"{path} is not a parseable PE image ({error})") from error

    # -- parsing ---------------------------------------------------------- #

    def _parse(self) -> None:
        data = self._data
        if len(data) < 0x40 or data[:2] != b"MZ":
            raise BundleError(f"{self.path} is not a PE image (missing MZ header)")
        pe_offset = _u32(data, 0x3C)
        if data[pe_offset : pe_offset + 4] != b"PE\0\0":
            raise BundleError(f"{self.path} is not a PE image (missing PE signature)")

        header = pe_offset + 4
        self.machine = _u16(data, header)
        number_of_sections = _u16(data, header + 2)
        size_of_optional_header = _u16(data, header + 16)
        optional = header + 20
        if size_of_optional_header < 2:
            raise BundleError(f"{self.path} has an empty optional header")

        magic = _u16(data, optional)
        if magic == 0x20B:
            self.pe32plus = True
            image_base = _u64(data, optional + 24)
            directory_offset = optional + 112
            count_offset = optional + 108
        elif magic == 0x10B:
            self.pe32plus = False
            image_base = _u32(data, optional + 28)
            directory_offset = optional + 96
            count_offset = optional + 92
        else:
            raise BundleError(f"{self.path} has an unsupported optional header magic 0x{magic:04x}")
        self.image_base = image_base
        self.size_of_headers = _u32(data, optional + 60)

        directory_count = min(_u32(data, count_offset), 16)
        self._directories = [
            (_u32(data, directory_offset + 8 * index), _u32(data, directory_offset + 8 * index + 4))
            for index in range(directory_count)
        ]

        sections = optional + size_of_optional_header
        self._sections = []
        for index in range(number_of_sections):
            entry = sections + 40 * index
            virtual_size = _u32(data, entry + 8)
            virtual_address = _u32(data, entry + 12)
            raw_size = _u32(data, entry + 16)
            raw_address = _u32(data, entry + 20)
            self._sections.append((virtual_address, max(virtual_size, raw_size), raw_address))

    def _directory(self, index: int) -> tuple[int, int]:
        if index >= len(self._directories):
            return (0, 0)
        return self._directories[index]

    def _rva_to_offset(self, rva: int) -> int:
        if rva < self.size_of_headers:
            return rva
        for virtual_address, size, raw_address in self._sections:
            if virtual_address <= rva < virtual_address + size:
                return raw_address + (rva - virtual_address)
        raise BundleError(f"{self.path} has an RVA 0x{rva:x} outside every section")

    def _cstring(self, rva: int) -> str:
        offset = self._rva_to_offset(rva)
        end = self._data.find(b"\0", offset)
        if end < 0:
            raise BundleError(f"{self.path} has an unterminated string at RVA 0x{rva:x}")
        return self._data[offset:end].decode("ascii", "replace")

    # -- public API ------------------------------------------------------- #

    @property
    def machine_name(self) -> str:
        return _MACHINE_NAMES.get(self.machine, f"0x{self.machine:04x}")

    def imported_modules(self) -> list[str]:
        """Import-directory and delay-load import-directory module names, in order."""
        names = self._import_names(self._directory(_IMPORT_DIRECTORY), 20, delay=False)
        names += self._import_names(self._directory(_DELAY_IMPORT_DIRECTORY), 32, delay=True)
        return names

    def _import_names(self, directory: tuple[int, int], size: int, *, delay: bool) -> list[str]:
        rva, _ = directory
        if rva == 0:
            return []
        offset = self._rva_to_offset(rva)
        names: list[str] = []
        for index in range((len(self._data) - offset) // size):
            entry = offset + size * index
            if not any(self._data[entry : entry + size]):
                break
            if delay:
                attributes = _u32(self._data, entry)
                name_rva = _u32(self._data, entry + 4)
                if not (attributes & 1):
                    # Pointer fields are virtual addresses, not RVAs.
                    name_rva -= self.image_base
            else:
                name_rva = _u32(self._data, entry + 12)
            if name_rva:
                names.append(self._cstring(name_rva))
        return names


# --------------------------------------------------------------------------- #
# filesystem helpers
# --------------------------------------------------------------------------- #


#: lowercased directory path -> {lowercased entry name: real path}.  Windows and
#: macOS resolve file names case-insensitively, so the requested spelling can
#: differ from the name on disk; the index always yields the real one.
_DIRECTORY_INDEX: dict[Path, dict[str, Path]] = {}


def _invalidate_dir_index(directory: Path) -> None:
    _DIRECTORY_INDEX.pop(directory, None)


def _dir_index(directory: Path) -> dict[str, Path]:
    index = _DIRECTORY_INDEX.get(directory)
    if index is None:
        try:
            entries = list(directory.iterdir())
        except OSError:
            entries = []
        index = {}
        for entry in entries:
            try:
                if entry.is_file():
                    index.setdefault(entry.name.lower(), entry)
            except OSError:
                continue
        _DIRECTORY_INDEX[directory] = index
    return index


def _find_in_dir(directory: Path, name: str) -> Path | None:
    """Case-insensitive file lookup returning the name actually present on disk."""
    return _dir_index(directory).get(name.lower())


def _find_matching_in_dir(directory: Path, pattern: str) -> list[Path]:
    wanted = pattern.lower()
    return [
        path
        for name, path in sorted(_dir_index(directory).items())
        if fnmatch.fnmatchcase(name, wanted)
    ]


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _system_directory() -> Path:
    return Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32"


def _redistributable_root() -> Path | None:
    value = os.environ.get("VCToolsRedistDir")
    return Path(value) if value else None


def _redistributable_directories(root: Path) -> list[Path]:
    """App-local MSVC redistributable directories for x64 (CRT, OpenMP, CXXAMP)."""
    platform_root = root / "x64"
    if not platform_root.is_dir():
        return []
    return [
        entry
        for entry in sorted(platform_root.iterdir())
        if entry.is_dir() and entry.name.lower().startswith("microsoft.vc")
    ]


def _looks_like_debug_artifact(path: Path) -> bool:
    if "debug" in (part.lower() for part in path.parts):
        return True
    name = path.name.lower()
    if name.endswith("d.dll") and len(name) > len("d.dll"):
        # vcpkg lays debug DLLs beside release ones; a release twin proves the pair.
        if _find_in_dir(path.parent, name[:-len("d.dll")] + ".dll") is not None:
            return True
    return False


# --------------------------------------------------------------------------- #
# bundler
# --------------------------------------------------------------------------- #


class Bundler:
    def __init__(self, binaries: Path, prefix: Path, output: Path) -> None:
        self.binaries = binaries
        self.prefix = prefix
        self.output = output
        self.redistributable_root = _redistributable_root()
        self.redistributable_dirs = (
            _redistributable_directories(self.redistributable_root)
            if self.redistributable_root
            else []
        )
        #: lowercased file name -> staged path (dict preserves insertion order)
        self.staged: dict[str, Path] = {}
        #: lowercased dependency name -> manifest entry (staged or Windows-supplied)
        self.dependencies: dict[str, dict] = {}
        #: modules whose imports have already been walked
        self.scanned: set[Path] = set()

    # -- validation ------------------------------------------------------- #

    def _validate_inputs(self) -> list[Path]:
        for label, path in (
            ("--binaries", self.binaries),
            ("--prefix", self.prefix),
            ("--output", self.output),
        ):
            if not path.is_dir():
                raise BundleError(f"{label} directory does not exist: {path}")
        if re.match(r"^(?:x86-|arm|arm64|win32)", self.prefix.name, re.IGNORECASE):
            raise BundleError(
                f"--prefix {self.prefix} does not look like an x64 vcpkg triplet (got "
                f"'{self.prefix.name}'); Windows bundles require the x64-windows family"
            )
        if not (self.prefix / "bin").is_dir():
            raise BundleError(
                f"--prefix {self.prefix} has no bin/ directory; it must be a vcpkg "
                f"installed/<triplet> tree with dynamic libraries"
            )
        if self.redistributable_root is None:
            raise BundleError(
                "VCToolsRedistDir is not set; run the build inside an MSVC developer "
                "environment (e.g. the msvc-dev-cmd CI step) so the app-local VC++ "
                "redistributables can be resolved"
            )
        if not self.redistributable_dirs:
            raise BundleError(
                f"VCToolsRedistDir={self.redistributable_root} contains no x64 "
                f"Microsoft.VC*.CRT / .OpenMP / .CXXAMP directories"
            )

        entry_points = []
        for stem in _ENTRY_POINTS:
            path = _find_in_dir(self.binaries, f"{stem}.exe") or _find_in_dir(self.binaries, stem)
            if path is None:
                raise BundleError(f"missing production entry point {stem}.exe in {self.binaries}")
            entry_points.append(path)
        return entry_points

    # -- staging ---------------------------------------------------------- #

    def _label(self, path: Path) -> str:
        for base, label in (
            (self.prefix, "prefix"),
            (self.binaries, "binaries"),
            (self.output, "bundle"),
        ):
            try:
                return f"{label}:{path.relative_to(base).as_posix()}"
            except ValueError:
                continue
        if self.redistributable_root is not None:
            try:
                return f"vctools-redist:{path.relative_to(self.redistributable_root).as_posix()}"
            except ValueError:
                pass
        return f"file:{path.as_posix()}"

    def _require_amd64(self, path: Path, role: str) -> None:
        pe = PeFile(path)
        if pe.machine != AMD64:
            raise BundleError(f"{role} {path.name} is {pe.machine_name}, expected AMD64")

    def _stage(self, source: Path) -> Path:
        """Copy ``source`` into the bundle root, failing on conflicting content."""
        destination = self.output / source.name
        source_hash = _hash_file(source)
        if destination.exists():
            if not destination.is_file():
                raise BundleError(f"bundle path {destination} exists and is not a file")
            if _hash_file(destination) != source_hash:
                raise BundleError(
                    f"collision: {destination.name} is already in the bundle with different "
                    f"content than {source}"
                )
            return destination
        try:
            shutil.copyfile(source, destination)
        except OSError as error:
            raise BundleError(f"cannot copy {source} to {destination}: {error}") from error
        if _hash_file(destination) != source_hash:
            raise BundleError(f"copy of {source} to {destination} produced different content")
        _invalidate_dir_index(self.output)
        return destination

    def _register(self, staged: Path) -> Path:
        self.staged[staged.name.lower()] = staged
        return staged

    def _record(self, name: str, source: str, bundled: bool, origin: Path | None = None) -> None:
        key = name.lower()
        self.dependencies[key] = {
            "name": key,
            "source": source,
            "bundled": bundled,
            "source_path": str(origin.resolve()) if origin is not None else None,
        }

    def _search_dirs(self, importer_dir: Path) -> list[Path]:
        ordered: list[Path] = []
        for candidate in (importer_dir, self.prefix / "bin", *self.redistributable_dirs, self.output):
            if candidate not in ordered:
                ordered.append(candidate)
        return ordered

    def _resolve(self, name: str, importer_dir: Path) -> Path | None:
        """Resolve one dependency name; returns the bundled path or None when Windows supplies it."""
        key = name.lower()
        existing = self.staged.get(key)
        if existing is not None:
            return existing
        if key in self.dependencies:
            return None

        if _API_SET.match(name):
            self._record(name, "system:windows", False)
            return None

        if _FORBIDDEN.match(name):
            raise BundleError(
                f"refusing to bundle compiler tooling: {name} (imported by {importer_dir})"
            )

        for directory in self._search_dirs(importer_dir):
            candidate = _find_in_dir(directory, name)
            if candidate is None:
                continue
            if _looks_like_debug_artifact(candidate):
                raise BundleError(
                    f"refusing to bundle debug artifact {candidate} (imported by {importer_dir})"
                )
            self._require_amd64(candidate, "dependency")
            staged = self._register(self._stage(candidate))
            self._record(staged.name, self._label(candidate), True, candidate)
            return staged

        if _NEVER_SYSTEM.match(name):
            raise BundleError(
                f"dependency {name} (imported by {importer_dir}) must ship with the bundle but "
                f"was not found in {self.prefix / 'bin'} or under {self.redistributable_root}; "
                f"refusing to rely on the build machine's System32/PATH copy"
            )
        if _find_in_dir(_system_directory(), name) is not None:
            self._record(name, "system:windows", False)
            return None
        raise BundleError(
            f"dependency {name} (imported by {importer_dir}) is neither bundled, provided by "
            f"the vcpkg prefix, nor present in {_system_directory()}"
        )

    def _walk(self, roots: list[Path]) -> None:
        """Follow the app-local closure from ``roots`` (re-resolving each staged module)."""
        pending = list(roots)
        while pending:
            current = pending.pop()
            if current in self.scanned:
                continue
            self.scanned.add(current)
            for name in PeFile(current).imported_modules():
                staged = self._resolve(name, current.parent)
                if staged is not None and staged not in self.scanned:
                    pending.append(staged)

    # -- post-conditions -------------------------------------------------- #

    def _stage_dynamic_loads(self) -> list[Path]:
        """Stage dynamically loaded candidates that our inputs actually provide."""
        staged: list[Path] = []
        search_dirs = (self.prefix / "bin", *self.redistributable_dirs)

        def consider(candidate: Path) -> None:
            self._require_amd64(candidate, "dynamic-load dependency")
            staged.append(self._register(self._stage(candidate)))
            self._record(candidate.name, self._label(candidate), True, candidate)

        for name in _DYNAMIC_LOADS:
            key = name.lower()
            if key in self.staged or key in self.dependencies:
                continue
            for directory in search_dirs:
                candidate = _find_in_dir(directory, name)
                if candidate is not None:
                    consider(candidate)
                    break
        for pattern in _DYNAMIC_LOAD_GLOBS:
            for directory in search_dirs:
                for candidate in _find_matching_in_dir(directory, pattern):
                    if candidate.name.lower() in self.staged:
                        continue
                    consider(candidate)
        return staged

    def _assert_closure(self) -> None:
        """Every import of every bundled module must be bundled or Windows-supplied."""
        for module in self.staged.values():
            for name in PeFile(module).imported_modules():
                if name.lower() in self.staged:
                    continue
                if _API_SET.match(name):
                    continue
                if _NEVER_SYSTEM.match(name):
                    raise BundleError(
                        f"{module.name} imports {name}, which must be bundled but is not in "
                        f"{self.output}"
                    )
                if _find_in_dir(_system_directory(), name) is None:
                    raise BundleError(
                        f"{module.name} imports {name}, which is neither in the bundle root nor "
                        f"a Windows-supplied DLL"
                    )

    def _assert_media_runtimes(self) -> None:
        names = set(self.staged)
        if not any(name.startswith("opencv") for name in names):
            raise BundleError(
                "no OpenCV runtime DLL (opencv_*.dll / opencv_world*.dll) was staged; the vcpkg "
                "triplet must build OpenCV as dynamic libraries"
            )
        missing = [
            token
            for token in ("avcodec", "avformat", "avutil", "swscale")
            if not any(token in name for name in names)
        ]
        if missing:
            raise BundleError(
                "FFmpeg runtime DLLs missing from the bundle: "
                + ", ".join(missing)
                + "; the vcpkg triplet must build FFmpeg as dynamic libraries"
            )

    # -- driver ----------------------------------------------------------- #

    def run(self) -> list[dict]:
        entry_points = self._validate_inputs()
        self.output.mkdir(parents=True, exist_ok=True)

        staged_entry_points = []
        for path in entry_points:
            self._require_amd64(path, "entry point")
            staged_entry_points.append(self._register(self._stage(path)))

        dynamic = self._stage_dynamic_loads()
        self._walk([*entry_points, *dynamic])

        self._assert_closure()
        self._assert_media_runtimes()

        manifest = [self.dependencies[key] for key in sorted(self.dependencies)]
        manifest_path = self.output / _MANIFEST_NAME
        manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")

        print(
            f"staged {len(staged_entry_points)} executables and "
            f"{len(self.staged) - len(staged_entry_points)} app-local DLLs"
        )
        print(f"manifest: {manifest_path}")
        for entry in manifest:
            print(f"  {entry['source']:<48} {entry['name']}"
                  + ("" if entry["bundled"] else "  (not bundled)"))
        for path in sorted(dynamic, key=lambda item: item.name.lower()):
            source = self.dependencies[path.name.lower()]["source"]
            print(f"  dynamic-load staged: {path.name} <- {source}")
        for name in sorted(_DYNAMIC_LOADS, key=str.lower):
            if name.lower() in self.staged or name.lower() in self.dependencies:
                continue
            where = (
                "Windows-supplied"
                if _find_in_dir(_system_directory(), name) is not None
                else "not present (optional host backend)"
            )
            print(f"  dynamic-load {name.lower():<28} {where}")
        return manifest


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        prog="bundle_windows.py",
        description=(
            "Stage the Windows release bundle: executables in the bundle root, app-local DLLs "
            "beside them, plus native-dependencies.json."
        ),
    )
    parser.add_argument("--binaries", required=True, type=Path, metavar="DIR",
                        help="directory holding rigcal-camera.exe and rigcal-gui.exe")
    parser.add_argument("--prefix", required=True, type=Path, metavar="DIR",
                        help="vcpkg installed/<triplet> prefix (bin/ holds the release DLLs)")
    parser.add_argument("--output", required=True, type=Path, metavar="DIR",
                        help="bundle directory to stage into (created if missing)")
    arguments = parser.parse_args(argv)

    try:
        Bundler(
            binaries=arguments.binaries.resolve(),
            prefix=arguments.prefix.resolve(),
            output=arguments.output.resolve(),
        ).run()
    except BundleError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
