#!/usr/bin/env python3
"""Stage a relocatable Linux bundle for rigcal-camera / rigcal-gui.

This is the Linux half of the release packaging step and needs nothing but the
Python 3.11+ standard library plus the binutils/patchelf tools that are part of
the CI image.  It deliberately does *not* build, install, strip, archive, write
licenses or touch the vcpkg prefix: it only stages the native executable/library
closure and proves that the staged closure still resolves.

Layout produced inside ``--output`` (any other content already present, such as
config, docs or LICENSES staged by the workflow, is left alone)::

    rigcal-camera              executable, DT_RPATH = $ORIGIN/lib
    rigcal-gui                 executable, DT_RPATH = $ORIGIN/lib
    lib/<soname>               every bundled third-party library, DT_RPATH = $ORIGIN
    native-dependencies.json   {"name", "source", "bundled", "source_path"} per dependency

``name`` is the file name the loader looks up (``DT_NEEDED`` string or dlopen
soname).  For every bundled dependency ``source`` is the absolute path the bytes
were read from on the packaging host -- vcpkg libraries live under ``--prefix``,
the rest are host files that the parent maps back to their distribution package
with ``dpkg-query -S`` -- and ``source_path`` repeats it so both platforms expose
the same key set.  Host-supplied objects get ``source: "system:linux"`` and a
``null`` ``source_path``: they are glibc core, the ELF interpreter and GPU vendor
driver implementations, none of which is shipped.  The executables and ``lib/``
are the namespace this script owns and refreshes on every run; anything else
already present in ``--output`` (config, docs, LICENSES) is left untouched.

Resolution rules
----------------
* The closure is the ELF ``DT_NEEDED`` closure of both executables, resolved by
  the host loader (``ldd``) with ``LD_LIBRARY_PATH`` pinned to the vcpkg prefix
  lib directories.  The ambient ``LD_LIBRARY_PATH``/``LD_PRELOAD``/``LD_AUDIT``
  are never inherited or trusted.
* Libraries that are only reached through ``dlopen`` are invisible to ``ldd``,
  so ``DYNAMIC_SEEDS`` lists every library the compiled Slint/winit/glutin stack
  opens at runtime together with the crate that opens it.  Seeds are resolved
  through the loader cache and bundled exactly like a ``DT_NEEDED`` dependency.
* Only glibc core objects, the ELF interpreter and *GPU vendor* driver
  implementations stay host-supplied.  Everything else -- libstdc++/libgcc_s,
  X11, xcb, Wayland, xkbcommon, EGL/GL dispatch, OpenCV, FFmpeg, ... -- is
  copied into ``lib/`` under the name the loader will look up (the
  ``DT_NEEDED`` string) and, when the file's own ``DT_SONAME`` differs, under
  that name as well, so transitive lookups keep working from a flat directory.
* After staging, every staged object is re-resolved with ``LD_LIBRARY_PATH``
  unset, so resolution has to succeed through the bundle's own relative RPATHs.
  Anything that still lands outside ``lib/`` and is not on the host allowlist is
  a hard failure.
* Membership in the closure follows from the object carrying an ELF *dynamic
  section*, not from how many libraries it links.  A shared object with zero
  ``DT_NEEDED`` entries (Debian/Ubuntu's ``libX11-xcb.so.1.0.0`` is one) is a
  normal loadable ``ET_DYN`` that simply contributes no further dependencies.
  The loader reports it as ``statically linked`` -- the very same message it
  prints for a static executable -- so the two cases are told apart by probing
  the dynamic section, never by that message.

Failure behaviour
-----------------
Missing executables, architecture mismatch, unreadable tools, an empty closure,
a prefix that contributes no OpenCV/FFmpeg library, an unresolved dependency,
an unresolved ``dlopen`` seed, a non-relocatable ``DT_NEEDED`` containing a
path, and two different source files claiming the same library name are all
fatal.  Nothing is skipped best-effort.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import re
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

EXECUTABLE_NAMES = ("rigcal-camera", "rigcal-gui")
NATIVE_DEPENDENCIES_NAME = "native-dependencies.json"
LIB_DIR_NAME = "lib"

EM_X86_64 = 62
EM_AARCH64 = 183
ELF_MACHINE_NAMES = {EM_X86_64: "x86_64", EM_AARCH64: "aarch64"}
HOST_MACHINES = {
    "x86_64": EM_X86_64,
    "amd64": EM_X86_64,
    "aarch64": EM_AARCH64,
    "arm64": EM_AARCH64,
}
MULTIARCH_SUFFIX = {EM_X86_64: "x86_64-linux-gnu", EM_AARCH64: "aarch64-linux-gnu"}
LDCONF_ARCH_TAG = {EM_X86_64: "x86-64", EM_AARCH64: "aarch64"}

# glibc core objects plus the ELF interpreter: always taken from the host, never
# bundled.  libgcc_s/libstdc++/libgomp are deliberately *not* here -- they are
# compiler runtime libraries and belong to the bundle.
GLIBC_SONAMES = frozenset(
    {
        "ld-linux.so.2",
        "libBrokenLocale.so.1",
        "libanl.so.1",
        "libc.so.6",
        "libc_malloc_debug.so.0",
        "libdl.so.2",
        "libm.so.6",
        "libmemusage.so",
        "libmvec.so.1",
        "libnsl.so.1",
        "libpcprofile.so",
        "libpthread.so.0",
        "libresolv.so.2",
        "librt.so.1",
        "libthread_db.so.1",
        "libutil.so.1",
    }
)

# GPU *vendor* implementations.  The vendor-neutral glvnd dispatch libraries
# (libEGL.so.1, libGL.so.1, libGLX.so.0, libGLdispatch.so.0, libOpenGL.so.0) are
# bundled; the per-vendor backends are not, because they are bound to the
# runtime kernel driver and are loaded by the dispatch layer itself.
GPU_VENDOR_SONAME_RE = re.compile(
    r"^(?:"
    r"libGLX_.+|libEGL_.+|libnvidia-.+|libcuda\.so.+|libnvcuvid\.so.+|"
    r"libnvoptix\.so.+|libvdpau_.+|libglapi\.so.+|libgallium-.+|libvulkan_.+"
    r")$"
)
GPU_VENDOR_DIRS = frozenset({"dri", "DRI", "vdpau"})

# dlopen-only dependencies of the compiled Slint stack.  Evidence:
#   x11-dl 2.21  src/{xlib,xcursor,xinput2,xlib_xcb}.rs  (winit 0.30 x11 backend)
#   x11rb 0.13   src/xcb_ffi/raw_ffi/ffi.rs dl-libxcb -> libxcb.so.1
#   xkbcommon-dl 0.4  src/lib.rs + src/x11.rs
#   wayland-sys 0.31  src/{client,cursor,egl}.rs  (wayland-backend/dlopen)
#   glutin 0.32  src/api/egl/mod.rs -> libEGL.so.1, src/api/glx/mod.rs -> libGL.so.1
#   glutin 0.32  src/platform/x11.rs -> x11-dl::xrender -> libXrender.so.1
#   (glvnd pulls libGLX.so.0/libGLdispatch.so.0 as *linked* deps of those two, so
#   they arrive through the ordinary ldd closure.)
# The femtovg/OpenGL renderer reaches GL only through those dispatch libraries;
# libVulkan.so.1 would only be needed if the renderer were switched to
# renderer-femtovg-wgpu, which this crate does not enable.
DYNAMIC_SEEDS = (
    ("libX11.so.6", "x11-dl xlib (winit x11 backend)", "libx11-6"),
    ("libX11-xcb.so.1", "x11-dl xlib_xcb (winit x11 backend)", "libx11-xcb1"),
    ("libxcb.so.1", "x11rb dl-libxcb (winit x11 backend)", "libxcb1"),
    ("libXcursor.so.1", "x11-dl xcursor (winit x11 backend)", "libxcursor1"),
    ("libXi.so.6", "x11-dl xinput2 (winit x11 backend)", "libxi6"),
    ("libxkbcommon.so.0", "xkbcommon-dl (winit x11 and wayland)", "libxkbcommon0"),
    ("libxkbcommon-x11.so.0", "xkbcommon-dl x11 (winit x11 backend)", "libxkbcommon-x11-0"),
    ("libwayland-client.so.0", "wayland-sys dlopen (winit wayland backend)", "libwayland-client0"),
    ("libwayland-cursor.so.0", "wayland-sys dlopen (winit wayland backend)", "libwayland-cursor0"),
    ("libwayland-egl.so.1", "wayland-sys egl (glutin wayland EGL surface)", "libwayland-egl1"),
    ("libEGL.so.1", "glutin egl (Slint femtovg renderer)", "libegl1"),
    ("libGL.so.1", "glutin glx (Slint femtovg renderer)", "libgl1"),
    ("libXrender.so.1", "x11-dl xrender (glutin x11 backend)", "libxrender1"),
)

REQUIRED_TOOLS = ("ldd", "patchelf", "readelf")

_SONAME_RE = re.compile(r"\(SONAME\)\s+Library soname: \[(.*)\]")
_LDCONF_RE = re.compile(r"^\s*(\S+)\s+\(([^)]*)\)\s+=>\s+(\S+)\s*$")
_OPENCV_RE = re.compile(r"opencv", re.IGNORECASE)
_FFMPEG_RE = re.compile(r"^(?:libav(?:codec|format|util|filter|device|image)|libsw(?:scale|resample))")


class BundleError(RuntimeError):
    """Fatal packaging failure."""


@dataclass(frozen=True)
class Dependency:
    """A resolved native dependency.

    ``name`` is the lookup name (the ``DT_NEEDED`` string, or the soname for a
    dlopen seed) and also the file name the library is staged under.  ``path``
    is the real file the bytes were read from; ``category`` records which of the
    three bundled provenance classes it came from (vcpkg prefix, the directory
    holding the executables, or the host filesystem) or that it is host-supplied
    and not shipped at all.
    """

    name: str
    path: Path
    bundled: bool
    category: str
    soname: str | None = None


CATEGORY_PREFIX = "prefix"
CATEGORY_BINARIES = "binaries"
CATEGORY_HOST = "host"
CATEGORY_SYSTEM = "system"
SYSTEM_SOURCE = "system:linux"


# --------------------------------------------------------------------------- #
# pure helpers (directly exercisable without a Linux host)
# --------------------------------------------------------------------------- #


def parse_ldd_output(text: str) -> dict[str, str | None]:
    """Map lookup name -> resolved path (``None`` when the loader reported it missing)."""

    resolved: dict[str, str | None] = {}
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith(("linux-vdso", "linux-gate", "ldd:", "statically linked")):
            continue
        if "=>" in line:
            name, _, right = line.partition("=>")
            name = name.strip()
            location = right.strip().split(" (", 1)[0].strip()
            if not name:
                continue
            resolved[name] = None if location == "not found" else location
        elif line.startswith("/"):
            location = line.split(" (", 1)[0].strip()
            resolved[Path(location).name] = location
    return resolved


def parse_ldconfig_output(text: str, arch_tag: str) -> dict[str, list[str]]:
    """Map soname -> candidate paths from ``ldconfig -p``, filtered to one architecture."""

    wanted = arch_tag.casefold()
    cache: dict[str, list[str]] = {}
    for line in text.splitlines():
        match = _LDCONF_RE.match(line)
        if match is None:
            continue
        name, tags, path = match.group(1), match.group(2), match.group(3)
        if wanted not in tags.casefold():
            continue
        cache.setdefault(name, []).append(path)
    return cache


def is_core_system_name(name: str) -> bool:
    """True when the host is expected to supply ``name`` instead of the bundle."""

    if name in GLIBC_SONAMES:
        return True
    if name.startswith("ld-linux") or name.startswith("ld.so") or name.startswith("libnss_"):
        return True
    return GPU_VENDOR_SONAME_RE.match(name) is not None


def is_gpu_vendor_path(path: Path) -> bool:
    return any(part in GPU_VENDOR_DIRS for part in path.parts[:-1])


def is_within(path: Path, directory: Path) -> bool:
    return path == directory or directory in path.parents


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def elf_machine(path: Path) -> int:
    """Return the ELF ``e_machine`` of a 64-bit little-endian object."""

    with path.open("rb") as handle:
        header = handle.read(20)
    if len(header) < 20 or header[:4] != b"\x7fELF":
        raise BundleError(f"{path}: not an ELF object")
    if header[4] != 2:
        raise BundleError(f"{path}: ELF class {header[4]} is not 64-bit")
    if header[5] != 1:
        raise BundleError(f"{path}: ELF data encoding {header[5]} is not little-endian")
    return struct.unpack_from("<H", header, 18)[0]


def format_machine(machine: int) -> str:
    return ELF_MACHINE_NAMES.get(machine, f"e_machine={machine}")


def readelf_dynamic_output(readelf: str, path: Path, env: dict[str, str]) -> str | None:
    """Return ``readelf -d`` output, or ``None`` when the object has no dynamic section.

    The dynamic section -- not the number of ``DT_NEEDED`` entries -- is what
    makes an object part of a dynamic closure: a shared object may legitimately
    link nothing at all while still being a normal, loadable ``ET_DYN`` object.
    """

    result = subprocess.run(
        [readelf, "-d", str(path)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
        check=False,
    )
    if result.returncode != 0:
        raise BundleError(
            f"readelf -d failed on {path} (exit {result.returncode}): {(result.stderr or result.stdout).strip()}"
        )
    output = f"{result.stdout}\n{result.stderr}"
    if "There is no dynamic section" in output:
        return None
    return output


def readelf_soname(readelf: str, path: Path, env: dict[str, str]) -> str | None:
    output = readelf_dynamic_output(readelf, path, env)
    if output is None:
        raise BundleError(f"{path}: no dynamic section, cannot be part of a dynamic closure")
    match = _SONAME_RE.search(output)
    if match is not None:
        return match.group(1)
    return None


# --------------------------------------------------------------------------- #
# packager
# --------------------------------------------------------------------------- #


class LinuxBundler:
    def __init__(self, binaries: Path, prefix: Path, output: Path) -> None:
        self.binaries = binaries.resolve()
        self.prefix = prefix.resolve()
        self.output = output.resolve()
        self.lib_dir = self.output / LIB_DIR_NAME

        self.machine = self._host_machine()
        self.tools = self._find_tools()
        self.env = self._clean_env()

        self.prefix_lib_dirs: list[Path] = []
        self.default_lib_dirs: list[Path] = []
        self.ld_cache: dict[str, list[str]] = {}

        self.deps: dict[str, Dependency] = {}
        self.staged: dict[str, tuple[Path, str]] = {}
        self._queue: list[tuple[str, Path]] = []
        self._walked: set[Path] = set()
        self._missing: list[str] = []

    # -- setup ------------------------------------------------------------- #

    @staticmethod
    def _host_machine() -> int:
        raw = platform.machine().strip().lower()
        machine = HOST_MACHINES.get(raw)
        if machine is None:
            raise BundleError(
                f"unsupported packaging host architecture {platform.machine()!r}; "
                "expected x86_64 or aarch64"
            )
        return machine

    @staticmethod
    def _find_tools() -> dict[str, str]:
        tools: dict[str, str] = {}
        missing: list[str] = []
        for name in REQUIRED_TOOLS:
            found = shutil.which(name)
            if found is None:
                missing.append(name)
            else:
                tools[name] = found
        if missing:
            raise BundleError(
                "missing required tool(s): " + ", ".join(missing) + " (install binutils and patchelf)"
            )
        return tools

    def _clean_env(self) -> dict[str, str]:
        """Environment without loader injection variables from the caller."""

        return {
            key: value
            for key, value in os.environ.items()
            if key not in ("LD_LIBRARY_PATH", "LD_PRELOAD", "LD_AUDIT")
        }

    def _validate_layout(self) -> None:
        if not self.binaries.is_dir():
            raise BundleError(f"--binaries {self.binaries} is not a directory")
        if not self.prefix.is_dir():
            raise BundleError(f"--prefix {self.prefix} is not a directory")
        if self.output.exists() and not self.output.is_dir():
            raise BundleError(f"--output {self.output} exists and is not a directory")
        for guard in (self.prefix, self.binaries):
            if is_within(self.output, guard):
                raise BundleError(f"--output {self.output} must not live inside {guard}")
            if is_within(guard, self.output):
                raise BundleError(f"{guard} must not live inside --output {self.output}")

        for name in ("lib", "lib64"):
            candidate = self.prefix / name
            if candidate.is_dir():
                self.prefix_lib_dirs.append(candidate)
        if not self.prefix_lib_dirs:
            raise BundleError(
                f"--prefix {self.prefix} has no lib/ or lib64/ directory; "
                "expected the vcpkg installed/<triplet> prefix"
            )

        multiarch = MULTIARCH_SUFFIX[self.machine]
        self.default_lib_dirs = [
            Path(f"/lib/{multiarch}"),
            Path(f"/usr/lib/{multiarch}"),
            Path("/lib64"),
            Path("/usr/lib64"),
            Path("/usr/local/lib"),
            Path("/lib"),
            Path("/usr/lib"),
        ]
        ldconfig = shutil.which("ldconfig")
        if ldconfig is not None:
            result = subprocess.run(
                [ldconfig, "-p"],
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=self.env,
                check=False,
            )
            if result.returncode == 0:
                self.ld_cache = parse_ldconfig_output(result.stdout, LDCONF_ARCH_TAG[self.machine])

    # -- resolution -------------------------------------------------------- #

    def _ldd(self, obj: Path, ld_library_path: list[Path] | None) -> dict[str, str | None]:
        env = dict(self.env)
        if ld_library_path:
            env["LD_LIBRARY_PATH"] = os.pathsep.join(str(path) for path in ld_library_path)
        result = subprocess.run(
            [self.tools["ldd"], str(obj)],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            check=False,
        )
        output = f"{result.stdout}\n{result.stderr}"
        if "not a dynamic executable" in output:
            raise BundleError(f"{obj} is not a dynamically linked ELF object")
        if result.returncode != 0:
            raise BundleError(f"ldd failed on {obj} (exit {result.returncode}): {output.strip()}")
        if "statically linked" in output:
            # The loader prints this whenever the object has no DT_NEEDED at all
            # (glibc elf/rtld.c: ``! main_map->l_info[DT_NEEDED]``), which
            # includes a shared object that simply links nothing.  Such an
            # object is a normal closure member with an empty dependency set;
            # only a missing dynamic section puts it outside a dynamic closure.
            if readelf_dynamic_output(self.tools["readelf"], obj, self.env) is None:
                raise BundleError(f"{obj} is not a dynamically linked ELF object")
        return parse_ldd_output(output)

    def _resolve_soname(self, name: str) -> Path | None:
        for directory in self.prefix_lib_dirs:
            candidate = directory / name
            if candidate.is_file():
                return candidate.resolve()
        for cached in self.ld_cache.get(name, ()):
            candidate = Path(cached)
            if candidate.is_file():
                return candidate.resolve()
        for directory in self.default_lib_dirs:
            candidate = directory / name
            if candidate.is_file():
                return candidate.resolve()
        return None

    def _category_for(self, path: Path) -> str:
        path = path.resolve()
        if is_within(path, self.output):
            raise BundleError(f"refusing to bundle {path}: it is already inside --output")
        if is_within(path, self.prefix):
            return CATEGORY_PREFIX
        if is_within(path, self.binaries):
            return CATEGORY_BINARIES
        return CATEGORY_HOST

    def _record(self, dependency: Dependency) -> None:
        existing = self.deps.get(dependency.name)
        if existing is not None:
            if existing.path != dependency.path:
                raise BundleError(
                    f"conflicting copies of {dependency.name}:\n"
                    f"  {existing.path} ({existing.category})\n"
                    f"  {dependency.path} ({dependency.category})"
                )
            return
        self.deps[dependency.name] = dependency

    def _enqueue(self, label: str, path: Path) -> None:
        """Queue an object for closure walking, once per distinct file."""

        if path not in self._walked:
            self._walked.add(path)
            self._queue.append((label, path))

    def _drain(self) -> None:
        while self._queue:
            label, path = self._queue.pop(0)
            for name, location in self._ldd(path, self.prefix_lib_dirs).items():
                if location is None:
                    self._missing.append(f"{name} (needed by {label})")
                    continue
                if "/" in name:
                    raise BundleError(
                        f"{label} needs {name!r}: DT_NEEDED entries containing a path cannot be "
                        "relocated"
                    )
                resolved = Path(location).resolve()
                if is_core_system_name(name) or is_gpu_vendor_path(resolved):
                    self._record(Dependency(name, resolved, False, CATEGORY_SYSTEM))
                    continue
                category = self._category_for(resolved)
                soname = readelf_soname(self.tools["readelf"], resolved, self.env)
                self._record(Dependency(name, resolved, True, category, soname))
                self._enqueue(name, resolved)

    def _resolve_seed(self, soname: str, loaded_by: str, apt_package: str) -> Path:
        resolved = self._resolve_soname(soname)
        if resolved is None:
            raise BundleError(
                f"dynamic seed {soname} not found on this host "
                f"(loaded at runtime by {loaded_by}); install {apt_package}"
            )
        return resolved

    def _build_closure(self) -> list[Path]:
        executable_paths = [self.output / name for name in EXECUTABLE_NAMES]
        for path in executable_paths:
            self._enqueue(path.name, path)
        self._drain()

        for soname, loaded_by, apt_package in DYNAMIC_SEEDS:
            if soname in self.deps:
                continue
            resolved = self._resolve_seed(soname, loaded_by, apt_package)
            if is_core_system_name(soname) or is_gpu_vendor_path(resolved):
                # A vendor-provided dispatch library (for example a distribution
                # where libEGL.so.1 is not glvnd) stays host-supplied.
                self._record(Dependency(soname, resolved, False, CATEGORY_SYSTEM))
                continue
            self._record(
                Dependency(
                    soname,
                    resolved,
                    True,
                    self._category_for(resolved),
                    readelf_soname(self.tools["readelf"], resolved, self.env),
                )
            )
            self._enqueue(soname, resolved)
        self._drain()

        if self._missing:
            raise BundleError(
                "unresolved dynamic dependencies:\n  "
                + "\n  ".join(sorted(set(self._missing)))
                + f"\n(searched: prefix lib dirs of {self.prefix} + ld.so.cache + system lib dirs)"
            )
        if not self.deps:
            raise BundleError("dependency closure is empty; the executables link nothing")
        return executable_paths

    # -- checks ------------------------------------------------------------ #

    def _check_architectures(self) -> None:
        for name in EXECUTABLE_NAMES:
            path = self.binaries / name
            machine = elf_machine(path)
            if machine != self.machine:
                raise BundleError(
                    f"{path} is {format_machine(machine)} but this host is {format_machine(self.machine)}; "
                    "refusing to package a foreign architecture"
                )
        for dependency in self.deps.values():
            if not dependency.bundled:
                continue
            machine = elf_machine(dependency.path)
            if machine != self.machine:
                raise BundleError(
                    f"{dependency.name} ({dependency.path}) is {format_machine(machine)}, "
                    f"expected {format_machine(self.machine)}"
                )

    def _check_prefix_contribution(self) -> None:
        from_prefix = [dep for dep in self.deps.values() if dep.category == CATEGORY_PREFIX]
        if not from_prefix:
            raise BundleError(f"no bundled dependency was resolved from --prefix {self.prefix}")
        if not any(_OPENCV_RE.search(dep.name) for dep in from_prefix):
            raise BundleError(f"--prefix {self.prefix} contributed no OpenCV library to the closure")
        media = [dep for dep in self.deps.values() if _OPENCV_RE.search(dep.name) or _FFMPEG_RE.search(dep.name)]
        foreign = [dep.name for dep in media if dep.category != CATEGORY_PREFIX]
        if foreign:
            raise BundleError("media libraries resolved outside the pinned SDK: " + ", ".join(sorted(foreign)))
        for component in ("libavcodec.so.", "libavformat.so.", "libavutil.so.", "libswscale.so."):
            if not any(dep.name.startswith(component) for dep in from_prefix):
                raise BundleError(f"--prefix {self.prefix} contributed no {component} library to the closure")

    # -- staging ----------------------------------------------------------- #

    @staticmethod
    def _install(source: Path, destination: Path) -> None:
        """Copy source bytes over destination.

        The executables and ``lib/`` are the namespace this script owns, so a
        re-run over a previously staged bundle refreshes them instead of
        tripping over our own earlier RPATH patch.  Anything that is not a
        regular file is refused, so a planted symlink can never redirect a copy.
        """

        if destination.is_symlink() or (destination.exists() and not destination.is_file()):
            raise BundleError(f"{destination} is not a regular file; refusing to overwrite it")
        shutil.copyfile(source, destination)
        destination.chmod(0o755)

    def _stage_executables(self) -> list[Path]:
        self.output.mkdir(parents=True, exist_ok=True)
        staged: list[Path] = []
        for name in EXECUTABLE_NAMES:
            source = self.binaries / name
            if not source.is_file():
                raise BundleError(f"missing executable {source}")
            elf_machine(source)
            destination = self.output / name
            self._install(source, destination)
            staged.append(destination)
        return staged

    def _stage_file(self, source: Path, name: str) -> None:
        if not name or name in (".", "..") or "/" in name or os.sep in name or "\\" in name:
            raise BundleError(f"unsafe library name {name!r} resolved from {source}")
        digest = sha256_of(source)
        known = self.staged.get(name)
        if known is not None:
            if known[1] != digest:
                raise BundleError(
                    f"collision: {name} would be staged from two different files: "
                    f"{known[0]} and {source}"
                )
            return
        self._install(source, self.lib_dir / name)
        self.staged[name] = (source, digest)

    def _stage_libraries(self) -> None:
        self.lib_dir.mkdir(parents=True, exist_ok=True)
        for dependency in sorted(self.deps.values(), key=lambda dep: dep.name):
            if not dependency.bundled:
                continue
            self._stage_file(dependency.path, dependency.name)
            if dependency.soname and dependency.soname != dependency.name:
                self._stage_file(dependency.path, dependency.soname)

    # -- rpath + verification ---------------------------------------------- #

    def _read_rpath(self, path: Path) -> str:
        result = subprocess.run(
            [self.tools["patchelf"], "--print-rpath", str(path)],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=self.env,
            check=False,
        )
        if result.returncode != 0:
            raise BundleError(
                f"patchelf --print-rpath failed on {path} (exit {result.returncode}): "
                f"{(result.stderr or result.stdout).strip()}"
            )
        return result.stdout.strip()

    def _set_rpath(self, path: Path, value: str) -> None:
        current = self._read_rpath(path)
        commands: list[list[str]] = []
        if current:
            commands.append([self.tools["patchelf"], "--remove-rpath", str(path)])
        commands.append([self.tools["patchelf"], "--set-rpath", value, "--force-rpath", str(path)])
        for command in commands:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                env=self.env,
                check=False,
            )
            if result.returncode != 0:
                raise BundleError(
                    f"{' '.join(command[:3])} failed on {path} (exit {result.returncode}): "
                    f"{(result.stderr or result.stdout).strip()}"
                )
        if self._read_rpath(path) != value:
            raise BundleError(f"{path}: RPATH is not {value} after patching")

    def _assert_resolves_from_bundle(self, obj: Path) -> None:
        for name, location in self._ldd(obj, None).items():
            if location is None:
                raise BundleError(f"{obj}: {name} does not resolve with LD_LIBRARY_PATH unset")
            resolved = Path(location).resolve()
            if is_within(resolved, self.lib_dir):
                if name not in self.staged:
                    raise BundleError(
                        f"{obj}: {name} resolved to {resolved}, which this script did not stage"
                    )
                continue
            if is_core_system_name(name) or is_gpu_vendor_path(resolved):
                continue
            raise BundleError(
                f"{obj}: {name} resolves outside the bundle to {resolved} "
                "(not on the host-supplied allowlist)"
            )

    def _patch_and_verify(self, executables: list[Path]) -> None:
        for executable in executables:
            self._set_rpath(executable, "$ORIGIN/lib")
        library_paths = [self.lib_dir / name for name in sorted(self.staged)]
        for library in library_paths:
            self._set_rpath(library, "$ORIGIN")
        for obj in [*executables, *library_paths]:
            self._assert_resolves_from_bundle(obj)

    # -- output ------------------------------------------------------------ #

    def _write_manifest(self) -> Path:
        # ``source`` is the absolute path the bytes were read from, which is what
        # the parent's license/source assembly feeds to ``dpkg-query -S`` (and how
        # it recognises vcpkg libraries, by containment in the prefix).
        # ``source_path`` repeats it so both platforms expose an identical key set.
        records = [
            {
                "name": dep.name,
                "source": str(dep.path) if dep.bundled else SYSTEM_SOURCE,
                "bundled": dep.bundled,
                "source_path": str(dep.path) if dep.bundled else None,
            }
            for dep in sorted(self.deps.values(), key=lambda dep: (dep.name, str(dep.path)))
        ]
        destination = self.output / NATIVE_DEPENDENCIES_NAME
        destination.write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
        return destination

    # -- entry point ------------------------------------------------------- #

    def run(self) -> int:
        self._validate_layout()
        executables = self._stage_executables()
        self._build_closure()
        self._check_prefix_contribution()
        self._check_architectures()
        self._stage_libraries()
        self._patch_and_verify(executables)
        manifest = self._write_manifest()

        bundled = [dep for dep in self.deps.values() if dep.bundled]
        hosted = [dep for dep in self.deps.values() if not dep.bundled]
        seeds = sum(1 for soname, _, _ in DYNAMIC_SEEDS if soname in self.deps and self.deps[soname].bundled)
        print(f"staged {', '.join(EXECUTABLE_NAMES)} -> {self.output}")
        print(f"bundled {len(bundled)} libraries into {self.lib_dir} ({seeds} dlopen seeds included)")
        print(f"host-supplied (not bundled): {len(hosted)} object(s), glibc/loader/GPU vendor only")
        print(f"wrote {manifest}")
        print(
            "runtime smoke: run the relocated/unpacked bundle with LD_LIBRARY_PATH unset; "
            "$ORIGIN/lib must win for every bundled library"
        )
        return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="bundle_linux.py",
        description=(
            "Stage the Linux native dependency closure of rigcal-camera/rigcal-gui into a "
            "relocatable bundle (executables in the root, third-party libraries in lib/)."
        ),
    )
    parser.add_argument("--binaries", required=True, type=Path, help="directory with rigcal-camera and rigcal-gui")
    parser.add_argument("--prefix", required=True, type=Path, help="vcpkg installed/<triplet> prefix")
    parser.add_argument("--output", required=True, type=Path, help="bundle directory to stage into")
    args = parser.parse_args(argv)

    try:
        return LinuxBundler(args.binaries, args.prefix, args.output).run()
    except BundleError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
