#!/usr/bin/env python3
"""Focused tests for the Linux bundler's dynamic-closure validation.

    python3 -m unittest discover -s tools/release -p 'test_*.py'

The distinction under test is structural and runs on any host: glibc's loader
prints ``statically linked`` for *every* object without a ``DT_NEEDED`` entry,
including a shared object that legitimately links nothing (Debian/Ubuntu's
``libX11-xcb.so.1.0.0``, a valid ``ET_DYN`` with a dynamic section and zero
needed libraries).  ``ldd``/``readelf`` are therefore answered from fixed
transcripts via ``subprocess.run``, so no Linux host, loader or binutils is
required.
"""

from __future__ import annotations

import subprocess
import unittest
from pathlib import Path
from unittest import mock

import bundle_linux
from bundle_linux import BundleError, LinuxBundler, parse_ldd_output, readelf_dynamic_output, readelf_soname

STATE_DIR = Path("/usr/lib/x86_64-linux-gnu")
X11_XCB = STATE_DIR / "libX11-xcb.so.1.0.0"
STAGED = Path("/stage/lib")

NO_DYNAMIC_SECTION = "There is no dynamic section in this file.\n"

# readelf -d of Debian/Ubuntu libX11-xcb.so.1.0.0: dynamic section, SONAME, no
# NEEDED entry.
DYNAMIC_WITHOUT_NEEDED = """
Dynamic section at offset 0x2d80 contains 19 entries:
  Tag        Type                         Name/Value
 0x000000000000000e (SONAME)             Library soname: [libX11-xcb.so.1]
"""

DYNAMIC_WITH_NEEDED = """
Dynamic section at offset 0x2dd0 contains 27 entries:
  Tag        Type                         Name/Value
 0x0000000000000001 (NEEDED)             Shared library: [libc.so.6]
 0x000000000000000e (SONAME)             Library soname: [libX11.so.6]
"""

STATICALLY_LINKED = "\tstatically linked\n"
NOT_DYNAMIC = "\tnot a dynamic executable\n"


def fake_tools(ldd, readelf):
    """A ``subprocess.run`` replacement answering ``ldd``/``readelf`` from transcripts."""

    transcripts = {"ldd": ldd, "readelf": readelf}
    calls: list[list[str]] = []

    def run(command, **kwargs):
        tool = Path(command[0]).name
        code, stdout, stderr = transcripts[tool]
        calls.append(list(command))
        return subprocess.CompletedProcess(command, code, stdout, stderr)

    run.calls = calls
    return run


def make_bundler():
    """A bundler without touching the host: only the fields ``_ldd`` needs."""

    bundler = object.__new__(LinuxBundler)
    bundler.tools = {
        "ldd": "/usr/bin/ldd",
        "readelf": "/usr/bin/readelf",
        "patchelf": "/usr/bin/patchelf",
    }
    bundler.env = {"PATH": "/usr/bin"}
    bundler.lib_dir = STAGED
    bundler.staged = {}
    return bundler


class DependencyFreeSharedObjectTests(unittest.TestCase):
    def test_dependency_free_shared_object_is_a_valid_closure_member(self):
        run = fake_tools(ldd=(0, STATICALLY_LINKED, ""), readelf=(0, DYNAMIC_WITHOUT_NEEDED, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            self.assertEqual(make_bundler()._ldd(X11_XCB, None), {})
        # The loader's message is not trusted: the dynamic section is probed.
        self.assertEqual([Path(call[0]).name for call in run.calls], ["ldd", "readelf"])

    def test_statically_linked_without_dynamic_section_is_rejected(self):
        run = fake_tools(ldd=(0, STATICALLY_LINKED, ""), readelf=(0, NO_DYNAMIC_SECTION, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                make_bundler()._ldd(STATE_DIR / "libstatic.a", None)

    def test_static_executable_is_rejected(self):
        run = fake_tools(ldd=(1, "", NOT_DYNAMIC), readelf=(0, NO_DYNAMIC_SECTION, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                make_bundler()._ldd(STATE_DIR / "rigcal-camera-static", None)

    def test_ldd_failure_is_not_swallowed(self):
        run = fake_tools(
            ldd=(1, "", "ldd: /usr/lib/x86_64-linux-gnu/libX11-xcb.so.1.0.0: No such file or directory\n"),
            readelf=(0, DYNAMIC_WITHOUT_NEEDED, ""),
        )
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                make_bundler()._ldd(X11_XCB, None)

    def test_readelf_failure_is_not_swallowed(self):
        run = fake_tools(
            ldd=(0, STATICALLY_LINKED, ""),
            readelf=(1, "", "readelf: Error: Not an ELF file - it has the wrong magic bytes at the start\n"),
        )
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                make_bundler()._ldd(X11_XCB, None)

    def test_dependency_map_still_reports_missing_libraries(self):
        run = fake_tools(
            ldd=(0, "\tlibX11.so.6 => /usr/lib/x86_64-linux-gnu/libX11.so.6 (0x00007f)\n\tlibmissing.so.1 => not found\n", ""),
            readelf=(0, DYNAMIC_WITH_NEEDED, ""),
        )
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            self.assertEqual(
                make_bundler()._ldd(X11_XCB, None),
                {
                    "libX11.so.6": "/usr/lib/x86_64-linux-gnu/libX11.so.6",
                    "libmissing.so.1": None,
                },
            )


class RelocationCheckTests(unittest.TestCase):
    """The post-staging re-resolve shares ``_ldd``, so it exercises the same rule."""

    def test_dependency_free_staged_object_verifies(self):
        run = fake_tools(ldd=(0, STATICALLY_LINKED, ""), readelf=(0, DYNAMIC_WITHOUT_NEEDED, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            make_bundler()._assert_resolves_from_bundle(STAGED / "libX11-xcb.so.1")

    def test_object_resolving_outside_the_bundle_is_rejected(self):
        run = fake_tools(
            ldd=(0, "\tlibrigcal-extra.so.1 => /opt/extra/librigcal-extra.so.1 (0x00007f)\n", ""),
            readelf=(0, DYNAMIC_WITH_NEEDED, ""),
        )
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                make_bundler()._assert_resolves_from_bundle(STAGED / "libX11-xcb.so.1")


class ReadelfDynamicProbeTests(unittest.TestCase):
    def test_missing_dynamic_section_is_none(self):
        run = fake_tools(ldd=(0, "", ""), readelf=(0, NO_DYNAMIC_SECTION, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            self.assertIsNone(readelf_dynamic_output("/usr/bin/readelf", X11_XCB, {}))

    def test_dynamic_section_output_is_returned(self):
        run = fake_tools(ldd=(0, "", ""), readelf=(0, DYNAMIC_WITHOUT_NEEDED, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            self.assertIn("SONAME", readelf_dynamic_output("/usr/bin/readelf", X11_XCB, {}) or "")

    def test_readelf_error_raises(self):
        run = fake_tools(ldd=(0, "", ""), readelf=(1, "", "readelf: Error: Not an ELF file\n"))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                readelf_dynamic_output("/usr/bin/readelf", X11_XCB, {})

    def test_soname_still_requires_a_dynamic_section(self):
        run = fake_tools(ldd=(0, "", ""), readelf=(0, NO_DYNAMIC_SECTION, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            with self.assertRaises(BundleError):
                readelf_soname("/usr/bin/readelf", X11_XCB, {})
        run = fake_tools(ldd=(0, "", ""), readelf=(0, DYNAMIC_WITHOUT_NEEDED, ""))
        with mock.patch.object(bundle_linux.subprocess, "run", run):
            self.assertEqual(readelf_soname("/usr/bin/readelf", X11_XCB, {}), "libX11-xcb.so.1")


class DynamicSeedTests(unittest.TestCase):
    def test_libxrender_seed_is_retained(self):
        # The Slint x11 renderer reaches libXrender only through dlopen, so it can
        # never appear in the ldd closure; dropping the seed ships a broken bundle.
        self.assertIn("libXrender.so.1", [soname for soname, _, _ in bundle_linux.DYNAMIC_SEEDS])

    def test_statically_linked_lines_do_not_become_dependencies(self):
        self.assertEqual(parse_ldd_output(STATICALLY_LINKED), {})


if __name__ == "__main__":
    unittest.main()
