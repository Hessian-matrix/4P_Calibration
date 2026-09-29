#!/usr/bin/env python3
"""Focused tests for the MSVC runtime notice packaging in ``release.py``.

    python3 -m unittest discover -s tools/release -p 'test_*.py'

The Burn bundle (``0`` manifest plus ``u<N>`` payloads) is synthesised here, so the
tests need no 7-Zip, no Windows host and no real redistributable installer.  They
pin the parts a wrong implementation gets wrong silently: a bundle embeds one
translated ``license.rtf`` per language and only the untranslated one is the notice
that applies, the manifest's own size/SHA-1 have to be re-checked against the
payload, and the recorded provenance must not point outside ``VCToolsRedistDir``.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import release

BURN = "http://schemas.microsoft.com/wix/2008/Burn"
LICENSE = b"{\\rtf1\\ansi\\deff0 Microsoft Visual C++ 2022 Redistributable}\n"


def payload(file_path, source, data, *, size=None, digest=None):
    return (
        '<Payload Id="pay" FilePath="{}" FileSize="{}" Hash="{}" '
        'Packaging="embedded" SourcePath="{}" />'
    ).format(
        file_path,
        len(data) if size is None else size,
        hashlib.sha1(data).hexdigest().upper() if digest is None else digest,
        source,
    )


def manifest(body, version="14.44.35211.0"):
    registration = "" if version is None else '<Registration Id="{{x}}" Version="{}" />'.format(version)
    return (
        '<?xml version="1.0" encoding="utf-8"?>'
        '<BurnManifest xmlns="{}"><UX>{}</UX>{}</BurnManifest>'.format(BURN, body, registration)
    )


class BurnLicenseTests(unittest.TestCase):
    """``read_burn_license`` against a synthesised extracted bundle."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="rigcal burn test ")
        self.addCleanup(temporary.cleanup)
        self.extracted = Path(temporary.name)

    def write(self, body, *, version="14.44.35211.0", member="u4", data=LICENSE):
        (self.extracted / "0").write_text(manifest(body, version), encoding="utf-8")
        (self.extracted / member).write_bytes(data)

    def read(self):
        return release.read_burn_license(self.extracted, Path("vc_redist.x64.exe"))

    def test_original_license_is_read_among_translated_copies(self):
        translated = "".join(
            payload("{}license.rtf".format(1028 + index), "u{}".format(100 + index), b"{\\rtf1 other}\n")
            for index in range(3)
        )
        self.write(translated + payload("license.rtf", "u4", LICENSE))
        self.assertEqual(self.read(), (LICENSE, "14.44.35211.0"))

    def test_tampered_payload_is_rejected(self):
        self.write(payload("license.rtf", "u4", LICENSE, digest=hashlib.sha1(b"other").hexdigest().upper()))
        with self.assertRaises(RuntimeError):
            self.read()

    def test_size_mismatch_is_rejected(self):
        self.write(payload("license.rtf", "u4", LICENSE, size=len(LICENSE) + 1))
        with self.assertRaises(RuntimeError):
            self.read()

    def test_non_rtf_payload_is_rejected(self):
        self.write(payload("license.rtf", "u4", b"MZ not a license"))
        with self.assertRaises(RuntimeError):
            self.read()

    def test_translated_only_bundle_is_rejected(self):
        self.write(payload("1028\\license.rtf", "u4", LICENSE))
        with self.assertRaises(RuntimeError):
            self.read()

    def test_payload_member_must_be_an_embedded_member(self):
        self.write(payload("license.rtf", "../u4", LICENSE))
        with self.assertRaises(RuntimeError):
            self.read()

    def test_missing_registration_version_is_rejected(self):
        self.write(payload("license.rtf", "u4", LICENSE), version=None)
        with self.assertRaises(RuntimeError):
            self.read()

    def test_absent_manifest_is_reported(self):
        with self.assertRaises(RuntimeError):
            self.read()


class MsvcLicensePackagingTests(unittest.TestCase):
    """``msvc_licenses``: installer discovery, provenance and fail-closed guards."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="rigcal msvc test ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name) / "Redist" / "MSVC" / "14.44.35211"
        self.crt = self.root / "x64" / "Microsoft.VC143.CRT"
        self.crt.mkdir(parents=True)
        self.runtime = self.crt / "msvcp140.dll"
        self.runtime.write_bytes(b"MZ msvcp140")
        self.bundle = Path(temporary.name) / "bundle"
        self.bundle.mkdir()
        self.write_manifest([self.runtime_record(), self.opencv_record()])

    def runtime_record(self, source_path=None):
        return {
            "name": "msvcp140.dll",
            "source": "vctools-redist:x64/Microsoft.VC143.CRT/msvcp140.dll",
            "bundled": True,
            "source_path": str(self.runtime if source_path is None else source_path),
        }

    def opencv_record(self):
        return {
            "name": "opencv_world4120.dll",
            "source": "prefix:bin/opencv_world4120.dll",
            "bundled": True,
            "source_path": str(self.bundle / "opencv_world4120.dll"),
        }

    def write_manifest(self, records):
        (self.bundle / "native-dependencies.json").write_text(
            json.dumps(records, indent=2) + "\n", encoding="utf-8"
        )

    def installer(self, *parts):
        path = self.root.parent.joinpath(*parts)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"MZ installer")
        return path

    @staticmethod
    def record_license(installer, destination):
        destination.mkdir(parents=True, exist_ok=True)
        (destination / "license.rtf").write_bytes(LICENSE)
        return {"installer": str(installer.resolve()), "license_sha256": "0" * 64}

    def run_msvc(self):
        with mock.patch.dict(os.environ, {"VCToolsRedistDir": str(self.root)}):
            return release.msvc_licenses(self.bundle)

    def test_toolset_alias_installer_is_found_and_recorded(self):
        installer = self.installer("v143", "vc_redist.x64.exe")
        with mock.patch.object(
            release, "extract_msvc_license", mock.Mock(side_effect=self.record_license)
        ) as extract:
            provenance = self.run_msvc()
        self.assertEqual(extract.call_args.args[0], installer.resolve())
        self.assertEqual(provenance["redistributable_root"], str(self.root.resolve()))
        self.assertEqual([record["name"] for record in provenance["libraries"]], ["msvcp140.dll"])
        self.assertEqual(provenance["license_sha256"], "0" * 64)
        self.assertTrue(provenance["redistribution_terms"].startswith("https://"))
        written = json.loads((self.bundle / "LICENSES" / "msvc" / "provenance.json").read_text(encoding="utf-8"))
        self.assertEqual(written, provenance)
        self.assertEqual((self.bundle / "LICENSES" / "msvc" / "license.rtf").read_bytes(), LICENSE)

    def test_moved_installer_is_found_under_the_versioned_tree(self):
        installer = self.installer("14.44.35211", "installers", "vc_redist.x64.exe")
        with mock.patch.object(
            release, "extract_msvc_license", mock.Mock(side_effect=self.record_license)
        ) as extract:
            self.run_msvc()
        self.assertEqual(extract.call_args.args[0], installer.resolve())

    def test_missing_installer_names_every_searched_location(self):
        with self.assertRaises(FileNotFoundError) as caught:
            self.run_msvc()
        message = str(caught.exception)
        # The fixed candidate and what the two redistributable directories actually
        # hold both have to be in the message: that is what makes a layout change
        # diagnosable from the packaging log alone.
        self.assertIn(str(self.root.resolve() / "vc_redist.x64.exe"), message)
        self.assertIn("Microsoft.VC143.CRT", message)
        self.assertIn(str(self.root.resolve().parent), message)

    def test_runtime_outside_the_redistributable_root_is_rejected(self):
        elsewhere = Path(self.bundle.parent) / "elsewhere" / "msvcp140.dll"
        elsewhere.parent.mkdir()
        elsewhere.write_bytes(b"MZ elsewhere")
        self.write_manifest([self.runtime_record(elsewhere), self.opencv_record()])
        with self.assertRaises(RuntimeError):
            self.run_msvc()

    def test_bundle_without_msvc_runtimes_is_rejected(self):
        self.write_manifest([self.opencv_record()])
        with self.assertRaises(RuntimeError):
            self.run_msvc()

    def test_redistributable_root_is_required(self):
        with mock.patch.dict(os.environ, {"VCToolsRedistDir": ""}):
            with self.assertRaises(RuntimeError):
                release.msvc_licenses(self.bundle)


class PublishGateTests(unittest.TestCase):
    """``publish``: the checksum/matrix gate that runs on a tag push only."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="rigcal publish test ")
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        for target in release.TARGETS:
            label = target["label"]
            extension = ".zip" if label.startswith("windows") else ".tar.gz"
            name = f"rigcal-0.0.1-{label}"
            lines = []
            for filename in (name + extension, name + "-sources.tar.gz"):
                path = self.directory / filename
                path.write_bytes(f"payload of {filename}".encode())
                lines.append(f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {filename}")
            self.write_sums(label, lines)
        self.args = SimpleNamespace(directory=self.directory, prefix="rigcal-0.0.1", tag="v0.0.1")

    def write_sums(self, label, lines):
        (self.directory / f"SHA256SUMS-{label}.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def publish(self):
        with mock.patch.object(release, "run") as run:
            release.publish(self.args)
        return run

    def test_complete_matrix_aggregates_checksums_and_creates_a_draft(self):
        run = self.publish()
        summary = (self.directory / "SHA256SUMS.txt").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(summary), 6)
        self.assertEqual(summary, sorted(summary))
        notes = (self.directory / "release-notes.txt").read_text(encoding="utf-8")
        self.assertIn("UNLICENSED", notes)
        self.assertIn("Slint", notes)
        self.assertEqual(run.call_args.args[:4], ("gh", "release", "create", "v0.0.1"))
        self.assertIn("--draft", run.call_args.args)

    def test_tampered_asset_is_rejected(self):
        (self.directory / "rigcal-0.0.1-linux-x86_64.tar.gz").write_bytes(b"tampered")
        with self.assertRaises(ValueError):
            self.publish()
        self.assertFalse((self.directory / "SHA256SUMS.txt").exists())

    def test_incomplete_matrix_is_rejected(self):
        label = "linux-aarch64"
        binary = f"rigcal-0.0.1-{label}.tar.gz"
        digest = hashlib.sha256((self.directory / binary).read_bytes()).hexdigest()
        self.write_sums(label, [f"{digest}  {binary}"])
        with self.assertRaises(ValueError):
            self.publish()

    def test_unexpected_checksum_entry_is_rejected(self):
        label = "linux-aarch64"
        path = self.directory / f"SHA256SUMS-{label}.txt"
        path.write_text(path.read_text(encoding="utf-8") + "0" * 64 + "  extra.tar.gz\n", encoding="utf-8")
        with self.assertRaises(ValueError):
            self.publish()


class RelocatedSmokeTests(unittest.TestCase):
    """``smoke``: unpack the produced archive elsewhere and run both entry points."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="rigcal smoke test ")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.vcpkg = self.root / "vcpkg"
        (self.vcpkg / "installed" / "x64-linux-release" / "lib").mkdir(parents=True)

    def build_archive(self, body, *, label="linux-x86_64"):
        prefix = "rigcal-0.0.1"
        name = f"{prefix}-{label}"
        stage = self.root / "stage" / name
        stage.mkdir(parents=True)
        for executable in release.BINARIES:
            path = stage / executable
            path.write_text("#!/bin/sh\n" + body, encoding="utf-8")
            path.chmod(0o755)
        dist = self.root / "dist"
        dist.mkdir(exist_ok=True)
        shutil.make_archive(str(dist / name), "gztar", stage.parent, stage.name)
        return SimpleNamespace(vcpkg=self.vcpkg, prefix=prefix, label=label)

    def smoke(self, args):
        with mock.patch.object(release, "ROOT", self.root):
            release.smoke(args)

    def test_relocated_entry_points_run_and_the_sdk_is_restored(self):
        self.smoke(self.build_archive('echo "rigcal $1"; exit 0\n'))
        self.assertTrue((self.vcpkg / "installed").is_dir())
        self.assertFalse((self.vcpkg / "installed-smoke-hidden").exists())

    def test_failing_entry_point_aborts_and_restores_the_sdk(self):
        with self.assertRaises(RuntimeError):
            self.smoke(self.build_archive("exit 1\n"))
        self.assertTrue((self.vcpkg / "installed").is_dir())

    def test_build_prefix_leak_is_rejected(self):
        leak = self.vcpkg / "installed" / "x64-linux-release" / "lib"
        with self.assertRaises(RuntimeError):
            self.smoke(self.build_archive(f'echo "{leak}"; exit 0\n'))


if __name__ == "__main__":
    unittest.main()
