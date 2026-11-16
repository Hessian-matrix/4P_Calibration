from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

import build_web  # noqa: E402
import package_web  # noqa: E402


SINGLE_GLUE = "const wasmUrl = new URL('four_p_calibration_bg.wasm', import.meta.url);\nfetch(wasmUrl);\n"
HELPER_RELATIVE = "snippets/wasm-bindgen-rayon-deadbeef/src/workerHelpers.no-bundler.js"
THREAD_GLUE = (
    f"import {{ startWorkers }} from './{HELPER_RELATIVE}';\n"
    "const wasmUrl = new URL('four_p_calibration_bg.wasm', import.meta.url);\n"
    "fetch(wasmUrl);\n"
)
HELPER = """const pkg = await import(data.mainJS);
const workerInit = { mainJS: builder.mainJS() };
const scriptBlob = await fetch(import.meta.url);
const worker = new Worker(url, { type: 'module' });
"""
CSP = "default-src 'none'; script-src 'self'"


class WebPackagingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.app = self.root / "app"
        self.engine = self.app / "engine"
        self.engine.mkdir(parents=True)
        for relative in (*package_web.SHELL_FILES, *package_web.VENDOR_FILES):
            path = self.app / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"asset:{relative}\n", encoding="utf-8")
        (self.app / "index.html").write_text(
            f'<meta http-equiv="Content-Security-Policy" content="{CSP}">\n', encoding="utf-8"
        )
        (self.app / "coi-serviceworker.js").write_text(
            f"const CONTENT_SECURITY_POLICY = {json.dumps(CSP)};\n", encoding="utf-8"
        )
        self.single_id = self._write_build("manifest.json", "cpu-single", {"four_p_calibration.js": SINGLE_GLUE.encode(), "four_p_calibration_bg.wasm": b"wasm"})
        self.thread_id = self._write_build(
            "manifest-threads.json",
            "cpu-rayon",
            {
                "four_p_calibration.js": THREAD_GLUE.encode(),
                "four_p_calibration_bg.wasm": b"thread-wasm",
                HELPER_RELATIVE: HELPER.encode(),
            },
        )

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def _write_build(self, manifest_name: str, profile: str, contents: dict[str, bytes]) -> str:
        digest = hashlib.sha256()
        rows = []
        for relative, content in sorted(contents.items()):
            sha256 = hashlib.sha256(content).hexdigest()
            digest.update(relative.encode() + b"\0" + bytes.fromhex(sha256))
            role = "glue" if relative.endswith(".js") and relative == "four_p_calibration.js" else "wasm" if relative.endswith(".wasm") else "module"
            rows.append({"role": role, "url": relative, "bytes": str(len(content)), "sha256": sha256})
        build_id = digest.hexdigest()
        for row in rows:
            relative = row["url"]
            path = self.engine / build_id / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(contents[relative])
            row["url"] = f"{build_id}/{relative}"
        manifest = {
            "manifestVersion": 1,
            "releaseId": "test",
            "buildId": build_id,
            "engineApi": package_web.ENGINE_API,
            "executionProfile": profile,
            "resources": rows,
        }
        (self.engine / manifest_name).write_text(json.dumps(manifest), encoding="utf-8")
        return build_id

    def test_rejects_resource_corruption(self) -> None:
        (self.engine / self.single_id / "four_p_calibration_bg.wasm").write_bytes(b"oops")
        with self.assertRaisesRegex(SystemExit, "SHA-256 mismatch"):
            package_web.verify_manifest(self.engine, "manifest.json", "cpu-single")

    def test_rejects_missing_declared_dependency(self) -> None:
        (self.engine / self.single_id / "four_p_calibration_bg.wasm").unlink()
        with self.assertRaisesRegex(SystemExit, "missing engine resource"):
            package_web.verify_manifest(self.engine, "manifest.json", "cpu-single")

    def test_rejects_unlisted_generated_dependency(self) -> None:
        old = self.engine / self.single_id
        manifest = self.engine / "manifest.json"
        old_manifest = json.loads(manifest.read_text())
        for row in old_manifest["resources"]:
            relative = row["url"].split("/", 1)[1]
            if relative == "four_p_calibration.js":
                contents = SINGLE_GLUE + "import('./private.js');\n"
                old.joinpath(relative).write_text(contents)
        # Rebuild the manifest so hashes and buildId are valid; dependency closure must still reject it.
        files = {path.relative_to(old).as_posix(): path.read_bytes() for path in old.rglob("*") if path.is_file()}
        for path in sorted(old.rglob("*"), reverse=True):
            if path.is_file():
                path.unlink()
            elif path.is_dir():
                path.rmdir()
        old.rmdir()
        self._write_build("manifest.json", "cpu-single", files)
        with self.assertRaisesRegex(SystemExit, "unsupported dependency-bearing construct"):
            package_web.verify_manifest(self.engine, "manifest.json", "cpu-single")

    def test_rejects_traversal_resource_url(self) -> None:
        path = self.engine / "manifest.json"
        manifest = json.loads(path.read_text())
        manifest["resources"][0]["url"] = f"{self.single_id}/../private.js"
        path.write_text(json.dumps(manifest))
        with self.assertRaisesRegex(SystemExit, "escapes or does not match"):
            package_web.verify_manifest(self.engine, "manifest.json", "cpu-single")

    def test_rejects_html_and_worker_csp_drift(self) -> None:
        (self.app / "coi-serviceworker.js").write_text(
            'const CONTENT_SECURITY_POLICY = "default-src \'self\'";\n', encoding="utf-8"
        )
        with self.assertRaisesRegex(SystemExit, "CSP policies differ"):
            package_web.package(self.root / "output", self.app)

    def test_package_excludes_unallowlisted_and_stale_files(self) -> None:
        (self.app / ".private-key").write_text("secret")
        (self.app / "vendor" / "private.js").write_text("secret")
        stale = self.engine / ("f" * 64)
        stale.mkdir()
        (stale / "private.bin").write_bytes(b"secret")
        destination = self.root / "output"
        package_web.package(destination, self.app)
        self.assertFalse((destination / ".private-key").exists())
        self.assertFalse((destination / "vendor" / "private.js").exists())
        self.assertFalse((destination / "engine" / stale.name).exists())
        self.assertEqual(
            {path.name for path in (destination / "engine").iterdir()},
            {"manifest.json", "manifest-threads.json", self.single_id, self.thread_id},
        )

    def test_immutable_build_reuse_rejects_extra_file(self) -> None:
        output = self.engine / self.single_id
        (output / "private.txt").write_text("secret")
        entries = []
        for path in output.rglob("*"):
            if path.is_file() and path.name != "private.txt":
                content = path.read_bytes()
                entries.append({"url": path.relative_to(output).as_posix(), "bytes": str(len(content)), "sha256": hashlib.sha256(content).hexdigest()})
        with self.assertRaisesRegex(SystemExit, "missing or extra entries"):
            build_web.verify_immutable_build(output, entries)


if __name__ == "__main__":
    unittest.main()
