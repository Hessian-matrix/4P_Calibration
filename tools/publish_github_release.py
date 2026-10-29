#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import mimetypes
import os
from pathlib import Path
import urllib.error
import urllib.parse
import urllib.request

REPOSITORY = "Hessian-matrix/4P_Calibration"
EXPECTED_ASSETS = {
    "four-p-calibration-x86_64-unknown-linux-gnu",
    "four-p-calibration-aarch64-unknown-linux-gnu",
    "four-p-calibration-x86_64-pc-windows-gnu.exe",
    "four-p-calibration-aarch64-apple-darwin",
}


def request(url: str, token: str, *, method: str = "GET", data: bytes | None = None,
            content_type: str = "application/vnd.github+json") -> dict[str, object]:
    headers = {
        "Accept": "application/vnd.github+json",
        "Authorization": f"Bearer {token}",
        "X-GitHub-Api-Version": "2022-11-28",
        "Content-Type": content_type,
        "User-Agent": "4p-calibration-gitlab-release",
    }
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=120) as response:
            body = response.read()
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise SystemExit(f"GitHub API {method} {url} failed: {error.code} {detail}") from error
    return json.loads(body) if body else {}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", required=True)
    parser.add_argument("--assets", type=Path, required=True)
    args = parser.parse_args()
    token = os.environ.get("GITHUB_RELEASE_TOKEN")
    if not token:
        raise SystemExit("GITHUB_RELEASE_TOKEN is required")

    binaries = {
        path.name: path for path in args.assets.iterdir()
        if path.is_file() and path.name != "SHA256SUMS"
    }
    if set(binaries) != EXPECTED_ASSETS:
        raise SystemExit(f"release assets mismatch: expected={sorted(EXPECTED_ASSETS)}, got={sorted(binaries)}")
    checksums = args.assets / "SHA256SUMS"
    checksums.write_text(
        "".join(f"{hashlib.sha256(binaries[name].read_bytes()).hexdigest()}  {name}\n" for name in sorted(binaries)),
        encoding="utf-8",
    )
    assets = {**binaries, checksums.name: checksums}

    api = f"https://api.github.com/repos/{REPOSITORY}"
    main_ref = request(f"{api}/git/ref/heads/main", token)
    target = main_ref["object"]["sha"]
    payload = json.dumps({
        "tag_name": args.tag,
        "target_commitish": target,
        "name": f"4P Calibration {args.tag}",
        "body": "Built and verified by the internal GitLab four-platform release pipeline.",
        "draft": False,
        "prerelease": False,
        "generate_release_notes": True,
    }).encode()
    release = request(f"{api}/releases", token, method="POST", data=payload)
    upload_url = str(release["upload_url"]).split("{", 1)[0]
    for name, path in sorted(assets.items()):
        query = urllib.parse.urlencode({"name": name})
        request(
            f"{upload_url}?{query}", token, method="POST", data=path.read_bytes(),
            content_type=mimetypes.guess_type(name)[0] or "application/octet-stream",
        )
    print(release["html_url"])


if __name__ == "__main__":
    main()
