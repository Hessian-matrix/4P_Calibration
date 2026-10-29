#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import urllib.error
import urllib.parse
import urllib.request

EXPECTED_BINARIES = {
    "four-p-calibration-x86_64-unknown-linux-gnu",
    "four-p-calibration-aarch64-unknown-linux-gnu",
    "four-p-calibration-x86_64-pc-windows-gnu.exe",
}


def api_request(url: str, token: str, *, method: str = "GET", data: bytes | None = None,
                content_type: str = "application/json") -> dict[str, object]:
    request = urllib.request.Request(
        url,
        data=data,
        headers={"JOB-TOKEN": token, "Content-Type": content_type},
        method=method,
    )
    try:
        with urllib.request.urlopen(request, timeout=120) as response:
            body = response.read()
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", errors="replace")
        raise SystemExit(f"GitLab API {method} {url} failed: {error.code} {detail}") from error
    return json.loads(body) if body else {}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag", required=True)
    parser.add_argument("--assets", type=Path, required=True)
    args = parser.parse_args()
    api = os.environ.get("CI_API_V4_URL")
    project = os.environ.get("CI_PROJECT_ID")
    token = os.environ.get("CI_JOB_TOKEN")
    if not api or not project or not token:
        raise SystemExit("CI_API_V4_URL, CI_PROJECT_ID and CI_JOB_TOKEN are required")

    binaries = {path.name: path for path in args.assets.iterdir() if path.is_file() and path.name != "SHA256SUMS"}
    if set(binaries) != EXPECTED_BINARIES:
        raise SystemExit(f"release assets mismatch: expected={sorted(EXPECTED_BINARIES)}, got={sorted(binaries)}")
    checksum_path = args.assets / "SHA256SUMS"
    checksum_path.write_text(
        "".join(f"{hashlib.sha256(binaries[name].read_bytes()).hexdigest()}  {name}\n" for name in sorted(binaries)),
        encoding="utf-8",
    )
    assets = {**binaries, checksum_path.name: checksum_path}
    encoded_project = urllib.parse.quote(project, safe="")
    encoded_tag = urllib.parse.quote(args.tag, safe="")
    links = []
    for name, path in sorted(assets.items()):
        encoded_name = urllib.parse.quote(name, safe="")
        package_url = f"{api}/projects/{encoded_project}/packages/generic/four-p-calibration/{encoded_tag}/{encoded_name}"
        api_request(package_url, token, method="PUT", data=path.read_bytes(), content_type="application/octet-stream")
        links.append({"name": name, "url": package_url.replace(api, os.environ["CI_SERVER_URL"] + "/api/v4"), "link_type": "package"})

    payload = json.dumps({
        "name": f"4P Calibration {args.tag}",
        "tag_name": args.tag,
        "description": "Four-platform build verified by the internal GitLab pipeline.",
        "assets": {"links": links},
    }).encode()
    release = api_request(
        f"{api}/projects/{encoded_project}/releases", token, method="POST", data=payload,
    )
    print(release["_links"]["self"])


if __name__ == "__main__":
    main()
