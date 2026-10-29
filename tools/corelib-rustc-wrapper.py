#!/usr/bin/env python3
from __future__ import annotations

import os
from pathlib import Path
import sys

CORELIB_CRATES = ("ccl_calib", "ccl_geometry", "ccl_image", "ccl_rosbag")
SHARED_CRATES = ("anyhow", "image", "serde", "serde_json", "serde_yaml")




def main() -> int:
    if len(sys.argv) < 2:
        raise SystemExit("rustc wrapper requires the real rustc path")
    rustc, *args = sys.argv[1:]
    bundle_text = os.environ.get("FOUR_P_CORELIB_BUNDLE")
    if not bundle_text:
        raise SystemExit("FOUR_P_CORELIB_BUNDLE is required; run tools/build.py")
    bundle = Path(bundle_text).resolve()
    shared = {}
    for crate in SHARED_CRATES:
        matches = list((bundle / "deps").glob(f"lib{crate}-*.rlib"))
        if len(matches) != 1:
            raise SystemExit(f"bundle must contain exactly one {crate} rlib, found {len(matches)}")
        shared[crate] = matches[0]
    for index, value in enumerate(args[:-1]):
        if value == "--extern":
            crate = str(args[index + 1]).split("=", 1)[0]
            if crate in shared:
                args[index + 1] = f"{crate}={shared[crate]}"
    args.extend(["-L", f"dependency={bundle / 'deps'}", "-L", f"dependency={bundle / 'host-deps'}"])
    if os.environ.get("CARGO_PRIMARY_PACKAGE") == "1":
        for crate in CORELIB_CRATES:
            args.extend(["--extern", f"{crate}={bundle / 'lib' / f'lib{crate}.rlib'}"])
    os.execv(rustc, [rustc, *args])
    return 127


if __name__ == "__main__":
    raise SystemExit(main())
