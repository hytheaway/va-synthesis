#!/usr/bin/env python3
"""Copy BRT Wall.hpp and return plane helpers by value.

BRTLibrary's CWall::GetNormal and CWall::GetProjectionPoint return references
to locals. Optimized builds then feed garbage normals into image-source
generation, so every reflection fails the visibility self-check.
"""

from __future__ import annotations

import sys
from pathlib import Path


REPLACEMENTS = (
    (b"\tCommon::CVector3& GetNormal() const {",
     b"\tCommon::CVector3 GetNormal() const {"),
    (b"\tCommon::CVector3 & GetProjectionPoint(float & x0, float & y0, float & z0) const {",
     b"\tCommon::CVector3 GetProjectionPoint(float & x0, float & y0, float & z0) const {"),
    (b"\tCommon::CVector3 & GetProjectionPoint(const Common::CVector3 & point) const {",
     b"\tCommon::CVector3 GetProjectionPoint(const Common::CVector3 & point) const {"),
)


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: patch_brt_wall.py SRC DST", file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    destination = Path(sys.argv[2])
    data = source.read_bytes()
    for old, new in REPLACEMENTS:
        if old not in data:
            print(f"BRT Wall.hpp did not contain expected declaration: {old!r}",
                  file=sys.stderr)
            return 1
        data = data.replace(old, new, 1)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
