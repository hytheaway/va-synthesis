#!/usr/bin/env python3
"""Copy BRT Wall.hpp and return plane helpers by value.

BRTLibrary's CWall::GetNormal and CWall::GetProjectionPoint return references
to locals. Optimized builds then feed garbage normals into image-source
generation, so every reflection fails the visibility self-check.
"""

import sys
from pathlib import Path


METHODS = (
    b"GetNormal\\(\\)\\s*const",
    b"GetProjectionPoint\\(float\\s*&\\s*x0,\\s*float\\s*&\\s*y0,\\s*float\\s*&\\s*z0\\)\\s*const",
    b"GetProjectionPoint\\(const\\s*Common::CVector3\\s*&\\s*point\\)\\s*const",
)


def patch_method(data, method_pattern):
    by_ref = re.compile(
        rb"(^[ \t]*Common::CVector3)\s*&\s*(" + method_pattern + rb"\s*\{)",
        re.M,
    )
    by_value = re.compile(
        rb"^[ \t]*Common::CVector3\s+" + method_pattern + rb"\s*\{",
        re.M,
    )
    match = by_ref.search(data)
    if match:
        return by_ref.sub(rb"\1 \2", data, count=1), True
    if by_value.search(data):
        return data, True
    return data, False


def nearby_lines(data, needle):
    lines = data.decode("latin1").splitlines()
    hits = [line for line in lines if needle.decode("ascii") in line]
    return hits[:8]


def main():
    if len(sys.argv) != 3:
        print("usage: patch_brt_wall.py SRC DST", file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    destination = Path(sys.argv[2])
    data = source.read_bytes()
    missing = []
    for method in METHODS:
        data, found = patch_method(data, method)
        if not found:
            missing.append(method)
    if missing:
        print(
            f"BRT Wall.hpp did not contain expected declarations in {source}:",
            file=sys.stderr,
        )
        for method in missing:
            print(f"  {method.decode('ascii')}", file=sys.stderr)
        for line in nearby_lines(data, b"GetNormal"):
            print(f"  found: {line!r}", file=sys.stderr)
        return 1
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_bytes(data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
