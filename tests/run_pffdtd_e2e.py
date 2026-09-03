#!/usr/bin/env python3
"""Run a tiny headless VA -> PFFDTD -> VA integration fixture."""

import argparse
import os
import subprocess
import tempfile
from pathlib import Path


def run(*command: str | Path, cwd: Path | None = None) -> None:
    subprocess.run([str(part) for part in command], cwd=cwd, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--prepare", type=Path, required=True)
    parser.add_argument("--bridge", type=Path, required=True)
    parser.add_argument("--python", type=Path, required=True)
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="va-pffdtd-e2e-") as temporary:
        work = Path(temporary)
        os.environ["MPLCONFIGDIR"] = str(work / "matplotlib")
        model = work / "room.json"
        job = work / "job"
        run(args.fixture, "export", model)
        run(
            args.python,
            args.prepare,
            "--repository", args.repository,
            "--model", model,
            "--output", job,
            "--maximum-frequency", "200",
            "--points-per-wavelength", "6",
            "--duration", "0.05",
            "--processes", "1",
        )
        run(
            args.fixture,
            "validate",
            args.repository,
            job,
            args.bridge,
            args.python,
        )


if __name__ == "__main__":
    main()
