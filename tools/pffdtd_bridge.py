#!/usr/bin/env python3
"""Convert PFFDTD HDF5 receiver output to va-synthesis' small binary IR format."""

import argparse
import struct
import sys
from pathlib import Path


def extract(data_dir: Path, output: Path, pffdtd_python: Path,
            output_rate: float, maximum_frequency: float,
            air_absorption: bool) -> None:
    try:
        import h5py
        import numpy as np
    except ImportError as error:
        raise SystemExit(
            "PFFDTD extraction requires numpy and h5py in the selected Python environment"
        ) from error

    sys.path.insert(0, str(pffdtd_python))
    try:
        from fdtd.process_outputs import ProcessOutputs
    except ImportError as error:
        raise SystemExit(
            "PFFDTD post-processing dependencies are missing from the selected environment"
        ) from error

    processor = ProcessOutputs(data_dir)
    processor.initial_process()
    processor.apply_lowpass(maximum_frequency)
    processor.resample(output_rate)
    if air_absorption:
        processor.apply_modal_filter()
    responses = processor.r_out_f
    sample_rate = processor.Fs_f

    responses = np.asarray(responses, dtype="<f4", order="C")
    if responses.ndim != 2:
        raise SystemExit("expected a receiver-by-sample PFFDTD output matrix")
    with output.open("wb") as handle:
        handle.write(struct.pack("<4sQQd", b"VAIR", responses.shape[0],
                                 responses.shape[1], sample_rate))
        handle.write(responses.tobytes(order="C"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pffdtd-python", type=Path, required=True)
    parser.add_argument("--output-rate", type=float, required=True)
    parser.add_argument("--maximum-frequency", type=float, required=True)
    parser.add_argument("--air-absorption", action="store_true")
    args = parser.parse_args()
    extract(args.data_dir, args.output, args.pffdtd_python, args.output_rate,
            args.maximum_frequency, args.air_absorption)


if __name__ == "__main__":
    main()
