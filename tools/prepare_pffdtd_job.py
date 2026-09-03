#!/usr/bin/env python3
"""Prepare a PFFDTD HDF5 job from a va-synthesis PFFDTD model export."""

import argparse
import json
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-frequency", type=float, required=True)
    parser.add_argument("--points-per-wavelength", type=float, default=8.0)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--source", type=int, default=1)
    parser.add_argument("--processes", type=int)
    parser.add_argument("--fcc", action="store_true")
    parser.add_argument("--differentiate-source", action="store_true")
    args = parser.parse_args()
    if args.processes is not None and args.processes < 1:
        parser.error("--processes must be at least 1")

    python_root = args.repository.resolve() / "python"
    sys.path.insert(0, str(python_root))
    try:
        import numpy as np
        from materials.adm_funcs import fit_to_Sabs_oct_11
        from sim_setup import sim_setup
    except ImportError as error:
        raise SystemExit("run this tool in the PFFDTD conda environment") from error

    with args.model.open(encoding="utf-8") as handle:
        model = json.load(handle)
    material_values = model.get("va_materials", {})
    material_names = set(model["mats_hash"]) - {"_RIGID"}
    if material_names != set(material_values):
        raise SystemExit("every non-rigid geometry material needs va_materials coefficients")

    material_dir = args.output / "materials"
    material_dir.mkdir(parents=True, exist_ok=True)
    material_files = {}
    for index, (name, coefficients) in enumerate(material_values.items()):
        filename = f"material_{index:03d}.h5"
        fit_to_Sabs_oct_11(np.asarray(coefficients, dtype=np.float64),
                           filename=material_dir / filename, plot=False)
        material_files[name] = filename

    sim_setup(
        model_json_file=args.model.resolve(),
        mat_folder=material_dir,
        mat_files_dict=material_files,
        source_num=args.source,
        insig_type="impulse",
        diff_source=args.differentiate_source,
        duration=args.duration,
        fcc_flag=args.fcc,
        PPW=args.points_per_wavelength,
        fmax=args.maximum_frequency,
        save_folder=args.output,
        compress=0,
        draw_vox=False,
        Nprocs=args.processes,
    )


if __name__ == "__main__":
    main()
