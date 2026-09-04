#!/usr/bin/env python3
"""Prepare a PFFDTD HDF5 job from a model_export.json file.

VA scene dumps include a va_materials section with 11-band Sabine coefficients.
Native PFFDTD exports (for example CTK_Church) only name surfaces in mats_hash;
pass the matching HDF5 impedance files with --materials-dir and --material.
"""

import argparse
import json
import sys
from pathlib import Path


def parse_material_assignment(text: str) -> tuple[str, str]:
    name, separator, filename = text.partition("=")
    if not separator or not name or not filename:
        raise argparse.ArgumentTypeError(
            f"--material must be NAME=FILE, got {text!r}")
    return name, filename


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Native PFFDTD example (CTK Church ships impedance HDF5 files):

  python tools/prepare_pffdtd_job.py \\
    --repository submodules/pffdtd \\
    --model submodules/pffdtd/data/models/CTK_Church/model_export.json \\
    --output jobs/ctk-church \\
    --maximum-frequency 1400 --points-per-wavelength 10.5 --duration 3.0 \\
    --source 1 --differentiate-source \\
    --materials-dir submodules/pffdtd/data/materials \\
    --material AcousticPanel=ctk_acoustic_panel.h5 \\
    --material Altar=ctk_altar.h5 \\
    --material Carpet=ctk_carpet.h5 \\
    --material Ceiling=ctk_ceiling.h5 \\
    --material Glass=ctk_window.h5 \\
    --material PlushChair=ctk_chair.h5 \\
    --material Tile=ctk_tile.h5 \\
    --material Walls=ctk_walls.h5
""")
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
    parser.add_argument("--materials-dir", type=Path,
                        help="Folder of existing PFFDTD material HDF5 files")
    parser.add_argument("--material", action="append", default=[],
                        type=parse_material_assignment, metavar="NAME=FILE",
                        help="Map a mats_hash surface name to an HDF5 file "
                             "(repeat for each non-rigid material)")
    args = parser.parse_args()
    if args.processes is not None and args.processes < 1:
        parser.error("--processes must be at least 1")

    with args.model.open(encoding="utf-8") as handle:
        model = json.load(handle)
    if "mats_hash" not in model:
        raise SystemExit("model JSON is missing mats_hash")
    material_names = set(model["mats_hash"]) - {"_RIGID"}
    assigned = dict(args.material)
    va_materials = model.get("va_materials") or {}

    material_files = None
    material_dir = None
    if assigned:
        missing = material_names - set(assigned)
        extra = set(assigned) - material_names
        if missing or extra:
            raise SystemExit(
                "--material names must match the model's non-rigid surfaces.\n"
                f"  missing: {sorted(missing) or '-'}\n"
                f"  unused: {sorted(extra) or '-'}")
        materials_dir = (args.materials_dir or
                         (args.repository.resolve() / "data" / "materials"))
        missing_files = [name for name, filename in assigned.items()
                         if not (materials_dir / filename).is_file()]
        if missing_files:
            raise SystemExit(
                f"material HDF5 files not found in {materials_dir}: "
                + ", ".join(f"{name}={assigned[name]}" for name in missing_files))
        material_files = assigned
        material_dir = materials_dir
    elif set(va_materials) != material_names:
        raise SystemExit(
            "every non-rigid geometry material needs absorption data.\n"
            "This model has no va_materials section (native PFFDTD exports "
            "usually do not). Map existing HDF5 fits with --materials-dir and "
            "--material NAME=FILE.h5 for: "
            + ", ".join(sorted(material_names)))

    python_root = args.repository.resolve() / "python"
    sys.path.insert(0, str(python_root))
    try:
        import numpy as np
        from materials.adm_funcs import fit_to_Sabs_oct_11
        from sim_setup import sim_setup
    except ImportError as error:
        raise SystemExit("run this tool in the PFFDTD conda environment") from error

    if material_files is None:
        material_dir = args.output / "materials"
        material_dir.mkdir(parents=True, exist_ok=True)
        material_files = {}
        for index, (name, coefficients) in enumerate(va_materials.items()):
            filename = f"material_{index:03d}.h5"
            fit_to_Sabs_oct_11(np.asarray(coefficients, dtype=np.float64),
                               filename=material_dir / filename, plot=False)
            material_files[name] = filename

    args.output.mkdir(parents=True, exist_ok=True)
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
