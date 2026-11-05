#!/usr/bin/env python3
"""Run PFFDTD's Python FDTD engine while exporting live pressure slices.

Produces the usual sim_outs.h5 for the C++ adapter, and writes PNG slices plus
meta.json into --viz-dir so the browser can show field plots without Mayavi/Qt.

Display scaling deliberately ignores the near-source hotspot and compresses
dynamic range so spherical-spreading wavefronts stay visible.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np


def write_json_atomic(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, separators=(",", ":"), allow_nan=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def safe_peak(field: np.ndarray) -> float:
    if field.size == 0:
        return 0.0
    peak = float(np.nanmax(np.abs(field)))
    return peak if np.isfinite(peak) else 0.0


def display_clim(
    field: np.ndarray,
    source_rc: tuple[int, int],
    exclude_radius: int,
) -> float:
    """Choose a clim that follows the traveling front, not the source spike."""
    abs_f = np.abs(np.nan_to_num(field, nan=0.0, posinf=0.0, neginf=0.0))
    if abs_f.size == 0:
        return 1e-12

    masked = abs_f.copy()
    row, col = source_rc
    r0 = max(0, row - exclude_radius)
    r1 = min(field.shape[0], row + exclude_radius + 1)
    c0 = max(0, col - exclude_radius)
    c1 = min(field.shape[1], col + exclude_radius + 1)
    masked[r0:r1, c0:c1] = 0.0
    outside = masked[masked > 0.0]

    if outside.size >= 32:
        peak = float(np.percentile(outside, 99.0))
    else:
        # Pulse still near the source: use a high percentile of the whole slice.
        peak = float(np.percentile(abs_f, 99.7))
    if not np.isfinite(peak) or peak <= 0.0:
        peak = float(np.max(abs_f)) if abs_f.size else 1e-12
    return max(peak, 1e-12)


def write_plane_png(
    path: Path,
    field: np.ndarray,
    source_rc: tuple[int, int],
    exclude_radius: int,
) -> float:
    """Render a pressure slice with PFFDTD-style red/blue seismic coloring."""
    import matplotlib

    matplotlib.use("Agg")
    from matplotlib.image import imsave

    data = np.ascontiguousarray(
        np.nan_to_num(field, nan=0.0, posinf=0.0, neginf=0.0),
        dtype=np.float64,
    )
    clim = display_clim(data, source_rc, exclude_radius)

    # Downsample for snappy live updates.
    max_side = 280
    height, width = data.shape
    if max(height, width) > max_side:
        step = int(np.ceil(max(height, width) / float(max_side)))
        data = data[::step, ::step]

    # Compress dynamic range so a 1/r front is not crushed to white.
    mapped = np.tanh(data / clim * 2.0)

    temporary = path.with_suffix(path.suffix + ".tmp")
    imsave(
        temporary,
        mapped,
        cmap="seismic",
        vmin=-1.0,
        vmax=1.0,
        origin="lower",
        format="png",
    )
    temporary.replace(path)
    return clim


def run_with_viz(
    *,
    data_dir: Path,
    viz_dir: Path,
    every: int,
    nthreads: int | None,
) -> None:
    from fdtd.sim_fdtd import SimEngine, ind2sub3d

    eng = SimEngine(str(data_dir), energy_on=False, nthreads=nthreads)
    eng.load_h5_data()
    eng.setup_mask()
    eng.allocate_mem()
    eng.set_coeffs()
    eng.checks()

    nx, ny, nz, nt = eng.Nx, eng.Ny, eng.Nz, eng.Nt
    ix, iy, iz = ind2sub3d(eng.in_ixyz, nx, ny, nz)
    ix_in = int(np.median(ix))
    iy_in = int(np.median(iy))
    iz_in = int(np.median(iz))
    xv, yv, zv = eng.xv, eng.yv, eng.zv

    # Exclude roughly half a meter around the source when choosing clim.
    exclude_radius = max(8, int(round(0.5 / max(float(eng.h), 1e-6))))

    out_linear = np.asarray(eng.out_ixyz, dtype=np.int64).reshape(-1)[:1]
    out_ix, out_iy, out_iz = ind2sub3d(out_linear, nx, ny, nz)
    receiver_world = [
        float(xv[int(np.asarray(out_ix).reshape(-1)[0])]),
        float(yv[int(np.asarray(out_iy).reshape(-1)[0])]),
        float(zv[int(np.asarray(out_iz).reshape(-1)[0])]),
    ]
    source_world = [float(xv[ix_in]), float(yv[iy_in]), float(zv[iz_in])]

    # After .T for imshow-style arrays, source indices are (row, col):
    # xy: (y, x), xz: (z, x), yz: (z, y)
    source_rc = {
        "xy": (iy_in, ix_in),
        "xz": (iz_in, ix_in),
        "yz": (iz_in, iy_in),
    }

    viz_dir.mkdir(parents=True, exist_ok=True)
    meta = {
        "nx": int(nx),
        "ny": int(ny),
        "nz": int(nz),
        "nt": int(nt),
        "h": float(eng.h),
        "excludeRadius": int(exclude_radius),
        "sourceIndex": [ix_in, iy_in, iz_in],
        "sourceWorld": source_world,
        "receiverWorld": receiver_world,
        "extent": {
            "x": [float(xv[0]), float(xv[-1])],
            "y": [float(yv[0]), float(yv[-1])],
            "z": [float(zv[0]), float(zv[-1])],
        },
        "planes": ["xy", "xz", "yz"],
    }
    write_json_atomic(viz_dir / "meta.json", meta)

    every = max(1, every)
    started = time.monotonic()
    frame_index = 0
    print(
        f"--VIZ: exporting slices every {every} steps "
        f"(source-exclude={exclude_radius} cells) to {viz_dir}",
        flush=True,
    )

    n = 0
    while n < nt:
        if n < max(every * 16, 512):
            batch = max(1, min(every, 4))
        else:
            batch = every
        nrun = min(batch, nt - n)
        eng.run_steps(n, nrun)
        n += nrun
        step = n
        frame_index += 1

        # Same gather + transpose as fdtd.sim_fdtd.run_plot for matplotlib.
        planes = {
            "xy": np.ascontiguousarray(eng.gather_slice(iz=iz_in).T),
            "xz": np.ascontiguousarray(eng.gather_slice(iy=iy_in).T),
            "yz": np.ascontiguousarray(eng.gather_slice(ix=ix_in).T),
        }
        clims = {}
        for name, field in planes.items():
            clims[name] = write_plane_png(
                viz_dir / f"{name}.png",
                field,
                source_rc[name],
                exclude_radius,
            )

        elapsed = float(time.monotonic() - started)
        write_json_atomic(
            viz_dir / "latest.json",
            {
                "ready": True,
                "step": int(step),
                "frame": int(frame_index),
                "nt": int(nt),
                "progress": float(step / max(nt, 1)),
                "elapsed": elapsed,
                "rev": int(elapsed * 1000) + frame_index,
                "sourceIndex": [ix_in, iy_in, iz_in],
                "sourceWorld": source_world,
                "receiverWorld": receiver_world,
                "extent": meta["extent"],
                "peaks": {name: safe_peak(field) for name, field in planes.items()},
                "clims": clims,
                "images": {
                    "xy": "xy.png",
                    "xz": "xz.png",
                    "yz": "yz.png",
                },
            },
        )
        print(
            f"--VIZ: frame={frame_index} step {step}/{nt} "
            f"({100.0 * step / nt:.1f}%) clim_xy={clims['xy']:.3e}",
            flush=True,
        )

    eng.save_outputs()
    eng.print_last_samples(5)
    print("--VIZ: simulation complete", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repository", type=Path, required=True,
                        help="PFFDTD repository root (contains python/)")
    parser.add_argument("--data-dir", type=Path, required=True,
                        help="Prepared PFFDTD job directory")
    parser.add_argument("--viz-dir", type=Path, required=True,
                        help="Directory for PNG slices / meta.json")
    parser.add_argument("--every", type=int, default=16,
                        help="Write a viz frame every N time steps")
    parser.add_argument("--max-side", type=int, default=192,
                        help="Ignored (kept for CLI compatibility)")
    parser.add_argument("--nthreads", type=int, default=None)
    args = parser.parse_args()
    if args.every < 1:
        parser.error("--every must be >= 1")

    python_root = args.repository.resolve() / "python"
    if not python_root.is_dir():
        raise SystemExit(f"PFFDTD python folder not found: {python_root}")
    sys.path.insert(0, str(python_root))

    try:
        run_with_viz(
            data_dir=args.data_dir.resolve(),
            viz_dir=args.viz_dir.resolve(),
            every=args.every,
            nthreads=args.nthreads,
        )
    except ImportError as error:
        raise SystemExit(
            "run this tool in the PFFDTD conda environment "
            f"(failed to import: {error})"
        ) from error


if __name__ == "__main__":
    main()
