# PFFDTD integration

PFFDTD is pinned at `submodules/pffdtd` and remains an independently buildable MIT-licensed project. `va-synthesis` treats its files and HDF5 layout as a private backend detail.

## Distribution of roles

`va-synthesis` owns the public scene, source, receiver, RIR, convolution, and hybrid APIs. 

PFFDTD owns mesh voxelization, impedance fitting, Cartesian/FCC execution, energy-aware numerical machinery, and its post-processing filters.

The compact `FDTDSolver` is for fast tests and comparison. It is _not_ intended to duplicate or compete with PFFDTD's room solver.

## Prepare PFFDTD

_BE AWARE: This is outdated! You are better off following the instrutions in the README.md at the repo root._

Create the environment (using conda here):

```sh
conda env create -f submodules/pffdtd/python/conda_pffdtd.yml
conda activate pffdtd
```

The fork that `va-synthesis` uses is my fork of Brian Hamilton's original PFFDTD, which I slightly optimized for Arch with CPU rendering. PFFDTD's voxelizer (and its optional Mayavi visualization) uses nested multiprocessing workers that cannot be pickled. That is fine under Linux `fork`, but macOS and Windows default to `spawn` and fail with `Can't pickle local object '...process_voxels'`. `prepare_pffdtd_job.py` therefore forces `--processes 1` on those platforms. Visualization is not part of `va-synthesis`. The adapter stays optional so core `va-synthesis` remains portable.

## Prepare a job

Construct a `va::Scene` with triangle geometry, material IDs, 11 octave-band Sabine absorption coefficients, sources, and receivers. Export it with:

```cpp
va::wave::write_pffdtd_model(scene, "model.json");
```

Then run the adapter's preparation utility inside the PFFDTD environment:

```sh
python tools/prepare_pffdtd_job.py \
  --repository submodules/pffdtd \
  --model model.json \
  --output jobs/source-1 \
  --maximum-frequency 1000 \
  --points-per-wavelength 8 \
  --duration 2.0 \
  --source 1
```

On macOS and Windows, add `--processes 1` (the tool does this automatically if you omit it). Add `--fcc` for PFFDTD's FCC preparation and `--differentiate-source` for its single-precision safeguard. Prepare one cached job per source; voxelized scene data can be reused by future orchestration work, although PFFDTD's current setup script packages one source per job.

Native PFFDTD `model_export.json` files (for example `data/models/CTK_Church`) have surface names in `mats_hash` but no `va_materials` coefficients, but the same tool can be used as long as you point it as PFFDTD's fitted HDF5 materials instead of a `va-synthesis` scene dump:

```sh
python tools/prepare_pffdtd_job.py \
  --repository submodules/pffdtd \
  --model submodules/pffdtd/data/models/CTK_Church/model_export.json \
  --output jobs/ctk-church \
  --maximum-frequency 1400 \
  --points-per-wavelength 10.5 \
  --duration 3.0 \
  --source 1 \
  --processes 1 \
  --differentiate-source \
  --materials-dir submodules/pffdtd/data/materials \
  --material AcousticPanel=ctk_acoustic_panel.h5 \
  --material Altar=ctk_altar.h5 \
  --material Carpet=ctk_carpet.h5 \
  --material Ceiling=ctk_ceiling.h5 \
  --material Glass=ctk_window.h5 \
  --material PlushChair=ctk_chair.h5 \
  --material Tile=ctk_tile.h5 \
  --material Walls=ctk_walls.h5
```

## Execute and import

Choose `prepared_output` when `sim_outs.h5` already exists, `python_cpu` to run PFFDTD's Numba engine, or a native CPU mode after building the corresponding binary in `submodules/pffdtd/c_cuda`.

```cpp
va::wave::PFFDTDSettings pffdtd;
pffdtd.repository = "submodules/pffdtd";
pffdtd.data_directory = "jobs/source-1";
pffdtd.bridge_script = "tools/pffdtd_bridge.py";
pffdtd.python_executable = "/path/to/conda/env/bin/python";
pffdtd.execution = va::wave::PFFDTDExecution::python_cpu;
pffdtd.valid_bandwidth = 1000.0;
pffdtd.apply_air_absorption = true;

va::Engine engine(std::make_unique<va::wave::PFFDTDBackend>(pffdtd));
const va::AudioProgram program{48'000.0, {mono_samples}};
const auto rendered = engine.render(scene, program, {48'000.0, 2.0});
```

For an installed CMake package, `VA_SYNTHESIS_PFFDTD_BRIDGE_SCRIPT` contains the installed bridge path. Configure that path into the host application or otherwise assign it to `PFFDTDSettings::bridge_script`.

The bridge invokes PFFDTD's receiver recombination, DC/integration handling, valid-bandwidth low-pass, high-quality resampling, and optional modal air absorption before converting the HDF5 results into `va-synthesis` impulse responses.

The preprocessed PFFDTD job is authoritative for geometry and positions. The adapter checks source and receiver counts, but it cannot prove that the passed `va::Scene` coordinates match an arbitrary pre-existing HDF5 job.

## End-to-end validation

Because the PFFDTD integration test is dependency-heavy, it's optional. You can configure it with the Python interpreter from the PFFDTD environment:

```sh
cmake -S . -B build-pffdtd-e2e -G "Unix Makefiles" \
  -DVA_BUILD_APPS=OFF \
  -DVA_ENABLE_PFFDTD_E2E_TESTS=ON \
  -DVA_PFFDTD_PYTHON_EXECUTABLE=/path/to/pffdtd/environment/bin/python
cmake --build build-pffdtd-e2e -j
ctest --test-dir build-pffdtd-e2e --output-on-failure
```

The fixture creates a temporary 2m rigid room and runs scene export, single-process voxelization, Python CPU simulation, bridge post-processing, RIR import, and mono rendering. It checks the imported RIR's layout, sample rate, length, finite values, and non-silence. Temporary simulation data is removed after the test. **On restricted hosts, PFFDTD's voxelizer must be allowed to create a POSIX shared-memory segment even when configured for one process.**

## Current limitations

- End-to-end PFFDTD is dependency heavy
- The host must provision PFFDTD's Python/HDF5 dependencies itself (I've done my best to make it as easy as possible)
- Native PFFDTD binaries retain their upstream working-directory convention
- No GPU execution right now (it's left to PFFDTD and is not exposed in `PFFDTDExecution`)
