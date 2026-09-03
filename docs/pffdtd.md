# PFFDTD integration

PFFDTD is pinned at `submodules/pffdtd` and remains an independently buildable
MIT-licensed project. `va-synthesis` treats its files and HDF5 layout as a
private backend detail.

## Responsibilities

VA owns the public scene, source, receiver, RIR, convolution, and hybrid APIs.
PFFDTD owns mesh voxelization, impedance fitting, Cartesian/FCC execution,
energy-aware numerical machinery, and its post-processing filters.

The compact `FDTDSolver` remains useful for fast tests and comparison. It is
not intended to duplicate PFFDTD's room solver.

## Provision PFFDTD

Create the environment described by the pinned fork:

```sh
conda env create -f submodules/pffdtd/python/conda_pffdtd.yml
conda activate pffdtd
```

The fork's README notes current Linux/Arch constraints. The external adapter is
optional so core VA builds remain portable.

## Prepare a job

Construct a `va::Scene` with triangle geometry, material IDs, 11 octave-band
Sabine absorption coefficients, sources, and receivers. Export it with:

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

Add `--fcc` for PFFDTD's FCC preparation and `--differentiate-source` for its
single-precision safeguard. Prepare one cached job per source; voxelized scene
data can be reused by future orchestration work, although PFFDTD's current
setup script packages one source per job.

## Execute and import

Choose `prepared_output` when `sim_outs.h5` already exists, `python_cpu` to run
PFFDTD's Numba engine, or a native CPU mode after building the corresponding
binary in `submodules/pffdtd/c_cuda`.

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

For an installed CMake package, `VA_SYNTHESIS_PFFDTD_BRIDGE_SCRIPT` contains
the installed bridge path. Configure that path into the host application or
otherwise assign it to `PFFDTDSettings::bridge_script`.

The bridge invokes PFFDTD's receiver recombination, DC/integration handling,
valid-bandwidth low-pass, high-quality resampling, and optional modal air
absorption before converting the HDF5 results into VA impulse responses.

The preprocessed PFFDTD job is authoritative for geometry and positions. The
adapter checks source and receiver counts, but it cannot prove that the passed
`va::Scene` coordinates match an arbitrary pre-existing HDF5 job.

## End-to-end validation

The dependency-heavy PFFDTD integration test is opt-in. Configure it with the
Python interpreter from the PFFDTD environment:

```sh
cmake -S . -B build-pffdtd-e2e -G "Unix Makefiles" \
  -DVA_BUILD_APPS=OFF \
  -DVA_ENABLE_PFFDTD_E2E_TESTS=ON \
  -DVA_PFFDTD_PYTHON_EXECUTABLE=/path/to/pffdtd/environment/bin/python
cmake --build build-pffdtd-e2e -j
ctest --test-dir build-pffdtd-e2e --output-on-failure
```

The fixture creates a temporary two-metre rigid room and runs scene export,
single-process voxelization, Python CPU simulation, bridge post-processing,
RIR import, and mono rendering. It checks the imported RIR's layout, sample
rate, length, finite values, and non-silence. Temporary simulation data is
removed after the test. On restricted hosts, PFFDTD's voxelizer must be allowed
to create a POSIX shared-memory segment even when configured for one process.

## Current limitations

- End-to-end PFFDTD execution is not part of the dependency-free test suite.
- The host must provision PFFDTD's Python/HDF5 dependencies.
- Native PFFDTD binaries retain their upstream working-directory convention.
- Multi-source orchestration currently means one backend/job per source.
- GPU execution is left to PFFDTD and is not yet exposed in `PFFDTDExecution`.
