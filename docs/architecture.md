# Architecture

The engine separates *what* is simulated from *how* propagation is solved. `va::Scene`, `va::AudioProgram`, `va::ImpulseResponseSet`, and `va::SimulationResult` are the interchange model; `va::PropagationSolver` is the backend; and `va::Engine` owns the selected backend. Applications don't need to know if a result came from geometrical acoustics, FDTD, or a future FEM/BEM implementation.

```text
    application / language bindings
                  |
              va::Engine
                  |
                 va::PropagationSolver
                 /        |          \
       BRT/geometrical  wave RIR  (future) FEM/BEM
                          /  \
             reference FDTD  PFFDTD adapter
                    \       /
                    hybrid crossover
                           |
                  convolution/rendering
```

## Directory layout

```text
include/va/core/          solver-neutral public API
include/va/geometrical/   geometrical backend API and BRT adapter
include/va/wave/          wave-solver APIs
include/va/hybrid/        wave/geometrical RIR combination
src/                      backend implementation
apps/                     thin hosts
tests/                    smoke tests
submodules/BRTLibrary/    BRTLibrary fork; do not modify for va-synthesis features
submodules/pffdtd/        PFFDTD fork; do not modify for va-synthesis features
```

## Geometrical path

`BRTGeometricalSolver` prevents BRT-specific graph and audio-buffer types from becoming engine-wide types. Selectable are free field, BRTLibrary's recursive image source model, BRTLibrary's scattering delay network, and a basic stochastic specular ray tracer. The image-source adapter maps triangle geometry into BRT rooms and uses BRT visibility and nine-band reflection gains. The SDN uses the scene bounds as a shoebox and generates a dense reverberant response. The ray tracer handles triangle meshes directly and uses a spherical receiver with deterministic ray directions. See `geometrical.md` for more info.

BRTLibrary is GPL-3.0-or-later. Before distributing a linked product, choose project licensing and deployment boundaries that are compatible with BRT's license.

## Wave path

`FDTDSolver` is the compact, deterministic reference implementation. It solves the scalar wave equation on a uniform Cartesian grid. Grid spacing is derived from maximum valid frequency and points per wavelength. Its internal rate is then derived from a `0.999` margin on the 3D Courant stability condition:

```text
c * dt / dx <= 1 / sqrt(3)
```

It returns an `ImpulseResponseSet` at the requested RIR rate. Dry mono program audio is supplied separately and rendered afterward using FFT convolution and band-limited resampling. RIR duration is independent of program and rendered file duration, and rendering can retain or truncate the reverberation tail. The implementation uses two full pressure grids, trilinear source/receiver interpolation, and PFFDTD-compatible Cartesian source scaling.

`PFFDTDBackend` is the production-oriented path. `va-synthesis` exports neutral triangle geometry and 11-band material absorption to PFFDTD's JSON schema. PFFDTD owns voxelization, frequency-dependent impedance boundaries, Cartesian/FCC kernels, and raw HDF5 output. `va-synthesis`'s bridge calls PFFDTD post-processing and converts the result into `ImpulseResponseSet`. See `pffdtd.md` for more info.

`HybridSolver` applies a complementary split to the low-frequency wave RIR and high-frequency geometrical RIR. The current one-pole crossover is a bare minimum. More ideal would be a higher-order phase-matched crossover and direct-arrival alignment.

If I end up doing it, FEM and BEM should implement `PropagationSolver` directly. Shared mesh, material, sparse-linear-algebra, and frequency-domain concepts should only be promoted into `core` after at least two backends demonstrate the same need. That would avoid forcing FDTD's grid model onto unstructured-meshes.

## Remaining increments

1. Validate BRT ISM arrival times against shoebox-room suite (and add frequency-dependent path filtering to the current broadband RIR taps)
2. Provision opt-in PFFDTD end to end fixture
3. (Maybe) replace the reference solver's pedagogical boundary with ABC
4. Add partitioned FFT convolution for bound-memory streaming (the current whole-buffer FFT path is only really intended for offline rendering)
5. (Maybe) add OpenMP to the reference CPU solver; use PFFDTD for FCC/CUDA execution
6. (Maybe) add optional Python bindings after the C++ API is complete
