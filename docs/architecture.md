# Architecture

## Design goals

The engine separates *what* is simulated from *how* propagation is solved.
`va::Scene`, `va::AudioProgram`, `va::ImpulseResponseSet`, and
`va::SimulationResult` are the interchange model;
`va::PropagationSolver` is the backend contract; and `va::Engine` owns the
selected backend. Applications therefore do not need to know whether a result
came from geometrical acoustics, FDTD, or a future FEM/BEM implementation.

```text
application / future language bindings
                  |
              va::Engine
                  |
                 va::PropagationSolver
                 /        |         \
       BRT/geometrical  wave RIR   future FEM/BEM
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
include/va/geometrical/   geometrical backend API and BRT adapter boundary
include/va/wave/          wave-solver APIs
include/va/hybrid/        wave/geometrical RIR combination
src/                      backend implementation details
apps/                     thin executable hosts
tests/                    fast contract and numerical smoke tests
submodules/BRTLibrary/    upstream BRTLibrary; do not modify for VA features
submodules/pffdtd/         pinned PFFDTD fork; do not modify for VA features
```

## Geometrical path

`BRTGeometricalSolver` prevents BRT-specific graph and audio-buffer types from
becoming engine-wide types. With BRT enabled, coordinates are translated to
BRT vectors inside the backend; the free-field route is usable as a reference
and tests the shared contract. The next integration step is to add private adapter
implementations for BRT's `ISMEnvironmentModel` and `SDNEnvironmentModel`,
mapping the neutral scene into BRT rooms, sources, and listeners. Ray tracing
should be another strategy behind the same class or a sibling solver if its
configuration diverges substantially.

BRTLibrary is GPL-3.0-or-later. Before distributing a linked product, choose
project licensing and deployment boundaries that are compatible with BRT's
license.

## Wave path

`FDTDSolver` is the compact, deterministic reference implementation. It solves
the scalar wave equation on a uniform Cartesian grid. Grid spacing is derived
from maximum valid frequency and points per wavelength. Its internal rate is
then derived from a `0.999` margin on the 3D Courant stability condition:

```text
c * dt / dx <= 1 / sqrt(3)
```

It returns an `ImpulseResponseSet` at the requested RIR rate. Dry mono program
audio is supplied separately and rendered afterward using FFT convolution and
band-limited resampling. RIR duration is independent of program and rendered
file duration, and rendering can retain or truncate the reverberation tail.
The implementation uses two full
pressure grids, trilinear source/receiver interpolation, and PFFDTD-compatible
Cartesian source scaling.

`PFFDTDBackend` is the production-oriented path. VA exports neutral triangle
geometry and 11-band material absorption to PFFDTD's JSON schema. PFFDTD owns
voxelization, frequency-dependent impedance boundaries, Cartesian/FCC kernels,
and raw HDF5 output. VA's bridge calls PFFDTD post-processing and converts the
result into `ImpulseResponseSet`. See `pffdtd.md`.

`HybridSolver` applies a complementary split to the low-frequency wave RIR and
high-frequency geometrical RIR. The current one-pole crossover is a functional
integration baseline; production work still needs a higher-order phase-matched
crossover and explicit direct-arrival alignment.

FEM and BEM should implement `PropagationSolver` directly. Shared mesh,
material, sparse-linear-algebra, and frequency-domain concepts should only be
promoted into `core` after at least two backends demonstrate the same need.
That avoids forcing FDTD's grid model onto unstructured-mesh methods.

## Remaining increments

1. Implement BRT ISM end-to-end and compare its impulse response with an
   analytic shoebox-room case.
2. Add an end-to-end PFFDTD fixture once the PFFDTD Conda environment is
   provisioned in CI; the current tests validate the shared Cartesian update.
3. Replace the reference solver's pedagogical boundary with a validated ABC.
4. Add partitioned FFT convolution for bounded-memory streaming; the current
   whole-buffer FFT path is intended for offline rendering.
5. Add OpenMP to the reference CPU solver; use PFFDTD for FCC/CUDA execution.
6. Add optional Python bindings after the C++ API settles.
