# va-synthesis

A modular virtual acoustics synthesis engine for geometrical and wave-based propagation methods.

The core is C++20 and can later be wrapped for Python, JUCE, game engines, or other hosts. Solver-neutral scene, source, receiver, and result types keep that choice from leaking into the scientific backends.

## Included

- A common `PropagationSolver` API and runtime-selectable `Engine`.
- Per-source/per-receiver impulse responses, FFT convolution, band-limited resampling, and complementary wave/geometrical crossover support.
- A geometrical backend boundary designed around the BRTLibrary submodule. Its reference free-field implementation provides distance attenuation and propagation delay while BRT ISM/SDN and a future ray tracer are integrated.
- A compact 7-point Cartesian reference FDTD solver with bandwidth-derived grid spacing, PFFDTD-compatible source scaling, trilinear interpolation, and a conservative Courant margin.
- `hytheaway/pffdtd`-provided scene export, job preparation, HDF5 result extraction, PFFDTD post-processing, and external Python/native-CPU execution adapters.
- A command-line example and dependency-free smoke tests.

The FDTD solver is not a production room simulator: it uses a uniform 3D Cartesian grid and does not yet model interior obstacles or frequency-dependent impedance.

## Build and run

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/va_demo fdtd
./build/va_demo geometrical
./build/va_demo hybrid
```

Install the CMake package for use from another repository:

```sh
cmake --install build --prefix /path/to/va-synthesis
```

The other project can then use `find_package(va-synthesis CONFIG REQUIRED)` and link `va::synthesis`. Set `CMAKE_PREFIX_PATH` to the install prefix when it is outside CMake's normal search locations.

Program audio is intentionally separate from the acoustic scene. A scene defines source positions and gains; `va::AudioProgram` supplies one mono signal per source. `va::ImpulseResponseSettings` controls RIR rate and duration, while `va::RenderSettings` independently controls output rate and whether the full reverberation tail is retained. File decoding and encoding, therefore, belong to the host application.

Use `-DVA_ENABLE_BRT=OFF` or `-DVA_ENABLE_PFFDTD=OFF` to disable either integration. PFFDTD execution needs its separate Python/Conda and HDF5 dependencies; the core library does not. See [docs/architecture.md](docs/architecture.md) and [docs/pffdtd.md](docs/pffdtd.md) for more info.
