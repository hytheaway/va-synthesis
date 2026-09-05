# va-synthesis

A modular virtual acoustics synthesis engine for geometrical and wave-based propagation methods.

The core is C++20 and can later be wrapped for Python, JUCE, game engines, or other hosts. Solver-neutral scene, source, receiver, and result types keep that choice from leaking into backends.

## Included

- A common `PropagationSolver` API and runtime-selectable `Engine`.
- Per-source/per-receiver impulse responses, FFT convolution, band-limited resampling, and complementary wave/geometrical crossover support.
- A geometrical backend with free-field propagation, BRTLibrary image sources, BRTLibrary scattering delay networks, and stochastic specular ray tracing. It models triangle room geometry, material absorption, reflections, and reverberant paths.
- A minimal 7-point Cartesian FDTD solver with bandwidth-derived grid spacing, PFFDTD-compatible source scaling, trilinear interpolation, and a conservative Courant margin.
- `hytheaway/pffdtd`-provided scene export, job preparation, HDF5 result extraction, PFFDTD post-processing, and external Python/native-CPU execution adapters.
- A command-line example and dependency-free smoke tests.

_The minimal FDTD is not a production room simulator: it's based on a uniform 3D Cartesian grid and does not model interior obstacles or frequency-dependent impedance._

## Build and run

First, make sure PFFDTD has everything it needs by creating a virtual Python environment and installing the relevant packages. Conda with miniforge3 is recommended.

```sh
cd submodules/pffdtd
conda create -n va-pffdtd -c conda-forge \
    python=3.11.15 \
    numpy=1.26.4 \
    h5py \
    scipy \
    numba \
    resampy \
    tqdm \
    psutil \
    memory_profiler \
    matplotlib \
    pytest
conda activate va-pffdtd
```

Validate the environment from the repository root:

```sh
python -m pip check
python -c "import numpy, h5py, scipy, numba, resampy, tqdm, psutil, memory_profiler, matplotlib"

python tools/prepare_pffdtd_job.py --help
python tools/pffdtd_bridge.py --help
```

If the CLI help menu appears for each of these files, the packages have been installed successfully. Then, check imports across the PFFDTD integration:

```sh
PYTHONPATH="$PWD/submodules/pffdtd/python" \
python -c "from sim_setup import sim_setup; from fdtd.sim_fdtd import SimEngine; from fdtd.process_outputs import ProcessOutputs; print('PFFDTD imports passed')"
```

After this has succeeded, you build properly:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure

./build/va_demo fdtd
./build/va_demo geometrical
./build/va_demo image-source
./build/va_demo sdn
./build/va_demo ray-tracing
./build/va_demo hybrid
```

Install the CMake package for use from another repository:

```sh
cmake --install build --prefix /path/to/va-synthesis
```

The other project can then use `find_package(va-synthesis CONFIG REQUIRED)` and link `va::synthesis`. Set `CMAKE_PREFIX_PATH` to the install prefix when it is outside CMake's normal search locations.

Program audio is intentionally separate from the acoustic scene. A scene defines source positions and gains; `va::AudioProgram` supplies one mono signal per source. `va::ImpulseResponseSettings` controls RIR rate and duration, while `va::RenderSettings` independently controls output rate and whether the full reverberation tail is retained. File decoding and encoding, therefore, belong to the host application.

Use `-DVA_ENABLE_BRT=OFF` or `-DVA_ENABLE_PFFDTD=OFF` to disable either integration. PFFDTD execution needs its separate Python/Conda and HDF5 dependencies; the core library does not. See [docs/architecture.md](docs/architecture.md), [docs/geometrical.md](docs/geometrical.md), and [docs/pffdtd.md](docs/pffdtd.md) for more info.

## License

This project is licensed under the GNU General Public License v3.0. See [LICENSE](LICENSE).

Building with BRTLibrary enabled produces a combined work under GPL-3.0. Use `-DVA_ENABLE_BRT=OFF` if you need a build that does not link BRT.

## Third-party attributions

A short inventory is also in [THIRD_PARTY.md](THIRD_PARTY.md).

### BRTLibrary (GPL-3.0-or-later)

[BRTLibrary](https://github.com/GrupoDiana/BRTLibrary) (Binaural Rendering Toolbox) is used for image-source and scattering-delay-network geometrical acoustics. It is licensed under the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

Copyright for each module belongs to its respective authors. The library is developed by a team coordinated by [Arcadio Reyes-Lecuona](https://github.com/areyesl) ([Diana Research Group, University of Málaga](https://www.diana.uma.es/3di-diana/)) and [Lorenzo Picinali](https://github.com/lpicinali) ([Audio Experience Design Team, Imperial College London](https://www.axdesign.co.uk/)). Current authors include [María Cuevas-Rodríguez](https://github.com/mariacuevas), [Daniel González-Toledo](https://github.com/dgonzalezt), and [Luis Molina-Tanco](https://github.com/lmtanco). The SDN environment model includes contributions from [Marco Fontana](https://github.com/MarcoFontana) ([Laboratorio di Informatica Musicale, Università degli Studi di Milano](https://www.lim.di.unimi.it/)).

BRTLibrary includes code from the [3D Tune-In Toolkit](https://github.com/3DTune-In/3dti_AudioToolkit), also under GPLv3, copyright University of Málaga and Imperial College London.

Vendored at [`submodules/BRTLibrary`](submodules/BRTLibrary) (fork: [hytheaway/BRTLibrary](https://github.com/hytheaway/BRTLibrary)). The full license text is in [`submodules/BRTLibrary/LICENSE`](submodules/BRTLibrary/LICENSE).

### PFFDTD (MIT)

[PFFDTD](https://github.com/bsxfun/pffdtd) (pretty fast FDTD) by [Brian Hamilton](https://github.com/bsxfun) is used for wave-based room simulation setup, execution, and post-processing. It is licensed under the MIT License:

```
Copyright (c) 2021 bsxfun

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

Vendored at [`submodules/pffdtd`](submodules/pffdtd) (fork: [hytheaway/pffdtd](https://github.com/hytheaway/pffdtd)). The same notice is in [`submodules/pffdtd/LICENSE`](submodules/pffdtd/LICENSE). SketchUp models bundled with PFFDTD may use different licenses; see the README files in that project's `data/models` folders.
