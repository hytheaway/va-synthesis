# Geometrical acoustics

`va::geometrical::BRTGeometricalSolver` exposes four strategies through
`BRTSettings::method`:

- `free_field` produces the direct arrival with propagation delay and distance
  attenuation.
- `image_source` maps `Scene::geometry` into a BRT room and uses BRTLibrary's
  recursive image-source generation, polygon visibility, and material-derived
  reflection gains. Axis-aligned shoebox meshes are collapsed to six rectangular
  walls so coplanar triangles do not emit duplicate images. BRT's algorithm
  assumes a convex room with consistently oriented boundary faces. Optimized
  builds use a compatibility header that returns BRT wall normals by value;
  upstream returns a reference to a local, which drops reflected arrivals.
- `scattering_delay_network` runs BRTLibrary's six-wall SDN. It uses
  `Scene::bounds` as the shoebox, requires sources and receivers strictly
  inside those bounds, and requires an RIR sample rate above 32 kHz because its
  absorption filters extend to the 16 kHz octave band. Until scene surfaces
  carry semantic wall roles, the adapter applies the mean material absorption
  across the scene to each of the six SDN walls.
- `ray_tracing` emits deterministic Fibonacci-distributed rays, traces
  specular reflections over triangle geometry, and captures paths with a
  spherical receiver. Increase `ray_count` or `receiver_radius` to reduce
  sparse estimates; decrease the radius and increase the ray count for better
  spatial precision.

All room modes honor `enable_direct_path` and `enable_reverberation`.
`reflection_order` controls image-source recursion depth and ray bounces;
`maximum_paths` prevents accidental image-tree explosion. Material absorption
uses the nine octave bands from 62.5 Hz through 16 kHz in the scene's existing
11-band coefficients. The current image-source and ray RIR taps collapse those
bands to a broadband reflection gain; frequency-dependent path filtering is a
planned refinement. The SDN applies BRT's frequency-dependent wall filters.

Example:

```cpp
va::geometrical::BRTSettings configuration;
configuration.method = va::geometrical::Method::image_source;
configuration.reflection_order = 2;
configuration.enable_direct_path = true;
configuration.enable_reverberation = true;

va::Engine engine(
    std::make_unique<va::geometrical::BRTGeometricalSolver>(configuration));
const auto responses = engine.compute_impulse_responses(
    scene, va::ImpulseResponseSettings{48'000.0, 1.5});
```

The stochastic ray tracer is deterministic for a given `random_seed`, making
room changes and regression tests repeatable. It is currently a CPU reference
implementation with brute-force triangle intersection and an uncalibrated
receiver-capture energy estimate; an acceleration structure, physical
calibration, and diffuse-scattering model are natural next steps for large
rooms.
