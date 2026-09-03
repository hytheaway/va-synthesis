#pragma once

#include "va/core/types.hpp"

namespace va::audio {

[[nodiscard]] AudioBuffer convolve(const AudioBuffer& signal, const AudioBuffer& impulse_response,
                                   std::size_t output_frames = 0);
[[nodiscard]] AudioBuffer resample(const AudioBuffer& input, double input_rate,
                                   double output_rate);
[[nodiscard]] SimulationResult render_sources(
    const Scene& scene, const AudioProgram& program, const RenderSettings& settings,
    const ImpulseResponseSet& impulse_responses);

// Complementary one-pole split. The low and high outputs sum to the input.
[[nodiscard]] AudioBuffer low_pass(const AudioBuffer& input, double sample_rate,
                                   double cutoff_hz);
[[nodiscard]] AudioBuffer high_pass_complement(const AudioBuffer& input, double sample_rate,
                                               double cutoff_hz);

} // namespace va::audio
