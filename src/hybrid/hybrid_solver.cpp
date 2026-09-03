#include "va/hybrid/hybrid_solver.hpp"

#include "va/core/audio.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace va::hybrid {

HybridSolver::HybridSolver(std::unique_ptr<PropagationSolver> wave_solver,
                           std::unique_ptr<PropagationSolver> geometrical_solver,
                           HybridSettings settings)
    : wave_solver_(std::move(wave_solver)),
      geometrical_solver_(std::move(geometrical_solver)),
      settings_(settings) {
    if (!wave_solver_ || !geometrical_solver_) {
        throw std::invalid_argument("hybrid solver requires wave and geometrical solvers");
    }
}

std::string_view HybridSolver::name() const noexcept {
    return "hybrid wave/geometrical acoustics";
}

void HybridSolver::validate(
    const Scene& scene, const ImpulseResponseSettings& settings) const {
    if (settings_.crossover_frequency <= 0.0 ||
        settings_.crossover_frequency >= settings.impulse_response_sample_rate * 0.5) {
        throw std::invalid_argument("hybrid crossover must be between zero and Nyquist");
    }
    wave_solver_->validate(scene, settings);
    geometrical_solver_->validate(scene, settings);
}

ImpulseResponseSet HybridSolver::compute_impulse_responses(
    const Scene& scene, const ImpulseResponseSettings& settings) {
    validate(scene, settings);
    auto wave = wave_solver_->compute_impulse_responses(scene, settings);
    auto geometrical = geometrical_solver_->compute_impulse_responses(scene, settings);
    if (wave.sample_rate != geometrical.sample_rate ||
        wave.source_count != geometrical.source_count ||
        wave.receiver_count != geometrical.receiver_count) {
        throw std::runtime_error("hybrid backend impulse-response layouts do not match");
    }
    if (settings_.crossover_frequency > wave.valid_bandwidth) {
        throw std::invalid_argument("hybrid crossover exceeds wave solver valid bandwidth");
    }

    ImpulseResponseSet result{wave.sample_rate,
                              std::max(wave.valid_bandwidth, geometrical.valid_bandwidth),
                              wave.source_count, wave.receiver_count,
                              std::vector<AudioBuffer>(wave.responses.size())};
    for (std::size_t index = 0; index < result.responses.size(); ++index) {
        auto low = audio::low_pass(wave.responses[index], wave.sample_rate,
                                   settings_.crossover_frequency);
        auto high = audio::high_pass_complement(geometrical.responses[index],
                                                geometrical.sample_rate,
                                                settings_.crossover_frequency);
        const auto frames = std::max(low.size(), high.size());
        low.resize(frames);
        high.resize(frames);
        result.responses[index].resize(frames);
        for (std::size_t frame = 0; frame < frames; ++frame) {
            result.responses[index][frame] = low[frame] + high[frame];
        }
    }
    return result;
}

} // namespace va::hybrid
