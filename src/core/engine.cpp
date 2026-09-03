#include "va/core/engine.hpp"

#include "va/core/audio.hpp"

#include <stdexcept>
#include <utility>

namespace va {

Engine::Engine(std::unique_ptr<PropagationSolver> solver) {
    set_solver(std::move(solver));
}

void Engine::set_solver(std::unique_ptr<PropagationSolver> solver) {
    if (!solver) {
        throw std::invalid_argument("engine requires a propagation solver");
    }
    solver_ = std::move(solver);
}

const PropagationSolver& Engine::solver() const noexcept {
    return *solver_;
}

ImpulseResponseSet Engine::compute_impulse_responses(
    const Scene& scene, const ImpulseResponseSettings& settings) {
    solver_->validate(scene, settings);
    return solver_->compute_impulse_responses(scene, settings);
}

SimulationResult Engine::render(
    const Scene& scene, const AudioProgram& program,
    const ImpulseResponseSettings& simulation, const RenderSettings& rendering) {
    return audio::render_sources(
        scene, program, rendering, compute_impulse_responses(scene, simulation));
}

} // namespace va
