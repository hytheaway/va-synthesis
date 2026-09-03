#pragma once

#include "va/core/propagation_solver.hpp"

#include <memory>

namespace va {

class Engine {
public:
    explicit Engine(std::unique_ptr<PropagationSolver> solver);

    void set_solver(std::unique_ptr<PropagationSolver> solver);
    [[nodiscard]] const PropagationSolver& solver() const noexcept;
    [[nodiscard]] ImpulseResponseSet compute_impulse_responses(
        const Scene& scene,
        const ImpulseResponseSettings& settings);
    [[nodiscard]] SimulationResult render(
        const Scene& scene,
        const AudioProgram& program,
        const ImpulseResponseSettings& simulation,
        const RenderSettings& rendering = {});

private:
    std::unique_ptr<PropagationSolver> solver_;
};

} // namespace va
