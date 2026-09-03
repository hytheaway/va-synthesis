#pragma once

#include "va/core/propagation_solver.hpp"

#include <memory>
#include <string_view>

namespace va::hybrid {

struct HybridSettings {
    double crossover_frequency{500.0};
};

class HybridSolver final : public PropagationSolver {
public:
    HybridSolver(std::unique_ptr<PropagationSolver> wave_solver,
                 std::unique_ptr<PropagationSolver> geometrical_solver,
                 HybridSettings settings = {});

    [[nodiscard]] std::string_view name() const noexcept override;
    void validate(const Scene& scene, const ImpulseResponseSettings& settings) const override;
    [[nodiscard]] ImpulseResponseSet compute_impulse_responses(
        const Scene& scene, const ImpulseResponseSettings& settings) override;
private:
    std::unique_ptr<PropagationSolver> wave_solver_;
    std::unique_ptr<PropagationSolver> geometrical_solver_;
    HybridSettings settings_;
};

} // namespace va::hybrid
