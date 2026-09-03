#pragma once

#include "va/core/propagation_solver.hpp"

#include <string_view>

namespace va::wave {

enum class GridScheme {
    cartesian_7_point,
    fcc_13_point,
};

struct FDTDSettings {
    double maximum_frequency{1'000.0};
    double points_per_wavelength{8.0};
    double courant_safety_factor{0.999};
    GridScheme scheme{GridScheme::cartesian_7_point};
    // Energy absorption at the six faces of the rectangular boundary: 0 reflects,
    // 1 is maximally absorbing in this first-order boundary model.
    double boundary_absorption{0.2};
};

struct GridParameters {
    double cell_size{};
    double internal_sample_rate{};
    double courant_number{};
};

class FDTDSolver final : public PropagationSolver {
public:
    explicit FDTDSolver(FDTDSettings settings = {});

    [[nodiscard]] std::string_view name() const noexcept override;
    void validate(const Scene& scene, const ImpulseResponseSettings& settings) const override;
    [[nodiscard]] ImpulseResponseSet compute_impulse_responses(
        const Scene& scene,
        const ImpulseResponseSettings& settings) override;
    [[nodiscard]] const FDTDSettings& settings() const noexcept;
    [[nodiscard]] GridParameters grid_parameters(const Scene& scene) const;

private:
    FDTDSettings settings_;
};

} // namespace va::wave
