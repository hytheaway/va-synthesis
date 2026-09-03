#pragma once

#include "va/core/propagation_solver.hpp"

#include <cstddef>
#include <string_view>

namespace va::geometrical {

enum class Method {
    free_field,
    image_source,
    scattering_delay_network,
    ray_tracing,
};

struct BRTSettings {
    Method method{Method::free_field};
    std::size_t reflection_order{2};
    bool propagation_delay{true};
    bool distance_attenuation{true};
};

// Stable VA-facing adapter boundary. The free-field path is operational now;
// BRT room models and a future ray tracer can be connected behind this class
// without exposing BRT types to the rest of the engine.
class BRTGeometricalSolver final : public PropagationSolver {
public:
    explicit BRTGeometricalSolver(BRTSettings settings = {});

    [[nodiscard]] std::string_view name() const noexcept override;
    void validate(const Scene& scene, const ImpulseResponseSettings& settings) const override;
    [[nodiscard]] ImpulseResponseSet compute_impulse_responses(
        const Scene& scene,
        const ImpulseResponseSettings& settings) override;
    [[nodiscard]] const BRTSettings& settings() const noexcept;
    [[nodiscard]] static bool brt_headers_available() noexcept;

private:
    BRTSettings settings_;
};

} // namespace va::geometrical
