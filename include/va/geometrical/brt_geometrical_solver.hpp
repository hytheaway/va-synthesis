#pragma once

#include "va/core/propagation_solver.hpp"

#include <cstddef>
#include <cstdint>
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
    std::size_t maximum_paths{100'000};
    bool propagation_delay{true};
    bool distance_attenuation{true};
    bool enable_direct_path{true};
    bool enable_reverberation{true};
    double default_absorption{0.2};
    std::size_t ray_count{32'768};
    double receiver_radius{0.15};
    std::uint64_t random_seed{0x56415f524159ULL};
};

// Stable VA-facing adapter boundary. BRT types remain private to the implementation.
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
