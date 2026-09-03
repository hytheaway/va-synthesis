#pragma once

#include "va/core/types.hpp"

#include <string_view>

namespace va {

class PropagationSolver {
public:
    virtual ~PropagationSolver() = default;

    [[nodiscard]] virtual std::string_view name() const noexcept = 0;
    virtual void validate(const Scene& scene, const ImpulseResponseSettings& settings) const = 0;
    [[nodiscard]] virtual ImpulseResponseSet compute_impulse_responses(
        const Scene& scene,
        const ImpulseResponseSettings& settings) = 0;
};

} // namespace va
