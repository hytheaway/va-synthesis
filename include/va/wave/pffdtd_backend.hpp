#pragma once

#include "va/core/propagation_solver.hpp"

#include <filesystem>
#include <string>
#include <string_view>

namespace va::wave {

enum class PFFDTDExecution {
    prepared_output,
    python_cpu,
    native_cpu_double,
    native_cpu_single,
};

struct PFFDTDSettings {
    std::filesystem::path repository{"submodules/pffdtd"};
    std::filesystem::path data_directory;
    std::filesystem::path bridge_script;
    std::filesystem::path python_executable{"python"};
    PFFDTDExecution execution{PFFDTDExecution::prepared_output};
    double valid_bandwidth{1'000.0};
    bool apply_air_absorption{false};
};

// Adapter for a PFFDTD job prepared by its Python voxelization pipeline.
// PFFDTD currently prepares one source and any number of receivers per job.
class PFFDTDBackend final : public PropagationSolver {
public:
    explicit PFFDTDBackend(PFFDTDSettings settings);

    [[nodiscard]] std::string_view name() const noexcept override;
    void validate(const Scene& scene, const ImpulseResponseSettings& settings) const override;
    [[nodiscard]] ImpulseResponseSet compute_impulse_responses(
        const Scene& scene, const ImpulseResponseSettings& settings) override;
    [[nodiscard]] static bool submodule_available() noexcept;

private:
    PFFDTDSettings settings_;
};

} // namespace va::wave
