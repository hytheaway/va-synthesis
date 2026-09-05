#include "va/wave/pffdtd_backend.hpp"

#include "va/core/audio.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace va::wave {
namespace {

std::string shell_quote(const std::filesystem::path& path) {
    const auto value = path.string();
    std::string quoted{"'"};
    for (const auto character : value) {
        if (character == '\0') {
            throw std::invalid_argument("path contains a null byte");
        }
        quoted += character == '\'' ? "'\\''" : std::string(1, character);
    }
    return quoted + "'";
}

void run_command(const std::string& command, const char* description) {
    if (std::system(command.c_str()) != 0) {
        throw std::runtime_error(std::string(description) + " failed");
    }
}

template <class Value>
void read_value(std::ifstream& stream, Value& value) {
    stream.read(reinterpret_cast<char*>(&value), sizeof(Value));
    if (!stream) {
        throw std::runtime_error("truncated PFFDTD bridge output");
    }
}

} // namespace

PFFDTDBackend::PFFDTDBackend(PFFDTDSettings settings) : settings_(std::move(settings)) {}

std::string_view PFFDTDBackend::name() const noexcept {
    return "PFFDTD external backend";
}

bool PFFDTDBackend::submodule_available() noexcept {
#if VA_HAS_PFFDTD
    return true;
#else
    return false;
#endif
}

void PFFDTDBackend::validate(
    const Scene& scene, const ImpulseResponseSettings& settings) const {
    if (!submodule_available()) {
        throw std::logic_error("PFFDTD submodule support is disabled");
    }
    if (scene.sources.size() != 1) {
        throw std::invalid_argument("one PFFDTD job represents exactly one source");
    }
    if (scene.receivers.empty()) {
        throw std::invalid_argument("scene requires at least one receiver");
    }
    if (settings.impulse_response_sample_rate <= 0.0 ||
        settings.impulse_response_duration_seconds <= 0.0 ||
        settings_.valid_bandwidth <= 0.0) {
        throw std::invalid_argument("sample rate, duration, and bandwidth must be positive");
    }
    if (settings_.valid_bandwidth >= settings.impulse_response_sample_rate * 0.5) {
        throw std::invalid_argument("PFFDTD valid bandwidth must be below output Nyquist");
    }
    if (settings_.bridge_script.empty() ||
        !std::filesystem::exists(settings_.bridge_script)) {
        throw std::invalid_argument("PFFDTD bridge script was not configured or does not exist");
    }
    for (const auto* filename : {"comms_out.h5", "sim_consts.h5"}) {
        if (!std::filesystem::exists(settings_.data_directory / filename)) {
            throw std::invalid_argument(
                "PFFDTD data directory is missing required prepared HDF5 files: " +
                (settings_.data_directory / filename).string());
        }
    }
    if (settings_.execution == PFFDTDExecution::prepared_output &&
        !std::filesystem::exists(settings_.data_directory / "sim_outs.h5")) {
        throw std::invalid_argument("prepared PFFDTD output sim_outs.h5 was not found");
    }
}

ImpulseResponseSet PFFDTDBackend::compute_impulse_responses(
    const Scene& scene, const ImpulseResponseSettings& settings) {
    validate(scene, settings);
    const auto repository = std::filesystem::absolute(settings_.repository);
    const auto data_directory = std::filesystem::absolute(settings_.data_directory);

    if (settings_.execution == PFFDTDExecution::python_cpu) {
        const auto command = "cd " + shell_quote(repository / "python") + " && " +
            shell_quote(settings_.python_executable) + " -m fdtd.sim_fdtd --data_dir " +
            shell_quote(data_directory);
        run_command(command, "PFFDTD Python simulation");
    } else if (settings_.execution == PFFDTDExecution::native_cpu_double ||
               settings_.execution == PFFDTDExecution::native_cpu_single) {
        const auto executable = repository / "c_cuda" /
            (settings_.execution == PFFDTDExecution::native_cpu_double
                 ? "fdtd_main_cpu_double.x"
                 : "fdtd_main_cpu_single.x");
        const auto command = "cd " + shell_quote(data_directory) + " && " +
                             shell_quote(executable);
        run_command(command, "PFFDTD native CPU simulation");
    }

    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto bridge_output = std::filesystem::temp_directory_path() /
                               ("va-pffdtd-" + std::to_string(nonce) + ".bin");
    const auto bridge = std::filesystem::absolute(settings_.bridge_script);
    const auto extract_command = shell_quote(settings_.python_executable) + " " +
        shell_quote(bridge) + " --data-dir " + shell_quote(data_directory) +
        " --output " + shell_quote(bridge_output) + " --pffdtd-python " +
        shell_quote(repository / "python") + " --output-rate " +
        std::to_string(settings.impulse_response_sample_rate) + " --maximum-frequency " +
        std::to_string(settings_.valid_bandwidth) +
        (settings_.apply_air_absorption ? " --air-absorption" : "");
    run_command(extract_command, "PFFDTD HDF5 extraction");

    std::ifstream input(bridge_output, std::ios::binary);
    std::array<char, 4> magic{};
    std::uint64_t receiver_count{};
    std::uint64_t frame_count{};
    double input_rate{};
    read_value(input, magic);
    read_value(input, receiver_count);
    read_value(input, frame_count);
    read_value(input, input_rate);
    if (magic != std::array<char, 4>{'V', 'A', 'I', 'R'} ||
        receiver_count != scene.receivers.size()) {
        throw std::runtime_error("PFFDTD output does not match the requested scene");
    }

    ImpulseResponseSet result{settings.impulse_response_sample_rate,
                              settings_.valid_bandwidth, 1,
                              scene.receivers.size(),
                              std::vector<AudioBuffer>(scene.receivers.size())};
    const auto output_frames = static_cast<std::size_t>(
        std::ceil(settings.impulse_response_sample_rate *
                  settings.impulse_response_duration_seconds));
    for (std::size_t receiver = 0; receiver < result.receiver_count; ++receiver) {
        AudioBuffer raw(static_cast<std::size_t>(frame_count));
        input.read(reinterpret_cast<char*>(raw.data()),
                   static_cast<std::streamsize>(raw.size() * sizeof(Sample)));
        if (!input) {
            throw std::runtime_error("truncated PFFDTD receiver data");
        }
        auto resampled = audio::resample(raw, input_rate,
                                         settings.impulse_response_sample_rate);
        resampled.resize(output_frames);
        for (auto& sample : resampled) {
            sample *= static_cast<Sample>(scene.sources.front().gain);
        }
        result.response(0, receiver) = std::move(resampled);
    }
    input.close();
    std::error_code ignored;
    std::filesystem::remove(bridge_output, ignored);
    return result;
}

} // namespace va::wave
