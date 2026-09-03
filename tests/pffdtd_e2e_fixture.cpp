#include "va/core/audio.hpp"
#include "va/core/engine.hpp"
#include "va/wave/pffdtd_backend.hpp"
#include "va/wave/pffdtd_scene.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string_view>

namespace {

va::Scene room_scene() {
    va::Scene scene;
    scene.bounds = {{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    scene.sources.push_back({{0.75, 1.0, 1.0}, 1.0});
    scene.receivers.push_back({{1.25, 1.0, 1.0}});

    const auto add = [&scene](va::Vec3 a, va::Vec3 b, va::Vec3 c) {
        scene.geometry.push_back({{{a, b, c}}, "_RIGID", 0});
    };
    add({0, 0, 0}, {0, 2, 0}, {2, 2, 0});
    add({0, 0, 0}, {2, 2, 0}, {2, 0, 0});
    add({0, 0, 2}, {2, 2, 2}, {0, 2, 2});
    add({0, 0, 2}, {2, 0, 2}, {2, 2, 2});
    add({0, 0, 0}, {2, 0, 2}, {0, 0, 2});
    add({0, 0, 0}, {2, 0, 0}, {2, 0, 2});
    add({0, 2, 0}, {0, 2, 2}, {2, 2, 2});
    add({0, 2, 0}, {2, 2, 2}, {2, 2, 0});
    add({0, 0, 0}, {0, 0, 2}, {0, 2, 2});
    add({0, 0, 0}, {0, 2, 2}, {0, 2, 0});
    add({2, 0, 0}, {2, 2, 2}, {2, 0, 2});
    add({2, 0, 0}, {2, 2, 0}, {2, 2, 2});
    return scene;
}

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 3 && std::string_view(argv[1]) == "export") {
            va::wave::write_pffdtd_model(room_scene(), argv[2]);
            return 0;
        }
        if (argc != 6 || std::string_view(argv[1]) != "validate") {
            throw std::invalid_argument(
                "usage: fixture export MODEL | validate REPOSITORY JOB BRIDGE PYTHON");
        }

        auto scene = room_scene();
        va::wave::PFFDTDSettings settings;
        settings.repository = std::filesystem::path(argv[2]);
        settings.data_directory = std::filesystem::path(argv[3]);
        settings.bridge_script = std::filesystem::path(argv[4]);
        settings.python_executable = std::filesystem::path(argv[5]);
        settings.execution = va::wave::PFFDTDExecution::python_cpu;
        settings.valid_bandwidth = 200.0;

        va::Engine engine(std::make_unique<va::wave::PFFDTDBackend>(settings));
        const auto responses = engine.compute_impulse_responses(
            scene, va::ImpulseResponseSettings{8'000.0, 0.05});
        require(responses.source_count == 1, "unexpected PFFDTD source count");
        require(responses.receiver_count == 1, "unexpected PFFDTD receiver count");
        require(responses.sample_rate == 8'000.0, "unexpected PFFDTD sample rate");
        require(responses.response(0, 0).size() == 400, "unexpected PFFDTD RIR length");

        bool finite = true;
        bool nonzero = false;
        for (const auto sample : responses.response(0, 0)) {
            finite = finite && std::isfinite(sample);
            nonzero = nonzero || std::abs(sample) > 1.0e-10F;
        }
        require(finite, "PFFDTD RIR contains non-finite samples");
        require(nonzero, "PFFDTD RIR is silent");

        const va::AudioProgram program{8'000.0, {{1.0F}}};
        const auto rendered = va::audio::render_sources(
            scene, program, va::RenderSettings{8'000.0, true}, responses);
        require(rendered.receiver_signals.size() == 1, "unexpected rendered receiver count");
        require(!rendered.receiver_signals.front().empty(), "rendered signal is empty");
        std::cout << "PFFDTD end-to-end fixture passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "PFFDTD fixture failure: " << error.what() << '\n';
        return 1;
    }
}
