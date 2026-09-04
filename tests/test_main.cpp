#include "va/core/engine.hpp"
#include "va/core/audio.hpp"
#include "va/geometrical/brt_geometrical_solver.hpp"
#include "va/hybrid/hybrid_solver.hpp"
#include "va/wave/fdtd_solver.hpp"
#include "va/wave/pffdtd_backend.hpp"
#include "va/wave/pffdtd_scene.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numeric>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

va::Scene test_scene() {
    va::Scene scene;
    scene.bounds = {{0.0, 0.0, 0.0}, {2.0, 2.0, 2.0}};
    scene.sources.push_back({{0.5, 1.0, 1.0}, 1.0});
    scene.receivers.push_back({{1.5, 1.0, 1.0}});
    return scene;
}

va::Scene shoebox_scene() {
    va::Scene scene;
    scene.bounds = {{0.0, 0.0, 0.0}, {4.0, 3.0, 2.5}};
    scene.sources.push_back({{1.0, 1.5, 1.25}, 1.0});
    scene.receivers.push_back({{3.0, 1.5, 1.25}});
    va::AcousticMaterial wall;
    wall.id = "wall";
    wall.octave_absorption.fill(0.2);
    scene.materials.push_back(wall);
    const auto add = [&scene](va::Vec3 a, va::Vec3 b, va::Vec3 c) {
        scene.geometry.push_back({{{a, b, c}}, "wall", 1});
    };
    add({0, 0, 0}, {0, 3, 0}, {4, 3, 0});
    add({0, 0, 0}, {4, 3, 0}, {4, 0, 0});
    add({0, 0, 2.5}, {4, 3, 2.5}, {0, 3, 2.5});
    add({0, 0, 2.5}, {4, 0, 2.5}, {4, 3, 2.5});
    add({0, 0, 0}, {4, 0, 2.5}, {0, 0, 2.5});
    add({0, 0, 0}, {4, 0, 0}, {4, 0, 2.5});
    add({0, 3, 0}, {0, 3, 2.5}, {4, 3, 2.5});
    add({0, 3, 0}, {4, 3, 2.5}, {4, 3, 0});
    add({0, 0, 0}, {0, 0, 2.5}, {0, 3, 2.5});
    add({0, 0, 0}, {0, 3, 2.5}, {0, 3, 0});
    add({4, 0, 0}, {4, 3, 2.5}, {4, 0, 2.5});
    add({4, 0, 0}, {4, 3, 0}, {4, 3, 2.5});
    return scene;
}

std::size_t count_nonzero(const va::AudioBuffer& buffer, float threshold = 1.0e-8F) {
    return static_cast<std::size_t>(std::count_if(buffer.begin(), buffer.end(),
        [threshold](float sample) { return std::abs(sample) > threshold; }));
}

void test_geometrical_delay() {
    auto scene = test_scene();
    va::ImpulseResponseSettings settings{48'000.0, 0.01};
    va::Engine engine(std::make_unique<va::geometrical::BRTGeometricalSolver>());
    const auto result = engine.compute_impulse_responses(scene, settings);
    const auto expected = static_cast<std::size_t>(
        std::floor(settings.impulse_response_sample_rate / scene.speed_of_sound));
    require(result.receiver_count == 1, "geometrical receiver count");
    require(std::abs(result.response(0, 0)[expected] +
                     result.response(0, 0)[expected + 1] - 1.0F) < 1.0e-6F,
            "geometrical delay or gain");
}

#if VA_HAS_BRT
void test_brt_image_sources_and_reverb_switch() {
    auto scene = shoebox_scene();
    va::geometrical::BRTSettings configuration;
    configuration.method = va::geometrical::Method::image_source;
    configuration.reflection_order = 1;
    va::geometrical::BRTGeometricalSolver solver(configuration);
    const auto reverberant = solver.compute_impulse_responses(scene, {48'000.0, 0.08});
    require(count_nonzero(reverberant.response(0, 0)) > 2,
            "BRT image-source model produced no reflections");

    configuration.enable_reverberation = false;
    va::geometrical::BRTGeometricalSolver direct_only(configuration);
    const auto direct = direct_only.compute_impulse_responses(scene, {48'000.0, 0.08});
    require(count_nonzero(direct.response(0, 0)) <= 2,
            "image-source reverberation switch did not remove reflections");

    const auto low_absorption_energy = std::accumulate(
        reverberant.response(0, 0).begin(), reverberant.response(0, 0).end(), 0.0,
        [](double sum, float sample) { return sum + std::abs(sample); });
    scene.materials.front().octave_absorption.fill(0.8);
    configuration.enable_reverberation = true;
    va::geometrical::BRTGeometricalSolver absorptive_solver(configuration);
    const auto absorptive = absorptive_solver.compute_impulse_responses(
        scene, {48'000.0, 0.08});
    const auto high_absorption_energy = std::accumulate(
        absorptive.response(0, 0).begin(), absorptive.response(0, 0).end(), 0.0,
        [](double sum, float sample) { return sum + std::abs(sample); });
    require(high_absorption_energy < low_absorption_energy,
            "BRT image-source reflections do not respond to material absorption");
}

void test_brt_sdn_reverberation() {
    const auto scene = shoebox_scene();
    va::geometrical::BRTSettings configuration;
    configuration.method = va::geometrical::Method::scattering_delay_network;
    va::geometrical::BRTGeometricalSolver solver(configuration);
    const auto result = solver.compute_impulse_responses(scene, {48'000.0, 0.08});
    require(count_nonzero(result.response(0, 0)) > 2,
            "BRT SDN produced no reverberant response");
}
#endif

void test_geometrical_ray_tracing() {
    const auto scene = shoebox_scene();
    va::geometrical::BRTSettings configuration;
    configuration.method = va::geometrical::Method::ray_tracing;
    configuration.reflection_order = 2;
    configuration.ray_count = 16'384;
    configuration.receiver_radius = 0.3;
    va::geometrical::BRTGeometricalSolver solver(configuration);
    const auto result = solver.compute_impulse_responses(scene, {48'000.0, 0.08});
    require(count_nonzero(result.response(0, 0)) > 2,
            "geometrical ray tracer produced no reflections");
    va::geometrical::BRTGeometricalSolver repeated_solver(configuration);
    const auto repeated = repeated_solver.compute_impulse_responses(scene, {48'000.0, 0.08});
    require(result.response(0, 0) == repeated.response(0, 0),
            "geometrical ray tracer is not deterministic for a fixed seed");
}

void test_fdtd_impulse_propagates() {
    auto scene = test_scene();
    // A z-only separation proves the update is not confined to an x-y plane.
    scene.sources[0].position = {1.0, 1.0, 0.5};
    scene.receivers[0].position = {1.0, 1.0, 0.7};
    va::ImpulseResponseSettings settings{48'000.0, 0.01};
    va::wave::FDTDSettings fdtd_settings;
    fdtd_settings.maximum_frequency = 300.0;
    fdtd_settings.points_per_wavelength = 6.0;
    va::wave::FDTDSolver solver(fdtd_settings);
    const auto result = solver.compute_impulse_responses(scene, settings);
    bool nonzero = false;
    for (const auto sample : result.response(0, 0)) {
        nonzero = nonzero || std::abs(sample) > 1.0e-7F;
    }
    require(nonzero, "FDTD impulse did not reach receiver");
    require(result.valid_bandwidth == 300.0, "FDTD valid bandwidth metadata");
}

void test_fdtd_derives_stable_grid() {
    auto scene = test_scene();
    va::wave::FDTDSettings fdtd_settings;
    fdtd_settings.maximum_frequency = 1'000.0;
    fdtd_settings.points_per_wavelength = 8.0;
    va::wave::FDTDSolver solver(fdtd_settings);
    const auto grid = solver.grid_parameters(scene);
    require(std::abs(grid.cell_size - scene.speed_of_sound / 8'000.0) < 1.0e-12,
            "FDTD cell size was not derived from bandwidth");
    require(grid.courant_number < 1.0 / std::sqrt(3.0),
            "FDTD Courant margin was not applied");
}

void test_fdtd_matches_cartesian_reference_update() {
    va::Scene scene;
    scene.bounds = {{0.0, 0.0, 0.0}, {4.0, 4.0, 4.0}};
    scene.speed_of_sound = 300.0;
    scene.sources.push_back({{2.0, 2.0, 2.0}, 1.0});
    scene.receivers.push_back({{2.0, 2.0, 2.0}});
    va::wave::FDTDSettings fdtd_settings;
    fdtd_settings.maximum_frequency = 50.0;
    fdtd_settings.points_per_wavelength = 6.0;
    fdtd_settings.boundary_absorption = 0.0;
    va::wave::FDTDSolver solver(fdtd_settings);
    const auto grid = solver.grid_parameters(scene);
    const va::ImpulseResponseSettings simulation{grid.internal_sample_rate,
                                                  4.0 / grid.internal_sample_rate};
    const auto result = solver.compute_impulse_responses(scene, simulation);
    const auto lambda_squared = grid.courant_number * grid.courant_number;
    const auto source_scale = lambda_squared / grid.cell_size;
    const auto& response = result.response(0, 0);
    require(std::abs(response[0] - source_scale) < 1.0e-6,
            "FDTD reference source injection");
    require(std::abs(response[1] - source_scale * (2.0 - 6.0 * lambda_squared)) < 1.0e-6,
            "FDTD reference Cartesian leapfrog update");
}

void test_fdtd_rejects_invalid_courant_margin() {
    auto scene = test_scene();
    va::wave::FDTDSettings fdtd_settings;
    fdtd_settings.courant_safety_factor = 1.01;
    va::wave::FDTDSolver solver(fdtd_settings);
    bool rejected = false;
    try {
        solver.validate(scene, {48'000.0, 0.01});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "invalid FDTD Courant margin was accepted");
}

void test_audio_convolution() {
    const auto output = va::audio::convolve({1.0F, 2.0F}, {1.0F, 0.5F}, 3);
    const va::AudioBuffer expected{1.0F, 2.5F, 1.0F};
    require(output.size() == expected.size(), "audio convolution size");
    for (std::size_t index = 0; index < expected.size(); ++index) {
        require(std::abs(output[index] - expected[index]) < 1.0e-6F,
                "audio convolution value");
    }
}

void test_audio_render_tail_and_resampling() {
    auto scene = test_scene();
    const va::AudioProgram program{24'000.0, {{1.0F, 1.0F}}};
    const va::ImpulseResponseSet responses{48'000.0, 20'000.0, 1, 1,
                                            {{1.0F, 0.5F}}};
    const auto with_tail = va::audio::render_sources(
        scene, program, {48'000.0, true}, responses);
    const auto without_tail = va::audio::render_sources(
        scene, program, {48'000.0, false}, responses);
    require(with_tail.receiver_signals.front().size() == 5,
            "render did not preserve the convolution tail");
    require(without_tail.receiver_signals.front().size() == 4,
            "tail-disabled render length does not match the program");
    for (const auto sample : va::audio::resample(va::AudioBuffer(64, 1.0F), 48'000.0,
                                                 32'000.0)) {
        require(std::abs(sample - 1.0F) < 1.0e-5F, "resampler does not preserve DC");
    }
}

void test_audio_program_dimension_validation() {
    const auto scene = test_scene();
    const va::ImpulseResponseSet responses{48'000.0, 20'000.0, 1, 1, {{1.0F}}};
    bool rejected = false;
    try {
        static_cast<void>(va::audio::render_sources(
            scene, va::AudioProgram{48'000.0, {}}, {}, responses));
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "mismatched mono source signals were accepted");
}

void test_hybrid_complementary_crossover() {
    auto scene = test_scene();
    const va::ImpulseResponseSettings settings{48'000.0, 0.01};
    va::geometrical::BRTGeometricalSolver reference;
    const auto expected = reference.compute_impulse_responses(scene, settings);
    va::hybrid::HybridSolver solver(
        std::make_unique<va::geometrical::BRTGeometricalSolver>(),
        std::make_unique<va::geometrical::BRTGeometricalSolver>(),
        va::hybrid::HybridSettings{500.0});
    const auto actual = solver.compute_impulse_responses(scene, settings);
    const auto& expected_ir = expected.response(0, 0);
    const auto& actual_ir = actual.response(0, 0);
    require(expected_ir.size() == actual_ir.size(), "hybrid IR size");
    for (std::size_t index = 0; index < expected_ir.size(); ++index) {
        require(std::abs(expected_ir[index] - actual_ir[index]) < 1.0e-6F,
                "hybrid crossover is not complementary");
    }
}

void test_pffdtd_model_export() {
    auto scene = test_scene();
    scene.geometry.push_back({{{{0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {0.0, 2.0, 0.0}}},
                              "wall", 1});
    va::AcousticMaterial wall;
    wall.id = "wall";
    wall.octave_absorption.fill(0.2);
    scene.materials.push_back(wall);
    const auto output = std::filesystem::temp_directory_path() / "va-pffdtd-model-test.json";
    va::wave::write_pffdtd_model(scene, output);
    std::ifstream input(output);
    const std::string contents((std::istreambuf_iterator<char>(input)),
                               std::istreambuf_iterator<char>());
    std::filesystem::remove(output);
    require(contents.find("\"mats_hash\"") != std::string::npos, "PFFDTD materials export");
    require(contents.find("\"va_materials\"") != std::string::npos, "PFFDTD coefficients export");
}

void test_fdtd_rejects_position_outside_z_domain() {
    auto scene = test_scene();
    scene.receivers[0].position.z = 2.1;
    va::wave::FDTDSolver solver;
    bool rejected = false;
    try {
        solver.validate(scene, {48'000.0, 0.01});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "receiver outside z domain was accepted");
}

} // namespace

int main() {
    try {
        test_geometrical_delay();
#if VA_HAS_BRT
        test_brt_image_sources_and_reverb_switch();
        test_brt_sdn_reverberation();
#endif
        test_geometrical_ray_tracing();
        test_fdtd_impulse_propagates();
        test_fdtd_derives_stable_grid();
        test_fdtd_matches_cartesian_reference_update();
        test_fdtd_rejects_invalid_courant_margin();
        test_fdtd_rejects_position_outside_z_domain();
        test_audio_convolution();
        test_audio_render_tail_and_resampling();
        test_audio_program_dimension_validation();
        test_hybrid_complementary_crossover();
#if VA_HAS_PFFDTD
        require(va::wave::PFFDTDBackend::submodule_available(),
                "PFFDTD submodule was not enabled");
#endif
        test_pffdtd_model_export();
        std::cout << "all tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "test failure: " << error.what() << '\n';
        return 1;
    }
}
