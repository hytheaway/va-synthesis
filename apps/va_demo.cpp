#include "va/core/engine.hpp"
#include "va/geometrical/brt_geometrical_solver.hpp"
#include "va/hybrid/hybrid_solver.hpp"
#include "va/wave/fdtd_solver.hpp"

#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <string_view>

int main(int argc, char** argv) {
    try {
        const std::string_view mode = argc > 1 ? argv[1] : "fdtd";
        va::Scene scene;
        scene.bounds = {{0.0, 0.0, 0.0}, {4.0, 3.0, 2.5}};
        scene.sources.push_back({{1.0, 1.5, 1.2}, 1.0});
        scene.receivers.push_back({{3.0, 1.5, 1.8}});
        const va::ImpulseResponseSettings impulse_response{48'000.0, 0.05};
        const va::AudioProgram program{48'000.0, {va::AudioBuffer(1, 1.0F)}};

        std::unique_ptr<va::PropagationSolver> solver;
        if (mode == "fdtd") {
            va::wave::FDTDSettings fdtd;
            fdtd.maximum_frequency = 500.0;
            fdtd.points_per_wavelength = 6.0;
            solver = std::make_unique<va::wave::FDTDSolver>(fdtd);
        } else if (mode == "geometrical") {
            solver = std::make_unique<va::geometrical::BRTGeometricalSolver>();
        } else if (mode == "hybrid") {
            va::wave::FDTDSettings fdtd;
            fdtd.maximum_frequency = 500.0;
            fdtd.points_per_wavelength = 6.0;
            solver = std::make_unique<va::hybrid::HybridSolver>(
                std::make_unique<va::wave::FDTDSolver>(fdtd),
                std::make_unique<va::geometrical::BRTGeometricalSolver>(),
                va::hybrid::HybridSettings{300.0});
        } else {
            std::cerr << "usage: va_demo [fdtd|geometrical|hybrid]\n";
            return 2;
        }

        va::Engine engine(std::move(solver));
        const auto result = engine.render(scene, program, impulse_response);
        const auto peak = *std::max_element(
            result.receiver_signals.front().begin(), result.receiver_signals.front().end(),
            [](float a, float b) { return std::abs(a) < std::abs(b); });
        std::cout << engine.solver().name() << ": " << result.receiver_signals.front().size()
                  << " samples, peak=" << peak << '\n';
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
}
