#include "va/core/engine.hpp"
#include "va/geometrical/brt_geometrical_solver.hpp"

#include <memory>

int main() {
    va::Scene scene;
    scene.sources.push_back({{0.0, 0.0, 0.0}, 1.0});
    scene.receivers.push_back({{1.0, 0.0, 0.0}});
    const va::AudioProgram program{48'000.0, {{1.0F}}};
    va::Engine engine(std::make_unique<va::geometrical::BRTGeometricalSolver>());
    const auto output = engine.render(scene, program, {48'000.0, 0.01});
    return output.receiver_signals.size() == 1 ? 0 : 1;
}
