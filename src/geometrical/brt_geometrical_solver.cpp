#include "va/geometrical/brt_geometrical_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#if VA_HAS_BRT
#include <Common/Vector3.hpp>
#endif

namespace va::geometrical {
namespace {

std::size_t sample_count(const ImpulseResponseSettings& settings) {
    return static_cast<std::size_t>(std::ceil(
        settings.impulse_response_sample_rate * settings.impulse_response_duration_seconds));
}

double geometrical_distance(const Vec3& a, const Vec3& b) {
#if VA_HAS_BRT
    // Translate at the backend boundary, keeping BRT coordinate types private.
    const Common::CVector3 from(static_cast<float>(a.x), static_cast<float>(a.y),
                                static_cast<float>(a.z));
    const Common::CVector3 to(static_cast<float>(b.x), static_cast<float>(b.y),
                              static_cast<float>(b.z));
    return (from - to).GetDistance();
#else
    return distance(a, b);
#endif
}

} // namespace

BRTGeometricalSolver::BRTGeometricalSolver(BRTSettings settings) : settings_(settings) {}

std::string_view BRTGeometricalSolver::name() const noexcept {
    return "BRT geometrical acoustics";
}

void BRTGeometricalSolver::validate(
    const Scene& scene,
    const ImpulseResponseSettings& settings) const {
    if (settings.impulse_response_sample_rate <= 0.0 ||
        settings.impulse_response_duration_seconds <= 0.0) {
        throw std::invalid_argument("sample rate and duration must be positive");
    }
    if (scene.speed_of_sound <= 0.0) {
        throw std::invalid_argument("speed of sound must be positive");
    }
    if (scene.receivers.empty()) {
        throw std::invalid_argument("scene requires at least one receiver");
    }
    if (settings_.method != Method::free_field) {
        throw std::logic_error(
            "this backend currently implements free-field propagation; "
            "BRT ISM/SDN and ray-tracing adapters are extension points");
    }
}

ImpulseResponseSet BRTGeometricalSolver::compute_impulse_responses(
    const Scene& scene,
    const ImpulseResponseSettings& settings) {
    const auto frames = sample_count(settings);
    ImpulseResponseSet result{settings.impulse_response_sample_rate,
                              settings.impulse_response_sample_rate * 0.5,
                              scene.sources.size(), scene.receivers.size(),
                              std::vector<AudioBuffer>(scene.sources.size() * scene.receivers.size(),
                                                       AudioBuffer(frames))};

    for (std::size_t source_index = 0; source_index < scene.sources.size(); ++source_index) {
        const auto& source = scene.sources[source_index];
        for (std::size_t receiver_index = 0; receiver_index < scene.receivers.size(); ++receiver_index) {
            auto& output = result.response(source_index, receiver_index);
            const auto metres = geometrical_distance(
                source.position, scene.receivers[receiver_index].position);
            const auto delay = settings_.propagation_delay
                ? static_cast<std::size_t>(std::llround(
                      metres * settings.impulse_response_sample_rate / scene.speed_of_sound))
                : 0U;
            const auto attenuation = settings_.distance_attenuation
                ? 1.0 / std::max(1.0, metres)
                : 1.0;
            if (delay < frames) {
                output[delay] = static_cast<Sample>(source.gain * attenuation);
            }
        }
    }
    return result;
}

const BRTSettings& BRTGeometricalSolver::settings() const noexcept {
    return settings_;
}

bool BRTGeometricalSolver::brt_headers_available() noexcept {
#if VA_HAS_BRT
    return true;
#else
    return false;
#endif
}

} // namespace va::geometrical
