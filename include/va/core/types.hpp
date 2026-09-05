#pragma once

#include <cmath>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace va {

using Sample = float;
using AudioBuffer = std::vector<Sample>;

struct Vec3 {
    double x{};
    double y{};
    double z{};
};

[[nodiscard]] inline double distance(const Vec3& a, const Vec3& b) noexcept {
    const auto dx = a.x - b.x;
    const auto dy = a.y - b.y;
    const auto dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct Source {
    Vec3 position;
    double gain{1.0};
};

struct Receiver {
    Vec3 position;
    double yaw_degrees{};
    double pitch_degrees{};
    double roll_degrees{};
};

struct Bounds {
    Vec3 minimum{};
    Vec3 maximum{10.0, 10.0, 3.0};
};

struct Triangle {
    std::array<Vec3, 3> vertices;
    std::string material_id{"_RIGID"};
    int active_side{1};
};

struct AcousticMaterial {
    std::string id;
    // Sabine absorption at 16 Hz through 16 kHz octave centers, as expected by PFFDTD.
    std::array<double, 11> octave_absorption{};
};

struct Scene {
    Bounds bounds;
    double speed_of_sound{343.0};
    std::vector<Source> sources;
    std::vector<Receiver> receivers;
    std::vector<Triangle> geometry;
    std::vector<AcousticMaterial> materials;
};

struct ImpulseResponseSettings {
    double impulse_response_sample_rate{48'000.0};
    double impulse_response_duration_seconds{1.0};
};

// Mono program material, one signal for each acoustic source in the scene.
struct AudioProgram {
    double sample_rate{48'000.0};
    std::vector<AudioBuffer> source_signals;
};

struct RenderSettings {
    double output_sample_rate{48'000.0};
    bool include_reverb_tail{true};
};

struct SimulationResult {
    double sample_rate{};
    std::vector<AudioBuffer> receiver_signals;
};

// Source-major collection: response(source, receiver).
struct ImpulseResponseSet {
    double sample_rate{};
    double valid_bandwidth{};
    std::size_t source_count{};
    std::size_t receiver_count{};
    std::vector<AudioBuffer> responses;

    [[nodiscard]] const AudioBuffer& response(
        std::size_t source_index, std::size_t receiver_index) const {
        if (source_index >= source_count || receiver_index >= receiver_count) {
            throw std::out_of_range("impulse-response index out of range");
        }
        return responses[source_index * receiver_count + receiver_index];
    }

    [[nodiscard]] AudioBuffer& response(
        std::size_t source_index, std::size_t receiver_index) {
        if (source_index >= source_count || receiver_index >= receiver_count) {
            throw std::out_of_range("impulse-response index out of range");
        }
        return responses[source_index * receiver_count + receiver_index];
    }
};

} // namespace va
