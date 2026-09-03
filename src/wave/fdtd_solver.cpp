#include "va/wave/fdtd_solver.hpp"

#include "va/core/audio.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace va::wave {
namespace {

struct WeightedPoint {
    std::size_t index{};
    Sample weight{};
};

using InterpolationPoint = std::array<WeightedPoint, 8>;

std::size_t flatten(std::size_t x, std::size_t y, std::size_t z,
                    std::size_t width, std::size_t height) {
    return x + width * (y + height * z);
}

void ensure_inside(const Vec3& point, const Bounds& bounds, const char* label) {
    if (point.x < bounds.minimum.x || point.x > bounds.maximum.x ||
        point.y < bounds.minimum.y || point.y > bounds.maximum.y ||
        point.z < bounds.minimum.z || point.z > bounds.maximum.z) {
        throw std::invalid_argument(std::string(label) + " is outside the FDTD domain");
    }
}

std::size_t checked_volume(std::size_t width, std::size_t height, std::size_t depth) {
    constexpr auto maximum = std::numeric_limits<std::size_t>::max();
    if (height != 0 && width > maximum / height) {
        throw std::length_error("FDTD grid dimensions overflow addressable memory");
    }
    const auto slice = width * height;
    if (depth != 0 && slice > maximum / depth) {
        throw std::length_error("FDTD grid dimensions overflow addressable memory");
    }
    return slice * depth;
}

InterpolationPoint interpolation_point(
    const Vec3& point, const Bounds& bounds, double cell_size,
    std::size_t width, std::size_t height, std::size_t depth) {
    const std::array<double, 3> coordinates{
        (point.x - bounds.minimum.x) / cell_size,
        (point.y - bounds.minimum.y) / cell_size,
        (point.z - bounds.minimum.z) / cell_size,
    };
    const std::array<std::size_t, 3> dimensions{width, height, depth};
    std::array<std::size_t, 3> lower{};
    std::array<std::size_t, 3> upper{};
    std::array<Sample, 3> fraction{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        lower[axis] = std::min(static_cast<std::size_t>(std::floor(coordinates[axis])),
                               dimensions[axis] - 1);
        upper[axis] = std::min(lower[axis] + 1, dimensions[axis] - 1);
        fraction[axis] = upper[axis] == lower[axis]
            ? 0.0F
            : static_cast<Sample>(coordinates[axis] - static_cast<double>(lower[axis]));
    }

    InterpolationPoint result{};
    for (std::size_t corner = 0; corner < result.size(); ++corner) {
        const bool high_x = (corner & 1U) != 0;
        const bool high_y = (corner & 2U) != 0;
        const bool high_z = (corner & 4U) != 0;
        const auto x = high_x ? upper[0] : lower[0];
        const auto y = high_y ? upper[1] : lower[1];
        const auto z = high_z ? upper[2] : lower[2];
        const auto wx = high_x ? fraction[0] : 1.0F - fraction[0];
        const auto wy = high_y ? fraction[1] : 1.0F - fraction[1];
        const auto wz = high_z ? fraction[2] : 1.0F - fraction[2];
        result[corner] = {flatten(x, y, z, width, height), wx * wy * wz};
    }
    return result;
}

AudioBuffer run_impulse(
    const Scene& scene, const FDTDSettings& settings, const GridParameters& grid,
    const Source& source, const Receiver& receiver, double duration_seconds) {
    const auto width = static_cast<std::size_t>(
                           std::ceil((scene.bounds.maximum.x - scene.bounds.minimum.x) /
                                     grid.cell_size)) + 1;
    const auto height = static_cast<std::size_t>(
                            std::ceil((scene.bounds.maximum.y - scene.bounds.minimum.y) /
                                      grid.cell_size)) + 1;
    const auto depth = static_cast<std::size_t>(
                           std::ceil((scene.bounds.maximum.z - scene.bounds.minimum.z) /
                                     grid.cell_size)) + 1;
    if (width < 3 || height < 3 || depth < 3) {
        throw std::invalid_argument("FDTD domain must contain at least 3 by 3 by 3 cells");
    }
    const auto grid_size = checked_volume(width, height, depth);
    const auto frames = static_cast<std::size_t>(
        std::ceil(grid.internal_sample_rate * duration_seconds));
    const auto courant_squared = static_cast<Sample>(
        grid.courant_number * grid.courant_number);
    const auto reflection = static_cast<Sample>(std::sqrt(1.0 - settings.boundary_absorption));
    const auto source_scale = static_cast<Sample>(
        grid.courant_number * grid.courant_number / grid.cell_size);
    const auto source_point = interpolation_point(source.position, scene.bounds, grid.cell_size,
                                                  width, height, depth);
    const auto receiver_point = interpolation_point(receiver.position, scene.bounds, grid.cell_size,
                                                     width, height, depth);

    // The next state overwrites the previous state, so only two full grids are needed.
    std::vector<Sample> previous(grid_size);
    std::vector<Sample> current(grid_size);
    AudioBuffer output(frames);
    const auto slice = width * height;

    for (std::size_t frame = 0; frame < frames; ++frame) {
        for (std::size_t z = 1; z + 1 < depth; ++z) {
            for (std::size_t y = 1; y + 1 < height; ++y) {
                for (std::size_t x = 1; x + 1 < width; ++x) {
                    const auto index = flatten(x, y, z, width, height);
                    const auto laplacian = current[index - 1] + current[index + 1] +
                                           current[index - width] + current[index + width] +
                                           current[index - slice] + current[index + slice] -
                                           6.0F * current[index];
                    previous[index] = 2.0F * current[index] - previous[index] +
                                      courant_squared * laplacian;
                }
            }
        }

        for (std::size_t z = 0; z < depth; ++z) {
            for (std::size_t y = 0; y < height; ++y) {
                for (std::size_t x = 0; x < width; ++x) {
                    if (x != 0 && x + 1 != width && y != 0 && y + 1 != height &&
                        z != 0 && z + 1 != depth) {
                        continue;
                    }
                    const auto interior_x = std::clamp(x, std::size_t{1}, width - 2);
                    const auto interior_y = std::clamp(y, std::size_t{1}, height - 2);
                    const auto interior_z = std::clamp(z, std::size_t{1}, depth - 2);
                    previous[flatten(x, y, z, width, height)] =
                        reflection * previous[flatten(interior_x, interior_y, interior_z,
                                                       width, height)];
                }
            }
        }

        if (frame == 0) {
            for (const auto& point : source_point) {
                previous[point.index] += point.weight * source_scale *
                                         static_cast<Sample>(source.gain);
            }
        }
        for (const auto& point : receiver_point) {
            output[frame] += point.weight * previous[point.index];
        }
        previous.swap(current);
    }
    return output;
}

} // namespace

FDTDSolver::FDTDSolver(FDTDSettings settings) : settings_(settings) {}

std::string_view FDTDSolver::name() const noexcept {
    return "3D acoustic FDTD";
}

GridParameters FDTDSolver::grid_parameters(const Scene& scene) const {
    if (settings_.maximum_frequency <= 0.0 || settings_.points_per_wavelength <= 0.0) {
        throw std::invalid_argument("maximum frequency and points per wavelength must be positive");
    }
    if (settings_.courant_safety_factor <= 0.0 || settings_.courant_safety_factor > 1.0) {
        throw std::invalid_argument("Courant safety factor must be in (0, 1]");
    }
    if (settings_.scheme != GridScheme::cartesian_7_point) {
        throw std::logic_error("FCC execution is reserved for the PFFDTD backend");
    }
    const auto cell_size = scene.speed_of_sound /
                           (settings_.maximum_frequency * settings_.points_per_wavelength);
    const auto courant = settings_.courant_safety_factor / std::sqrt(3.0);
    return {cell_size, scene.speed_of_sound / (courant * cell_size), courant};
}

void FDTDSolver::validate(
    const Scene& scene, const ImpulseResponseSettings& settings) const {
    if (settings.impulse_response_sample_rate <= 0.0 ||
        settings.impulse_response_duration_seconds <= 0.0) {
        throw std::invalid_argument("sample rate and duration must be positive");
    }
    if (scene.speed_of_sound <= 0.0) {
        throw std::invalid_argument("speed of sound must be positive");
    }
    if (settings_.boundary_absorption < 0.0 || settings_.boundary_absorption > 1.0) {
        throw std::invalid_argument("boundary absorption must be in [0, 1]");
    }
    if (settings_.maximum_frequency >= settings.impulse_response_sample_rate * 0.5) {
        throw std::invalid_argument("maximum FDTD frequency must be below output Nyquist");
    }
    if (scene.bounds.maximum.x <= scene.bounds.minimum.x ||
        scene.bounds.maximum.y <= scene.bounds.minimum.y ||
        scene.bounds.maximum.z <= scene.bounds.minimum.z) {
        throw std::invalid_argument("FDTD domain must have positive x, y, and z dimensions");
    }
    if (scene.receivers.empty()) {
        throw std::invalid_argument("scene requires at least one receiver");
    }
    static_cast<void>(grid_parameters(scene));
    for (const auto& source : scene.sources) {
        ensure_inside(source.position, scene.bounds, "source");
    }
    for (const auto& receiver : scene.receivers) {
        ensure_inside(receiver.position, scene.bounds, "receiver");
    }
}

ImpulseResponseSet FDTDSolver::compute_impulse_responses(
    const Scene& scene, const ImpulseResponseSettings& settings) {
    validate(scene, settings);
    const auto grid = grid_parameters(scene);
    const auto output_frames = static_cast<std::size_t>(
        std::ceil(settings.impulse_response_sample_rate *
                  settings.impulse_response_duration_seconds));
    ImpulseResponseSet result{settings.impulse_response_sample_rate,
                              settings_.maximum_frequency,
                              scene.sources.size(), scene.receivers.size(),
                              std::vector<AudioBuffer>(scene.sources.size() * scene.receivers.size())};
    for (std::size_t source = 0; source < scene.sources.size(); ++source) {
        for (std::size_t receiver = 0; receiver < scene.receivers.size(); ++receiver) {
            auto internal = run_impulse(scene, settings_, grid, scene.sources[source],
                                        scene.receivers[receiver],
                                        settings.impulse_response_duration_seconds);
            auto output = audio::resample(internal, grid.internal_sample_rate,
                                          settings.impulse_response_sample_rate);
            output.resize(output_frames);
            result.response(source, receiver) = std::move(output);
        }
    }
    return result;
}

const FDTDSettings& FDTDSolver::settings() const noexcept {
    return settings_;
}

} // namespace va::wave
