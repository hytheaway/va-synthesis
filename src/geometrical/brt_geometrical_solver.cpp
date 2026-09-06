#include "va/geometrical/brt_geometrical_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>

#if VA_HAS_BRT
#include <Common/Buffer.hpp>
#include <Common/FFTCalculator.hpp>
#include <Common/GlobalParameters.hpp>
#include <Common/Transform.hpp>
#include <Common/Vector3.hpp>
#include <ServiceModules/InterpolationAuxiliarMethods.hpp>
#include <Common/Waveguide.hpp>
#include <ProcessingModules/DistanceAttenuator.hpp>
#include <EnvironmentModels/ISMEnvironment/ISMEnvironment.hpp>
#include <EnvironmentModels/SDNEnvironment/SDNEnvironment.hpp>
#include <ServiceModules/Room.hpp>
#endif

namespace va::geometrical {
namespace {

constexpr double epsilon = 1.0e-7;

std::size_t sample_count(const ImpulseResponseSettings& settings) {
    return static_cast<std::size_t>(std::ceil(
        settings.impulse_response_sample_rate * settings.impulse_response_duration_seconds));
}

Vec3 add(const Vec3& a, const Vec3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 subtract(const Vec3& a, const Vec3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 multiply(const Vec3& value, double factor) {
    return {value.x * factor, value.y * factor, value.z * factor};
}
double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
Vec3 normalized(const Vec3& value) {
    const auto length = std::sqrt(dot(value, value));
    if (length <= epsilon) throw std::invalid_argument("zero-length ray direction");
    return multiply(value, 1.0 / length);
}

double geometrical_distance(const Vec3& a, const Vec3& b) {
#if VA_HAS_BRT
    const Common::CVector3 from(static_cast<float>(a.x), static_cast<float>(a.y),
                                static_cast<float>(a.z));
    const Common::CVector3 to(static_cast<float>(b.x), static_cast<float>(b.y),
                              static_cast<float>(b.z));
    return (from - to).GetDistance();
#else
    return distance(a, b);
#endif
}

void add_arrival(AudioBuffer& output, double sample_position, double amplitude) {
    if (sample_position < 0.0 || !std::isfinite(sample_position) ||
        !std::isfinite(amplitude)) return;
    const auto first = static_cast<std::size_t>(std::floor(sample_position));
    if (first >= output.size()) return;
    const auto fraction = sample_position - static_cast<double>(first);
    output[first] += static_cast<Sample>(amplitude * (1.0 - fraction));
    if (first + 1 < output.size()) {
        output[first + 1] += static_cast<Sample>(amplitude * fraction);
    }
}

const AcousticMaterial* find_material(const Scene& scene, const std::string& id) {
    const auto item = std::find_if(scene.materials.begin(), scene.materials.end(),
        [&id](const AcousticMaterial& material) { return material.id == id; });
    return item == scene.materials.end() ? nullptr : &*item;
}

double mean_absorption(const Scene& scene, const std::string& material_id,
                       double fallback) {
    const auto* material = find_material(scene, material_id);
    if (!material) return fallback;
    double total = 0.0;
    for (std::size_t band = 2; band < material->octave_absorption.size(); ++band) {
        total += material->octave_absorption[band];
    }
    return total / 9.0;
}

[[maybe_unused]] double scene_mean_absorption(const Scene& scene, double fallback) {
    if (scene.materials.empty()) return fallback;
    double total = 0.0;
    std::size_t count = 0;
    for (const auto& material : scene.materials) {
        for (std::size_t band = 2; band < material.octave_absorption.size(); ++band) {
            total += material.octave_absorption[band];
            ++count;
        }
    }
    return count == 0 ? fallback : total / static_cast<double>(count);
}

struct RayHit {
    double distance{std::numeric_limits<double>::infinity()};
    std::size_t triangle{};
    Vec3 normal{};
};

bool intersect_triangle(const Vec3& origin, const Vec3& direction,
                        const Triangle& triangle, double& ray_distance, Vec3& normal) {
    const auto edge1 = subtract(triangle.vertices[1], triangle.vertices[0]);
    const auto edge2 = subtract(triangle.vertices[2], triangle.vertices[0]);
    const auto p = cross(direction, edge2);
    const auto determinant = dot(edge1, p);
    if (std::abs(determinant) < epsilon) return false;
    const auto inverse = 1.0 / determinant;
    const auto t = subtract(origin, triangle.vertices[0]);
    const auto u = dot(t, p) * inverse;
    if (u < 0.0 || u > 1.0) return false;
    const auto q = cross(t, edge1);
    const auto v = dot(direction, q) * inverse;
    if (v < 0.0 || u + v > 1.0) return false;
    ray_distance = dot(edge2, q) * inverse;
    if (ray_distance <= epsilon) return false;
    normal = normalized(cross(edge1, edge2));
    if (dot(normal, direction) > 0.0) normal = multiply(normal, -1.0);
    return true;
}

RayHit nearest_hit(const Scene& scene, const Vec3& origin, const Vec3& direction) {
    RayHit result;
    for (std::size_t index = 0; index < scene.geometry.size(); ++index) {
        double hit_distance{};
        Vec3 normal;
        if (intersect_triangle(origin, direction, scene.geometry[index], hit_distance, normal) &&
            hit_distance < result.distance) {
            result = {hit_distance, index, normal};
        }
    }
    return result;
}

bool segment_occluded(const Scene& scene, const Vec3& from, const Vec3& to) {
    const auto delta = subtract(to, from);
    const auto length = std::sqrt(dot(delta, delta));
    if (length <= epsilon) return false;
    return nearest_hit(scene, from, multiply(delta, 1.0 / length)).distance < length - epsilon;
}

double receiver_intersection(const Vec3& origin, const Vec3& direction,
                             const Vec3& receiver, double radius) {
    const auto offset = subtract(origin, receiver);
    const auto b = dot(offset, direction);
    const auto c = dot(offset, offset) - radius * radius;
    const auto discriminant = b * b - c;
    if (discriminant < 0.0) return std::numeric_limits<double>::infinity();
    const auto near = -b - std::sqrt(discriminant);
    const auto far = -b + std::sqrt(discriminant);
    if (near > epsilon) return near;
    return far > epsilon ? far : std::numeric_limits<double>::infinity();
}

void add_direct_path(const Scene& scene, const Source& source, const Receiver& receiver,
                     const BRTSettings& configuration,
                     const ImpulseResponseSettings& settings, AudioBuffer& output) {
    if (!configuration.enable_direct_path ||
        (configuration.method != Method::free_field && !scene.geometry.empty() &&
         segment_occluded(scene, source.position, receiver.position))) {
        return;
    }
    const auto metres = geometrical_distance(source.position, receiver.position);
    const auto delay = configuration.propagation_delay
        ? metres * settings.impulse_response_sample_rate / scene.speed_of_sound : 0.0;
    const auto attenuation = configuration.distance_attenuation
        ? 1.0 / std::max(1.0, metres) : 1.0;
    add_arrival(output, delay, source.gain * attenuation);
}

#if VA_HAS_BRT
Common::CVector3 to_brt(const Vec3& value) {
    return {static_cast<float>(value.x), static_cast<float>(value.y),
            static_cast<float>(value.z)};
}

std::vector<float> brt_absorption(const Scene& scene, const std::string& material_id,
                                  double fallback) {
    std::vector<float> result(9, static_cast<float>(fallback));
    if (const auto* material = find_material(scene, material_id)) {
        for (std::size_t band = 0; band < result.size(); ++band) {
            result[band] = static_cast<float>(std::clamp(
                material->octave_absorption[band + 2], 0.0, 0.999999));
        }
    }
    return result;
}

int aabb_face(const Scene& scene, const Triangle& triangle) {
    const auto on = [](double value, double bound) {
        return std::abs(value - bound) <= 1.0e-6;
    };
    const auto& lo = scene.bounds.minimum;
    const auto& hi = scene.bounds.maximum;
    bool x_min = true, x_max = true, y_min = true, y_max = true, z_min = true, z_max = true;
    for (const auto& vertex : triangle.vertices) {
        x_min = x_min && on(vertex.x, lo.x);
        x_max = x_max && on(vertex.x, hi.x);
        y_min = y_min && on(vertex.y, lo.y);
        y_max = y_max && on(vertex.y, hi.y);
        z_min = z_min && on(vertex.z, lo.z);
        z_max = z_max && on(vertex.z, hi.z);
    }
    if (x_min) return 0;
    if (x_max) return 1;
    if (y_min) return 2;
    if (y_max) return 3;
    if (z_min) return 4;
    if (z_max) return 5;
    return -1;
}

std::shared_ptr<BRTServices::CRoom> make_brt_room(const Scene& scene,
                                                   double fallback_absorption) {
    BRTServices::TRoomGeometry geometry;
    std::vector<std::string> wall_materials;
    const auto add_wall = [&](const std::array<Vec3, 4>& corners, const std::string& material_id) {
        const auto base = static_cast<int>(geometry.corners.size());
        for (const auto& corner : corners) geometry.corners.push_back(to_brt(corner));
        geometry.walls.push_back({base, base + 1, base + 2, base + 3});
        wall_materials.push_back(material_id);
    };

    std::array<int, 6> face_triangle;
    face_triangle.fill(-1);
    bool aabb_hull = !scene.geometry.empty();
    for (std::size_t index = 0; index < scene.geometry.size(); ++index) {
        const auto face = aabb_face(scene, scene.geometry[index]);
        if (face < 0) {
            aabb_hull = false;
            break;
        }
        if (face_triangle[static_cast<std::size_t>(face)] < 0) {
            face_triangle[static_cast<std::size_t>(face)] = static_cast<int>(index);
        }
    }

    if (aabb_hull) {
        const auto& lo = scene.bounds.minimum;
        const auto& hi = scene.bounds.maximum;
        const auto material_of = [&](int face) {
            return scene.geometry[static_cast<std::size_t>(face_triangle[static_cast<std::size_t>(face)])].material_id;
        };
        // Corner order matches BRT's SetupShoeBox so wall normals point inward.
        if (face_triangle[1] >= 0) {
            add_wall({{{hi.x, hi.y, hi.z}, {hi.x, hi.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}}},
                     material_of(1));
        }
        if (face_triangle[3] >= 0) {
            add_wall({{{lo.x, hi.y, hi.z}, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}}},
                     material_of(3));
        }
        if (face_triangle[2] >= 0) {
            add_wall({{{hi.x, lo.y, hi.z}, {hi.x, lo.y, lo.z}, {lo.x, lo.y, lo.z}, {lo.x, lo.y, hi.z}}},
                     material_of(2));
        }
        if (face_triangle[0] >= 0) {
            add_wall({{{lo.x, lo.y, hi.z}, {lo.x, lo.y, lo.z}, {lo.x, hi.y, lo.z}, {lo.x, hi.y, hi.z}}},
                     material_of(0));
        }
        if (face_triangle[4] >= 0) {
            add_wall({{{hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z}, {lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}}},
                     material_of(4));
        }
        if (face_triangle[5] >= 0) {
            add_wall({{{hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z}}},
                     material_of(5));
        }
    } else {
        const Vec3 center{(scene.bounds.minimum.x + scene.bounds.maximum.x) * 0.5,
                          (scene.bounds.minimum.y + scene.bounds.maximum.y) * 0.5,
                          (scene.bounds.minimum.z + scene.bounds.maximum.z) * 0.5};
        for (const auto& triangle : scene.geometry) {
            auto vertices = triangle.vertices;
            const auto normal = cross(subtract(vertices[1], vertices[0]),
                                      subtract(vertices[2], vertices[0]));
            const auto centroid = multiply(add(add(vertices[0], vertices[1]), vertices[2]), 1.0 / 3.0);
            if (dot(normal, subtract(center, centroid)) < 0.0) std::swap(vertices[1], vertices[2]);
            add_wall({{vertices[0], vertices[1], vertices[2],
                       multiply(add(vertices[2], vertices[0]), 0.5)}},
                     triangle.material_id);
        }
    }

    auto room = std::make_shared<BRTServices::CRoom>();
    room->SetupRoomGeometry(geometry);
    for (std::size_t wall = 0; wall < wall_materials.size(); ++wall) {
        room->SetWallAbsortion(static_cast<int>(wall),
            brt_absorption(scene, wall_materials[wall], fallback_absorption));
    }
    return room;
}

class OfflineSDN final : public BRTEnvironmentModel::SDNEnvironment {
public:
    void prepare(double sample_rate, const Common::CVector3& dimensions,
                 const Common::CTransform& source, const Common::CTransform& receiver,
                 std::vector<Common::CTransform>& positions) {
        Prepare(sample_rate, dimensions, source, receiver, positions);
    }
    void set_absorption(const std::vector<float>& values) {
        for (auto& wall : wallNodes) wall.SetFreqAbsortion(values);
    }
};
#endif

void compute_image_source(const Scene& scene, const Source& source,
                          const Receiver& receiver, const BRTSettings& configuration,
                          const ImpulseResponseSettings& settings, AudioBuffer& output) {
#if VA_HAS_BRT
    add_direct_path(scene, source, receiver, configuration, settings, output);
    if (!configuration.enable_reverberation || configuration.reflection_order == 0) return;
    Common::CGlobalParameters globals;
    globals.SetSampleRate(static_cast<int>(std::llround(settings.impulse_response_sample_rate)));
    globals.SetSoundSpeed(static_cast<float>(scene.speed_of_sound));
    auto room = make_brt_room(scene, configuration.default_absorption);
    BRTEnvironmentModel::CISMEnvironment model;
    const auto max_distance = static_cast<float>(
        scene.speed_of_sound * settings.impulse_response_duration_seconds);
    if (!model.Setup(static_cast<int>(configuration.reflection_order), max_distance,
                     0.01F, room, Common::CTransform(to_brt(source.position)),
                     Common::CTransform(to_brt(receiver.position)))) {
        throw std::runtime_error("BRT image-source setup failed");
    }
    const auto image_data = model.getImageSourceData();
    for (const auto& image : image_data) {
        if (!image.visible) continue;
        const Vec3 position{image.location.x, image.location.y, image.location.z};
        const auto metres = geometrical_distance(position, receiver.position);
        const auto delay = configuration.propagation_delay
            ? metres * settings.impulse_response_sample_rate / scene.speed_of_sound : 0.0;
        double reflection_gain = image.visibility;
        if (!image.reflectionBands.empty()) {
            reflection_gain *= std::accumulate(image.reflectionBands.begin(),
                image.reflectionBands.end(), 0.0) / image.reflectionBands.size();
        }
        const auto distance_gain = configuration.distance_attenuation
            ? 1.0 / std::max(1.0, metres) : 1.0;
        add_arrival(output, delay, source.gain * reflection_gain * distance_gain);
    }
#else
    static_cast<void>(scene); static_cast<void>(source); static_cast<void>(receiver);
    static_cast<void>(configuration); static_cast<void>(settings); static_cast<void>(output);
    throw std::logic_error("image-source modeling requires BRTLibrary support");
#endif
}

void compute_sdn(const Scene& scene, const Source& source, const Receiver& receiver,
                 const BRTSettings& configuration,
                 const ImpulseResponseSettings& settings, AudioBuffer& output) {
#if VA_HAS_BRT
    const auto dimensions = subtract(scene.bounds.maximum, scene.bounds.minimum);
    const auto local_source = subtract(source.position, scene.bounds.minimum);
    const auto local_receiver = subtract(receiver.position, scene.bounds.minimum);
    std::vector<Common::CTransform> positions(7);
    OfflineSDN model;
    model.prepare(settings.impulse_response_sample_rate, to_brt(dimensions),
                  Common::CTransform(to_brt(local_source)),
                  Common::CTransform(to_brt(local_receiver)), positions);
    const auto absorption = std::clamp(
        scene_mean_absorption(scene, configuration.default_absorption), 0.0, 0.999999);
    model.set_absorption(std::vector<float>(8, static_cast<float>(absorption)));
    CMonoBuffer<float> impulse(output.size(), 0.0F);
    if (!impulse.empty()) impulse[0] = static_cast<float>(source.gain);
    std::vector<CMonoBuffer<float>> paths(7, CMonoBuffer<float>(output.size(), 0.0F));
    model.Process(impulse, Common::CTransform(to_brt(local_source)),
                  Common::CTransform(to_brt(local_receiver)), paths, positions);
    for (std::size_t path = 0; path < paths.size(); ++path) {
        const bool direct = path == 6;
        if ((direct && !configuration.enable_direct_path) ||
            (!direct && !configuration.enable_reverberation)) continue;
        for (std::size_t frame = 0; frame < output.size(); ++frame) {
            output[frame] += paths[path][frame];
        }
    }
#else
    static_cast<void>(scene); static_cast<void>(source); static_cast<void>(receiver);
    static_cast<void>(configuration); static_cast<void>(settings); static_cast<void>(output);
    throw std::logic_error("SDN modeling requires BRTLibrary support");
#endif
}

void compute_ray_tracing(const Scene& scene, const Source& source,
                         const Receiver& receiver, const BRTSettings& configuration,
                         const ImpulseResponseSettings& settings, AudioBuffer& output) {
    add_direct_path(scene, source, receiver, configuration, settings, output);
    if (!configuration.enable_reverberation || configuration.reflection_order == 0) return;
    std::mt19937_64 generator(configuration.random_seed);
    std::uniform_real_distribution<double> rotation(0.0, 2.0 * kPi);
    const auto phase = rotation(generator);
    const double golden_angle = kPi * (3.0 - std::sqrt(5.0));
    for (std::size_t ray = 0; ray < configuration.ray_count; ++ray) {
        const auto z = 1.0 - 2.0 * (static_cast<double>(ray) + 0.5) / configuration.ray_count;
        const auto radial = std::sqrt(std::max(0.0, 1.0 - z * z));
        const auto azimuth = phase + golden_angle * static_cast<double>(ray);
        Vec3 direction{radial * std::cos(azimuth), radial * std::sin(azimuth), z};
        Vec3 origin = source.position;
        double travelled = 0.0;
        double energy = source.gain / static_cast<double>(configuration.ray_count);
        for (std::size_t bounce = 0; bounce <= configuration.reflection_order; ++bounce) {
            const auto hit = nearest_hit(scene, origin, direction);
            const auto receiver_distance = receiver_intersection(
                origin, direction, receiver.position, configuration.receiver_radius);
            if (bounce > 0 && receiver_distance < hit.distance) {
                const auto path_length = travelled + receiver_distance;
                const auto delay = configuration.propagation_delay
                    ? path_length * settings.impulse_response_sample_rate / scene.speed_of_sound
                    : 0.0;
                const auto distance_gain = configuration.distance_attenuation
                    ? 1.0 / std::max(1.0, path_length) : 1.0;
                const auto capture_compensation =
                    4.0 / (configuration.receiver_radius * configuration.receiver_radius);
                add_arrival(output, delay, energy * distance_gain * capture_compensation);
            }
            if (!std::isfinite(hit.distance)) break;
            if (bounce == configuration.reflection_order) break;
            travelled += hit.distance;
            const auto& triangle = scene.geometry[hit.triangle];
            const auto absorption = std::clamp(mean_absorption(
                scene, triangle.material_id, configuration.default_absorption), 0.0, 1.0);
            energy *= std::sqrt(1.0 - absorption);
            const auto point = add(origin, multiply(direction, hit.distance));
            direction = normalized(subtract(direction,
                multiply(hit.normal, 2.0 * dot(direction, hit.normal))));
            origin = add(point, multiply(direction, epsilon * 10.0));
        }
    }
}

} // namespace

BRTGeometricalSolver::BRTGeometricalSolver(BRTSettings settings) : settings_(settings) {}

std::string_view BRTGeometricalSolver::name() const noexcept {
    switch (settings_.method) {
    case Method::free_field: return "BRT free-field acoustics";
    case Method::image_source: return "BRT image-source acoustics";
    case Method::scattering_delay_network: return "BRT scattering delay network";
    case Method::ray_tracing: return "VA/BRT geometrical ray tracing";
    }
    return "BRT geometrical acoustics";
}

void BRTGeometricalSolver::validate(
    const Scene& scene, const ImpulseResponseSettings& settings) const {
    if (!std::isfinite(settings.impulse_response_sample_rate) ||
        !std::isfinite(settings.impulse_response_duration_seconds) ||
        settings.impulse_response_sample_rate <= 0.0 ||
        settings.impulse_response_duration_seconds <= 0.0) {
        throw std::invalid_argument("sample rate and duration must be finite and positive");
    }
    if (!std::isfinite(scene.speed_of_sound) || scene.speed_of_sound <= 0.0) {
        throw std::invalid_argument("speed of sound must be finite and positive");
    }
    if (scene.sources.empty() || scene.receivers.empty()) {
        throw std::invalid_argument("scene requires at least one source and receiver");
    }
    if (settings_.default_absorption < 0.0 || settings_.default_absorption >= 1.0) {
        throw std::invalid_argument("default absorption must be in [0, 1)");
    }
    for (const auto& material : scene.materials) {
        for (const auto absorption : material.octave_absorption) {
            if (!std::isfinite(absorption) || absorption < 0.0 || absorption > 1.0) {
                throw std::invalid_argument("material absorption must be finite and in [0, 1]");
            }
        }
    }
    if (settings_.method != Method::free_field &&
        settings_.method != Method::scattering_delay_network && scene.geometry.empty()) {
        throw std::invalid_argument("selected geometrical method requires triangle geometry");
    }
    if (settings_.method == Method::image_source) {
#if !VA_HAS_BRT
        throw std::logic_error("image-source modeling requires BRTLibrary support");
#endif
        if (settings_.reflection_order > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
            throw std::invalid_argument("image-source reflection order is too large");
        }
        std::size_t paths = 1;
        std::size_t level = 1;
        for (std::size_t order = 0; order < settings_.reflection_order; ++order) {
            if (level > settings_.maximum_paths / scene.geometry.size()) {
                throw std::invalid_argument("image-source path limit exceeded");
            }
            level *= scene.geometry.size();
            if (paths > settings_.maximum_paths - level) {
                throw std::invalid_argument("image-source path limit exceeded");
            }
            paths += level;
        }
    }
    if (settings_.method == Method::scattering_delay_network) {
#if !VA_HAS_BRT
        throw std::logic_error("SDN modeling requires BRTLibrary support");
#endif
        const auto dimensions = subtract(scene.bounds.maximum, scene.bounds.minimum);
        if (dimensions.x <= 0.0 || dimensions.y <= 0.0 || dimensions.z <= 0.0) {
            throw std::invalid_argument("SDN requires positive shoebox scene bounds");
        }
        const auto inside = [&scene](const Vec3& point) {
            return point.x > scene.bounds.minimum.x && point.x < scene.bounds.maximum.x &&
                   point.y > scene.bounds.minimum.y && point.y < scene.bounds.maximum.y &&
                   point.z > scene.bounds.minimum.z && point.z < scene.bounds.maximum.z;
        };
        for (const auto& source : scene.sources) {
            if (!inside(source.position)) {
                throw std::invalid_argument("SDN source must be inside the shoebox bounds");
            }
        }
        for (const auto& receiver : scene.receivers) {
            if (!inside(receiver.position)) {
                throw std::invalid_argument("SDN receiver must be inside the shoebox bounds");
            }
        }
        if (settings.impulse_response_sample_rate <= 32'000.0) {
            throw std::invalid_argument("BRT SDN requires a sample rate above 32 kHz");
        }
        if (!settings_.propagation_delay || !settings_.distance_attenuation) {
            throw std::invalid_argument(
                "BRT SDN always applies propagation delay and distance attenuation");
        }
    }
    if (settings_.method == Method::ray_tracing &&
        (settings_.ray_count == 0 || settings_.receiver_radius <= 0.0 ||
         !std::isfinite(settings_.receiver_radius))) {
        throw std::invalid_argument("ray tracing requires rays and a positive receiver radius");
    }
}

ImpulseResponseSet BRTGeometricalSolver::compute_impulse_responses(
    const Scene& scene, const ImpulseResponseSettings& settings) {
    validate(scene, settings);
    const auto frames = sample_count(settings);
    ImpulseResponseSet result{settings.impulse_response_sample_rate,
                              settings.impulse_response_sample_rate * 0.5,
                              scene.sources.size(), scene.receivers.size(),
                              std::vector<AudioBuffer>(scene.sources.size() * scene.receivers.size(),
                                                       AudioBuffer(frames))};
    for (std::size_t source_index = 0; source_index < scene.sources.size(); ++source_index) {
        for (std::size_t receiver_index = 0; receiver_index < scene.receivers.size();
             ++receiver_index) {
            auto& output = result.response(source_index, receiver_index);
            const auto& source = scene.sources[source_index];
            const auto& receiver = scene.receivers[receiver_index];
            switch (settings_.method) {
            case Method::free_field:
                add_direct_path(scene, source, receiver, settings_, settings, output);
                break;
            case Method::image_source:
                compute_image_source(scene, source, receiver, settings_, settings, output);
                break;
            case Method::scattering_delay_network:
                compute_sdn(scene, source, receiver, settings_, settings, output);
                break;
            case Method::ray_tracing:
                compute_ray_tracing(scene, source, receiver, settings_, settings, output);
                break;
            }
        }
    }
    return result;
}

const BRTSettings& BRTGeometricalSolver::settings() const noexcept { return settings_; }

bool BRTGeometricalSolver::brt_headers_available() noexcept {
#if VA_HAS_BRT
    return true;
#else
    return false;
#endif
}

} // namespace va::geometrical
