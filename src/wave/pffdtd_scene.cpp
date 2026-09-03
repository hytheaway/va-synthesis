#include "va/wave/pffdtd_scene.hpp"

#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace va::wave {
namespace {

std::string json_string(const std::string& input) {
    std::string output{"\""};
    for (const auto character : input) {
        switch (character) {
        case '\\': output += "\\\\"; break;
        case '\"': output += "\\\""; break;
        case '\n': output += "\\n"; break;
        case '\r': output += "\\r"; break;
        case '\t': output += "\\t"; break;
        default:
            if (static_cast<unsigned char>(character) < 0x20) {
                throw std::invalid_argument("identifier contains an unsupported control character");
            }
            output += character;
        }
    }
    return output + "\"";
}

void write_vec3(std::ostream& output, const Vec3& value) {
    output << '[' << value.x << ',' << value.y << ',' << value.z << ']';
}

} // namespace

void write_pffdtd_model(const Scene& scene, const std::filesystem::path& output_path) {
    if (scene.geometry.empty() || scene.sources.empty() || scene.receivers.empty()) {
        throw std::invalid_argument("PFFDTD export requires geometry, sources, and receivers");
    }
    std::map<std::string, std::vector<const Triangle*>> groups;
    for (const auto& triangle : scene.geometry) {
        groups[triangle.material_id].push_back(&triangle);
    }

    std::ofstream output(output_path);
    if (!output) {
        throw std::runtime_error("could not create PFFDTD model file");
    }
    output.precision(17);
    output << "{\n\"mats_hash\":{";
    bool first_group = true;
    for (const auto& [material, triangles] : groups) {
        if (!first_group) output << ',';
        first_group = false;
        output << '\n' << json_string(material) << ":{\"tris\":[";
        for (std::size_t index = 0; index < triangles.size(); ++index) {
            if (index != 0) output << ',';
            output << '[' << index * 3 << ',' << index * 3 + 1 << ',' << index * 3 + 2 << ']';
        }
        output << "],\"pts\":[";
        bool first_point = true;
        for (const auto* triangle : triangles) {
            for (const auto& point : triangle->vertices) {
                if (!first_point) output << ',';
                first_point = false;
                write_vec3(output, point);
            }
        }
        output << "],\"color\":[128,128,128],\"sides\":[";
        for (std::size_t index = 0; index < triangles.size(); ++index) {
            if (index != 0) output << ',';
            output << triangles[index]->active_side;
        }
        output << "]}";
    }
    output << "\n},\n\"sources\":[";
    for (std::size_t index = 0; index < scene.sources.size(); ++index) {
        if (index != 0) output << ',';
        output << "{\"xyz\":";
        write_vec3(output, scene.sources[index].position);
        output << ",\"name\":\"S" << index + 1 << "\"}";
    }
    output << "],\n\"receivers\":[";
    for (std::size_t index = 0; index < scene.receivers.size(); ++index) {
        if (index != 0) output << ',';
        output << "{\"xyz\":";
        write_vec3(output, scene.receivers[index].position);
        output << ",\"name\":\"R" << index + 1 << "\"}";
    }
    output << "],\n\"va_materials\":{";
    for (std::size_t index = 0; index < scene.materials.size(); ++index) {
        if (index != 0) output << ',';
        output << '\n' << json_string(scene.materials[index].id) << ":[";
        for (std::size_t band = 0; band < scene.materials[index].octave_absorption.size(); ++band) {
            if (band != 0) output << ',';
            const auto absorption = scene.materials[index].octave_absorption[band];
            if (absorption < 0.0 || absorption > 1.0) {
                throw std::invalid_argument("material absorption must be in [0, 1]");
            }
            output << absorption;
        }
        output << ']';
    }
    output << "\n}\n}\n";
}

} // namespace va::wave
