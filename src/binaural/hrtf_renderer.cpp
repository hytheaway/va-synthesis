#include "va/binaural/hrtf_renderer.hpp"

#include "va/core/audio.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#if VA_HAS_MYSOFA
#include <mysofa.h>
#endif

#if VA_HAS_BRT
#include <Common/Transform.hpp>
#endif

namespace va::binaural {
namespace {

Vec3 source_in_listener_frame(const Vec3& source, const Receiver& listener) {
#if VA_HAS_BRT
    Common::CTransform listener_transform;
    listener_transform.SetPosition(Common::CVector3(
        static_cast<float>(listener.position.x), static_cast<float>(listener.position.y),
        static_cast<float>(listener.position.z)));
    const auto deg2rad = static_cast<float>(std::numbers::pi / 180.0);
    listener_transform.SetOrientation(Common::CQuaternion::FromYawPitchRoll(
        static_cast<float>(listener.yaw_degrees) * deg2rad,
        static_cast<float>(listener.pitch_degrees) * deg2rad,
        static_cast<float>(listener.roll_degrees) * deg2rad));
    Common::CTransform source_transform;
    source_transform.SetPosition(Common::CVector3(
        static_cast<float>(source.x), static_cast<float>(source.y), static_cast<float>(source.z)));
    const auto local = listener_transform.GetVectorTo(source_transform);
    return {local.x, local.y, local.z};
#else
    const auto deg2rad = std::numbers::pi / 180.0;
    const auto yaw = listener.yaw_degrees * deg2rad;
    const auto pitch = listener.pitch_degrees * deg2rad;
    const auto roll = listener.roll_degrees * deg2rad;
    const auto cy = std::cos(yaw);
    const auto sy = std::sin(yaw);
    const auto cp = std::cos(pitch);
    const auto sp = std::sin(pitch);
    const auto cr = std::cos(roll);
    const auto sr = std::sin(roll);
    const auto dx = source.x - listener.position.x;
    const auto dy = source.y - listener.position.y;
    const auto dz = source.z - listener.position.z;
    // Inverse of BRT yaw-pitch-roll with Ambisonic axes (forward +X, left +Y, up +Z).
    const auto t0 = cy * dx + sy * dy;
    const auto t1 = -sy * dx + cy * dy;
    const auto t2 = dz;
    const auto u0 = cp * t0 + sp * t2;
    const auto u1 = t1;
    const auto u2 = -sp * t0 + cp * t2;
    return {u0, cr * u1 - sr * u2, sr * u1 + cr * u2};
#endif
}

AudioBuffer apply_delay(const float* ir, int length, float delay_seconds, double sample_rate) {
    auto delay_samples = static_cast<int>(std::llround(static_cast<double>(delay_seconds) * sample_rate));
    if (delay_samples < 0) delay_samples = 0;
    AudioBuffer output(static_cast<std::size_t>(length) + static_cast<std::size_t>(delay_samples));
    for (int index = 0; index < length; ++index) {
        output[static_cast<std::size_t>(delay_samples + index)] = ir[index];
    }
    return output;
}

} // namespace

bool sofa_reader_available() noexcept {
#if VA_HAS_MYSOFA
    return true;
#else
    return false;
#endif
}

StereoBuffer spatialize_receiver(
    const AudioBuffer& mono, double sample_rate, const std::filesystem::path& sofa_path,
    const Vec3& source_position, const Receiver& listener) {
    if (mono.empty()) return {};
    if (!std::isfinite(sample_rate) || sample_rate <= 0.0) {
        throw std::invalid_argument("HRTF sample rate must be positive");
    }
    if (sofa_path.empty()) {
        throw std::invalid_argument("HRTF SOFA path is empty");
    }
#if !VA_HAS_MYSOFA
    throw std::runtime_error(
        "This renderer was built without libmysofa, so HRTF SOFA files cannot be loaded");
#else
    int filter_length = 0;
    int error = 0;
    struct MYSOFA_EASY* hrtf = mysofa_open(sofa_path.string().c_str(),
                                           static_cast<float>(sample_rate), &filter_length, &error);
    if (!hrtf || error != MYSOFA_OK || filter_length <= 0) {
        if (hrtf) mysofa_close(hrtf);
        throw std::runtime_error("could not read HRTF SOFA file (" + sofa_path.string() +
                                 "), libmysofa error " + std::to_string(error));
    }

    const auto local = source_in_listener_frame(source_position, listener);
    const auto distance = std::sqrt(local.x * local.x + local.y * local.y + local.z * local.z);
    if (!(distance > 1.0e-6)) {
        mysofa_close(hrtf);
        return {mono, mono};
    }

    std::vector<float> left_ir(static_cast<std::size_t>(filter_length));
    std::vector<float> right_ir(static_cast<std::size_t>(filter_length));
    float delay_left = 0.0F;
    float delay_right = 0.0F;
    mysofa_getfilter_float(hrtf, static_cast<float>(local.x), static_cast<float>(local.y),
                           static_cast<float>(local.z), left_ir.data(), right_ir.data(),
                           &delay_left, &delay_right);
    mysofa_close(hrtf);

    auto left_hrir = apply_delay(left_ir.data(), filter_length, delay_left, sample_rate);
    auto right_hrir = apply_delay(right_ir.data(), filter_length, delay_right, sample_rate);
    const auto frames = mono.size() + std::max(left_hrir.size(), right_hrir.size()) - 1;
    StereoBuffer stereo;
    stereo.left = audio::convolve(mono, left_hrir, frames);
    stereo.right = audio::convolve(mono, right_hrir, frames);
    return stereo;
#endif
}

} // namespace va::binaural
