#pragma once

#include "va/core/types.hpp"

#include <filesystem>

namespace va::binaural {

struct StereoBuffer {
    AudioBuffer left;
    AudioBuffer right;
};

// BRTLibrary's binaural path is a real-time graph: CSOFAReader loads a SOFA HRTF
// into CSphericalInterpolatedFIRTable, then CHRTFConvolver / CListenerDirectHRTFConvolution
// spatialize block by block. That graph needs libmysofa (not vendored in this BRT checkout)
// and a running CBRTManager. Offline we use the same SOFA/HRTF model at the end of the
// pipeline: look up a left/right HRIR for the source direction in the listener's head
// frame and convolve the already rendered mono receiver signal.
[[nodiscard]] bool sofa_reader_available() noexcept;
[[nodiscard]] StereoBuffer spatialize_receiver(
    const AudioBuffer& mono, double sample_rate, const std::filesystem::path& sofa_path,
    const Vec3& source_position, const Receiver& listener);

} // namespace va::binaural
