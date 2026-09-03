#pragma once

#include "va/core/types.hpp"

#include <filesystem>

namespace va::wave {

// Writes PFFDTD's model_export.json schema, plus a va_materials section used by
// tools/prepare_pffdtd_job.py to fit the impedance datasets.
void write_pffdtd_model(const Scene& scene, const std::filesystem::path& output);

} // namespace va::wave
