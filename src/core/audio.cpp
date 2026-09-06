#include "va/core/audio.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <stdexcept>
#include <vector>

namespace va::audio {
namespace {

using Complex = std::complex<double>;

std::size_t next_power_of_two(std::size_t value) {
    std::size_t result = 1;
    while (result < value) {
        if (result > std::numeric_limits<std::size_t>::max() / 2) {
            throw std::length_error("audio buffer is too large for FFT convolution");
        }
        result <<= 1U;
    }
    return result;
}

void fft(std::vector<Complex>& values, bool inverse) {
    const auto count = values.size();
    for (std::size_t index = 1, reversed = 0; index < count; ++index) {
        std::size_t bit = count >> 1U;
        for (; reversed & bit; bit >>= 1U) reversed ^= bit;
        reversed ^= bit;
        if (index < reversed) std::swap(values[index], values[reversed]);
    }
    for (std::size_t length = 2; length <= count; length <<= 1U) {
        const auto angle = (inverse ? 2.0 : -2.0) * kPi /
                           static_cast<double>(length);
        const Complex root(std::cos(angle), std::sin(angle));
        for (std::size_t offset = 0; offset < count; offset += length) {
            Complex factor{1.0, 0.0};
            for (std::size_t index = 0; index < length / 2; ++index) {
                const auto even = values[offset + index];
                const auto odd = values[offset + index + length / 2] * factor;
                values[offset + index] = even + odd;
                values[offset + index + length / 2] = even - odd;
                factor *= root;
            }
        }
    }
    if (inverse) {
        for (auto& value : values) value /= static_cast<double>(count);
    }
}

double sinc(double value) {
    if (std::abs(value) < 1.0e-12) return 1.0;
    const auto argument = kPi * value;
    return std::sin(argument) / argument;
}

} // namespace

AudioBuffer convolve(const AudioBuffer& signal, const AudioBuffer& impulse_response,
                     std::size_t output_frames) {
    if (signal.empty() || impulse_response.empty()) return {};
    if (signal.size() > std::numeric_limits<std::size_t>::max() - impulse_response.size() + 1) {
        throw std::length_error("convolution output size overflow");
    }
    const auto full_size = signal.size() + impulse_response.size() - 1;
    const auto fft_size = next_power_of_two(full_size);
    std::vector<Complex> signal_spectrum(fft_size);
    std::vector<Complex> response_spectrum(fft_size);
    for (std::size_t index = 0; index < signal.size(); ++index) signal_spectrum[index] = signal[index];
    for (std::size_t index = 0; index < impulse_response.size(); ++index) {
        response_spectrum[index] = impulse_response[index];
    }
    fft(signal_spectrum, false);
    fft(response_spectrum, false);
    for (std::size_t index = 0; index < fft_size; ++index) {
        signal_spectrum[index] *= response_spectrum[index];
    }
    fft(signal_spectrum, true);

    const auto frames = output_frames == 0 ? full_size : output_frames;
    AudioBuffer output(frames);
    for (std::size_t index = 0; index < std::min(frames, full_size); ++index) {
        output[index] = static_cast<Sample>(signal_spectrum[index].real());
    }
    return output;
}

AudioBuffer resample(const AudioBuffer& input, double input_rate, double output_rate) {
    if (!std::isfinite(input_rate) || !std::isfinite(output_rate) ||
        input_rate <= 0.0 || output_rate <= 0.0) {
        throw std::invalid_argument("resampling rates must be positive");
    }
    if (input.empty() || input_rate == output_rate) return input;

    const auto exact_output_size =
        std::ceil(static_cast<double>(input.size()) * output_rate / input_rate);
    if (!std::isfinite(exact_output_size) ||
        exact_output_size > static_cast<double>(std::numeric_limits<std::size_t>::max())) {
        throw std::length_error("resampled audio buffer is too large");
    }
    const auto output_size = static_cast<std::size_t>(exact_output_size);
    AudioBuffer output(output_size);
    constexpr int radius = 24;
    const auto cutoff = 0.95 * std::min(1.0, output_rate / input_rate);
    for (std::size_t output_index = 0; output_index < output_size; ++output_index) {
        const auto position = static_cast<double>(output_index) * input_rate / output_rate;
        const auto center = static_cast<long long>(std::floor(position));
        double weighted_sum = 0.0;
        double weight_sum = 0.0;
        for (int tap = -radius + 1; tap <= radius; ++tap) {
            const auto input_index = center + tap;
            if (input_index < 0 || input_index >= static_cast<long long>(input.size())) continue;
            const auto distance = position - static_cast<double>(input_index);
            const auto window = 0.42 + 0.5 * std::cos(kPi * distance / radius) +
                                0.08 * std::cos(2.0 * kPi * distance / radius);
            const auto weight = cutoff * sinc(cutoff * distance) * window;
            weighted_sum += static_cast<double>(input[static_cast<std::size_t>(input_index)]) * weight;
            weight_sum += weight;
        }
        output[output_index] = weight_sum == 0.0
            ? 0.0F
            : static_cast<Sample>(weighted_sum / weight_sum);
    }
    return output;
}

SimulationResult render_sources(const Scene& scene, const AudioProgram& program,
                                const RenderSettings& settings,
                                const ImpulseResponseSet& impulse_responses) {
    if (!std::isfinite(program.sample_rate) ||
        !std::isfinite(settings.output_sample_rate) ||
        program.sample_rate <= 0.0 || settings.output_sample_rate <= 0.0 ||
        !std::isfinite(impulse_responses.sample_rate) ||
        impulse_responses.sample_rate <= 0.0) {
        throw std::invalid_argument("program and output sample rates must be positive");
    }
    if (impulse_responses.receiver_count != 0 &&
        impulse_responses.source_count >
            std::numeric_limits<std::size_t>::max() / impulse_responses.receiver_count) {
        throw std::invalid_argument("impulse-response dimensions overflow");
    }
    if (program.source_signals.size() != scene.sources.size() ||
        impulse_responses.source_count != scene.sources.size() ||
        impulse_responses.receiver_count != scene.receivers.size() ||
        impulse_responses.responses.size() !=
            impulse_responses.source_count * impulse_responses.receiver_count) {
        throw std::invalid_argument("audio program or impulse-response dimensions do not match scene");
    }

    std::vector<AudioBuffer> source_signals;
    source_signals.reserve(program.source_signals.size());
    std::size_t output_frames = 0;
    for (std::size_t source = 0; source < program.source_signals.size(); ++source) {
        source_signals.push_back(resample(program.source_signals[source], program.sample_rate,
                                          impulse_responses.sample_rate));
        auto frames = source_signals.back().size();
        if (settings.include_reverb_tail && frames != 0) {
            std::size_t longest_ir = 0;
            for (std::size_t receiver = 0; receiver < scene.receivers.size(); ++receiver) {
                longest_ir = std::max(longest_ir,
                    impulse_responses.response(source, receiver).size());
            }
            if (longest_ir != 0) {
                if (frames > std::numeric_limits<std::size_t>::max() - longest_ir + 1) {
                    throw std::length_error("rendered audio buffer is too large");
                }
                frames += longest_ir - 1;
            }
        }
        output_frames = std::max(output_frames, frames);
    }

    SimulationResult result{impulse_responses.sample_rate,
                            std::vector<AudioBuffer>(scene.receivers.size(),
                                                     AudioBuffer(output_frames))};
    for (std::size_t source = 0; source < scene.sources.size(); ++source) {
        for (std::size_t receiver = 0; receiver < scene.receivers.size(); ++receiver) {
            const auto rendered = convolve(source_signals[source],
                                           impulse_responses.response(source, receiver),
                                           output_frames);
            for (std::size_t frame = 0; frame < rendered.size(); ++frame) {
                result.receiver_signals[receiver][frame] += rendered[frame];
            }
        }
    }
    if (settings.output_sample_rate != result.sample_rate) {
        for (auto& receiver : result.receiver_signals) {
            receiver = resample(receiver, result.sample_rate, settings.output_sample_rate);
        }
        result.sample_rate = settings.output_sample_rate;
    }
    return result;
}

AudioBuffer low_pass(const AudioBuffer& input, double sample_rate, double cutoff_hz) {
    if (sample_rate <= 0.0 || cutoff_hz <= 0.0 || cutoff_hz >= sample_rate * 0.5) {
        throw std::invalid_argument("low-pass cutoff must be between zero and Nyquist");
    }
    AudioBuffer output(input.size());
    if (input.empty()) return output;
    const auto alpha = static_cast<Sample>(
        1.0 - std::exp(-2.0 * kPi * cutoff_hz / sample_rate));
    output[0] = alpha * input[0];
    for (std::size_t index = 1; index < input.size(); ++index) {
        output[index] = output[index - 1] + alpha * (input[index] - output[index - 1]);
    }
    return output;
}

AudioBuffer high_pass_complement(const AudioBuffer& input, double sample_rate, double cutoff_hz) {
    const auto low = low_pass(input, sample_rate, cutoff_hz);
    AudioBuffer output(input.size());
    for (std::size_t index = 0; index < input.size(); ++index) output[index] = input[index] - low[index];
    return output;
}

} // namespace va::audio
