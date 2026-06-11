#include "fobos_dmr_voice_backend.h"

// SPDX-License-Identifier: GPL-2.0-or-later

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>

int main() {
    const fobos_dmr_voice_backend_info *info = fobos_dmr_voice_get_backend_info();
    if (!info || info->abi_version != FOBOS_DMR_VOICE_ABI_VERSION) {
        std::cerr << "backend ABI mismatch\n";
        return 1;
    }

    std::cout << info->display_name << "\n";
    std::cout << "  id: " << info->backend_id << "\n";
    std::cout << "  version: " << info->version << "\n";
    std::cout << "  capabilities: 0x" << std::hex << info->capability_flags << std::dec << "\n";

    fobos_dmr_voice_context *context = fobos_dmr_voice_create();
    if (!context) {
        std::cerr << "failed to create backend context\n";
        return 1;
    }

    std::array<uint8_t, 7> empty_payload = {};
    std::array<int16_t, 160> pcm = {};
    int errors = 0;
    const int decode_status =
        fobos_dmr_voice_decode_ambe2450_payload49(context,
                                                  empty_payload.data(),
                                                  0,
                                                  pcm.data(),
                                                  &errors);
    std::cout << "  empty payload decode status: " << decode_status
              << ", errors: " << errors << "\n";
    if (decode_status != FOBOS_DMR_VOICE_OK) {
        std::cout << "  last error: " << fobos_dmr_voice_get_last_error(context) << "\n";
    }

    std::array<uint8_t, 7> encoded_payload = {};
    const int encode_status =
        fobos_dmr_voice_encode_ambe2450_payload49(context,
                                                  pcm.data(),
                                                  encoded_payload.data());
    std::cout << "  payload encode status: " << encode_status << "\n";

    std::array<uint8_t, 9> canonical_frame = {};
    const int canonical_encode_status =
        fobos_dmr_voice_encode_canonical_ambe_frame72(context,
                                                      pcm.data(),
                                                      canonical_frame.data());
    std::cout << "  canonical encode status: " << canonical_encode_status << "\n";

    std::array<int16_t, 160> canonical_pcm = {};
    int canonical_errors = 0;
    const int canonical_decode_status =
        fobos_dmr_voice_decode_canonical_ambe_frame72(context,
                                                      canonical_frame.data(),
                                                      canonical_pcm.data(),
                                                      &canonical_errors);
    std::cout << "  canonical decode status: " << canonical_decode_status
              << ", errors: " << canonical_errors << "\n";

    const uint32_t canonical_caps =
        FOBOS_DMR_VOICE_CAP_ENCODE_CANONICAL_AMBE_FRAME72 |
        FOBOS_DMR_VOICE_CAP_DECODE_CANONICAL_AMBE_FRAME72;
    if ((info->capability_flags & canonical_caps) == canonical_caps) {
        constexpr int sample_rate = 8000;
        constexpr int samples_per_frame = 160;
        constexpr int frame_count = 50;
        constexpr double pi = 3.14159265358979323846;
        constexpr double tone_hz = 440.0;

        fobos_dmr_voice_reset(context);

        double input_energy = 0.0;
        double output_energy = 0.0;
        int ok_frames = 0;
        int failed_frames = 0;
        int total_errors = 0;
        int nonzero_output_samples = 0;

        for (int frame = 0; frame < frame_count; ++frame) {
            std::array<int16_t, samples_per_frame> tone = {};
            for (int i = 0; i < samples_per_frame; ++i) {
                const int sample_index = frame * samples_per_frame + i;
                const double t = static_cast<double>(sample_index) /
                                 static_cast<double>(sample_rate);
                const double voiced =
                    0.82 * std::sin(2.0 * pi * tone_hz * t) +
                    0.18 * std::sin(2.0 * pi * tone_hz * 2.0 * t);
                const int sample = static_cast<int>(std::lrint(9000.0 * voiced));
                tone[static_cast<std::size_t>(i)] =
                    static_cast<int16_t>((std::max)(-32768, (std::min)(32767, sample)));
                input_energy += static_cast<double>(tone[static_cast<std::size_t>(i)]) *
                                static_cast<double>(tone[static_cast<std::size_t>(i)]);
            }

            std::array<uint8_t, 9> ambe = {};
            const int encode_frame_status =
                fobos_dmr_voice_encode_canonical_ambe_frame72(context,
                                                              tone.data(),
                                                              ambe.data());
            if (encode_frame_status != FOBOS_DMR_VOICE_OK) {
                ++failed_frames;
                continue;
            }

            std::array<int16_t, samples_per_frame> decoded = {};
            int frame_errors = 0;
            const int decode_frame_status =
                fobos_dmr_voice_decode_canonical_ambe_frame72(context,
                                                              ambe.data(),
                                                              decoded.data(),
                                                              &frame_errors);
            if (decode_frame_status != FOBOS_DMR_VOICE_OK) {
                ++failed_frames;
                continue;
            }

            ++ok_frames;
            total_errors += frame_errors;
            for (const int16_t sample : decoded) {
                output_energy += static_cast<double>(sample) *
                                 static_cast<double>(sample);
                if (sample != 0) {
                    ++nonzero_output_samples;
                }
            }
        }

        const double denominator =
            static_cast<double>(frame_count * samples_per_frame);
        const double input_rms = std::sqrt(input_energy / denominator);
        const double output_rms = std::sqrt(output_energy / denominator);
        std::cout << "  canonical tone frames: " << ok_frames
                  << "/" << frame_count
                  << ", failed: " << failed_frames
                  << ", errors: " << total_errors
                  << ", input RMS: " << input_rms
                  << ", output RMS: " << output_rms
                  << ", output nonzero: " << nonzero_output_samples
                  << "\n";
    }

    fobos_dmr_voice_destroy(context);
    return 0;
}
