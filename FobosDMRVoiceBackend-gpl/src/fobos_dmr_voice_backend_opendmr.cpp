#include "fobos_dmr_voice_backend.h"

// SPDX-License-Identifier: GPL-2.0-only

#include <array>
#include <cstdint>
#include <cstring>

#include "opendmr.h"

struct fobos_dmr_voice_context {
    opendmr_decoder_t *decoder = nullptr;
    opendmr_encoder_t *encoder = nullptr;
    char last_error[192] = {};
};

namespace {

constexpr int PCM_8K_SAMPLES = 160;
constexpr int CANONICAL_FRAME_BYTES = 9;
constexpr int AMBE_PAYLOAD_BYTES = 7;

const fobos_dmr_voice_backend_info BACKEND_INFO = {
    FOBOS_DMR_VOICE_ABI_VERSION,
    FOBOS_DMR_VOICE_CAP_DECODE_CANONICAL_AMBE_FRAME72 |
        FOBOS_DMR_VOICE_CAP_ENCODE_CANONICAL_AMBE_FRAME72,
    "fobos.dmr.voice.opendmr",
    "Fobos DMR Voice Backend (OpenDMR/OP25)",
    "0.1.0",
    "GPL-2.0",
    "Optional AMBE/AMBE+2 encode/decode backend. AMBE technologies may be patent-encumbered in some jurisdictions."
};

void set_error(fobos_dmr_voice_context *context, const char *message) {
    if (!context) {
        return;
    }
    std::strncpy(context->last_error, message, sizeof(context->last_error) - 1);
    context->last_error[sizeof(context->last_error) - 1] = '\0';
}

bool ensure_decoder(fobos_dmr_voice_context *context) {
    if (!context) {
        return false;
    }
    if (!context->decoder) {
        context->decoder = opendmr_decoder_create();
    }
    if (!context->decoder) {
        set_error(context, "failed to create OpenDMR decoder");
        return false;
    }
    return true;
}

bool ensure_encoder(fobos_dmr_voice_context *context) {
    if (!context) {
        return false;
    }
    if (!context->encoder) {
        context->encoder = opendmr_encoder_create();
    }
    if (!context->encoder) {
        set_error(context, "failed to create OpenDMR encoder");
        return false;
    }
    return true;
}

} // namespace

extern "C" {

FOBOS_DMR_VOICE_API const fobos_dmr_voice_backend_info *
fobos_dmr_voice_get_backend_info(void) {
    return &BACKEND_INFO;
}

FOBOS_DMR_VOICE_API fobos_dmr_voice_context *
fobos_dmr_voice_create(void) {
    fobos_dmr_voice_context *context = new fobos_dmr_voice_context();
    context->decoder = opendmr_decoder_create();
    context->encoder = opendmr_encoder_create();
    if (!context->decoder || !context->encoder) {
        set_error(context, "failed to initialize OpenDMR backend");
    }
    return context;
}

FOBOS_DMR_VOICE_API void
fobos_dmr_voice_destroy(fobos_dmr_voice_context *context) {
    if (!context) {
        return;
    }
    opendmr_decoder_destroy(context->decoder);
    opendmr_encoder_destroy(context->encoder);
    delete context;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_reset(fobos_dmr_voice_context *context) {
    if (!context) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    if (context->decoder) {
        opendmr_decoder_reset(context->decoder);
    }
    if (context->encoder) {
        opendmr_encoder_reset(context->encoder);
    }
    set_error(context, "");
    return FOBOS_DMR_VOICE_OK;
}

FOBOS_DMR_VOICE_API const char *
fobos_dmr_voice_get_last_error(fobos_dmr_voice_context *context) {
    if (!context) {
        return "invalid backend context";
    }
    return context->last_error;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_decode_dmr_ambe_frame72(fobos_dmr_voice_context *context,
                                        const uint8_t *frame72_bytes,
                                        int16_t *pcm8k_160,
                                        int *error_count) {
    if (!context) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    (void)frame72_bytes;
    (void)pcm8k_160;
    (void)error_count;
    set_error(context, "raw DMR over-the-air frame decode is not available in the OpenDMR backend");
    return FOBOS_DMR_VOICE_UNSUPPORTED;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_decode_dmr_ambe_soft_frame72(fobos_dmr_voice_context *context,
                                             const uint8_t *frame72_bytes,
                                             const uint8_t *reliability72,
                                             int16_t *pcm8k_160,
                                             int *error_count) {
    if (!context) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    (void)frame72_bytes;
    (void)reliability72;
    (void)pcm8k_160;
    (void)error_count;
    set_error(context, "soft DMR over-the-air frame decode is not available in the OpenDMR backend");
    return FOBOS_DMR_VOICE_UNSUPPORTED;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_decode_ambe2450_payload49(fobos_dmr_voice_context *context,
                                          const uint8_t *payload49_bytes,
                                          int corrected_errors,
                                          int16_t *pcm8k_160,
                                          int *error_count) {
    if (!context) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    (void)payload49_bytes;
    (void)corrected_errors;
    (void)pcm8k_160;
    (void)error_count;
    set_error(context, "AMBE2450 payload decode is not available in the OpenDMR backend");
    return FOBOS_DMR_VOICE_UNSUPPORTED;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_decode_canonical_ambe_frame72(fobos_dmr_voice_context *context,
                                              const uint8_t *canonical_frame72_bytes,
                                              int16_t *pcm8k_160,
                                              int *error_count) {
    if (!context || !canonical_frame72_bytes || !pcm8k_160) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    if (!ensure_decoder(context)) {
        return FOBOS_DMR_VOICE_NOT_AVAILABLE;
    }
    int errors = 0;
    const bool ok = opendmr_decode(context->decoder,
                                   canonical_frame72_bytes,
                                   pcm8k_160,
                                   &errors);
    if (!ok) {
        set_error(context, "OpenDMR failed to decode canonical AMBE frame");
        return FOBOS_DMR_VOICE_DECODE_FAILED;
    }
    if (error_count) {
        *error_count = errors;
    }
    set_error(context, "");
    return FOBOS_DMR_VOICE_OK;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_encode_ambe2450_payload49(fobos_dmr_voice_context *context,
                                          const int16_t *pcm8k_160,
                                          uint8_t *payload49_bytes) {
    if (!context) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    (void)pcm8k_160;
    if (payload49_bytes) {
        std::memset(payload49_bytes, 0, AMBE_PAYLOAD_BYTES);
    }
    set_error(context, "AMBE2450 payload encode is not available in the OpenDMR backend");
    return FOBOS_DMR_VOICE_UNSUPPORTED;
}

FOBOS_DMR_VOICE_API int
fobos_dmr_voice_encode_canonical_ambe_frame72(fobos_dmr_voice_context *context,
                                              const int16_t *pcm8k_160,
                                              uint8_t *canonical_frame72_bytes) {
    if (!context || !pcm8k_160 || !canonical_frame72_bytes) {
        return FOBOS_DMR_VOICE_INVALID_ARGUMENT;
    }
    if (!ensure_encoder(context)) {
        return FOBOS_DMR_VOICE_NOT_AVAILABLE;
    }
    std::memset(canonical_frame72_bytes, 0, CANONICAL_FRAME_BYTES);
    const bool ok = opendmr_encode(context->encoder,
                                   pcm8k_160,
                                   canonical_frame72_bytes);
    if (!ok) {
        set_error(context, "OpenDMR failed to encode canonical AMBE frame");
        return FOBOS_DMR_VOICE_ENCODE_FAILED;
    }
    set_error(context, "");
    return FOBOS_DMR_VOICE_OK;
}

} // extern "C"
