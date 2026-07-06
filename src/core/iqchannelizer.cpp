#include "iqchannelizer.h"

#include "channelizerutils.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double TWO_PI = 6.28318530717958647692;
constexpr float CHANNEL_IQ_TARGET_LEVEL = 0.45f;
}

void IqChannelizer::reset() {
    ncoPhase = 0.0;
    decimationSum = {0.0f, 0.0f};
    lowPassState = {0.0f, 0.0f};
    preLowPassStates.fill({0.0f, 0.0f});
    decimationCount = 0;
    agcLevel = 0.01f;
}

IqChannelizer::Result IqChannelizer::processFloatIq(const float *samples,
                                                    std::size_t floatCount,
                                                    const RadioSettings &settings,
                                                    std::vector<float> &output) {
    output.clear();

    Result result;
    result.inputRate = settings.sampleRate;
    result.centerFrequency = settings.centerFrequency;
    result.listeningFrequency = settings.listeningFrequency;
    result.frequencyShift = settings.listeningFrequency - settings.centerFrequency;

    if (!samples || floatCount < 2 || settings.sampleRate <= 0.0 ||
        !std::isfinite(settings.sampleRate)) {
        return result;
    }

    const std::size_t iqSamples = floatCount / 2U;
    const double targetRate = channelizerTargetRate(settings);
    const int decimationFactor =
        (std::max)(1, static_cast<int>(std::floor(settings.sampleRate / targetRate)));
    const double outputRate = settings.sampleRate / static_cast<double>(decimationFactor);
    const double cutoff = channelizerCutoff(settings, outputRate);
    const float preLowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * cutoff / settings.sampleRate),
        0.000001,
        1.0));
    const float lowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * cutoff / outputRate),
        0.000001,
        1.0));

    const double fShift = result.frequencyShift;
    const double phaseIncrement = -TWO_PI * fShift / settings.sampleRate;
    const bool noFrequencyShift = std::abs(fShift) < 0.5;
    float rotI = 1.0f;
    float rotQ = 0.0f;
    float rotStepI = 1.0f;
    float rotStepQ = 0.0f;
    if (!noFrequencyShift) {
        rotI = static_cast<float>(std::cos(ncoPhase));
        rotQ = static_cast<float>(std::sin(ncoPhase));
        rotStepI = static_cast<float>(std::cos(phaseIncrement));
        rotStepQ = static_cast<float>(std::sin(phaseIncrement));
    }

    output.reserve((iqSamples / static_cast<std::size_t>(decimationFactor) + 8U) * 2U);

    for (std::size_t n = 0; n < iqSamples; ++n) {
        float iSample = samples[2U * n];
        float qSample = samples[2U * n + 1U];
        if (!std::isfinite(iSample)) {
            iSample = 0.0f;
        }
        if (!std::isfinite(qSample)) {
            qSample = 0.0f;
        }

        if (settings.inputMode == INPUT_HF_COMBINED) {
            if (fShift < 0.0) {
                qSample = 0.0f;
            } else {
                iSample = qSample;
                qSample = 0.0f;
            }
        } else if (settings.inputMode == INPUT_HF1) {
            qSample = 0.0f;
        } else if (settings.inputMode == INPUT_HF2) {
            iSample = qSample;
            qSample = 0.0f;
        }

        std::complex<float> mixedSample;
        if (noFrequencyShift) {
            mixedSample = {iSample, qSample};
        } else {
            const float mixedI = iSample * rotI - qSample * rotQ;
            const float mixedQ = iSample * rotQ + qSample * rotI;
            mixedSample = {mixedI, mixedQ};
        }

        if (decimationFactor > 1) {
            preLowPassStates[0] += preLowPassAlpha * (mixedSample - preLowPassStates[0]);
            preLowPassStates[1] += preLowPassAlpha * (preLowPassStates[0] - preLowPassStates[1]);
            preLowPassStates[2] += preLowPassAlpha * (preLowPassStates[1] - preLowPassStates[2]);
            mixedSample = preLowPassStates[2];
        }

        decimationSum += mixedSample;
        ++decimationCount;

        if (!noFrequencyShift) {
            const float nextRotI = rotI * rotStepI - rotQ * rotStepQ;
            const float nextRotQ = rotI * rotStepQ + rotQ * rotStepI;
            rotI = nextRotI;
            rotQ = nextRotQ;
            if ((n & 4095U) == 4095U) {
                const float norm = std::sqrt(rotI * rotI + rotQ * rotQ);
                if (norm > 0.0f) {
                    rotI /= norm;
                    rotQ /= norm;
                }
            }
        }

        if (decimationCount < decimationFactor) {
            continue;
        }

        const float invCount = 1.0f / static_cast<float>(decimationCount);
        std::complex<float> channelSample = decimationSum * invCount;
        decimationSum = {0.0f, 0.0f};
        decimationCount = 0;

        lowPassState += lowPassAlpha * (channelSample - lowPassState);
        const float magnitude = std::abs(lowPassState);
        const float agcCoeff = magnitude > agcLevel ? 0.01f : 0.0002f;
        agcLevel += agcCoeff * (magnitude - agcLevel);
        agcLevel = (std::max)(agcLevel, 0.00001f);
        const float gain = CHANNEL_IQ_TARGET_LEVEL / agcLevel;
        output.push_back(std::clamp(std::real(lowPassState) * gain, -1.0f, 1.0f));
        output.push_back(std::clamp(std::imag(lowPassState) * gain, -1.0f, 1.0f));
    }

    ncoPhase = noFrequencyShift
                   ? 0.0
                   : std::remainder(ncoPhase + phaseIncrement * static_cast<double>(iqSamples),
                                    TWO_PI);
    if (ncoPhase < 0.0) {
        ncoPhase += TWO_PI;
    }

    result.valid = true;
    result.outputRate = outputRate;
    result.cutoff = cutoff;
    result.decimationFactor = decimationFactor;
    result.inputSamples = static_cast<int>(iqSamples);
    result.outputSamples = static_cast<int>(output.size() / 2U);
    result.agcLevel = agcLevel;
    return result;
}
