#include "fft.h"
#include "iqbuffer.h"

#include <QDebug>
#include <algorithm>
#include <cmath>
#include <cstddef>

extern int globalMode;

extern int fftLength;
extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;
extern double minFrequency;
extern double maxFrequency;

namespace {
constexpr float FFT_MAGNITUDE_FLOOR_DB = -160.0f;
constexpr float HF_NOISE_CANCEL_MAX_COEFF = 2.5f;

float estimateHfNoiseCancelCoefficient(const std::vector<float> &iqSnapshot,
                                       int sourceStart,
                                       int sampleCount) {
    if (sampleCount <= 8) {
        return 0.0f;
    }

    double sumMain = 0.0;
    double sumRef = 0.0;
    double sumCross = 0.0;
    double sumRefSquared = 0.0;
    int count = 0;

    for (int i = 0; i < sampleCount; ++i) {
        const int sourceIndex = sourceStart + i;
        const float mainSample = iqSnapshot[2 * sourceIndex];
        const float refSample = iqSnapshot[2 * sourceIndex + 1];
        if (!std::isfinite(mainSample) || !std::isfinite(refSample)) {
            continue;
        }
        sumMain += mainSample;
        sumRef += refSample;
        sumCross += static_cast<double>(mainSample) * refSample;
        sumRefSquared += static_cast<double>(refSample) * refSample;
        ++count;
    }

    if (count <= 8) {
        return 0.0f;
    }

    const double meanMain = sumMain / count;
    const double meanRef = sumRef / count;
    const double covariance = sumCross - static_cast<double>(count) * meanMain * meanRef;
    const double refVariance = sumRefSquared - static_cast<double>(count) * meanRef * meanRef;
    if (!std::isfinite(covariance) || !std::isfinite(refVariance) || refVariance <= 1.0e-12) {
        return 0.0f;
    }

    return (std::clamp)(static_cast<float>(covariance / refVariance),
                        -HF_NOISE_CANCEL_MAX_COEFF,
                        HF_NOISE_CANCEL_MAX_COEFF);
}

std::complex<float> clampComplexMagnitude(std::complex<float> value, float maxMagnitude) {
    const float magnitude = std::abs(value);
    if (!std::isfinite(magnitude) || magnitude <= maxMagnitude || magnitude <= 0.0f) {
        return value;
    }
    return value * (maxMagnitude / magnitude);
}
}

FFTResult::FFTResult(QObject *parent)
    : QObject(parent), fftIn(nullptr), fftOut(nullptr), plan(nullptr), planLength(0) {
    ensurePlan(fftLength);
}

FFTResult::~FFTResult() {
    releasePlan();
}

void FFTResult::resetHfNoiseCancelState() {
    hfNoiseCancelBins.clear();
    hfNoiseCancelCrossPower.clear();
    hfNoiseCancelMainPower.clear();
    hfNoiseCancelRefPower.clear();
}

void FFTResult::releasePlan() {
    if (plan) {
        fftwf_destroy_plan(plan);
        plan = nullptr;
    }
    if (fftIn) {
        fftwf_free(fftIn);
        fftIn = nullptr;
    }
    if (fftOut) {
        fftwf_free(fftOut);
        fftOut = nullptr;
    }
    planLength = 0;
}

bool FFTResult::ensurePlan(int length) {
    if (length <= 0) {
        releasePlan();
        return false;
    }

    if (plan && fftIn && fftOut && planLength == length) {
        return true;
    }

    releasePlan();
    fftIn = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * length);
    fftOut = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * length);
    if (!fftIn || !fftOut) {
        qDebug() << "Failed to allocate FFT buffers for length" << length;
        releasePlan();
        return false;
    }

    plan = fftwf_plan_dft_1d(length, fftIn, fftOut, FFTW_FORWARD, FFTW_ESTIMATE);
    if (!plan) {
        qDebug() << "Failed to create FFTW plan for length" << length;
        releasePlan();
        return false;
    }

    planLength = length;
    return true;
}


void FFTResult::performFFTInThread() {
    QtConcurrent::run(this, &FFTResult::storeFFTResults);
}

bool FFTResult::storeFFTResults(const RadioSettings &settings,
                                std::vector<float> &outFrequencies,
                                std::vector<float> &outMagnitudes,
                                std::vector<float> *outReferenceMagnitudes,
                                IqBuffer::BlockMetadata *outMetadata) {
    const int currentFftLength = settings.fftLength;
    const double sampleRate = settings.sampleRate;
    double centerFrequency = settings.centerFrequency;
    const int inputMode = settings.inputMode;

    if (currentFftLength <= 0 || sampleRate <= 0.0) {
        outMagnitudes.clear();
        outFrequencies.clear();
        return false;
    }

    if (!ensurePlan(currentFftLength)) {
        outMagnitudes.clear();
        outFrequencies.clear();
        return false;
    }

    outMagnitudes.resize(static_cast<std::size_t>(currentFftLength));
    if (outReferenceMagnitudes) {
        outReferenceMagnitudes->clear();
    }
    const std::size_t requestedSnapshotFloats =
        static_cast<std::size_t>((std::max)(1, currentFftLength)) * 2U;
    IqBuffer::BlockMetadata snapshotMetadata;
    if (!IqBuffer::snapshotRecent(iqSnapshotScratch, requestedSnapshotFloats, nullptr, &snapshotMetadata)) {
        return false;
    }
    if (outMetadata) {
        *outMetadata = snapshotMetadata;
    }
    if (snapshotMetadata.valid &&
        std::isfinite(snapshotMetadata.centerFrequencyHz) &&
        snapshotMetadata.centerFrequencyHz > 0.0) {
        centerFrequency = snapshotMetadata.centerFrequencyHz;
    }
    outFrequencies.resize(currentFftLength);
    for (int i = 0; i < currentFftLength; ++i) {
        outFrequencies[i] = (i - currentFftLength / 2) * (sampleRate / currentFftLength) + centerFrequency;
    }

    const int availableIqSamples = static_cast<int>(iqSnapshotScratch.size() / 2);
    if (availableIqSamples <= 0) {
        return false;
    }

    const int samplesToCopy = std::min(currentFftLength, availableIqSamples);
    const int sourceStart = availableIqSamples - samplesToCopy;

    auto magnitudeDb = [this, samplesToCopy](int index) {
        const float re = std::isfinite(fftOut[index][0]) ? fftOut[index][0] : 0.0f;
        const float im = std::isfinite(fftOut[index][1]) ? fftOut[index][1] : 0.0f;
        const float value =
            std::sqrt(re * re + im * im) /
            static_cast<float>((std::max)(1, samplesToCopy));
        if (std::isfinite(value) && value > 0.0f) {
            return (std::max)(FFT_MAGNITUDE_FLOOR_DB, 20.0f * std::log10(value));
        }
        return FFT_MAGNITUDE_FLOOR_DB;
    };

    if (inputMode == INPUT_HF_COMBINED) {
        const int halfLength = currentFftLength / 2;
        std::vector<float> hf1Positive(halfLength + 1, FFT_MAGNITUDE_FLOOR_DB);
        std::vector<float> hf2Positive(halfLength + 1, FFT_MAGNITUDE_FLOOR_DB);
        std::vector<float> shiftedMagnitude(currentFftLength, FFT_MAGNITUDE_FLOOR_DB);

        for (int i = 0; i < currentFftLength; ++i) {
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = 0.0f;
        }
        for (int i = 0; i < samplesToCopy; ++i) {
            const int sourceIndex = sourceStart + i;
            const float iValue = iqSnapshotScratch[2 * sourceIndex];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
        }
        fftwf_execute(plan);
        for (int k = 0; k <= halfLength; ++k) {
            hf1Positive[k] = magnitudeDb(k);
        }

        for (int i = 0; i < currentFftLength; ++i) {
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = 0.0f;
        }
        for (int i = 0; i < samplesToCopy; ++i) {
            const int sourceIndex = sourceStart + i;
            const float qValue = iqSnapshotScratch[2 * sourceIndex + 1];
            fftIn[i][0] = std::isfinite(qValue) ? qValue : 0.0f;
        }
        fftwf_execute(plan);
        for (int k = 0; k <= halfLength; ++k) {
            hf2Positive[k] = magnitudeDb(k);
        }

        for (int i = 0; i < currentFftLength; ++i) {
            if (i < halfLength) {
                shiftedMagnitude[i] = hf1Positive[halfLength - i];
            } else {
                shiftedMagnitude[i] = hf2Positive[i - halfLength];
            }
            outMagnitudes[(i + halfLength) % currentFftLength] = shiftedMagnitude[i];
            outFrequencies[i] =
                (i - currentFftLength / 2) * (sampleRate / currentFftLength) + centerFrequency;
        }
        return true;
    }

    if (inputMode == INPUT_HF_NOISE_CANCEL) {
        std::vector<std::complex<float>> mainSpectrum(currentFftLength);
        std::vector<std::complex<float>> refSpectrum(currentFftLength);

        for (int i = 0; i < currentFftLength; ++i) {
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = 0.0f;
        }
        for (int i = 0; i < samplesToCopy; ++i) {
            const int sourceIndex = sourceStart + i;
            const float iValue = iqSnapshotScratch[2 * sourceIndex];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
        }
        fftwf_execute(plan);
        for (int i = 0; i < currentFftLength; ++i) {
            mainSpectrum[i] = std::complex<float>(std::isfinite(fftOut[i][0]) ? fftOut[i][0] : 0.0f,
                                                  std::isfinite(fftOut[i][1]) ? fftOut[i][1] : 0.0f);
        }

        for (int i = 0; i < currentFftLength; ++i) {
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = 0.0f;
        }
        for (int i = 0; i < samplesToCopy; ++i) {
            const int sourceIndex = sourceStart + i;
            const float qValue = iqSnapshotScratch[2 * sourceIndex + 1];
            fftIn[i][0] = std::isfinite(qValue) ? qValue : 0.0f;
        }
        fftwf_execute(plan);
        for (int i = 0; i < currentFftLength; ++i) {
            refSpectrum[i] = std::complex<float>(std::isfinite(fftOut[i][0]) ? fftOut[i][0] : 0.0f,
                                                 std::isfinite(fftOut[i][1]) ? fftOut[i][1] : 0.0f);
        }

        if (hfNoiseCancelBins.size() != static_cast<std::size_t>(currentFftLength)) {
            hfNoiseCancelBins.assign(currentFftLength, std::complex<float>(0.0f, 0.0f));
            hfNoiseCancelCrossPower.assign(currentFftLength, std::complex<float>(0.0f, 0.0f));
            hfNoiseCancelMainPower.assign(currentFftLength, 0.0f);
            hfNoiseCancelRefPower.assign(currentFftLength, 0.0f);
        }

        constexpr float adaptiveAlphaCold = 0.18f;
        constexpr float adaptiveAlphaWarm = 0.035f;
        constexpr float adaptiveEpsilon = 1.0e-7f;
        constexpr float coherenceStart = 0.08f;
        constexpr float coherenceFull = 0.38f;
        auto binFrequency = [currentFftLength, sampleRate, centerFrequency](int index) {
            const int shiftedIndex = index <= currentFftLength / 2
                                         ? index
                                         : index - currentFftLength;
            return shiftedIndex * (sampleRate / currentFftLength) + centerFrequency;
        };
        const float noiseCancelDepth =
            static_cast<float>((std::clamp)(settings.hfNoiseCancelDepth, 0.0, 2.0));
        if (!settings.hfNoiseCancelFreeze) {
            for (int i = 0; i < currentFftLength; ++i) {
                const std::complex<float> adjustedRef =
                    hfNoiseCancelReferenceCoefficient(settings, binFrequency(i)) * refSpectrum[i];
                const float refPowerInstant = std::norm(adjustedRef);
                const float mainPowerInstant = std::norm(mainSpectrum[i]);
                if (!std::isfinite(refPowerInstant) ||
                    !std::isfinite(mainPowerInstant) ||
                    refPowerInstant <= adaptiveEpsilon) {
                    continue;
                }
                const std::complex<float> crossInstant = mainSpectrum[i] * std::conj(adjustedRef);
                const float alpha = hfNoiseCancelRefPower[i] <= adaptiveEpsilon
                                        ? adaptiveAlphaCold
                                        : adaptiveAlphaWarm;
                hfNoiseCancelCrossPower[i] += alpha * (crossInstant - hfNoiseCancelCrossPower[i]);
                hfNoiseCancelMainPower[i] += alpha * (mainPowerInstant - hfNoiseCancelMainPower[i]);
                hfNoiseCancelRefPower[i] += alpha * (refPowerInstant - hfNoiseCancelRefPower[i]);
                if (hfNoiseCancelRefPower[i] > adaptiveEpsilon) {
                    hfNoiseCancelBins[i] =
                        clampComplexMagnitude(hfNoiseCancelCrossPower[i] /
                                                  (hfNoiseCancelRefPower[i] + adaptiveEpsilon),
                                              HF_NOISE_CANCEL_MAX_COEFF);
                }
            }
        }

        auto complexMagnitudeDb = [samplesToCopy](std::complex<float> value) {
            const float magnitude = std::abs(value) / static_cast<float>((std::max)(1, samplesToCopy));
            if (std::isfinite(magnitude) && magnitude > 0.0f) {
                return (std::max)(FFT_MAGNITUDE_FLOOR_DB, 20.0f * std::log10(magnitude));
            }
            return FFT_MAGNITUDE_FLOOR_DB;
        };
        if (outReferenceMagnitudes) {
            outReferenceMagnitudes->assign(currentFftLength, FFT_MAGNITUDE_FLOOR_DB);
        }

        for (int i = 0; i < currentFftLength; ++i) {
            const std::complex<float> adjustedRef =
                hfNoiseCancelReferenceCoefficient(settings, binFrequency(i)) * refSpectrum[i];
            const float coherence =
                (std::norm(hfNoiseCancelCrossPower[i]) /
                 ((hfNoiseCancelMainPower[i] * hfNoiseCancelRefPower[i]) + adaptiveEpsilon));
            const float coherenceWeight =
                (std::clamp)((coherence - coherenceStart) / (coherenceFull - coherenceStart),
                             0.0f,
                             1.0f);
            const std::complex<float> effectiveRef =
                coherenceWeight * hfNoiseCancelBins[i] * adjustedRef;
            const std::complex<float> cleaned =
                mainSpectrum[i] - noiseCancelDepth * effectiveRef;
            outMagnitudes[i] = complexMagnitudeDb(cleaned);
            if (outReferenceMagnitudes) {
                (*outReferenceMagnitudes)[i] = complexMagnitudeDb(adjustedRef);
            }
            outFrequencies[i] =
                (i - currentFftLength / 2) * (sampleRate / currentFftLength) + centerFrequency;
        }
        return true;
    }

    if (!hfNoiseCancelBins.empty()) {
        hfNoiseCancelBins.clear();
        hfNoiseCancelCrossPower.clear();
        hfNoiseCancelMainPower.clear();
        hfNoiseCancelRefPower.clear();
    }
    for (int i = 0; i < samplesToCopy; ++i) {
        const int sourceIndex = sourceStart + i;
        if (inputMode == INPUT_HF1) {
            const float iValue = iqSnapshotScratch[2 * sourceIndex];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
            fftIn[i][1] = 0.0f;
        } else if (inputMode == INPUT_HF2) {
            const float qValue = iqSnapshotScratch[2 * sourceIndex + 1];
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = std::isfinite(qValue) ? qValue : 0.0f;
        } else {
            const float iValue = iqSnapshotScratch[2 * sourceIndex];
            const float qValue = iqSnapshotScratch[2 * sourceIndex + 1];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
            fftIn[i][1] = std::isfinite(qValue) ? qValue : 0.0f;
        }
    }
    for (int i = samplesToCopy; i < currentFftLength; ++i) {
        fftIn[i][0] = 0.0f;
        fftIn[i][1] = 0.0f;
    }

    fftwf_execute(plan);
    for (int i = 0; i < currentFftLength; ++i) {
        outMagnitudes[i] = magnitudeDb(i);
        outFrequencies[i] = (i - currentFftLength / 2) * (sampleRate / currentFftLength) + centerFrequency;
    }
    return true;
}

void FFTResult::storeFFTResults() {
    RadioSettings settings;
    settings.inputMode = globalMode;
    settings.centerFrequency = globalFrequency;
    settings.sampleRate = globalSampleRate;
    settings.fftLength = fftLength;
    storeFFTResults(settings, fftFrequencies, fftMagnitudes);
}
