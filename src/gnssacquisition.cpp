#include "gnssacquisition.h"

#include <fftw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <limits>
#include <memory>

namespace {

constexpr double kGpsCaChipRate = 1023000.0;
constexpr double kAcquisitionSampleRate = 4092000.0;
constexpr int kChannelizerTaps = 33;
constexpr int kSamplesPerMs = 4092;
constexpr int kSamplesPerChip = 4;
constexpr int kPeakExclusionSamples = kSamplesPerChip * 2;
constexpr int kGpsCaChips = 1023;
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr int kTopCandidateCount = 8;
constexpr int kDopplerMinHz = -50000;
constexpr int kDopplerMaxHz = 50000;
constexpr int kDopplerStepHz = 1000;
constexpr int kDopplerMinStepHz = 250;
constexpr int kMaxCoherentMs = 160;

struct FftwPlanScope {
    explicit FftwPlanScope(fftwf_plan plan = nullptr)
        : plan(plan) {
    }

    ~FftwPlanScope() {
        if (plan) {
            fftwf_destroy_plan(plan);
        }
    }

    FftwPlanScope(const FftwPlanScope &) = delete;
    FftwPlanScope &operator=(const FftwPlanScope &) = delete;

    fftwf_plan plan = nullptr;
};

using FftwBuffer = std::unique_ptr<fftwf_complex, decltype(&fftwf_free)>;

FftwBuffer makeFftwBuffer(int count) {
    return FftwBuffer(static_cast<fftwf_complex *>(fftwf_malloc(sizeof(fftwf_complex) *
                                                               static_cast<std::size_t>(count))),
                      &fftwf_free);
}

const std::array<std::array<int, 2>, 32> &gpsCaG2Taps() {
    static const std::array<std::array<int, 2>, 32> taps = {{
        {{2, 6}}, {{3, 7}}, {{4, 8}}, {{5, 9}},
        {{1, 9}}, {{2, 10}}, {{1, 8}}, {{2, 9}},
        {{3, 10}}, {{2, 3}}, {{3, 4}}, {{5, 6}},
        {{6, 7}}, {{7, 8}}, {{8, 9}}, {{9, 10}},
        {{1, 4}}, {{2, 5}}, {{3, 6}}, {{4, 7}},
        {{5, 8}}, {{6, 9}}, {{1, 3}}, {{4, 6}},
        {{5, 7}}, {{6, 8}}, {{7, 9}}, {{8, 10}},
        {{1, 6}}, {{2, 7}}, {{3, 8}}, {{4, 9}}
    }};
    return taps;
}

std::vector<float> gpsCaChips(int prn) {
    std::vector<int> g1(10, 1);
    std::vector<int> g2(10, 1);
    std::vector<float> chips(kGpsCaChips);

    const auto taps = gpsCaG2Taps()[static_cast<std::size_t>(prn - 1)];
    for (int chip = 0; chip < kGpsCaChips; ++chip) {
        const int g1Out = g1[9];
        const int g2Out = g2[taps[0] - 1] ^ g2[taps[1] - 1];
        const int caBit = g1Out ^ g2Out;
        chips[static_cast<std::size_t>(chip)] = caBit ? -1.0f : 1.0f;

        const int g1Feedback = g1[2] ^ g1[9];
        const int g2Feedback = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9];
        for (int i = 9; i > 0; --i) {
            g1[static_cast<std::size_t>(i)] = g1[static_cast<std::size_t>(i - 1)];
            g2[static_cast<std::size_t>(i)] = g2[static_cast<std::size_t>(i - 1)];
        }
        g1[0] = g1Feedback;
        g2[0] = g2Feedback;
    }

    return chips;
}

std::vector<float> gpsCaCodeSamples(int prn) {
    const std::vector<float> chips = gpsCaChips(prn);
    std::vector<float> sampled(kSamplesPerMs);
    for (int i = 0; i < kSamplesPerMs; ++i) {
        const int chipIndex = (std::min)(kGpsCaChips - 1,
                                         static_cast<int>((static_cast<long long>(i) * kGpsCaChips) /
                                                          kSamplesPerMs));
        sampled[static_cast<std::size_t>(i)] = chips[static_cast<std::size_t>(chipIndex)];
    }
    return sampled;
}

double sinc(double x) {
    if (std::abs(x) < 1.0e-12) {
        return 1.0;
    }
    const double pix = kPi * x;
    return std::sin(pix) / pix;
}

bool resampleAndShift(const std::vector<float> &interleavedIq,
                      double inputSampleRate,
                      double frequencyOffset,
                      double channelFilterCutoffHz,
                      int coherentMs,
                      std::vector<std::complex<float>> *out,
                      int *usedInputSamples) {
    if (!out || coherentMs <= 0 || inputSampleRate <= 0.0 || !std::isfinite(inputSampleRate)) {
        return false;
    }

    const int inputSamples = static_cast<int>(interleavedIq.size() / 2);
    const int outputSamples = coherentMs * kSamplesPerMs;
    const double ratio = inputSampleRate / kAcquisitionSampleRate;
    if (inputSamples <= 0 || outputSamples <= 0 || ratio <= 0.0 || !std::isfinite(ratio)) {
        return false;
    }

    out->assign(static_cast<std::size_t>(outputSamples), std::complex<float>(0.0f, 0.0f));
    double meanRe = 0.0;
    double meanIm = 0.0;
    int lastInput = 0;
    const int halfTaps = kChannelizerTaps / 2;
    const double requestedCutoffHz = std::isfinite(channelFilterCutoffHz) && channelFilterCutoffHz > 0.0
                                         ? channelFilterCutoffHz
                                         : 1800000.0;
    const double cutoffHz = (std::min)(requestedCutoffHz,
                                       kAcquisitionSampleRate * 0.45);
    const double normalizedCutoff = (std::min)(0.49, cutoffHz / inputSampleRate);

    for (int outIndex = 0; outIndex < outputSamples; ++outIndex) {
        const double inputCenter = static_cast<double>(outIndex) * ratio;
        const int centerIndex = static_cast<int>(std::floor(inputCenter));
        double re = 0.0;
        double im = 0.0;
        double weightSum = 0.0;

        for (int tap = -halfTaps; tap <= halfTaps; ++tap) {
            const int inputIndex = centerIndex + tap;
            if (inputIndex < 0 || inputIndex >= inputSamples) {
                continue;
            }
            const double distance = static_cast<double>(inputIndex) - inputCenter;
            const double window = 0.54 + 0.46 *
                                  std::cos(kTwoPi * static_cast<double>(tap) /
                                           static_cast<double>(kChannelizerTaps - 1));
            const double weight = 2.0 * normalizedCutoff *
                                  sinc(2.0 * normalizedCutoff * distance) *
                                  window;
            const int offset = inputIndex * 2;
            const double iSample = static_cast<double>(interleavedIq[static_cast<std::size_t>(offset)]);
            const double qSample = static_cast<double>(interleavedIq[static_cast<std::size_t>(offset + 1)]);
            if (!std::isfinite(iSample) || !std::isfinite(qSample)) {
                continue;
            }

            const double phase = kTwoPi * frequencyOffset *
                                 (static_cast<double>(inputIndex) / inputSampleRate);
            const double c = std::cos(phase);
            const double s = std::sin(phase);
            re += (iSample * c + qSample * s) * weight;
            im += (qSample * c - iSample * s) * weight;
            weightSum += weight;
        }

        if (std::abs(weightSum) > std::numeric_limits<double>::epsilon()) {
            re /= weightSum;
            im /= weightSum;
        }
        (*out)[static_cast<std::size_t>(outIndex)] =
            std::complex<float>(static_cast<float>(re), static_cast<float>(im));
        meanRe += re;
        meanIm += im;
        lastInput = (std::max)(lastInput,
                               (std::min)(inputSamples,
                                          centerIndex + halfTaps + 1));
    }

    if (outputSamples > 0) {
        meanRe /= static_cast<double>(outputSamples);
        meanIm /= static_cast<double>(outputSamples);
        const std::complex<float> mean(static_cast<float>(meanRe), static_cast<float>(meanIm));
        for (std::complex<float> &sample : *out) {
            sample -= mean;
        }
    }

    for (int ms = 0; ms < coherentMs; ++ms) {
        const int chunkOffset = ms * kSamplesPerMs;
        double power = 0.0;
        for (int i = 0; i < kSamplesPerMs; ++i) {
            const std::complex<float> sample = (*out)[static_cast<std::size_t>(chunkOffset + i)];
            power += static_cast<double>(std::norm(sample));
        }
        const double rms = std::sqrt(power / static_cast<double>(kSamplesPerMs));
        if (std::isfinite(rms) && rms > std::numeric_limits<double>::epsilon()) {
            const float gain = static_cast<float>(1.0 / rms);
            for (int i = 0; i < kSamplesPerMs; ++i) {
                (*out)[static_cast<std::size_t>(chunkOffset + i)] *= gain;
            }
        }
    }

    if (usedInputSamples) {
        *usedInputSamples = lastInput;
    }
    return true;
}

void insertCandidate(QVector<GnssAcquisitionCandidate> *candidates,
                     const GnssAcquisitionCandidate &candidate) {
    if (!candidates || candidate.prn <= 0 || candidate.metric <= 0.0 ||
        !std::isfinite(candidate.metric)) {
        return;
    }

    candidates->append(candidate);
    std::sort(candidates->begin(), candidates->end(), [](const auto &a, const auto &b) {
        return a.metric > b.metric;
    });
    while (candidates->size() > kTopCandidateCount) {
        candidates->removeLast();
    }
}

bool isCancelRequested(const std::atomic_bool *cancelRequested) {
    return cancelRequested && cancelRequested->load(std::memory_order_relaxed);
}

void markCancelled(GnssAcquisitionResult *result) {
    if (!result) {
        return;
    }
    result->valid = false;
    result->cancelled = true;
    result->status = QStringLiteral("GPS C/A acquisition cancelled");
}

} // namespace

GnssAcquisitionResult GnssAcquisition::acquireGpsL1Ca(const std::vector<float> &interleavedIq,
                                                      double inputSampleRate,
                                                      double centerFrequency,
                                                      double targetFrequency,
                                                      int maxCoherentMs,
                                                      double channelFilterCutoffHz,
                                                      const std::atomic_bool *cancelRequested,
                                                      int dopplerMinHz,
                                                      int dopplerMaxHz,
                                                      int dopplerStepHz) {
    GnssAcquisitionResult result;
    result.inputSampleRate = inputSampleRate;
    result.acquisitionSampleRate = kAcquisitionSampleRate;
    result.centerFrequency = centerFrequency;
    result.targetFrequency = targetFrequency;
    result.frequencyOffset = targetFrequency - centerFrequency;
    result.channelFilterCutoffHz =
        (std::clamp)(std::isfinite(channelFilterCutoffHz) ? channelFilterCutoffHz : 1800000.0,
                     300000.0,
                     kAcquisitionSampleRate * 0.45);
    result.channelizerTaps = kChannelizerTaps;
    result.millisecondAgc = true;
    result.inputSamples = static_cast<int>(interleavedIq.size() / 2);

    if (isCancelRequested(cancelRequested)) {
        markCancelled(&result);
        return result;
    }

    if (result.inputSamples <= 0) {
        result.status = QStringLiteral("No IQ samples");
        return result;
    }
    if (inputSampleRate <= 0.0 || !std::isfinite(inputSampleRate)) {
        result.status = QStringLiteral("Invalid IQ sample rate");
        return result;
    }
    if (centerFrequency <= 0.0 || targetFrequency <= 0.0 ||
        !std::isfinite(centerFrequency) || !std::isfinite(targetFrequency)) {
        result.status = QStringLiteral("Invalid GNSS frequency");
        return result;
    }
    if (dopplerMinHz > dopplerMaxHz) {
        std::swap(dopplerMinHz, dopplerMaxHz);
    }
    dopplerMinHz = (std::clamp)(dopplerMinHz, kDopplerMinHz, kDopplerMaxHz);
    dopplerMaxHz = (std::clamp)(dopplerMaxHz, kDopplerMinHz, kDopplerMaxHz);
    dopplerStepHz = (std::clamp)(std::abs(dopplerStepHz), kDopplerMinStepHz, 10000);
    if (dopplerMinHz == dopplerMaxHz) {
        dopplerMaxHz = (std::min)(kDopplerMaxHz, dopplerMinHz + dopplerStepHz);
    }

    const double availableMs =
        (static_cast<double>(result.inputSamples) / inputSampleRate) * 1000.0;
    const int coherentMs = (std::clamp)(static_cast<int>(std::floor(availableMs)),
                                       1,
                                       (std::clamp)(maxCoherentMs, 1, kMaxCoherentMs));
    if (coherentMs <= 0) {
        result.status = QStringLiteral("Not enough IQ for one GPS C/A millisecond");
        return result;
    }

    std::vector<std::complex<float>> samples;
    if (!resampleAndShift(interleavedIq,
                          inputSampleRate,
                          result.frequencyOffset,
                          result.channelFilterCutoffHz,
                          coherentMs,
                          &samples,
                          &result.usedInputSamples)) {
        result.status = QStringLiteral("GNSS resample failed");
        return result;
    }
    result.coherentMs = coherentMs;
    if (isCancelRequested(cancelRequested)) {
        markCancelled(&result);
        return result;
    }

    auto dataIn = makeFftwBuffer(kSamplesPerMs);
    auto dataFft = makeFftwBuffer(kSamplesPerMs);
    auto codeIn = makeFftwBuffer(kSamplesPerMs);
    auto codeFft = makeFftwBuffer(kSamplesPerMs);
    auto product = makeFftwBuffer(kSamplesPerMs);
    auto corr = makeFftwBuffer(kSamplesPerMs);
    if (!dataIn || !dataFft || !codeIn || !codeFft || !product || !corr) {
        result.status = QStringLiteral("GNSS FFT allocation failed");
        return result;
    }

    FftwPlanScope dataPlan(fftwf_plan_dft_1d(kSamplesPerMs,
                                             dataIn.get(),
                                             dataFft.get(),
                                             FFTW_FORWARD,
                                             FFTW_ESTIMATE));
    FftwPlanScope codePlan(fftwf_plan_dft_1d(kSamplesPerMs,
                                             codeIn.get(),
                                             codeFft.get(),
                                             FFTW_FORWARD,
                                             FFTW_ESTIMATE));
    FftwPlanScope inversePlan(fftwf_plan_dft_1d(kSamplesPerMs,
                                                product.get(),
                                                corr.get(),
                                                FFTW_BACKWARD,
                                                FFTW_ESTIMATE));
    if (!dataPlan.plan || !codePlan.plan || !inversePlan.plan) {
        result.status = QStringLiteral("GNSS FFT plan failed");
        return result;
    }

    std::vector<double> dopplerBins;
    dopplerBins.reserve(static_cast<std::size_t>((dopplerMaxHz - dopplerMinHz) / dopplerStepHz + 1));
    for (int dopplerHz = dopplerMinHz; dopplerHz <= dopplerMaxHz; dopplerHz += dopplerStepHz) {
        dopplerBins.push_back(static_cast<double>(dopplerHz));
    }
    if (dopplerBins.empty() || dopplerBins.back() < static_cast<double>(dopplerMaxHz)) {
        dopplerBins.push_back(static_cast<double>(dopplerMaxHz));
    }
    result.dopplerBinsHz = QVector<double>(dopplerBins.begin(), dopplerBins.end());
    result.prnDopplerMetricDb.reserve(32 * static_cast<int>(dopplerBins.size()));
    std::vector<double> accumulatedPower(kSamplesPerMs, 0.0);
    double bestMetric = 0.0;
    std::vector<double> bestAccumulatedPower;

    for (int prn = 1; prn <= 32; ++prn) {
        if (isCancelRequested(cancelRequested)) {
            markCancelled(&result);
            return result;
        }
        const std::vector<float> code = gpsCaCodeSamples(prn);
        for (int i = 0; i < kSamplesPerMs; ++i) {
            codeIn.get()[i][0] = code[static_cast<std::size_t>(i)];
            codeIn.get()[i][1] = 0.0f;
        }
        fftwf_execute(codePlan.plan);

        for (double dopplerHz : dopplerBins) {
            if (isCancelRequested(cancelRequested)) {
                markCancelled(&result);
                return result;
            }
            std::fill(accumulatedPower.begin(), accumulatedPower.end(), 0.0);

            for (int ms = 0; ms < coherentMs; ++ms) {
                if (isCancelRequested(cancelRequested)) {
                    markCancelled(&result);
                    return result;
                }
                const int chunkOffset = ms * kSamplesPerMs;
                for (int i = 0; i < kSamplesPerMs; ++i) {
                    const std::complex<float> sample =
                        samples[static_cast<std::size_t>(chunkOffset + i)];
                    const double phase =
                        kTwoPi * dopplerHz *
                        (static_cast<double>(chunkOffset + i) / kAcquisitionSampleRate);
                    const double c = std::cos(phase);
                    const double s = std::sin(phase);
                    const double re = static_cast<double>(sample.real()) * c +
                                      static_cast<double>(sample.imag()) * s;
                    const double im = static_cast<double>(sample.imag()) * c -
                                      static_cast<double>(sample.real()) * s;
                    dataIn.get()[i][0] = static_cast<float>(re);
                    dataIn.get()[i][1] = static_cast<float>(im);
                }

                fftwf_execute(dataPlan.plan);
                for (int bin = 0; bin < kSamplesPerMs; ++bin) {
                    const float ar = dataFft.get()[bin][0];
                    const float ai = dataFft.get()[bin][1];
                    const float br = codeFft.get()[bin][0];
                    const float bi = codeFft.get()[bin][1];
                    product.get()[bin][0] = ar * br + ai * bi;
                    product.get()[bin][1] = ai * br - ar * bi;
                }
                fftwf_execute(inversePlan.plan);

                for (int phase = 0; phase < kSamplesPerMs; ++phase) {
                    const double re = corr.get()[phase][0];
                    const double im = corr.get()[phase][1];
                    accumulatedPower[static_cast<std::size_t>(phase)] += re * re + im * im;
                }
            }

            double sum = 0.0;
            double peak = 0.0;
            int peakPhase = 0;
            for (int phase = 0; phase < kSamplesPerMs; ++phase) {
                const double value = accumulatedPower[static_cast<std::size_t>(phase)];
                sum += value;
                if (value > peak) {
                    peak = value;
                    peakPhase = phase;
                }
            }
            const double average = (std::max)(std::numeric_limits<double>::min(),
                                              (sum - peak) /
                                                  static_cast<double>((std::max)(1, kSamplesPerMs - 1)));
            double secondPeak = 0.0;
            for (int phase = 0; phase < kSamplesPerMs; ++phase) {
                const int rawDistance = std::abs(phase - peakPhase);
                const int circularDistance = (std::min)(rawDistance, kSamplesPerMs - rawDistance);
                if (circularDistance <= kPeakExclusionSamples) {
                    continue;
                }
                secondPeak = (std::max)(secondPeak,
                                        accumulatedPower[static_cast<std::size_t>(phase)]);
            }
            secondPeak = (std::max)(secondPeak, std::numeric_limits<double>::min());
            GnssAcquisitionCandidate candidate;
            candidate.prn = prn;
            candidate.dopplerHz = dopplerHz;
            candidate.codePhaseSamples = peakPhase;
            candidate.metric = peak / average;
            candidate.peak = peak;
            candidate.secondPeak = secondPeak;
            candidate.peakToSecond = peak / secondPeak;
            candidate.average = average;
            const double metricDb =
                10.0 * std::log10((std::max)(candidate.metric,
                                             std::numeric_limits<double>::min()));
            result.prnDopplerMetricDb.push_back(metricDb);
            if (candidate.metric > bestMetric) {
                bestMetric = candidate.metric;
                bestAccumulatedPower = accumulatedPower;
            }
            insertCandidate(&result.topCandidates, candidate);
        }
    }

    if (!bestAccumulatedPower.empty()) {
        result.bestCorrelationProfileDb.reserve(static_cast<int>(bestAccumulatedPower.size()));
        double maxPower = 0.0;
        for (const double value : bestAccumulatedPower) {
            maxPower = (std::max)(maxPower, value);
        }
        maxPower = (std::max)(maxPower, std::numeric_limits<double>::min());
        for (const double value : bestAccumulatedPower) {
            const double normalized = (std::max)(value, std::numeric_limits<double>::min()) / maxPower;
            result.bestCorrelationProfileDb.push_back(static_cast<float>(10.0 * std::log10(normalized)));
        }
    }

    result.valid = !result.topCandidates.isEmpty();
    result.status = result.valid
                        ? QStringLiteral("GPS C/A acquisition finished")
                        : QStringLiteral("No GPS C/A candidates");
    return result;
}

std::vector<float> GnssAcquisition::makeSyntheticGpsL1CaIq(double inputSampleRate,
                                                           double centerFrequency,
                                                           double targetFrequency,
                                                           int prn,
                                                           double dopplerHz,
                                                           int durationMs,
                                                           double amplitude) {
    if (!std::isfinite(inputSampleRate) || inputSampleRate <= 0.0 ||
        !std::isfinite(centerFrequency) || centerFrequency <= 0.0 ||
        !std::isfinite(targetFrequency) || targetFrequency <= 0.0 ||
        !std::isfinite(dopplerHz) ||
        prn < 1 || prn > 32 ||
        durationMs <= 0) {
        return {};
    }

    amplitude = (std::clamp)(std::isfinite(amplitude) ? amplitude : 0.25, 0.001, 0.95);
    const int inputSamples =
        static_cast<int>(std::llround(inputSampleRate *
                                      (static_cast<double>(durationMs) / 1000.0)));
    if (inputSamples <= 0) {
        return {};
    }

    const std::vector<float> chips = gpsCaChips(prn);
    const double carrierHz = (targetFrequency - centerFrequency) + dopplerHz;
    constexpr double kSyntheticCodePhaseChips = 123.25;
    constexpr double kSyntheticInitialPhaseRad = 0.37;
    std::vector<float> iq;
    iq.reserve(static_cast<std::size_t>(inputSamples) * 2U);
    for (int n = 0; n < inputSamples; ++n) {
        const double t = static_cast<double>(n) / inputSampleRate;
        double chipPosition = std::fmod(t * kGpsCaChipRate + kSyntheticCodePhaseChips,
                                        static_cast<double>(kGpsCaChips));
        if (chipPosition < 0.0) {
            chipPosition += static_cast<double>(kGpsCaChips);
        }
        const int chipIndex =
            (std::clamp)(static_cast<int>(std::floor(chipPosition)), 0, kGpsCaChips - 1);
        const double code = static_cast<double>(chips[static_cast<std::size_t>(chipIndex)]);
        const double phase = kTwoPi * carrierHz * t + kSyntheticInitialPhaseRad;
        iq.push_back(static_cast<float>(amplitude * code * std::cos(phase)));
        iq.push_back(static_cast<float>(amplitude * code * std::sin(phase)));
    }
    return iq;
}
