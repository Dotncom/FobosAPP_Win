#include "fft.h"
#include "iqbuffer.h"

#include <QDebug>
#include <cmath>

extern int globalMode;

extern int fftLength;
extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;
extern double minFrequency;
extern double maxFrequency;

namespace {
constexpr float FFT_MAGNITUDE_FLOOR_DB = -160.0f;
}

FFTResult::FFTResult(QObject *parent)
    : QObject(parent), fftIn(nullptr), fftOut(nullptr), plan(nullptr), planLength(0) {
    ensurePlan(fftLength);
}

FFTResult::~FFTResult() {
    releasePlan();
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
                                std::vector<float> &outMagnitudes) {
    const int currentFftLength = settings.fftLength;
    const double sampleRate = settings.sampleRate;
    const double centerFrequency = settings.centerFrequency;
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

    outMagnitudes.assign(currentFftLength, 0.0f);
    outFrequencies.resize(currentFftLength);
    std::vector<float> magnitude(currentFftLength);
    std::vector<float> iqSnapshot;

    for (int i = 0; i < currentFftLength; ++i) {
        outFrequencies[i] = (i - currentFftLength / 2) * (sampleRate / currentFftLength) + centerFrequency;
    }

    if (!IqBuffer::snapshot(iqSnapshot)) {
        return false;
    }

    const int availableIqSamples = static_cast<int>(iqSnapshot.size() / 2);
    if (availableIqSamples <= 0) {
        return false;
    }

    for (int i = 0; i < currentFftLength; ++i) {
        fftIn[i][0] = 0.0f;
        fftIn[i][1] = 0.0f;
    }

    const int samplesToCopy = std::min(currentFftLength, availableIqSamples);
    const int sourceStart = availableIqSamples - samplesToCopy;
    for (int i = 0; i < samplesToCopy; ++i) {
        const int sourceIndex = sourceStart + i;
        if (inputMode == 2) {
            const float iValue = iqSnapshot[2 * sourceIndex];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
            fftIn[i][1] = 0.0f;
        } else if (inputMode == 3) {
            const float qValue = iqSnapshot[2 * sourceIndex + 1];
            fftIn[i][0] = 0.0f;
            fftIn[i][1] = std::isfinite(qValue) ? qValue : 0.0f;
        } else {
            const float iValue = iqSnapshot[2 * sourceIndex];
            const float qValue = iqSnapshot[2 * sourceIndex + 1];
            fftIn[i][0] = std::isfinite(iValue) ? iValue : 0.0f;
            fftIn[i][1] = std::isfinite(qValue) ? qValue : 0.0f;
        }
    }

    fftwf_execute(plan);
    const float fftNormalization = 1.0f / static_cast<float>((std::max)(1, samplesToCopy));
    for (int i = 0; i < currentFftLength; ++i) {
        const float re = std::isfinite(fftOut[i][0]) ? fftOut[i][0] : 0.0f;
        const float im = std::isfinite(fftOut[i][1]) ? fftOut[i][1] : 0.0f;
        magnitude[i] = std::sqrt(re * re + im * im) * fftNormalization;
        if (std::isfinite(magnitude[i]) && magnitude[i] > 0.0f) {
            outMagnitudes[i] = (std::max)(FFT_MAGNITUDE_FLOOR_DB, 20.0f * std::log10(magnitude[i]));
        } else {
            outMagnitudes[i] = FFT_MAGNITUDE_FLOOR_DB;
        }
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
