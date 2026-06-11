#ifndef RADIOSETTINGS_H
#define RADIOSETTINGS_H

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>

enum class RadioRunState {
    Idle,
    Starting,
    Running,
    Stopping
};

enum class FobosApiKind {
    Standard = 0,
    Agile = 1
};

enum ModulationMode {
    MOD_AM = 0,
    MOD_NFM = 1,
    MOD_SAM = 2,
    MOD_USB = 3,
    MOD_LSB = 4,
    MOD_DSB = 5,
    MOD_CW = 6,
    MOD_WFM = 7,
    MOD_FT8 = 8,
    MOD_RTTY = 9,
    MOD_FSK = 10,
    MOD_PSK = 11,
    MOD_ATV = 12,
    MOD_SSTV = 13,
    MOD_APT = 14,
    MOD_WEFAX = 15,
    MOD_LRPT = 16,
    MOD_DMR = 17
};

enum InputMode {
    INPUT_RF = 0,
    INPUT_HF_COMBINED = 1,
    INPUT_HF1 = 2,
    INPUT_HF2 = 3,
    INPUT_HF_NOISE_CANCEL = 4
};

inline bool isDirectInputMode(int inputMode) {
    return inputMode != INPUT_RF;
}

inline bool isSinglePositiveHfInputMode(int inputMode) {
    return inputMode == INPUT_HF1 ||
           inputMode == INPUT_HF2 ||
           inputMode == INPUT_HF_NOISE_CANCEL;
}

inline bool isUpperSidebandMode(int modulationType) {
    return modulationType == MOD_USB ||
           modulationType == MOD_FT8 ||
           modulationType == MOD_RTTY ||
           modulationType == MOD_PSK ||
           modulationType == MOD_SSTV ||
           modulationType == MOD_WEFAX;
}

inline bool isLowerSidebandMode(int modulationType) {
    return modulationType == MOD_LSB;
}

inline bool isFrequencyDiscriminatorMode(int modulationType) {
    return modulationType == MOD_NFM ||
           modulationType == MOD_WFM ||
           modulationType == MOD_FSK ||
           modulationType == MOD_ATV ||
           modulationType == MOD_APT ||
           modulationType == MOD_DMR;
}

constexpr int DMR_DEFAULT_BASEBAND_SAMPLE_RATE = 192000;

inline int normalizedDmrBasebandSampleRate(int sampleRate) {
    switch (sampleRate) {
    case 24000:
    case 48000:
    case 96000:
    case 192000:
    case 384000:
        return sampleRate;
    default:
        return DMR_DEFAULT_BASEBAND_SAMPLE_RATE;
    }
}

enum DmrAmbeLayout {
    DMR_AMBE_LAYOUT_AUTO = 0,
    DMR_AMBE_LAYOUT_LINEAR72 = 1,
    DMR_AMBE_LAYOUT_SPLIT36 = 2,
    DMR_AMBE_LAYOUT_DIBIT_STRIPE = 3,
    DMR_AMBE_LAYOUT_BIT_STRIPE = 4
};

constexpr int DMR_DEFAULT_AMBE_LAYOUT = DMR_AMBE_LAYOUT_AUTO;

inline int normalizedDmrAmbeLayout(int layout) {
    switch (layout) {
    case DMR_AMBE_LAYOUT_AUTO:
    case DMR_AMBE_LAYOUT_LINEAR72:
    case DMR_AMBE_LAYOUT_SPLIT36:
    case DMR_AMBE_LAYOUT_DIBIT_STRIPE:
    case DMR_AMBE_LAYOUT_BIT_STRIPE:
        return layout;
    default:
        return DMR_DEFAULT_AMBE_LAYOUT;
    }
}

inline const char *dmrAmbeLayoutName(int layout) {
    switch (normalizedDmrAmbeLayout(layout)) {
    case DMR_AMBE_LAYOUT_LINEAR72:
        return "Linear72";
    case DMR_AMBE_LAYOUT_SPLIT36:
        return "Split36";
    case DMR_AMBE_LAYOUT_DIBIT_STRIPE:
        return "DibitStripe";
    case DMR_AMBE_LAYOUT_BIT_STRIPE:
        return "BitStripe";
    case DMR_AMBE_LAYOUT_AUTO:
    default:
        return "Auto";
    }
}

struct RadioSettings {
    int deviceIndex = 0;
    int clockSource = 0;
    int inputMode = 0;
    double centerFrequency = 100000000.0;
    double actualFrequency = 100000000.0;
    double listeningFrequency = 100000000.0;
    double sampleRate = 80000000.0;
    double bandwidth = 10000.0;
    int modulationType = 0;
    int fftLength = 32768;
    int lnaGain = 1;
    int vgaGain = 3;
    int audioDeviceId = 0;
    double audioLowPassHz = 0.0;
    double audioHighPassHz = 0.0;
    double hfNoiseCancelDepth = 1.0;
    double hfNoiseCancelRefGainDb = 0.0;
    double hfNoiseCancelRefDelayNs = 0.0;
    double hfNoiseCancelRefTiltDb = 0.0;
    bool hfNoiseCancelFreeze = false;
    bool audioEnabled = false;
    bool syncEnabled = false;
    std::uint8_t gpoValue = 0;
    bool dmrLabEnabled = false;
    int dmrLabColorCode = -1;
    int dmrLabTimeslot = 0;
    int dmrLabSourceId = 0;
    int dmrLabTargetId = 0;
    int dmrBasebandSampleRate = DMR_DEFAULT_BASEBAND_SAMPLE_RATE;
    bool dmrManualTimingEnabled = false;
    int dmrManualTimingOffset = 0;
    double dmrSlicerRatio = 0.625;
    bool dmrAdaptiveSlicer = true;
    int dmrAmbeLayout = DMR_DEFAULT_AMBE_LAYOUT;
};

inline double hfNoiseCancelFrequencyForShaping(const RadioSettings &settings, double frequencyHz) {
    const double maxFrequency = (std::max)(1.0, settings.sampleRate * 0.5);
    if (!std::isfinite(frequencyHz)) {
        frequencyHz = settings.listeningFrequency;
    }
    return (std::clamp)(std::abs(frequencyHz), 1.0, maxFrequency);
}

inline double hfNoiseCancelReferenceGainDbAt(const RadioSettings &settings, double frequencyHz) {
    const double maxFrequency = (std::max)(1.0, settings.sampleRate * 0.5);
    const double shapedFrequency = hfNoiseCancelFrequencyForShaping(settings, frequencyHz);
    const double normalized = (std::clamp)(shapedFrequency / maxFrequency, 0.0, 1.0);
    const double tiltDb = settings.hfNoiseCancelRefTiltDb * (normalized - 0.5);
    return settings.hfNoiseCancelRefGainDb + tiltDb;
}

inline std::complex<float> hfNoiseCancelReferenceCoefficient(const RadioSettings &settings,
                                                             double frequencyHz) {
    constexpr double twoPi = 6.28318530717958647692;
    const double shapedFrequency = hfNoiseCancelFrequencyForShaping(settings, frequencyHz);
    const double gainDb = hfNoiseCancelReferenceGainDbAt(settings, shapedFrequency);
    const double gain = std::pow(10.0, gainDb / 20.0);
    const double delaySeconds = settings.hfNoiseCancelRefDelayNs * 1.0e-9;
    const double phase = -twoPi * shapedFrequency * delaySeconds;
    return std::complex<float>(static_cast<float>(gain * std::cos(phase)),
                               static_cast<float>(gain * std::sin(phase)));
}

#endif // RADIOSETTINGS_H
