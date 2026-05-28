#ifndef RADIOSETTINGS_H
#define RADIOSETTINGS_H

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
    MOD_LRPT = 16
};

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
           modulationType == MOD_APT;
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
    bool audioEnabled = false;
    bool syncEnabled = false;
    std::uint8_t gpoValue = 0;
};

#endif // RADIOSETTINGS_H
