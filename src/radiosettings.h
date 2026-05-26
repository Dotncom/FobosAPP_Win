#ifndef RADIOSETTINGS_H
#define RADIOSETTINGS_H

#include <cstdint>

enum class RadioRunState {
    Idle,
    Starting,
    Running,
    Stopping
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
    MOD_PSK = 11
};

inline bool isUpperSidebandMode(int modulationType) {
    return modulationType == MOD_USB ||
           modulationType == MOD_FT8 ||
           modulationType == MOD_PSK;
}

inline bool isLowerSidebandMode(int modulationType) {
    return modulationType == MOD_LSB;
}

inline bool isFrequencyDiscriminatorMode(int modulationType) {
    return modulationType == MOD_NFM ||
           modulationType == MOD_WFM ||
           modulationType == MOD_RTTY ||
           modulationType == MOD_FSK;
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
    bool audioEnabled = false;
    bool syncEnabled = false;
    std::uint8_t gpoValue = 0;
};

#endif // RADIOSETTINGS_H
