#ifndef IQCHANNELIZER_H
#define IQCHANNELIZER_H

#include "radiosettings.h"

#include <array>
#include <complex>
#include <cstddef>
#include <vector>

class IqChannelizer {
public:
    struct Result {
        bool valid = false;
        double inputRate = 0.0;
        double outputRate = 0.0;
        double centerFrequency = 0.0;
        double listeningFrequency = 0.0;
        double frequencyShift = 0.0;
        double cutoff = 0.0;
        int decimationFactor = 1;
        int inputSamples = 0;
        int outputSamples = 0;
        float agcLevel = 0.0f;
    };

    void reset();
    Result processFloatIq(const float *samples,
                          std::size_t floatCount,
                          const RadioSettings &settings,
                          std::vector<float> &output);

private:
    double ncoPhase = 0.0;
    std::complex<float> decimationSum = {0.0f, 0.0f};
    std::complex<float> lowPassState = {0.0f, 0.0f};
    std::array<std::complex<float>, 3> preLowPassStates = {};
    int decimationCount = 0;
    float agcLevel = 0.01f;
};

#endif // IQCHANNELIZER_H
