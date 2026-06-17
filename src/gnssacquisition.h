#ifndef GNSSACQUISITION_H
#define GNSSACQUISITION_H

#include <QString>
#include <QVector>

#include <atomic>
#include <vector>

struct GnssAcquisitionCandidate {
    int prn = 0;
    double dopplerHz = 0.0;
    int codePhaseSamples = 0;
    double metric = 0.0;
    double peak = 0.0;
    double secondPeak = 0.0;
    double peakToSecond = 0.0;
    double average = 0.0;
};

struct GnssAcquisitionResult {
    bool valid = false;
    bool cancelled = false;
    QString status;
    double inputSampleRate = 0.0;
    double acquisitionSampleRate = 0.0;
    double centerFrequency = 0.0;
    double targetFrequency = 0.0;
    double frequencyOffset = 0.0;
    double channelFilterCutoffHz = 0.0;
    int channelizerTaps = 0;
    bool millisecondAgc = false;
    int toneNotchesApplied = 0;
    double strongestToneNotchDb = 0.0;
    int inputSamples = 0;
    int usedInputSamples = 0;
    int coherentMs = 0;
    qint64 processingElapsedMs = 0;
    QVector<GnssAcquisitionCandidate> topCandidates;
    QVector<double> dopplerBinsHz;
    QVector<double> prnDopplerMetricDb;
    QVector<float> bestCorrelationProfileDb;
};

class GnssAcquisition {
public:
    static GnssAcquisitionResult acquireGpsL1Ca(const std::vector<float> &interleavedIq,
                                               double inputSampleRate,
                                               double centerFrequency,
                                               double targetFrequency,
                                               int maxCoherentMs = 4,
                                               double channelFilterCutoffHz = 1800000.0,
                                               const std::atomic_bool *cancelRequested = nullptr,
                                               int dopplerMinHz = -50000,
                                               int dopplerMaxHz = 50000,
                                               int dopplerStepHz = 1000);
    static std::vector<float> makeSyntheticGpsL1CaIq(double inputSampleRate,
                                                     double centerFrequency,
                                                     double targetFrequency,
                                                     int prn,
                                                     double dopplerHz,
                                                     int durationMs,
                                                     double amplitude = 0.25);
};

#endif // GNSSACQUISITION_H
