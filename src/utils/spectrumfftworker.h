#ifndef SPECTRUMFFTWORKER_H
#define SPECTRUMFFTWORKER_H

#include "iqbuffer.h"
#include "radiosettings.h"

#include <QString>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

struct SpectrumFftFrame {
    quint64 requestId = 0;
    quint64 generation = 0;
    bool valid = false;
    bool badAlloc = false;
    QString error;
    RadioSettings settings;
    std::vector<float> frequencies;
    std::vector<float> magnitudes;
    std::vector<float> referenceMagnitudes;
    IqBuffer::BlockMetadata metadata;
};

class SpectrumFftWorker {
public:
    SpectrumFftWorker();
    ~SpectrumFftWorker();

    SpectrumFftWorker(const SpectrumFftWorker &) = delete;
    SpectrumFftWorker &operator=(const SpectrumFftWorker &) = delete;

    void request(const RadioSettings &settings);
    bool takeLatest(SpectrumFftFrame &frame);
    void resetHfNoiseCancelState();

private:
    void run();

    std::thread workerThread;
    mutable std::mutex mutex;
    std::condition_variable wakeCondition;
    bool stopping = false;
    bool hasRequest = false;
    bool hasResult = false;
    bool resetRequested = false;
    quint64 nextRequestId = 0;
    quint64 generation = 0;
    quint64 pendingGeneration = 0;
    RadioSettings pendingSettings;
    SpectrumFftFrame latestResult;
};

#endif // SPECTRUMFFTWORKER_H
