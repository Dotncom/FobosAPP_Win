#include "spectrumfftworker.h"

#include "fft.h"

#include <exception>
#include <new>
#include <utility>

SpectrumFftWorker::SpectrumFftWorker()
    : workerThread(&SpectrumFftWorker::run, this) {
}

SpectrumFftWorker::~SpectrumFftWorker() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = true;
        hasRequest = false;
        hasResult = false;
        ++generation;
    }
    wakeCondition.notify_one();
    if (workerThread.joinable()) {
        workerThread.join();
    }
}

void SpectrumFftWorker::request(const RadioSettings &settings) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        pendingSettings = settings;
        pendingGeneration = generation;
        hasRequest = true;
        ++nextRequestId;
    }
    wakeCondition.notify_one();
}

bool SpectrumFftWorker::takeLatest(SpectrumFftFrame &frame) {
    std::lock_guard<std::mutex> lock(mutex);
    if (!hasResult) {
        return false;
    }
    frame = std::move(latestResult);
    hasResult = false;
    return true;
}

void SpectrumFftWorker::resetHfNoiseCancelState() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        hasRequest = false;
        hasResult = false;
        resetRequested = true;
        ++generation;
    }
    wakeCondition.notify_one();
}

void SpectrumFftWorker::run() {
    FFTResult fft;

    for (;;) {
        RadioSettings settings;
        quint64 requestId = 0;
        quint64 requestGeneration = 0;
        bool doReset = false;

        {
            std::unique_lock<std::mutex> lock(mutex);
            wakeCondition.wait(lock, [this]() {
                return stopping || hasRequest || resetRequested;
            });
            if (stopping) {
                break;
            }

            doReset = resetRequested;
            resetRequested = false;
            if (doReset) {
                fft.resetHfNoiseCancelState();
            }

            if (!hasRequest) {
                continue;
            }

            settings = pendingSettings;
            requestId = nextRequestId;
            requestGeneration = pendingGeneration;
            hasRequest = false;
        }

        SpectrumFftFrame frame;
        frame.requestId = requestId;
        frame.generation = requestGeneration;
        frame.settings = settings;

        try {
            frame.valid = fft.storeFFTResults(settings,
                                              frame.frequencies,
                                              frame.magnitudes,
                                              &frame.referenceMagnitudes,
                                              &frame.metadata);
        } catch (const std::bad_alloc &error) {
            frame.badAlloc = true;
            frame.error = QString::fromLatin1(error.what());
        } catch (const std::exception &error) {
            frame.error = QString::fromLatin1(error.what());
        } catch (...) {
            frame.error = QStringLiteral("unknown exception");
        }

        {
            std::lock_guard<std::mutex> lock(mutex);
            if (requestGeneration != generation || stopping) {
                continue;
            }
            latestResult = std::move(frame);
            hasResult = true;
        }
    }
}
