#include "dataprocessor.h"
#include "iqbuffer.h"
#include "diagnosticlogging.h"

#include <fobos_sdr.h>
#include <algorithm>
#include <limits>
#include <vector>
#include <Windows.h>

#define FLOATS_PER_IQ_SAMPLE 2

namespace {

constexpr double TWO_PI = 6.28318530717958647692;
constexpr bool FORCE_SYNC_READER = false;
constexpr double SYNC_BLOCKS_PER_SECOND = 20.0;
constexpr double ASYNC_BLOCKS_PER_SECOND = 400.0;
constexpr uint32_t MIN_SYNC_BLOCK_SAMPLES = 65536;
constexpr uint32_t MAX_SYNC_BLOCK_SAMPLES = 4000000;
constexpr uint32_t MIN_ASYNC_BLOCK_SAMPLES = 32768;
constexpr uint32_t MAX_ASYNC_BLOCK_SAMPLES = 262144;
constexpr int NETWORK_CHANNEL_FRAME_SAMPLES = 8192;
constexpr float NETWORK_IQ_QUANTIZE_GAIN = 1.0f;
constexpr float NETWORK_CHANNEL_IQ_TARGET_LEVEL = 0.45f;

double clampDouble(double value, double low, double high) {
    return (std::max)(low, (std::min)(value, high));
}

double networkChannelTargetRate(const RadioSettings &settings) {
    switch (settings.modulationType) {
    case MOD_WFM:
        return clampDouble((std::max)(384000.0, settings.bandwidth * 3.0), 384000.0, 768000.0);
    case MOD_NFM:
    case MOD_RTTY:
    case MOD_FSK:
        return clampDouble((std::max)(240000.0, settings.bandwidth * 4.0), 240000.0, 384000.0);
    case MOD_CW:
    case MOD_USB:
    case MOD_LSB:
    case MOD_FT8:
    case MOD_PSK:
    case MOD_AM:
    case MOD_SAM:
    case MOD_DSB:
    default:
        return 192000.0;
    }
}

double networkChannelCutoff(const RadioSettings &settings, double outputRate) {
    double requestedCutoff = settings.bandwidth * 0.6;
    switch (settings.modulationType) {
    case MOD_WFM:
        requestedCutoff = (std::max)(120000.0, settings.bandwidth * 0.6);
        break;
    case MOD_NFM:
    case MOD_RTTY:
    case MOD_FSK:
        requestedCutoff = (std::max)(12000.0, settings.bandwidth * 0.6);
        break;
    case MOD_USB:
    case MOD_LSB:
    case MOD_FT8:
    case MOD_PSK:
        requestedCutoff = (std::max)(3600.0, settings.bandwidth);
        break;
    case MOD_CW:
        requestedCutoff = (std::max)(1200.0, settings.bandwidth);
        break;
    case MOD_AM:
    case MOD_SAM:
    case MOD_DSB:
    default:
        requestedCutoff = (std::max)(10000.0, settings.bandwidth * 0.6);
        break;
    }
    return (std::min)(requestedCutoff, outputRate * 0.45);
}

char quantizeIqSample(float sample) {
    if (!std::isfinite(sample)) {
        sample = 0.0f;
    }
    const float scaled = (std::clamp)(sample * NETWORK_IQ_QUANTIZE_GAIN, -1.0f, 1.0f) * 127.0f;
    int value = scaled >= 0.0f
                    ? static_cast<int>(scaled + 0.5f)
                    : static_cast<int>(scaled - 0.5f);
    value = (std::clamp)(value, -128, 127);
    return static_cast<char>(value);
}

void appendInt16Le(QByteArray &buffer, float sample) {
    if (!std::isfinite(sample)) {
        sample = 0.0f;
    }
    const float clamped = (std::clamp)(sample, -1.0f, 1.0f);
    const auto value = static_cast<qint16>(std::lrint(clamped * 32767.0f));
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
}

uint32_t syncBlockSamplesForRate(double sampleRate) {
    if (sampleRate <= 0.0) {
        return 32768;
    }
    const double samplesPerBlock = sampleRate / SYNC_BLOCKS_PER_SECOND;
    return (std::clamp)(static_cast<uint32_t>(samplesPerBlock),
                        MIN_SYNC_BLOCK_SAMPLES,
                        MAX_SYNC_BLOCK_SAMPLES);
}

uint32_t asyncBlockSamplesForRate(double sampleRate) {
    if (sampleRate <= 0.0) {
        return 32768;
    }
    const double samplesPerBlock = sampleRate / ASYNC_BLOCKS_PER_SECOND;
    return (std::clamp)(static_cast<uint32_t>(samplesPerBlock),
                        MIN_ASYNC_BLOCK_SAMPLES,
                        MAX_ASYNC_BLOCK_SAMPLES);
}

uint32_t asyncBufferCountForRate(double sampleRate) {
    if (sampleRate >= 50000000.0) {
        return 16;
    }
    if (sampleRate >= 25000000.0) {
        return 24;
    }
    return 32;
}

int stopSyncSafely(fobos_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_stop_sync(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_stop_sync(dev);
#endif
}

int stopSyncAgileSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_stop_sync(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_stop_sync(dev);
#endif
}

int startSyncSafely(fobos_dev_t *dev, uint32_t blockSamples) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_start_sync(dev, blockSamples);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_start_sync(dev, blockSamples);
#endif
}

int startSyncAgileSafely(fobos_sdr_dev_t *dev, uint32_t blockSamples) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_start_sync(dev, blockSamples);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_start_sync(dev, blockSamples);
#endif
}

int readSyncSafely(fobos_dev_t *dev, float *buf, uint32_t *actual_buf_length) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_read_sync(dev, buf, actual_buf_length);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_read_sync(dev, buf, actual_buf_length);
#endif
}

int readSyncAgileSafely(fobos_sdr_dev_t *dev, float *buf, uint32_t *actual_buf_length) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_read_sync(dev, buf, actual_buf_length);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_read_sync(dev, buf, actual_buf_length);
#endif
}

int cancelAsyncSafely(fobos_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_cancel_async(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_cancel_async(dev);
#endif
}

int cancelAsyncAgileSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_cancel_async(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_cancel_async(dev);
#endif
}

} // namespace

DataProcessor::DataProcessor(QObject *parent)
    : QThread(parent),
      running(false),
      activeSyncMode(false),
      requestedSyncMode(false),
      requestedQueueAudioBlocks(false),
      requestedEmitIqFrames(false),
      requestedChannelizeIqFrames(false),
      networkIqResetRequested(false),
      requestedSampleRate(0.0),
      activeDevice(nullptr),
      activeApiKind(FobosApiKind::Standard),
      totalCallbackCounter(0) {
}

DataProcessor::~DataProcessor() {
    stop();
    wait();
}

void DataProcessor::startProcessing(void *device,
                                    FobosApiKind apiKind,
                                    bool syncEnabled,
                                    double sampleRate,
                                    bool queueAudioBlocks,
                                    bool emitIqFrames) {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] startProcessing enter"
                 << "device" << device
                 << "apiKind" << static_cast<int>(apiKind)
                 << "syncEnabled" << syncEnabled
                 << "sampleRate" << sampleRate
                 << "queueAudioBlocks" << queueAudioBlocks
                 << "emitIqFrames" << emitIqFrames
                 << "threadRunning" << QThread::isRunning()
                 << "runningFlag" << running.load();
    }
    if (QThread::isRunning()) {
        qDebug() << "Warning: DataProcessor is already running.";
        return;
    }
    if (!device) {
        qDebug() << "Cannot start DataProcessor without an active device.";
        return;
    }
    const bool useSyncReader = FORCE_SYNC_READER || syncEnabled;
    activeDevice = device;
    activeApiKind = apiKind;
    requestedSampleRate = sampleRate;
    requestedQueueAudioBlocks = queueAudioBlocks;
    requestedEmitIqFrames = emitIqFrames;
    networkIqResetRequested = true;
    requestedSyncMode = useSyncReader;
    activeSyncMode = useSyncReader;
    totalCallbackCounter = 0;
    if (FORCE_SYNC_READER) {
        qDebug() << "Using SDR++-style sync Fobos reader for sample rate:" << sampleRate;
    } else if (fobosVerboseLoggingEnabled()) {
        if (syncEnabled) {
            qDebug() << "Using sync Fobos reader for sample rate:" << sampleRate;
        } else {
            qDebug() << "Using async Fobos reader for sample rate:" << sampleRate;
        }
    }
    running = true;
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] QThread::start begin";
    }
    QThread::start();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] QThread::start end" << "threadRunning" << QThread::isRunning();
    }
}

void DataProcessor::run() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_ABOVE_NORMAL);
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] run enter";
    }
    void *readerDevice = activeDevice.load();
    const FobosApiKind readerApiKind = activeApiKind.load();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] run state"
                 << "readerDevice" << readerDevice
                 << "apiKind" << static_cast<int>(readerApiKind)
                 << "runningFlag" << running.load()
                 << "requestedSyncMode" << requestedSyncMode.load()
                 << "queueAudioBlocks" << requestedQueueAudioBlocks.load()
                 << "emitIqFrames" << requestedEmitIqFrames.load()
                 << "requestedSampleRate" << requestedSampleRate.load();
    }
    if (!running.load() || !readerDevice) {
        qDebug() << "Cannot start DataProcessor without an active device.";
        running = false;
        return;
    }
    IqBuffer::clear();
    const bool useSyncReader = FORCE_SYNC_READER || requestedSyncMode.load();
    const double sampleRate = requestedSampleRate.load();
    const uint32_t readBlockSamples = useSyncReader
                                          ? syncBlockSamplesForRate(sampleRate)
                                          : asyncBlockSamplesForRate(sampleRate);
    const uint32_t asyncBufferCount = asyncBufferCountForRate(sampleRate);
    activeSyncMode = useSyncReader;
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] selected reader"
                 << "useSyncReader" << useSyncReader
                 << "forceSync" << FORCE_SYNC_READER
                 << "buffers" << asyncBufferCount
                 << "readBlockSamples" << readBlockSamples;
    }

    if (!useSyncReader) {
        IqBuffer::setSampleRateEstimate(sampleRate);
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] fobos_rx_read_async begin"
                     << "device" << readerDevice
                     << "buffers" << asyncBufferCount
                     << "blockSamples" << readBlockSamples;
        }
        asyncMeasuredSamples = 0;
        asyncCallbackCounter = 0;
        asyncRateTimer.restart();
        int ret = FOBOS_ERR_NOT_OPEN;
        if (readerApiKind == FobosApiKind::Agile) {
            ret = fobos_sdr_read_async(static_cast<fobos_sdr_dev_t*>(readerDevice),
                                       [](float *buf, uint32_t buf_length, fobos_sdr_dev_t *, void *ctx) {
                                           auto *processor = static_cast<DataProcessor*>(ctx);
                                           processor->handleData(buf, buf_length);
                                       },
                                       this,
                                       asyncBufferCount,
                                       readBlockSamples);
        } else {
            ret = fobos_rx_read_async(static_cast<fobos_dev_t*>(readerDevice),
                                      [](float *buf, uint32_t buf_length, void *ctx) {
                                          auto *processor = static_cast<DataProcessor*>(ctx);
                                          processor->handleData(buf, buf_length);
                                      },
                                      this,
                                      asyncBufferCount,
                                      readBlockSamples);
        }
        const bool stoppedByRequest = !running.load();
        qDebug() << "[DataProcessor] fobos_rx_read_async end"
                 << "result" << ret
                 << "stoppedByRequest" << stoppedByRequest;
        if (ret != FOBOS_ERR_OK && running.load()) {
            qDebug() << "Failed to start async read, error code:" << ret;
        } else if (ret != FOBOS_ERR_OK && stoppedByRequest && fobosVerboseLoggingEnabled()) {
            qDebug() << "Async read stopped after cancel, result:" << ret;
        }
        running = false;
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "Async read finished.";
        }

    } else {
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] fobos_rx_start_sync begin"
                     << "device" << readerDevice
                     << "blockSamples" << readBlockSamples;
        }
        int ret = readerApiKind == FobosApiKind::Agile
                      ? startSyncAgileSafely(static_cast<fobos_sdr_dev_t*>(readerDevice), readBlockSamples)
                      : startSyncSafely(static_cast<fobos_dev_t*>(readerDevice), readBlockSamples);
        qDebug() << "[DataProcessor] fobos_rx_start_sync end" << "result" << ret;
        if (ret != FOBOS_ERR_OK) {
            qDebug() << "Failed to start sync mode, error code:" << ret;
            running = false;

        }
        std::vector<float> syncBuffer(static_cast<size_t>(readBlockSamples) * FLOATS_PER_IQ_SAMPLE);
        IqBuffer::setSampleRateEstimate(sampleRate);
        QElapsedTimer rateTimer;
        rateTimer.start();
        uint64_t measuredSamples = 0;
        uint64_t measuredReads = 0;
        qint64 accumulatedReadMs = 0;
        qint64 accumulatedPublishMs = 0;
        uint64_t readCounter = 0;
        while (running.load()) {
            uint32_t actual_buf_length = 0;
            const bool logRead = readCounter < 5 || (readCounter % 400) == 0;
            if (logRead && fobosVerboseLoggingEnabled()) {
                qDebug() << "[DataProcessor] fobos_rx_read_sync begin"
                         << "readCounter" << readCounter;
            }
            QElapsedTimer readTimer;
            readTimer.start();
            ret = readerApiKind == FobosApiKind::Agile
                      ? readSyncAgileSafely(static_cast<fobos_sdr_dev_t*>(readerDevice), syncBuffer.data(), &actual_buf_length)
                      : readSyncSafely(static_cast<fobos_dev_t*>(readerDevice), syncBuffer.data(), &actual_buf_length);
            accumulatedReadMs += readTimer.elapsed();
            if ((logRead && fobosVerboseLoggingEnabled()) || ret != FOBOS_ERR_OK) {
                qDebug() << "[DataProcessor] fobos_rx_read_sync end"
                         << "readCounter" << readCounter
                         << "result" << ret
                         << "actual_buf_length" << actual_buf_length;
            }
            if (ret != FOBOS_ERR_OK) {
                if (running.load()) {
                    qDebug() << "Failed to read sync data, error code:" << ret;
                } else {
                    qDebug() << "Sync read stopped after stop request, result:" << ret;
                }
                running = false;
                break;
            }
            ++readCounter;
            ++measuredReads;
            measuredSamples += actual_buf_length;
            const size_t floatCount = (std::min)(
                syncBuffer.size(),
                static_cast<size_t>(actual_buf_length) * FLOATS_PER_IQ_SAMPLE
                );
            QElapsedTimer publishTimer;
            publishTimer.start();
            IqBuffer::publish(syncBuffer.data(), floatCount, requestedQueueAudioBlocks.load());
            if (requestedEmitIqFrames.load()) {
                emitIqFrame(syncBuffer.data(), floatCount);
            }
            accumulatedPublishMs += publishTimer.elapsed();

            const qint64 elapsedMs = rateTimer.elapsed();
            if (elapsedMs >= 500) {
                const double measuredRate = static_cast<double>(measuredSamples) * 1000.0 /
                                            static_cast<double>((std::max)(qint64(1), elapsedMs));
                const double avgReadMs = measuredReads > 0
                                             ? static_cast<double>(accumulatedReadMs) / static_cast<double>(measuredReads)
                                             : 0.0;
                const double avgPublishMs = measuredReads > 0
                                                ? static_cast<double>(accumulatedPublishMs) / static_cast<double>(measuredReads)
                                                : 0.0;
                IqBuffer::setSampleRateEstimate(measuredRate);
                if (fobosVerboseLoggingEnabled()) {
                    qDebug() << "[DataProcessor] sync measured sample rate"
                             << "configured" << sampleRate
                             << "measured" << measuredRate
                             << "elapsedMs" << elapsedMs
                             << "readCounter" << readCounter
                             << "avgReadMs" << avgReadMs
                             << "avgPublishMs" << avgPublishMs;
                }
                measuredSamples = 0;
                measuredReads = 0;
                accumulatedReadMs = 0;
                accumulatedPublishMs = 0;
                rateTimer.restart();
            }
        }
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] sync read loop finished";
        }
    }
}

void DataProcessor::handleData(float *buf, uint32_t buf_length) {
    if (!running.load()) {
        return;
    }
    ++totalCallbackCounter;
    IqBuffer::publish(buf,
                      static_cast<size_t>(buf_length) * FLOATS_PER_IQ_SAMPLE,
                      requestedQueueAudioBlocks.load());
    if (requestedEmitIqFrames.load()) {
        emitIqFrame(buf, static_cast<size_t>(buf_length) * FLOATS_PER_IQ_SAMPLE);
    }
    ++asyncCallbackCounter;
    asyncMeasuredSamples += buf_length;
    const qint64 elapsedMs = asyncRateTimer.isValid() ? asyncRateTimer.elapsed() : 0;
    if (elapsedMs >= 500) {
        const double measuredRate = static_cast<double>(asyncMeasuredSamples) * 1000.0 /
                                    static_cast<double>((std::max)(qint64(1), elapsedMs));
        IqBuffer::setSampleRateEstimate(measuredRate);
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] async measured sample rate"
                     << "configured" << requestedSampleRate.load()
                     << "measured" << measuredRate
                     << "elapsedMs" << elapsedMs
                     << "callbackCounter" << asyncCallbackCounter
                     << "blockSamples" << buf_length;
        }
        asyncMeasuredSamples = 0;
        asyncCallbackCounter = 0;
        asyncRateTimer.restart();
    }
}

void DataProcessor::updateNetworkIqSettings(const RadioSettings &settings, bool channelizeFrames) {
    configureNetworkIqStreaming(settings, true, channelizeFrames);
}

void DataProcessor::configureNetworkIqStreaming(const RadioSettings &settings, bool emitFrames, bool channelizeFrames) {
    {
        std::lock_guard<std::mutex> lock(networkIqSettingsMutex);
        networkIqSettings = settings;
    }
    requestedEmitIqFrames = emitFrames;
    requestedChannelizeIqFrames = channelizeFrames;
    networkIqResetRequested = true;
}

void DataProcessor::resetNetworkIqState() {
    networkIqNcoPhase = 0.0;
    networkIqDecimationSum = std::complex<float>(0.0f, 0.0f);
    networkIqDecimationCount = 0;
    networkIqLowPassState = std::complex<float>(0.0f, 0.0f);
    networkIqAgcLevel = 0.01f;
    networkIqFrameBuffer.clear();
    networkIqFrameSampleRate = 0.0;
}

void DataProcessor::emitIqFrame(const float *samples, std::size_t floatCount) {
    if (!samples || floatCount == 0 || floatCount > static_cast<std::size_t>((std::numeric_limits<int>::max)())) {
        return;
    }

    if (networkIqResetRequested.exchange(false)) {
        resetNetworkIqState();
    }

    if (requestedChannelizeIqFrames.load()) {
        RadioSettings settings;
        {
            std::lock_guard<std::mutex> lock(networkIqSettingsMutex);
            settings = networkIqSettings;
        }
        emitChannelIqFrame(samples, floatCount, settings);
        return;
    }

    emitFullIqFrame(samples, floatCount);
}

void DataProcessor::emitFullIqFrame(const float *samples, std::size_t floatCount) {
    QByteArray quantized;
    quantized.resize(static_cast<int>(floatCount));
    char *dst = quantized.data();
    for (std::size_t i = 0; i < floatCount; ++i) {
        dst[i] = quantizeIqSample(samples[i]);
    }

    emit iqFrameReady(quantized,
                      requestedSampleRate.load(),
                      static_cast<int>(floatCount / FLOATS_PER_IQ_SAMPLE));
}

void DataProcessor::emitChannelIqFrame(const float *samples,
                                       std::size_t floatCount,
                                       const RadioSettings &settings) {
    const double inputRate = settings.sampleRate;
    if (inputRate <= 0.0 || floatCount < FLOATS_PER_IQ_SAMPLE) {
        return;
    }

    const std::size_t iqSamples = floatCount / FLOATS_PER_IQ_SAMPLE;
    const double targetRate = networkChannelTargetRate(settings);
    const int decimationFactor = (std::max)(1, static_cast<int>(std::floor(inputRate / targetRate)));
    const double outputRate = inputRate / static_cast<double>(decimationFactor);
    const double cutoff = networkChannelCutoff(settings, outputRate);
    const float lowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * cutoff / outputRate),
        0.000001,
        1.0
        ));

    if (std::abs(networkIqFrameSampleRate - outputRate) > 0.5) {
        networkIqFrameBuffer.clear();
        networkIqFrameSampleRate = outputRate;
    }

    const double fShift = settings.listeningFrequency - settings.centerFrequency;
    const double phaseIncrement = -TWO_PI * fShift / inputRate;
    float rotI = static_cast<float>(std::cos(networkIqNcoPhase));
    float rotQ = static_cast<float>(std::sin(networkIqNcoPhase));
    const float rotStepI = static_cast<float>(std::cos(phaseIncrement));
    const float rotStepQ = static_cast<float>(std::sin(phaseIncrement));

    std::complex<float> decimationSum = networkIqDecimationSum;
    int decimationCount = networkIqDecimationCount;
    std::complex<float> lowPass = networkIqLowPassState;

    for (std::size_t n = 0; n < iqSamples; ++n) {
        float iSample = samples[2 * n];
        float qSample = samples[2 * n + 1];
        if (!std::isfinite(iSample)) {
            iSample = 0.0f;
        }
        if (!std::isfinite(qSample)) {
            qSample = 0.0f;
        }
        if (settings.inputMode == 2) {
            qSample = 0.0f;
        } else if (settings.inputMode == 3) {
            iSample = qSample;
            qSample = 0.0f;
        }

        const float mixedI = iSample * rotI - qSample * rotQ;
        const float mixedQ = iSample * rotQ + qSample * rotI;
        decimationSum += std::complex<float>(mixedI, mixedQ);
        ++decimationCount;

        const float nextRotI = rotI * rotStepI - rotQ * rotStepQ;
        const float nextRotQ = rotI * rotStepQ + rotQ * rotStepI;
        rotI = nextRotI;
        rotQ = nextRotQ;
        if ((n & 4095) == 4095) {
            const float norm = std::sqrt(rotI * rotI + rotQ * rotQ);
            if (norm > 0.0f) {
                rotI /= norm;
                rotQ /= norm;
            }
        }

        if (decimationCount < decimationFactor) {
            continue;
        }

        const float invCount = 1.0f / static_cast<float>(decimationCount);
        std::complex<float> channelSample = decimationSum * invCount;
        decimationSum = std::complex<float>(0.0f, 0.0f);
        decimationCount = 0;

        lowPass += lowPassAlpha * (channelSample - lowPass);
        const float magnitude = std::abs(lowPass);
        const float agcCoeff = magnitude > networkIqAgcLevel ? 0.01f : 0.0002f;
        networkIqAgcLevel += agcCoeff * (magnitude - networkIqAgcLevel);
        networkIqAgcLevel = (std::max)(networkIqAgcLevel, 0.00001f);
        const float gain = NETWORK_CHANNEL_IQ_TARGET_LEVEL / networkIqAgcLevel;
        appendInt16Le(networkIqFrameBuffer, std::real(lowPass) * gain);
        appendInt16Le(networkIqFrameBuffer, std::imag(lowPass) * gain);

        if (networkIqFrameBuffer.size() >= NETWORK_CHANNEL_FRAME_SAMPLES * FLOATS_PER_IQ_SAMPLE * static_cast<int>(sizeof(qint16))) {
            emit iqFrameReady(networkIqFrameBuffer,
                              outputRate,
                              networkIqFrameBuffer.size() / (FLOATS_PER_IQ_SAMPLE * static_cast<int>(sizeof(qint16))));
            networkIqFrameBuffer.clear();
        }
    }

    networkIqDecimationSum = decimationSum;
    networkIqDecimationCount = decimationCount;
    networkIqLowPassState = lowPass;
    networkIqNcoPhase = std::remainder(networkIqNcoPhase + phaseIncrement * static_cast<double>(iqSamples), TWO_PI);
}

uint64_t DataProcessor::callbackCount() const {
    return totalCallbackCounter.load();
}


void DataProcessor::requestStop() {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] requestStop enter"
                 << "runningFlag" << running.load()
                 << "threadRunning" << QThread::isRunning()
                 << "activeSyncMode" << activeSyncMode.load()
                 << "activeDevice" << activeDevice.load();
    }
    const bool wasRunning = running.exchange(false);
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] running flag cleared" << "wasRunning" << wasRunning;
    }
    if (wasRunning || QThread::isRunning()) {
        void *readerDevice = activeDevice.load();
        if (!activeSyncMode.load()) {
            if (readerDevice) {
                if (fobosVerboseLoggingEnabled()) {
                    qDebug() << "[DataProcessor] fobos_rx_cancel_async begin" << readerDevice;
                }
                const int ret = activeApiKind.load() == FobosApiKind::Agile
                                    ? cancelAsyncAgileSafely(static_cast<fobos_sdr_dev_t*>(readerDevice))
                                    : cancelAsyncSafely(static_cast<fobos_dev_t*>(readerDevice));
                if (fobosVerboseLoggingEnabled() || ret != FOBOS_ERR_OK) {
                    qDebug() << "[DataProcessor] fobos_rx_cancel_async end" << "result" << ret;
                }
            }
        } else {
            if (readerDevice) {
                if (fobosVerboseLoggingEnabled()) {
                    qDebug() << "[DataProcessor] fobos_rx_stop_sync begin" << readerDevice;
                }
                const int ret = activeApiKind.load() == FobosApiKind::Agile
                                    ? stopSyncAgileSafely(static_cast<fobos_sdr_dev_t*>(readerDevice))
                                    : stopSyncSafely(static_cast<fobos_dev_t*>(readerDevice));
                if (fobosVerboseLoggingEnabled() || ret != FOBOS_ERR_OK) {
                    qDebug() << "[DataProcessor] fobos_rx_stop_sync end" << "result" << ret;
                }
            }
        }
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] QThread::quit begin";
        }
        QThread::quit();
    }
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] requestStop exit" << "threadRunning" << QThread::isRunning();
    }
}

void DataProcessor::finalizeStopped() {
    if (!QThread::isRunning()) {
        running = false;
        activeDevice = nullptr;
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DataProcessor] finalizeStopped: thread is stopped";
        }
    }
}

bool DataProcessor::forceStop(int timeoutMs) {
    qDebug() << "[DataProcessor] forceStop enter"
             << "timeoutMs" << timeoutMs
             << "threadRunning" << QThread::isRunning()
             << "activeDevice" << activeDevice.load();
    running = false;
    if (QThread::isRunning()) {
        qDebug() << "[DataProcessor] QThread::terminate begin";
        QThread::terminate();
        qDebug() << "[DataProcessor] QThread::wait after terminate begin" << "timeoutMs" << timeoutMs;
        if (!QThread::wait(timeoutMs)) {
            qDebug() << "[DataProcessor] QThread::terminate did not stop the thread within timeout.";
            return false;
        }
    }
    activeDevice = nullptr;
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] forceStop exit" << "threadRunning" << QThread::isRunning();
    }
    return !QThread::isRunning();
}

bool DataProcessor::stop(int timeoutMs) {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] stop enter" << "timeoutMs" << timeoutMs;
    }
    requestStop();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] QThread::wait begin" << "timeoutMs" << timeoutMs;
    }
    if (QThread::isRunning() && !QThread::wait(timeoutMs)) {
        qDebug() << "Error: DataProcessor thread did not quit within timeout.";
        return false;
    }
    finalizeStopped();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DataProcessor] stop exit" << "threadRunning" << QThread::isRunning();
    }
    return !QThread::isRunning();
}
