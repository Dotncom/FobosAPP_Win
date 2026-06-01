#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#include <fobos.h>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <cmath>
#include <QDebug>
#include <cstring>
#include <QVector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <QElapsedTimer>
#include <QByteArray>
#include <atomic>
#include <complex>
#include "radiosettings.h"
//#include <main.h>


class DataProcessor : public QThread {
    Q_OBJECT
public:
    explicit DataProcessor( QObject *parent = nullptr);
    ~DataProcessor();
    void handleData(float *buf, uint32_t buf_length);
    //void startit();
    //void runit();
    void run() override;
    //void stopit();
    void startProcessing(void *device,
                         FobosApiKind apiKind,
                         bool syncEnabled,
                         double sampleRate,
                         bool queueAudioBlocks,
                         bool emitIqFrames = false,
                         bool agileScanEnabled = false);
    void requestStop();
    void finalizeStopped();
    bool forceStop(int timeoutMs = 1000);
    bool stop(int timeoutMs = 5000);
    uint64_t callbackCount() const;
    void updateNetworkIqSettings(const RadioSettings &settings, bool channelizeFrames);
    void configureNetworkIqStreaming(const RadioSettings &settings, bool emitFrames, bool channelizeFrames);
signals:
    void iqFrameReady(const QByteArray &iqData, double sampleRate, int sampleCount);
    void readerFailed(int errorCode, bool stoppedByRequest);

private:
    void emitIqFrame(const float *samples, std::size_t floatCount);
    void emitFullIqFrame(const float *samples, std::size_t floatCount);
    void emitChannelIqFrame(const float *samples, std::size_t floatCount, const RadioSettings &settings);
    void resetNetworkIqState();

//signals:
    //void dataReady();
    //std::mutex dataMutex;
    std::atomic<bool> running;
    std::atomic<bool> activeSyncMode;
    std::atomic<bool> requestedSyncMode;
    std::atomic<bool> requestedQueueAudioBlocks;
    std::atomic<bool> requestedEmitIqFrames;
    std::atomic<bool> requestedChannelizeIqFrames;
    std::atomic<bool> requestedAgileScanEnabled;
    std::atomic<bool> networkIqResetRequested;
    std::atomic<double> requestedSampleRate;
    std::atomic<void*> activeDevice;
    std::atomic<FobosApiKind> activeApiKind;
    std::mutex networkIqSettingsMutex;
    RadioSettings networkIqSettings;
    double networkIqNcoPhase = 0.0;
    std::complex<float> networkIqDecimationSum = {0.0f, 0.0f};
    int networkIqDecimationCount = 0;
    std::complex<float> networkIqLowPassState = {0.0f, 0.0f};
    float networkIqAgcLevel = 0.01f;
    std::complex<float> networkHfNoiseCancelCoeff = {0.0f, 0.0f};
    std::complex<float> networkHfNoiseCancelRefDecimationSum = {0.0f, 0.0f};
    QByteArray networkIqFrameBuffer;
    double networkIqFrameSampleRate = 0.0;
    QElapsedTimer asyncRateTimer;
    uint64_t asyncMeasuredSamples = 0;
    uint64_t asyncCallbackCounter = 0;
    std::atomic<uint64_t> totalCallbackCounter;
//int reti;
};

#endif // DATAPROCESSOR_H
