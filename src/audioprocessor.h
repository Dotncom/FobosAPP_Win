#ifndef AUDIOPROCESSOR_H
#define AUDIOPROCESSOR_H

#define NUM_BUFFERS 3
#define BUFFER_SIZE 4800  // 100 ms at 48 kHz

#include <QTimer>
#include <QObject>
#include <QMutex>
#include <QByteArray>
#include <QWaitCondition>
#include <QThread>
#include <QDebug>
#include <complex>
#include <vector>
#include <array>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <string>
#include <mutex>
#include <thread>
#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#endif
#include "radiosettings.h"

//#include <QAudioDeviceInfo>
//#include <QAudio>
//#include <QAudioOutput>
//#include <QUdpSocket>
//#include <QHostAddress>
//#include <udp_sender.h>


//struct WAVData {
//    std::vector<short> leftChannel;
//    std::vector<short> rightChannel;
//    int sampleRate;
//};

class AudioProcessor : public QObject {
    Q_OBJECT

public:
    explicit AudioProcessor(QObject *parent = nullptr);
    ~AudioProcessor();
    void configure(const RadioSettings &settings);
    void resetHfNoiseCancelState();
    void setAudioDevice(int deviceID);
    void setLocalPlaybackEnabled(bool enabled);
    void decimateIQ(    const std::vector<std::complex<float>>& inputIQ, std::vector<std::complex<float>>& outputIQ, int decimationFactor);
    void downsampleAudio(const std::vector<float>& input, std::vector<float>& output, int downsampleFactor);
    void demodulateAM(const std::vector<std::complex<float>>& input, std::vector<float>& output);
    void demodulateFM(const std::vector<float>& input, std::vector<float>& output, float& lastPhase);
    void startAudioOutput();
    void processAudioBuffer();
//  void setUdpSocket(QUdpSocket *socket);
public:
    void setVolume(float volume);


public slots:
    void startDemodulation();
    void stopDemodulation();

signals:
    void audioFrameReady(const QByteArray &pcmData);

private:
    void SDRThread();
    bool openAudioDevice(int deviceID);
    void closeAudioDevice();
    void pauseAudioDevice();
    void prepareWaveHeaders();
    void unprepareWaveHeaders();
    void joinWorkerThreads();
    void resetDemodulatorState();
    RadioSettings currentSettingsSnapshot() const;
    void processDemodulatorBlock(const std::vector<float>& iqBlock, std::vector<short>& audioSamples, const RadioSettings &settings);
    size_t queuedAudioSamples() const;
    void discardAudioSamples(size_t count);
    void compactAudioBufferIfNeeded();
	std::vector<float> demodulateSSB(const std::vector<float>& lowPassFilteredData, double frequency, double globalBandwidth, double sampleRate);
	std::vector<float> demodulateFSK(const std::vector<float>& lowPassFilteredData, double frequency, double globalBandwidth, double sampleRate);
    std::vector<short> waveBuffers[NUM_BUFFERS];
    std::vector<short> audioBuffer;
    size_t audioBufferReadOffset = 0;
#ifdef _WIN32
    WAVEHDR waveHdrs[NUM_BUFFERS];
#endif
    std::atomic<bool> bufferReady[NUM_BUFFERS];
    std::atomic<int> currentBufferIndex = 0;
    std::atomic<float> outputVolume = 1.0f;
    std::atomic<bool> hfNoiseCancelResetRequested = false;
    std::condition_variable cv;
    std::mutex audioMutex;
#ifdef _WIN32
    std::mutex waveOutMutex;
#endif
    mutable std::mutex settingsMutex;
    std::thread sdrWorker;
    std::thread audioWorker;
    QThread *workerThread = nullptr;
    QMutex mutex;
    QWaitCondition condition;
    QTimer *update1Timer;

#ifdef _WIN32
    WAVEFORMATEX format;
    HWAVEOUT hWaveOut = nullptr;
#endif
    bool waveHeadersPrepared = false;
    int openedAudioDeviceID = -1;
    int selectedAudioDeviceID = 0;
    std::atomic<bool> audioDeviceClosing = false;
    std::atomic<bool> localPlaybackEnabled = true;
    RadioSettings audioSettings;
    double ncoPhase = 0.0;
    double audioResamplePhase = 0.0;
    std::complex<float> amLowPassState = {0.0f, 0.0f};
    std::array<std::complex<float>, 4> sidebandLowPassStates = {};
    std::complex<float> channelDecimationSum = {0.0f, 0.0f};
    int channelDecimationCount = 0;
    std::complex<float> fmPreviousSample = {1.0f, 0.0f};
    bool fmPreviousValid = false;
    float fmDeemphasisState = 0.0f;
    float demodAudioLowPassState = 0.0f;
    float demodAudioLowPassState2 = 0.0f;
    float demodAudioLowPassState3 = 0.0f;
    float demodAudioHighPassState = 0.0f;
    double samCarrierPhase = 0.0;
    double samCarrierFrequency = 0.0;
    double sidebandFilterPhase = 0.0;
    double cwTonePhase = 0.0;
    float amDcEstimate = 0.0f;
    float amAgcLevel = 0.05f;
    std::complex<float> hfNoiseCancelCoeff = {0.0f, 0.0f};
    std::complex<float> hfNoiseCancelRefDecimationSum = {0.0f, 0.0f};

	QByteArray byteArray;
    QString audioDeviceName;

    std::atomic<bool> running = false;

    //QUdpSocket *udpSocket;
    //QString serverIp;
    //quint16 serverPort;
    //UdpSender *udpSender;


#ifdef _WIN32
    static void CALLBACK WaveOutCallback(HWAVEOUT hwo, UINT uMsg, DWORD_PTR dwInstance,
                                         DWORD_PTR dwParam1, DWORD_PTR dwParam2);
#endif

};

#endif // AUDIOPROCESSOR_H
