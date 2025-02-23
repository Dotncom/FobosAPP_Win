#ifndef AUDIOPROCESSOR_H
#define AUDIOPROCESSOR_H

//#include <main.h>
#include <QTimer>
#include <QObject>
#include <QMutex>
#include <mutex>
#include <QWaitCondition>
#include <QThread>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
//#include <QAudioDeviceInfo>
//#include <QAudio>
//#include <QAudioOutput>
//#include <QUdpSocket>
#include <QByteArray>
//#include <QHostAddress>
//#include <udp_sender.h>
#include <cmath>
#include <chrono>
#include <QDebug>
//#include "decim/plans.h"
#include <windows.h>
#include <mmsystem.h>
#include <algorithm>
#include <condition_variable>  // Добавляем для синхронизации
#include <string>

extern double listeningFrequency;
extern float* iqData;
extern double audioSamplerate;
extern double globalSampleRate;
extern double globalBandwidth;
extern int globalModulationType;
extern int DEFAULT_BUF_LEN;
extern int globalMode;
extern double globalFrequency;

struct WAVData {
    std::vector<short> leftChannel;
    std::vector<short> rightChannel;
    int sampleRate;
};

class AudioProcessor : public QObject {
    Q_OBJECT

public:
    explicit AudioProcessor(QObject *parent = nullptr);
    ~AudioProcessor();
    void setAudioDevice(int deviceID);
//  void setUdpSocket(QUdpSocket *socket);
    WAVEFORMATEX format;
    HWAVEOUT hWaveOut; // assuming you're working with Windows audio API
    //AudioProcessor() : hWaveOut(nullptr) {} // Initialize it correctly

        bool loadWAV(const std::string& filename, int& sampleRate, int& numChannels);


public slots:
    void startDemodulation();
    void stopDemodulation();
private:
    void SDRThread();
    void AudioThread();
    void playAudio(const std::vector<short>& leftChannel, const std::vector<short>& rightChannel);
    void demodulateAM(const std::vector<float>& testIQ_AM, std::vector<float>& demodulatedData);
    void filterIQData(float* iqData, double centerFrequency, double globalSampleRate, double listeningFrequency, double globalBandwidth, std::vector<float>& filteredData);
    void applyLowPassFilter(const std::vector<float>& demodulatedData, std::vector<float>& lowPassFilteredData, float cutoffFreq, float sampleRate);
    void applyDeemphasisFilter(std::vector<float>& lowPassFilteredData, float sampleRate);
    void normalizeAudio(std::vector<float>& audioData);
    void resampleAudio(const std::vector<float>& input, std::vector<short>& output);
    std::vector<short> loadWAV(const std::string& filename, int& sampleRate);
    std::vector<float> demodulateFM(const std::vector<float>& testIQ_FM, float& lastPhase);
	std::vector<float> demodulateSSB(const std::vector<float>& lowPassFilteredData, double frequency, double globalBandwidth, double sampleRate);
	std::vector<float> demodulateFSK(const std::vector<float>& lowPassFilteredData, double frequency, double globalBandwidth, double sampleRate);
	//QUdpSocket *udpSocket;  
    //QString serverIp;       
    //quint16 serverPort;    
	//UdpSender *udpSender;
	QByteArray byteArray;
    //void initializeUdpSocket();
    //void sendAudioData(const QByteArray &data);
	//void handleSocketError(QAbstractSocket::SocketError socketError);	
    QString audioDeviceName;
    QThread *workerThread = nullptr;
    bool running;
    QMutex mutex;
    QWaitCondition condition;
    QTimer *update1Timer;
};

#endif // AUDIOPROCESSOR_H
