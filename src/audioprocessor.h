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

extern double listeningFrequency;
extern float* iqData;
extern double audioSamplerate;
extern double globalSampleRate;
extern double globalBandwidth;
extern int globalModulationType;
extern int DEFAULT_BUF_LEN;
extern int globalMode;
extern double globalFrequency;

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
public slots:
    void startDemodulation();
    void stopDemodulation();
private:
    void SDRThread();
    void AudioThread();
    void playAudio(const std::vector<short> &audioData);
    //void processAudioData(std::vector<short>& output);
    void processAudioData();
    //void resampleToAudioRate(std::vector<float>& demodulatedData, double sourceRate, double targetRate);
    void resampleToAudioRate(std::vector<float>& lowPassFilteredData, std::vector<float>& resampledData, double sourceRate, double targetRate);
    void filterIQData(float* iqData, double centerFrequency, double globalSampleRate, double listeningFrequency, double globalBandwidth, std::vector<float>& filteredData);
    void applyLowPassFilter(const std::vector<float>& demodulatedData, std::vector<float>& lowPassFilteredData, float cutoffFreq, float sampleRate);
    void applyDeemphasisFilter(std::vector<float>& lowPassFilteredData, float sampleRate);
    std::vector<float> demodulateAM(const std::vector<short>& filteredData);
	std::vector<float> demodulateFM(const std::vector<float>& filteredData, float& lastPhase);
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
