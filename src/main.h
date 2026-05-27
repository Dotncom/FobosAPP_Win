#ifndef MAIN_H
#define MAIN_H

#include <Windows.h>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QTimer>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QLabel>
#include <memory>
#include <QScrollArea>
#include <QCheckBox>
#include <QMainWindow>
//#include <QAudioOutput>
//#include <QAudioDeviceInfo>
//#include <QAudio>
#include <QWheelEvent>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <QCloseEvent>
#include <QMessageBox>
#include <QByteArray>
#include <QJsonObject>
#include <QVector>
#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <QRadioButton>
#include <QButtonGroup>
#include <QElapsedTimer>
#include <windows.h>
#include <mmsystem.h>
#include <cstdint>
#include "fft.h"
#include "dataprocessor.h"
#include "audioprocessor.h"
#include "networkcontroller.h"
#include "remoteaudioplayer.h"
#include "radiosettings.h"
#include "scalewidget.h"
#include "MyGraphWidget.h"
#include "MyWaterfallWidget.h"
//#include <fobos.h>
#include <fftw3.h>
//#include <libusb.h>

extern fobos_dev_t *device;
extern float* dataq;
extern float* iqData;
extern double globalFrequency; 
extern double actualFrequency; 
extern double listeningFrequency;
extern double globalSampleRate;
extern double globalBandwidth;
extern int globalModulationType;
extern int globalMode;
extern int fftLength;
extern double currentScale;
extern double minFrequency;
extern double maxFrequency;
extern float sensitivity;
extern float contrast;
class FFTResult;
extern int deviceID;

class YourClassName : public QMainWindow {
    Q_OBJECT

public:
   explicit YourClassName(QWidget *parent = nullptr);
    ~YourClassName();
     std::unique_ptr<FFTResult> fftResult;
    void onFrequencyEntered();
    void onListeningFrequencyEntered();
    void onScaleChanged(int value);
    void updateSpectrum();
    void onLnaGainChanged(int value);
    void onVgaGainChanged(int value);
    void onSensitivityChanged(int value);
    void onContrastChanged(int value);
    void onLevelMinChanged(int value);
    void onLevelMaxChanged(int value);
    void populateSampleRates();
    void populateAudioDevices();
private slots:
	void settingRange();
    void onDirectSamplingChanged(int index);
    void listFobosDevices();
    void startFobosProcessing();
    void stopFobosProcessing();
    void onSampleRateChanged(int index);
    void onfftLengthEntered();
    void onClkChanged(int index);
    void onCheckboxStateChanged(int state);
    void onAudioDeviceChanged(int index);
    void syncEnable(bool checked);
	void onModulationChanged(int id);
    void onBandwidthChanged();
    void updateFrequency();
    void updateCentralFrequency();
    void updateTuningFromScale(double tunedListeningFrequency, double tunedCenterFrequency);
    void doubleGraphEnable(bool checked);
    void colorGraphEnable(bool checked);
    void pollStopCompletion();
    void checkStreamStartup();
    void openNetworkSettingsDialog();
    void onNetworkStatusChanged(const QString &status);
    void onNetworkControlCommandReceived(const QJsonObject &command);
    void onAudioEnabledChanged(bool checked);
protected:
	void onWaterfallScaleChanged(int delta);
    void wheelEvent(QWheelEvent *event) override;
        //void closeEvent(QCloseEvent *event) override {
        //int reply = QMessageBox::question(this, "Acception", 
        //                                  "Close this program?",
        //                                  QMessageBox::Yes | QMessageBox::No);
        //if (reply == QMessageBox::Yes) { 
        //    event->accept();
        //} else {
        //    event->ignore();
        //}
		//};

private:
    QWidget *centralWidget;
    QStringList getFobosDevices();
    bool restartStreamForHardwareChange();
    bool openFobosSession();
    bool closeFobosSession(bool clearIq = true);
    bool applyFobosSettings();
    void refreshSettingsFromUi();
    void publishSettingsToGlobals();
    bool isIdle() const;
    bool isRunningOrTransitioning() const;
    void updateUiForRunState();
    void updateSpectrumTimerInterval();
    void revertHardwareControlsToSettings();
    void finishFobosStop(bool forcedRecovery);
    void recreateDataProcessor();
    uint8_t currentGpoValue() const;
    bool isNetworkClientMode() const;
    bool isClientIqProcessingMode() const;
    bool isChannelIqProcessingMode() const;
    bool isFullIqProcessingMode() const;
    void appendNetworkState(QJsonObject &command) const;
    void applyNetworkStateFromCommand(const QJsonObject &command);
    bool sendRemoteControlCommand(const QString &action);
    QJsonObject settingsToJson() const;
    void applySettingsFromJson(const QJsonObject &settingsJson);
    void updateUiFromPendingSettings();
    void applyLiveRemoteSettings(const RadioSettings &previousSettings);
    void connectDataProcessorSignals();
    void startNetworkClientProcessing();
    void stopNetworkClientProcessing();
    void sendNetworkSpectrumFrame(const std::vector<float> &frequencies, const std::vector<float> &magnitudes);
    void displayNetworkSpectrumFrame(const QJsonObject &frame);
    void sendNetworkAudioFrame(const QByteArray &pcmData);
    void playNetworkAudioFrame(const QJsonObject &frame);
    void sendNetworkIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);
    void receiveNetworkIqFrame(const QJsonObject &frame);
    
    QComboBox *clkBox;
    QComboBox *comboBox;
    QComboBox *modeBox;
    QComboBox *sampleBox;
    QComboBox *fftComboBox;
    QComboBox *audioDeviceComboBox;
    QButtonGroup *modulationButtonGroup;
    
    QPushButton *refreshButton;
    QPushButton *fobosButton;
    QPushButton *networkButton;
    QPushButton *startButton;
    QPushButton *stopButton;
    
    QCheckBox *spectrumCheckbox;
    QCheckBox *audioCheckbox;
    QCheckBox *syncCheckbox;
    QCheckBox *graphCheckbox;
    QCheckBox *colorCheckbox;
    QCheckBox *checkBoxes[8]; 
    
    QSlider *scaleSlider;
    QSlider *lnaGainSlider;
    QSlider *vgaGainSlider;
    QSlider *contrastSlider;
    QSlider *sensitivitySlider;
    QSlider *levelMinSlider;
    QSlider *levelMaxSlider;
    QSlider *volumeSlider;

    QLabel *volumeLabel;
    QLabel *lnaGainLabel;
    QLabel *centralFrequencyLabel;
    QLabel *listeningFrequencyLabel;
    QLabel *fftLabel;
    QLabel *contrastLabel;
    QLabel *sensitivityLabel;
    QLabel *levelMinLabel;
    QLabel *levelMaxLabel;
    QLabel *vgaGainLabel;
    QLabel *scaleLabel;
       
    QLineEdit *frequencyLineEdit;
    QLineEdit *listeningFrequencyLineEdit;
    QLineEdit *bandwidthLineEdit;
    
    QTimer *updateTimer;
    QTimer *stopPollTimer;
    QTimer *streamWatchdogTimer;

    DataProcessor *processor;
    AudioProcessor *audioProcessor;
    NetworkController *networkController;
    RemoteAudioPlayer *remoteAudioPlayer;
    MyGraphWidget *graphWidget;
    MyWaterfallWidget *waterfallWidget;
    ScaleWidget *scaleWidget;
    
    bool deviceOpened;
    int openedDeviceIndex = -1;
    double appliedSampleRate = 0.0;
    QElapsedTimer stopElapsedTimer;
    QElapsedTimer streamStartElapsedTimer;
    int stopCancelRetryCount = 0;
    int streamStartupRetryCount = 0;
    uint64_t streamStartCallbackCount = 0;
    bool restartAfterStartupWatchdog = false;
    bool automaticStreamRestart = false;
    bool pendingAudioStartAfterStreamReady = false;
    bool pendingNetworkAudioStartAfterIqPrebuffer = false;
    bool sampleRateReopenRequired = false;
    bool fobosCloseKnownUnsafe = false;
    RadioSettings pendingSettings;
    RadioSettings appliedHardwareSettings;
    bool hardwareSettingsApplied = false;
    int spectrumDebugFramesRemaining = 0;
    RadioRunState runState = RadioRunState::Idle;
    NetworkMode networkMode = NetworkMode::Disabled;
    NetworkProcessingMode networkProcessingMode = NetworkProcessingMode::ServerSide;
    QString networkServerAddress = "127.0.0.1";
    QString networkBindAddress = "0.0.0.0";
    quint16 networkControlPort = 21090;
    bool serverDisableLocalVisualAudio = true;
    QElapsedTimer networkSpectrumFrameTimer;
    uint64_t networkSpectrumFrameSequence = 0;
    uint64_t networkIqFrameSequence = 0;
    uint64_t networkIqFramesDropped = 0;
    float displayLevelMin = -120.0f;
    float displayLevelMax = 0.0f;
    int minScale = 1;
    int maxScale = 1000;
};

#endif // MAIN_H
