#ifndef MAIN_H
#define MAIN_H

#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QTimer>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QSlider>
#include <QDial>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QMap>
#include <memory>
#include <QScrollArea>
#include <QCheckBox>
#include <QMainWindow>
#include <QDockWidget>
#include <QPlainTextEdit>
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
#include <QList>
#include <QVector>
#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <QRadioButton>
#include <QButtonGroup>
#include <QElapsedTimer>
#include <QUdpSocket>
#include <QTcpServer>
#include <QTcpSocket>
#include <cstdint>
#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
#endif
#include "fft.h"
#include "dataprocessor.h"
#include "digitaldecoder.h"
#include "audioprocessor.h"
#include "networkcontroller.h"
#include "playbackmanager.h"
#include "recordingmanager.h"
#include "remoteaudioplayer.h"
#include "radiosettings.h"
#include "videoprocessor.h"
#include "videowidget.h"
#include "frequencycontrol.h"
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
class FineTuneScaleWidget;
class QStackedWidget;
class QToolButton;
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
    void onFineTuneDialChanged(int value);
    void onFineTuneDialReleased();
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
    void onDigitalTextDecoded(const QString &text);
    void onDigitalDecoderStatusChanged(const QString &status);
    void startRecording(bool momentary);
    void stopRecording(bool momentaryRelease);
    void startPlayback();
    void stopPlayback();
    void refreshPlaybackFiles();
    void onPlaybackStarted(const QString &filePath, PlaybackManager::WavInfo info);
    void onPlaybackStopped();
    void onPlaybackStatusChanged(const QString &status);
    void onVideoStatusChanged(const QString &status);
protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
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
    struct FobosDeviceInfo {
        FobosApiKind apiKind = FobosApiKind::Standard;
        int nativeIndex = 0;
        QString label;
        QString serial;
        QString hardwareRevision;
        QString firmwareVersion;
        QString product;
        QString manufacturer;
    };

    QWidget *centralWidget;
    QStringList getFobosDevices();
    void refreshFobosDeviceList();
    QString formatFobosDeviceLabel(const FobosDeviceInfo &info) const;
    FobosDeviceInfo selectedFobosDeviceInfo() const;
    bool restartStreamForHardwareChange();
    bool openFobosSession();
    bool closeFobosSession(bool clearIq = true);
    bool applyFobosSettings();
    bool applyAgileScanSettings(bool forceStop = false);
    QVector<double> agileScanFrequencyList(QString *error = nullptr) const;
    void updateAgileScanControls();
    void saveAgileScanPreset();
    void deleteAgileScanPreset();
    void ensureDefaultFrequencyPresets();
    void ensureDefaultBandMarkers();
    void updateFrequencyPresetControls();
    void updateGraphBandMarkers();
    QVector<QPair<QString, double>> presetMapToVector(const QMap<QString, double> &presets) const;
    void openPresetManager();
    void openApplicationSettings();
    double fineTuneRangeHz() const;
    double fineTuneStepHz() const;
    void updateFineTuneLabel();
    void updateFineTuneControlMode();
    void updateFineTuneScaleModeButton();
    void applyListeningFrequencyDelta(double deltaHz, int networkDelayMs = 80);
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
    bool isClientIqProcessingMode(NetworkProcessingMode mode) const;
    void appendNetworkState(QJsonObject &command) const;
    void applyNetworkStateFromCommand(const QJsonObject &command);
    bool sendRemoteControlCommand(const QString &action, const QJsonObject &extra = QJsonObject());
    void scheduleRemoteSettingsCommand(int delayMs = 120);
    void cancelPendingRemoteSettingsCommand();
    QJsonObject settingsToJson() const;
    void applySettingsFromJson(const QJsonObject &settingsJson);
    void updateUiFromPendingSettings();
    void loadUiTranslations();
    QString uiText(const QString &key, const QString &fallback) const;
    QString localizedStatusText(const QString &status) const;
    void markTranslatable(QWidget *widget, const QString &key, const QString &fallback);
    void applyUiLanguage();
    void setComboItemText(QComboBox *combo, const QVariant &data, const QString &key, const QString &fallback);
    void updateNetworkButtonText();
    void updateAudioFilterLabels();
    void updateHfNoiseCancelControls();
    void applyLiveRemoteSettings(const RadioSettings &previousSettings);
    void handleDataProcessorFailure(int errorCode, bool stoppedByRequest);
    void connectDataProcessorSignals();
    void applyServerLocalOutputPolicy();
    bool applyCenterFrequencyToHardwareIfNeeded(const RadioSettings &previousSettings, const char *reason);
    void resetNetworkIqReceptionState(bool clearGraph, bool clearWaterfall, bool restartAudioPrebuffer);
    void loadPersistentSettings();
    void savePersistentSettings();
    void startNetworkClientProcessing();
    void stopNetworkClientProcessing();
    void sendNetworkSpectrumFrame(const std::vector<float> &frequencies,
                                  const std::vector<float> &magnitudes,
                                  const std::vector<float> &referenceMagnitudes = {});
    void displayNetworkSpectrumFrame(const QJsonObject &frame);
    void displayNetworkSpectrumFrameBinary(const QJsonObject &frame, const QByteArray &payload);
    void sendNetworkAudioFrame(const QByteArray &pcmData);
    void playNetworkAudioFrame(const QJsonObject &frame);
    void updateAudioRelaySocket();
    void sendAudioRelayFrame(const QByteArray &pcmData);
    void receiveAudioRelayDatagrams();
    void updateAudioHttpStreamServer();
    void acceptAudioHttpClient();
    void removeAudioHttpClient(QTcpSocket *client);
    void sendAudioHttpFrame(const QByteArray &pcmData);
    void sendNetworkIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);
    void receiveNetworkIqFrame(const QJsonObject &frame);
    void receiveNetworkIqFrameBinary(const QJsonObject &frame, const QByteArray &iqData);
    void handleNetworkIqPayload(const QJsonObject &frame, QByteArray iqBytes);
    void processDigitalAudioFrame(const QByteArray &pcmData);
    void processSstvAudioFrame(const QByteArray &pcmData);
    void processAptAudioFrame(const QByteArray &pcmData);
    void processWefaxAudioFrame(const QByteArray &pcmData);
    void updateDigitalDecoderMode();
    bool isVideoDecodeActive() const;
    void processVideoIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);
    void processVideoSnapshotFrame();
    void updateVideoProcessorMode();
    RadioSettings audioProcessorSettings() const;
    RadioSettings spectrumProcessingSettings() const;
    RecordingManager::Mode selectedRecordingMode() const;
    bool isChannelIqRecordingActive() const;
    void updateIqFrameProducerSettings();
    void updateRecordingStatus(const QString &status);
    void handlePlaybackAudioFrame(const QByteArray &pcmData);
    void handlePlaybackIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);
    QString selectedPlaybackFilePath() const;
    QJsonObject recordingLabMetadata() const;
    void showTuneContextMenu(double frequency, const QPoint &globalPos);
    void tuneSignalCenterAt(double frequency);
    void tuneSidebandEdgeAt(double frequency, int modulationType);
    void centerReceiverAt(double frequency);
    bool applyFftLengthChange(int newFftLength, bool notifyRemote);
    
    QComboBox *clkBox = nullptr;
    QComboBox *comboBox = nullptr;
    QComboBox *modeBox = nullptr;
    QComboBox *sampleBox = nullptr;
    QComboBox *fftComboBox = nullptr;
    QComboBox *audioDeviceComboBox = nullptr;
    QComboBox *recordingModeCombo = nullptr;
    QComboBox *playbackFileCombo = nullptr;
    QComboBox *languageComboBox = nullptr;
    QComboBox *agileScanPresetCombo = nullptr;
    QComboBox *videoDemodCombo = nullptr;
    QComboBox *videoStandardCombo = nullptr;
    QComboBox *dmrLabColorCodeCombo = nullptr;
    QComboBox *dmrLabSlotCombo = nullptr;
    QComboBox *dmrLabCallTypeCombo = nullptr;
    QButtonGroup *modulationButtonGroup = nullptr;
    
    QPushButton *refreshButton = nullptr;
    QPushButton *fobosButton = nullptr;
    QPushButton *networkButton = nullptr;
    QPushButton *presetManagerButton = nullptr;
    QPushButton *appSettingsButton = nullptr;
    QPushButton *controlsToggleButton = nullptr;
    QPushButton *digitalToggleButton = nullptr;
    QPushButton *videoToggleButton = nullptr;
    QPushButton *recordButton = nullptr;
    QPushButton *playbackRefreshButton = nullptr;
    QPushButton *playbackButton = nullptr;
    QPushButton *startButton = nullptr;
    QPushButton *stopButton = nullptr;
    
    QCheckBox *spectrumCheckbox = nullptr;
    QCheckBox *audioCheckbox = nullptr;
    QCheckBox *syncCheckbox = nullptr;
    QCheckBox *graphCheckbox = nullptr;
    QCheckBox *colorCheckbox = nullptr;
    QCheckBox *digitalDecodeCheckbox = nullptr;
    QCheckBox *dmrLabCaptureCheckbox = nullptr;
    QCheckBox *videoDecodeCheckbox = nullptr;
    QCheckBox *videoInvertCheckbox = nullptr;
    QCheckBox *videoHSyncCheckbox = nullptr;
    QCheckBox *videoVSyncCheckbox = nullptr;
    QCheckBox *videoTestPatternCheckbox = nullptr;
    QCheckBox *hfNoiseCancelFreezeCheckbox = nullptr;
    QCheckBox *agileScanCheckbox = nullptr;
    QCheckBox *checkBoxes[8] = {};
    
    QSlider *scaleSlider = nullptr;
    QSlider *lnaGainSlider = nullptr;
    QSlider *vgaGainSlider = nullptr;
    QSlider *contrastSlider = nullptr;
    QSlider *sensitivitySlider = nullptr;
    QSlider *levelMinSlider = nullptr;
    QSlider *levelMaxSlider = nullptr;
    QDial *fineTuneDial = nullptr;
    FineTuneScaleWidget *fineTuneScaleWidget = nullptr;
    QStackedWidget *fineTuneStack = nullptr;
    QToolButton *fineTuneScaleModeButton = nullptr;
    QSlider *volumeSlider = nullptr;
    QSlider *audioLowPassSlider = nullptr;
    QSlider *audioHighPassSlider = nullptr;
    QSlider *hfNoiseCancelDepthSlider = nullptr;
    QSlider *hfNoiseCancelRefGainSlider = nullptr;
    QSlider *hfNoiseCancelRefDelaySlider = nullptr;
    QSlider *hfNoiseCancelRefTiltSlider = nullptr;

    QLabel *volumeLabel = nullptr;
    QLabel *audioLowPassLabel = nullptr;
    QLabel *audioHighPassLabel = nullptr;
    QLabel *hfNoiseCancelDepthLabel = nullptr;
    QLabel *hfNoiseCancelRefGainLabel = nullptr;
    QLabel *hfNoiseCancelRefDelayLabel = nullptr;
    QLabel *hfNoiseCancelRefTiltLabel = nullptr;
    QLabel *lnaGainLabel = nullptr;
    QLabel *centralFrequencyLabel = nullptr;
    QLabel *listeningFrequencyLabel = nullptr;
    QLabel *fftLabel = nullptr;
    QLabel *contrastLabel = nullptr;
    QLabel *sensitivityLabel = nullptr;
    QLabel *levelMinLabel = nullptr;
    QLabel *levelMaxLabel = nullptr;
    QLabel *fineTuneLabel = nullptr;
    QLabel *vgaGainLabel = nullptr;
    QLabel *scaleLabel = nullptr;
    QLabel *digitalStatusLabel = nullptr;
    QLabel *videoStatusLabel = nullptr;
    QLabel *recordingStatusLabel = nullptr;
    QLabel *playbackStatusLabel = nullptr;

    QJsonObject uiTranslations;
    QString uiLanguage = QStringLiteral("en");
       
    QLineEdit *dmrLabSourceIdEdit = nullptr;
    QLineEdit *dmrLabTargetIdEdit = nullptr;
    QLineEdit *dmrLabRadioEdit = nullptr;
    QLineEdit *dmrLabNotesEdit = nullptr;
    QLineEdit *agileScanRangesEdit = nullptr;
    QDoubleSpinBox *agileScanStepSpin = nullptr;
    QPushButton *agileScanSavePresetButton = nullptr;
    QPushButton *agileScanDeletePresetButton = nullptr;
    QLabel *agileScanStatusLabel = nullptr;

    FrequencyControl *frequencyControl = nullptr;
    FrequencyControl *listeningFrequencyControl = nullptr;
    FrequencyControl *bandwidthControl = nullptr;
    
    QTimer *updateTimer = nullptr;
    QTimer *stopPollTimer = nullptr;
    QTimer *streamWatchdogTimer = nullptr;
    QTimer *networkSettingsDebounceTimer = nullptr;
    QTimer *videoSnapshotTimer = nullptr;

    DataProcessor *processor = nullptr;
    AudioProcessor *audioProcessor = nullptr;
    DigitalDecoder *digitalDecoder = nullptr;
    QThread *digitalDecoderThread = nullptr;
    VideoProcessor *videoProcessor = nullptr;
    QThread *videoProcessorThread = nullptr;
    PlaybackManager *playbackManager = nullptr;
    RecordingManager *recordingManager = nullptr;
    NetworkController *networkController = nullptr;
    RemoteAudioPlayer *remoteAudioPlayer = nullptr;
    MyGraphWidget *graphWidget = nullptr;
    MyWaterfallWidget *waterfallWidget = nullptr;
    ScaleWidget *scaleWidget = nullptr;
    QDockWidget *controlsDock = nullptr;
    QDockWidget *digitalDock = nullptr;
    QDockWidget *videoDock = nullptr;
    QPlainTextEdit *digitalTextEdit = nullptr;
    VideoWidget *videoWidget = nullptr;
    
    bool deviceOpened;
    int openedDeviceIndex = -1;
    FobosApiKind openedDeviceApiKind = FobosApiKind::Standard;
    int openedNativeDeviceIndex = -1;
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
    bool networkClientIqProcessingActive = false;
    bool networkClientReconnectPending = false;
    NetworkProcessingMode activeNetworkClientProcessingMode = NetworkProcessingMode::ServerSide;
    bool networkIqStreamMetadataValid = false;
    bool networkIqStreamWasChannelized = false;
    double networkIqStreamSampleRate = 0.0;
    double networkIqStreamCenterFrequency = 0.0;
    double networkIqStreamListeningFrequency = 0.0;
    int networkIqStreamInputMode = 0;
    bool networkSpectrumFrameMetadataValid = false;
    double networkSpectrumFrameMinFrequency = 0.0;
    double networkSpectrumFrameMaxFrequency = 0.0;
    int networkSpectrumFrameFftLength = 0;
    QVector<FobosDeviceInfo> availableFobosDevices;
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
    bool networkFullResolutionSpectrumFrames = false;
    int fineTuneDialLastValue = 0;
    int fineTuneControlMode = 0;
    bool fineTuneScaleHoldMode = false;
    bool serverDisableLocalVisualAudio = true;
    bool digitalDecodeEnabled = true;
    bool videoDecodeEnabled = false;
    bool agileScanEnabled = false;
    bool agileScanRunning = false;
    QString agileScanRangesMhz = QStringLiteral("430-432");
    double agileScanStepMhz = 0.0125;
    QMap<QString, QString> agileScanPresets;
    QMap<QString, double> centerFrequencyPresets;
    QMap<QString, double> listeningFrequencyPresets;
    QMap<QString, double> bandwidthValuePresets;
    QVector<GraphBandMarker> bandMarkers;
    bool bandMarkersCustomized = false;
    bool showGeneralBandMarkers = false;
    bool showAmateurBandMarkers = false;
    bool compactBandMarkers = false;
    std::atomic_bool videoIqFramePending{false};
    bool momentaryRecordingActive = false;
    bool offlineIqPlaybackActive = false;
    bool offlineIqPlaybackHasMetadata = false;
    double offlineIqPlaybackSampleRate = 0.0;
    bool pendingPlaybackAudioStartAfterIqPrebuffer = false;
    bool playbackSettingsSaved = false;
    RadioSettings settingsBeforePlayback;
    int volumePercent = 100;
    QElapsedTimer networkSpectrumFrameTimer;
    uint64_t networkSpectrumFrameSequence = 0;
    uint64_t networkIqFrameSequence = 0;
    uint64_t networkIqFramesDropped = 0;
    QElapsedTimer networkClientSettingsGuardTimer;
    bool audioRelayTransmitEnabled = false;
    QString audioRelayHost = "127.0.0.1";
    quint16 audioRelayPort = 21091;
    bool audioRelayReceiveEnabled = false;
    quint16 audioRelayListenPort = 21091;
    uint32_t audioRelaySequence = 0;
    QUdpSocket *audioRelaySocket = nullptr;
    bool audioHttpStreamEnabled = false;
    quint16 audioHttpStreamPort = 21092;
    QTcpServer *audioHttpServer = nullptr;
    QList<QTcpSocket *> audioHttpClients;
    float displayLevelMin = -120.0f;
    float displayLevelMax = 0.0f;
    int minScale = 1;
    int maxScale = 1000;
};

#endif // MAIN_H
