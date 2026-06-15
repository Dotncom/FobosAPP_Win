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
#include <QSpinBox>
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
#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QStringList>
#include <QVector>
#include <QApplication>
#include <QCoreApplication>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <atomic>
#include <limits>
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
#include "dmrhunterdetector.h"
#include "fpvhunterdetector.h"
#include "digitalvideohunterdetector.h"
#include "spectrumhuntercontrols.h"
#include "scanvisualassembler.h"
#include "frequencycontrol.h"
#include "fobosbackend.h"
#include "rtlsdrbackend.h"
#include "gnssacquisition.h"
#include "gnsssignalmonitor.h"
#include "qthlocator.h"
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
class QDialog;
class QStackedWidget;
class QToolButton;
class QthMapWidget;
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
    void onDmrMetadataDetected(int colorCode,
                               int timeslot,
                               quint32 targetId,
                               quint32 sourceId,
                               int flco);
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
    void closeEvent(QCloseEvent *event) override;
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
    void refreshFobosDeviceList(bool recoverUsb = false);
    QString formatFobosDeviceLabel(const FobosDeviceInfo &info) const;
    FobosDeviceInfo selectedFobosDeviceInfo() const;
    bool isRtlTcpSelected() const;
    bool isRtlSdrNativeSelected() const;
    int selectedRtlSdrNativeIndex() const;
    bool isSoapySdrSelected() const;
    bool isRtlBackendSelected() const;
    bool isExternalReceiverBackendSelected() const;
    bool normalizeRtlSdrSettings();
    ReceiverStreamDescriptor makeRtlTcpStreamDescriptor(bool queueAudioBlocks,
                                                        bool publishIqSnapshot,
                                                        bool emitIqFrames) const;
    ReceiverStreamDescriptor makeRtlSdrNativeStreamDescriptor(bool queueAudioBlocks,
                                                              bool publishIqSnapshot,
                                                              bool emitIqFrames) const;
    ReceiverStreamDescriptor makeSoapySdrStreamDescriptor(bool queueAudioBlocks,
                                                          bool publishIqSnapshot,
                                                          bool emitIqFrames) const;
    bool restartStreamForHardwareChange();
    bool restartAgileReaderForCenterRetune(double previousFrequency,
                                           double requestedFrequency,
                                           const QString &reason);
    void abandonFobosSessionWithoutClose(const char *reason);
    bool openFobosSession();
    bool closeFobosSession(bool clearIq = true);
    bool applyFobosSettings();
    bool applyAgileScanSettings(bool forceStop = false);
    bool applyStandardScanSettings(bool forceStop = false);
    QVector<double> agileScanFrequencyList(QString *error = nullptr) const;
    double agileScanAutoStepMhz() const;
    void applyAgileScanAutoStep(bool updateSpin = true);
    QVector<double> standardScanFrequencyList(QString *error = nullptr) const;
    QVector<double> listeningScanFrequencyList(QString *error = nullptr) const;
    double currentAgileScanCenterFrequencyHz() const;
    double currentStandardScanCenterFrequencyHz() const;
    void resetStandardScanState(bool clearSegments = true);
    void resetListeningScanState();
    bool applyStandardScanRetune(double targetFrequencyHz, const char *reason);
    bool applyListeningScanTarget(double targetFrequencyHz, const char *reason);
    bool applyListeningScanSettings(bool forceStop = false);
    bool scheduleLiveAgileCenterRetune(const QString &reason);
    void flushQueuedLiveAgileCenterRetune();
    void advanceStandardScanIfNeeded();
    void advanceListeningScanIfNeeded();
    void normalizeStandardScanCentersUi(bool requireTwoCenters = false);
    void applyStandardScanRangeToCenters();
    void updateAgileScanControls();
    void updateListeningScanControls();
    void saveAgileScanPreset();
    void deleteAgileScanPreset();
    void updateScanMeasurement(const std::vector<float> &frequencies,
                               const std::vector<float> &magnitudes);
    std::vector<float> scanMeasurementOverlay(const std::vector<float> &frequencies,
                                              int dataCount) const;
    void updateScanMeasurementStatus();
    void resetScanMeasurementPeaks();
    void clearScanMeasurement();
    void exportScanMeasurementCsv();
    void updateFpvHunter(const std::vector<float> &frequencies,
                         const std::vector<float> &magnitudes);
    void updateFpvHunterControls();
    void applyFpvHunterPresetToScan();
    void tuneFpvHunterCandidate();
    void selectFpvHunterCandidate(int direction);
    void tuneFpvHunterCandidateIndex(int index);
    void tuneFpvHunterCandidateValue(const FpvHunterCandidate &candidate, bool saveSettings);
    void rememberFpvHunterCandidate(const FpvHunterCandidate &candidate, bool startNewEvent, qint64 nowMs);
    void updateFpvHunterHistoryControls();
    void tuneFpvHunterHistorySelection();
    void clearFpvHunterHistory();
    void updateDigitalVideoHunter(const std::vector<float> &frequencies,
                                  const std::vector<float> &magnitudes);
    void updateDigitalVideoHunterControls();
    void applyDigitalVideoHunterPresetToScan();
    void tuneDigitalVideoHunterCandidate();
    void selectDigitalVideoHunterCandidate(int direction);
    void tuneDigitalVideoHunterCandidateIndex(int index);
    void tuneDigitalVideoHunterCandidateValue(const DigitalVideoHunterCandidate &candidate, bool saveSettings);
    void updateDmrHunter(const std::vector<float> &frequencies,
                         const std::vector<float> &magnitudes);
    void updateDmrHunterControls();
    void applyDmrHunterPresetToScan();
    void tuneDmrHunterCandidate();
    void selectDmrHunterCandidate(int direction);
    void tuneDmrHunterCandidateIndex(int index);
    void startSpurCalibration();
    void clearSpurMask();
    void updateSpurCalibration(const std::vector<float> &frequencies,
                               const std::vector<float> &magnitudes,
                               double centerFrequency);
    void finishSpurCalibration();
    void applySpurSuppression(const std::vector<float> &frequencies,
                              std::vector<float> &magnitudes,
                              double centerFrequency) const;
    void updateSpurSuppressionStatus();
    void updateGnssSpurWatch(const std::vector<float> &frequencies,
                             const std::vector<float> &magnitudes,
                             double centerFrequency);
    void ensureDefaultFrequencyPresets();
    void ensureDefaultBandMarkers();
    void updateFrequencyPresetControls();
    void updateGraphBandMarkers();
    void setControlsPanelVisible(bool visible);
    QVector<QPair<QString, double>> presetMapToVector(const QMap<QString, double> &presets,
                                                      const QStringList &order) const;
    void openPresetManager();
    void openApplicationSettings();
    void openApplicationHelp();
    void exportSettingsBackup();
    void importSettingsBackup();
    void updateQthControls();
    void applyQthPositionFromUi();
    void pasteNmeaPositionFromClipboard();
    bool applyNmeaPositionText(const QString &text, QString *statusMessage = nullptr);
    void openQthMapWindow();
    void copyQthLocator();
    void updateGnssSystemSelection();
    void applyGnssSystemPresetToReceiver(const QString &systemId);
    void applyQthMapSearch();
    void tuneGnssL1Preset();
    void applyGnssScanPreset();
    void logGnssRawContext();
    bool isGnssMonitorActive() const;
    void resetGnssMonitor();
    void processGnssIqSnapshot(const RadioSettings &settings);
    void processGnssPackedIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);
    void updateGnssMonitorStatus(const GnssSignalReport &report, bool forceLog = false);
    void runGnssAcquisitionTest();
    void runGnssDeepAcquisitionTest();
    void runGnssOfflineReplayTest();
    void runGnssSyntheticSelfTest();
    void runGnssPositionSelfTest();
    void updateGnssAcquisitionStatus(const GnssAcquisitionResult &result);
    void updateGnssAcquisitionPlot(const GnssAcquisitionResult &result);
    void saveGnssAcquisitionArtifacts(const GnssAcquisitionResult &result, const QString &sourceLabel);
    void requestGnssNetworkTime();
    void handleGnssNetworkTimeResponse();
    void updateQthMapControls();
    void applyQthOnlineProviderPreset(const QString &providerId, bool applyTemplate);
    QString resolvedQthOnlineTileUrlTemplate() const;
    void selectQthTileDirectory();
    void openQthTileDirectory();
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
    QJsonObject networkSettingsPatch(const QJsonObject &settings) const;
    void applyAuthoritativeNetworkState(const QJsonObject &command);
    void sendSettingsAckToPeer(const QString &peerId,
                               bool ok,
                               const QString &requestId = QString(),
                               const QString &reason = QString());
    void startNetworkSettingsAckWait();
    void handleNetworkSettingsAckTimeout();
    void buildLocalReceiverDeviceChoices(QStringList &labels, QVector<int> &values) const;
    void rebuildReceiverDeviceCombo();
    QJsonArray receiverDeviceListToJson() const;
    bool applyReceiverDeviceListFromJson(const QJsonArray &devices);
    void clearRemoteReceiverDeviceList();
    void sendServerStateToClients();
    bool sendRemoteControlCommand(const QString &action, const QJsonObject &extra = QJsonObject());
    void scheduleRemoteSettingsCommand(int delayMs = 120);
    void cancelPendingRemoteSettingsCommand();
    QJsonObject settingsToJson() const;
    void applySettingsFromJson(const QJsonObject &settingsJson, bool normalizeAfterApply = true);
    void updateUiFromPendingSettings();
    void loadUiTranslations();
    QString normalizedUiLanguage(const QString &language) const;
    void populateLanguageCombo(QComboBox *combo) const;
    QString uiText(const QString &key, const QString &fallback) const;
    QString localizedStatusText(const QString &status) const;
    void markTranslatable(QWidget *widget, const QString &key, const QString &fallback);
    void applyUiLanguage();
    void applySpectrumHunterTranslations();
    void setComboItemText(QComboBox *combo, const QVariant &data, const QString &key, const QString &fallback);
    void updateNetworkButtonText();
    void updateAudioFilterLabels();
    void updateHfNoiseCancelControls();
    void applyLiveRemoteSettings(const RadioSettings &previousSettings);
    void handleDataProcessorFailure(int errorCode, bool stoppedByRequest);
    void clearLiveSpectrumSnapshot(bool clearVisualHistory = false, uint64_t iqEpoch = 0);
    void connectDataProcessorSignals();
    void applyServerLocalOutputPolicy();
    bool applyCenterFrequencyToHardwareIfNeeded(const RadioSettings &previousSettings, const char *reason);
    bool applyLiveAgileCenterRetune(uint64_t generation, const QString &reason);
    void resetNetworkIqReceptionState(bool clearGraph, bool clearWaterfall, bool restartAudioPrebuffer);
    void loadPersistentSettings();
    void savePersistentSettings();
    void flushPendingPersistentSettingsSave();
    void startNetworkClientProcessing();
    void stopNetworkClientProcessing();
    void sendNetworkSpectrumFrame(const std::vector<float> &frequencies,
                                  const std::vector<float> &magnitudes,
                                  const std::vector<float> &referenceMagnitudes = {},
                                  double frameCenterFrequency = std::numeric_limits<double>::quiet_NaN(),
                                  double frameMinFrequency = std::numeric_limits<double>::quiet_NaN(),
                                  double frameMaxFrequency = std::numeric_limits<double>::quiet_NaN(),
                                  const QVector<ScanVisualSegment> &scanSegments = QVector<ScanVisualSegment>(),
                                  bool frameFresh = true,
                                  bool showScanSegmentMarkers = true);
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
    void processDigitalAudioFrame(const QByteArray &pcmData, int sampleRate = 48000);
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
    bool stopAgileScanForNormalRf(const char *reason);
    
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
    QComboBox *scanVisualModeCombo = nullptr;
    QComboBox *standardScanPresetCombo = nullptr;
    QComboBox *listeningScanPresetCombo = nullptr;
    QComboBox *videoDemodCombo = nullptr;
    QComboBox *videoStandardCombo = nullptr;
    QComboBox *dmrLabColorCodeCombo = nullptr;
    QComboBox *dmrLabSlotCombo = nullptr;
    QComboBox *dmrLabCallTypeCombo = nullptr;
    QComboBox *dmrBasebandRateCombo = nullptr;
    QComboBox *dmrAmbeLayoutCombo = nullptr;
    QComboBox *qthSourceCombo = nullptr;
    QComboBox *gnssSystemCombo = nullptr;
    QCheckBox *dmrManualTimingCheckbox = nullptr;
    QSpinBox *dmrTimingOffsetSpin = nullptr;
    QDoubleSpinBox *dmrSlicerRatioSpin = nullptr;
    QSpinBox *gnssDopplerSpanSpin = nullptr;
    QSpinBox *gnssDopplerStepSpin = nullptr;
    QCheckBox *dmrAdaptiveSlicerCheckbox = nullptr;
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
    QPushButton *qthMapButton = nullptr;
    QPushButton *qthCopyButton = nullptr;
    QPushButton *qthPasteNmeaButton = nullptr;
    QPushButton *gnssTuneButton = nullptr;
    QPushButton *gnssScanButton = nullptr;
    QPushButton *gnssRawLogButton = nullptr;
    QPushButton *gnssAcquireButton = nullptr;
    QPushButton *gnssDeepAcquireButton = nullptr;
    QPushButton *gnssOfflineAcquireButton = nullptr;
    QPushButton *gnssSelfTestButton = nullptr;
    QPushButton *gnssPositionSelfTestButton = nullptr;
    QPushButton *gnssNetworkTimeButton = nullptr;
    QPushButton *gnssPlotButton = nullptr;
    QPushButton *gnssMonitorResetButton = nullptr;
    
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
    QCheckBox *standardScanCheckbox = nullptr;
    QCheckBox *listeningScanCheckbox = nullptr;
    QCheckBox *scanListeningLockCheckbox = nullptr;
    QCheckBox *scanMeasurementCheckbox = nullptr;
    QCheckBox *spurSuppressionCheckbox = nullptr;
    QCheckBox *gnssMonitorCheckbox = nullptr;
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
    SpectrumHunterControls *dmrHunterControls = nullptr;
    SpectrumHunterControls *fpvHunterControls = nullptr;
    SpectrumHunterControls *digitalVideoHunterControls = nullptr;

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
    QMap<QString, QString> uiLanguageNames;
    QStringList uiLanguageOrder;
    QString uiLanguage = QStringLiteral("en");
       
    QLineEdit *dmrLabSourceIdEdit = nullptr;
    QLineEdit *dmrLabTargetIdEdit = nullptr;
    QLineEdit *dmrLabRadioEdit = nullptr;
    QLineEdit *dmrLabNotesEdit = nullptr;
    QLineEdit *agileScanRangesEdit = nullptr;
    QLineEdit *standardScanCentersEdit = nullptr;
    QLineEdit *listeningScanTargetsEdit = nullptr;
    QLineEdit *standardScanRangeStartEdit = nullptr;
    QLineEdit *standardScanRangeEndEdit = nullptr;
    QLineEdit *qthTileDirectoryEdit = nullptr;
    QLineEdit *qthOnlineTileUrlEdit = nullptr;
    QLineEdit *qthOnlineAttributionEdit = nullptr;
    QLineEdit *qthOnlineApiKeyEdit = nullptr;
    QLineEdit *qthMapSearchEdit = nullptr;
    QDoubleSpinBox *agileScanStepSpin = nullptr;
    QCheckBox *agileScanAutoStepCheckbox = nullptr;
    QDoubleSpinBox *qthLatitudeSpin = nullptr;
    QDoubleSpinBox *qthLongitudeSpin = nullptr;
    QDoubleSpinBox *gnssChannelFilterSpin = nullptr;
    QSpinBox *standardScanDwellSpin = nullptr;
    QSpinBox *standardScanSettleSpin = nullptr;
    QSpinBox *listeningScanDwellSpin = nullptr;
    QSpinBox *listeningScanSettleSpin = nullptr;
    QSpinBox *gnssIntegrationSpin = nullptr;
    QDoubleSpinBox *scanMeasurementBinSpin = nullptr;
    QPushButton *agileScanSavePresetButton = nullptr;
    QPushButton *agileScanDeletePresetButton = nullptr;
    QPushButton *standardScanSavePresetButton = nullptr;
    QPushButton *standardScanDeletePresetButton = nullptr;
    QPushButton *listeningScanSavePresetButton = nullptr;
    QPushButton *listeningScanDeletePresetButton = nullptr;
    QPushButton *standardScanAddLowerButton = nullptr;
    QPushButton *standardScanAddUpperButton = nullptr;
    QPushButton *standardScanRemoveLowerButton = nullptr;
    QPushButton *standardScanRemoveUpperButton = nullptr;
    QPushButton *standardScanFillRangeButton = nullptr;
    QPushButton *scanMeasurementBaselineButton = nullptr;
    QPushButton *scanMeasurementResetPeakButton = nullptr;
    QPushButton *scanMeasurementExportButton = nullptr;
    QPushButton *spurCalibrateButton = nullptr;
    QPushButton *spurClearButton = nullptr;
    QPushButton *qthSelectTilesButton = nullptr;
    QPushButton *qthOpenTilesButton = nullptr;
    QPushButton *qthCenterMapButton = nullptr;
    QPushButton *qthUseOsmButton = nullptr;
    QPushButton *qthMapSearchButton = nullptr;
    QPushButton *fpvHunterHistoryTuneButton = nullptr;
    QPushButton *fpvHunterHistoryClearButton = nullptr;
    QComboBox *fpvHunterHistoryCombo = nullptr;
    QComboBox *qthMapLayerCombo = nullptr;
    QComboBox *qthOnlineProviderCombo = nullptr;
    QComboBox *qthGridPrecisionCombo = nullptr;
    QCheckBox *qthOnlineNoDiskCacheCheckbox = nullptr;
    QSpinBox *qthMapZoomSpin = nullptr;
    QLabel *fpvHunterHistoryLabel = nullptr;
    QLabel *agileScanStatusLabel = nullptr;
    QLabel *standardScanStatusLabel = nullptr;
    QLabel *listeningScanStatusLabel = nullptr;
    QLabel *scanMeasurementStatusLabel = nullptr;
    QLabel *spurSuppressionStatusLabel = nullptr;
    QLabel *qthLocatorLabel = nullptr;
    QLabel *qthStatusLabel = nullptr;
    QLabel *qthMapStatusLabel = nullptr;
    QLabel *gnssMonitorStatusLabel = nullptr;
    QLabel *gnssAcquireStatusLabel = nullptr;
    QLabel *gnssAcquisitionPlotLabel = nullptr;

    FrequencyControl *frequencyControl = nullptr;
    FrequencyControl *listeningFrequencyControl = nullptr;
    FrequencyControl *bandwidthControl = nullptr;
    
    QTimer *updateTimer = nullptr;
    QTimer *stopPollTimer = nullptr;
    QTimer *streamWatchdogTimer = nullptr;
    QTimer *networkSettingsDebounceTimer = nullptr;
    QTimer *networkSettingsAckTimer = nullptr;
    QTimer *standardScanAdvanceTimer = nullptr;
    QTimer *listeningScanAdvanceTimer = nullptr;
    QTimer *videoSnapshotTimer = nullptr;
    QTimer *agileLiveRetuneTimer = nullptr;
    QTimer *persistentSettingsSaveTimer = nullptr;
    QUdpSocket *gnssNtpSocket = nullptr;

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
    QDialog *qthMapDialog = nullptr;
    QthMapWidget *qthMapWidget = nullptr;
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
    bool clearSpectrumAfterStop = false;
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
    QStringList remoteReceiverDeviceLabels;
    QVector<int> remoteReceiverDeviceValues;
    bool remoteReceiverDeviceListValid = false;
    RadioSettings pendingSettings;
    RadioSettings appliedHardwareSettings;
    bool hardwareSettingsApplied = false;
    int spectrumDebugFramesRemaining = 0;
    int spectrumTuningDebugFramesRemaining = 0;
    int scanVisualDebugFramesRemaining = 0;
    uint64_t scanVisualDebugSequence = 0;
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
    bool digitalDecodeEnabled = false;
    QByteArray pendingDmrDecoderPcm;
    int pendingDmrDecoderSampleRate = 48000;
    std::atomic<uint64_t> digitalDecoderGeneration {0};
    std::atomic<int> pendingDigitalDecoderFrames {0};
    std::atomic<int> droppedDigitalDecoderFramesSinceLog {0};
    bool videoDecodeEnabled = false;
    bool closeShutdownInProgress = false;
    bool agileScanEnabled = false;
    bool agileScanRunning = false;
    bool standardScanEnabled = false;
    bool standardScanRunning = false;
    bool listeningScanEnabled = false;
    bool listeningScanRunning = false;
    bool scanListeningLockEnabled = true;
    bool agileScanAutoStepSampleRate = true;
    int scanVisualMode = static_cast<int>(ScanVisualMode::CompressedMosaic);
    QString agileScanRangesMhz = QStringLiteral("430-432");
    double agileScanStepMhz = 0.0125;
    QString standardScanCentersMhz = QStringLiteral("430, 480");
    int standardScanDwellMs = 650;
    int standardScanSettleMs = 60;
    QString standardScanRangeStartMhz;
    QString standardScanRangeEndMhz;
    QMap<QString, QString> standardScanPresets;
    QStringList standardScanPresetOrder;
    QString listeningScanTargetsMhz = QStringLiteral("1561.098, 1575.420, 1602.000");
    int listeningScanDwellMs = 3000;
    int listeningScanSettleMs = 100;
    QMap<QString, QString> listeningScanPresets;
    QStringList listeningScanPresetOrder;
    int spectrumUpdateIntervalMs = 0;
    int waterfallRowsPerFrame = 1;
    bool scanMeasurementEnabled = true;
    bool scanMeasurementBaselineRecording = false;
    double scanMeasurementBinMhz = 0.1;
    DmrHunterSettings dmrHunterSettings;
    DmrHunterResult dmrHunterLastResult;
    std::vector<DmrHunterCandidate> dmrHunterCandidates;
    int dmrHunterCandidateIndex = -1;
    FpvHunterSettings fpvHunterSettings;
    FpvHunterResult fpvHunterLastResult;
    std::vector<FpvHunterCandidate> fpvHunterCandidates;
    int fpvHunterCandidateIndex = -1;
    bool fpvHunterFollowEnabled = false;
    double fpvHunterLastFollowCenterHz = std::numeric_limits<double>::quiet_NaN();
    double fpvHunterLastFollowBandwidthHz = std::numeric_limits<double>::quiet_NaN();
    struct FpvHunterTrack {
        bool valid = false;
        bool stable = false;
        double centerHz = 0.0;
        double widthHz = 0.0;
        float peakDb = -160.0f;
        float averageDb = -160.0f;
        float excessDb = 0.0f;
        float score = 0.0f;
        int hits = 0;
        int misses = 0;
        uint64_t lastSeenSequence = 0;
        qint64 firstSeenMsec = -1;
        qint64 lastSeenMsec = -1;
        QString type;
    };
    FpvHunterTrack fpvHunterTrack;
    uint64_t fpvHunterFrameSequence = 0;
    QElapsedTimer fpvHunterClock;
    struct FpvHunterEvent {
        bool valid = false;
        quint64 id = 0;
        double centerHz = 0.0;
        double widthHz = 0.0;
        float peakDb = -160.0f;
        float averageDb = -160.0f;
        float excessDb = 0.0f;
        float score = 0.0f;
        int hits = 0;
        qint64 firstSeenMsec = -1;
        qint64 lastSeenMsec = -1;
        QString type;
    };
    QVector<FpvHunterEvent> fpvHunterEvents;
    int fpvHunterActiveEventIndex = -1;
    quint64 fpvHunterNextEventId = 1;
    DigitalVideoHunterSettings digitalVideoHunterSettings;
    DigitalVideoHunterResult digitalVideoHunterLastResult;
    std::vector<DigitalVideoHunterCandidate> digitalVideoHunterCandidates;
    int digitalVideoHunterCandidateIndex = -1;
    QMap<QString, QString> agileScanPresets;
    QStringList agileScanPresetOrder;
    QVector<double> activeAgileScanFrequencies;
    QVector<double> activeStandardScanFrequencies;
    QVector<double> activeListeningScanFrequencies;
    int standardScanIndex = 0;
    int listeningScanIndex = 0;
    ScanVisualAssembler scanVisualAssembler;

    struct ScanMeasurementBin {
        double frequencyHz = 0.0;
        float currentDb = -160.0f;
        float peakDb = -160.0f;
        float baselineDb = -160.0f;
        int baselineCount = 0;
        int seenCount = 0;
        uint64_t lastSequence = 0;
    };
    QMap<qint64, ScanMeasurementBin> scanMeasurementBins;
    uint64_t scanMeasurementSequence = 0;

    struct SpurMaskEntry {
        double offsetHz = 0.0;
        double widthHz = 0.0;
        float prominenceDb = 0.0f;
        int hits = 0;
    };
    struct SpurCalibrationBin {
        double offsetWeightedSum = 0.0;
        double weightSum = 0.0;
        float maxProminenceDb = 0.0f;
        int hits = 0;
    };
    struct GnssSpurWatchBin {
        float averageDb = -160.0f;
        int hits = 0;
        qint64 lastSeenMs = 0;
    };
    QVector<SpurMaskEntry> spurMaskEntries;
    QMap<qint64, SpurCalibrationBin> spurCalibrationBins;
    QMap<qint64, GnssSpurWatchBin> gnssSpurWatchBins;
    bool spurSuppressionEnabled = false;
    bool spurCalibrationActive = false;
    int spurCalibrationFramesDone = 0;
    int spurCalibrationTargetFrames = 32;
    double spurCalibrationBinHz = 0.0;
    QMap<QString, double> centerFrequencyPresets;
    QMap<QString, double> listeningFrequencyPresets;
    QMap<QString, double> bandwidthValuePresets;
    QStringList centerFrequencyPresetOrder;
    QStringList listeningFrequencyPresetOrder;
    QStringList bandwidthPresetOrder;
    int centerFrequencyUnitIndex = 2;
    int listeningFrequencyUnitIndex = 2;
    int bandwidthUnitIndex = 1;
    QString centerFrequencyStepName = QStringLiteral("1 MHz");
    QString listeningFrequencyStepName = QStringLiteral("1 kHz");
    QString bandwidthStepName = QStringLiteral("1 kHz");
    QString centerFrequencyPresetName;
    QString listeningFrequencyPresetName;
    QString bandwidthPresetName;
    bool frequencyControlUiStateRestorePending = false;
    QVector<GraphBandMarker> bandMarkers;
    bool bandMarkersCustomized = false;
    bool showGeneralBandMarkers = true;
    bool showAmateurBandMarkers = true;
    bool compactBandMarkers = false;
    bool diagnosticVerboseLogging = false;
    bool gnssMonitorEnabled = false;
    bool gnssAcquisitionRunning = false;
    std::shared_ptr<std::atomic_bool> gnssAcquisitionCancelFlag;
    QVector<double> gnssPeakToSecondHistoryDb;
    QString gnssSystemId = QStringLiteral("gps_l1_ca");
    QString gnssAcquisitionSource = QStringLiteral("live");
    int gnssAcquisitionIntegrationMs = 24;
    double gnssChannelFilterCutoffHz = 1800000.0;
    int gnssDopplerSpanHz = 25000;
    int gnssDopplerStepHz = 1000;
    GnssSignalMonitor gnssSignalMonitor;
    QElapsedTimer gnssMonitorSnapshotTimer;
    QElapsedTimer gnssMonitorLogTimer;
    QElapsedTimer gnssMonitorUiTimer;
    QElapsedTimer gnssSpurWatchTimer;
    QElapsedTimer gnssSpurLogTimer;
    QDialog *gnssAcquisitionPlotDialog = nullptr;
    double qthLatitude = 0.0;
    double qthLongitude = 0.0;
    QString qthSource = QStringLiteral("manual");
    QString qthTileDirectory;
    QString qthOnlineProviderId = QStringLiteral("osm");
    QString qthOnlineTileUrlTemplate = QStringLiteral("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
    QString qthOnlineAttribution = QString::fromUtf8("\xC2\xA9 OpenStreetMap contributors");
    QString qthOnlineApiKey;
    bool qthOnlineNoDiskCache = false;
    int qthMapLayer = 0;
    int qthMapZoom = 12;
    int qthGridPrecision = 6;
    QVector<qth::UserMarker> qthUserMarkers;
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
    QElapsedTimer liveRetuneSettleTimer;
    QElapsedTimer standardScanDwellTimer;
    QElapsedTimer listeningScanDwellTimer;
    QElapsedTimer listeningScanSettleTimer;
    qint64 liveRetuneSettleDurationMs = 80;
    uint64_t liveCenterRetuneGeneration = 0;
    uint64_t queuedLiveCenterRetuneGeneration = 0;
    QString queuedLiveCenterRetuneReason;
    QElapsedTimer agileLiveRetuneCommandTimer;
    int agileLiveRetuneCommandIntervalMs = 120;
    QElapsedTimer persistentSettingsLastSaveTimer;
    bool persistentSettingsSaveDeferred = false;
    bool persistentSettingsSaveInProgress = false;
    uint64_t networkSpectrumFrameSequence = 0;
    uint64_t networkIqFrameSequence = 0;
    uint64_t networkIqFramesDropped = 0;
    QJsonObject networkClientConfirmedSettingsJson;
    bool networkClientConfirmedSettingsValid = false;
    bool networkClientAwaitingSettingsAck = false;
    quint64 networkClientSettingsRequestCounter = 0;
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
    float displayLevelMin = -140.0f;
    float displayLevelMax = -30.0f;
    bool persistentSettingsReady = false;
    int minScale = 1;
    int maxScale = 1000;
};

#endif // MAIN_H
