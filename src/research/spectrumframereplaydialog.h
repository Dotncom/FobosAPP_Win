#ifndef SPECTRUMFRAMEREPLAYDIALOG_H
#define SPECTRUMFRAMEREPLAYDIALOG_H

#include <QDialog>
#include <QColor>
#include <QCheckBox>
#include <QComboBox>
#include <QImage>
#include <QLabel>
#include <QElapsedTimer>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QString>
#include <QTimer>
#include <atomic>
#include <thread>
#include <vector>

#include "spectrumframerecorder.h"

class MyGraphWidget;
class AudioProcessor;
class FrequencyControl;
class QResizeEvent;
class QScrollArea;
class QShowEvent;
class ScaleWidget;

class SpectrumFrameReplayDialog : public QDialog {
public:
    explicit SpectrumFrameReplayDialog(QWidget *parent = nullptr);
    ~SpectrumFrameReplayDialog() override;

    bool loadRecording(const QString &path, QString *errorMessage = nullptr);

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;

private:
    enum class ReplayIqSource {
        ChannelIq,
        FullIq
    };

    void rebuildWaterfallImage();
    void renderWaterfallPixmap();
    void updateFrameSelection(int index);
    void updateLabels();
    void setPlaybackEnabled(bool enabled);
    double replaySpeedMultiplier() const;
    void scheduleNextPlaybackFrame();
    void updateLinkedIqControls();
    void updateReplayMarker();
    void updateScaleWidget();
    void scheduleDeferredRender();
    void setReplayListenFrequency(double frequencyHz, bool fromUser);
    void openRecordingFromDialog();
    void loadPersistentUiSettings();
    void savePersistentUiSettings() const;
    void schedulePersistentUiSettingsSave();
    void updateSliderValueLabels();
    void startLocalIqPlayback();
    void stopLocalIqPlayback();
    void resetLocalIqPlaybackState(bool updateButton);
    void stopLocalIqWorker();
    RadioSettings replaySettingsForSource(double sampleRate) const;
    ReplayIqSource selectedReplayIqSource() const;
    QString replayIqPathForSource(ReplayIqSource source) const;
    QString replayIqSidecarPathForSource(ReplayIqSource source) const;
    double replayIqEffectiveSampleRate(ReplayIqSource source,
                                       double fallbackSampleRate,
                                       quint64 dataSize,
                                       int bytesPerSample) const;
    double replayIqNominalSampleRate(ReplayIqSource source, double fallbackSampleRate) const;
    double replayIqTimelineOffsetSeconds(ReplayIqSource source, double iqDurationSeconds) const;
    bool replayIqSourceAvailable(ReplayIqSource source) const;
    double frequencyForLocalX(int x, int width) const;
    static QColor colorForLevel(float value,
                                float minLevel,
                                float maxLevel,
                                float contrastValue,
                                float sensitivityValue);
    static QString formatFrequency(double hz);

    MyGraphWidget *graph = nullptr;
    ScaleWidget *scaleWidget = nullptr;
    QLabel *waterfallLabel = nullptr;
    QScrollArea *graphScroll = nullptr;
    QScrollArea *scaleScroll = nullptr;
    QScrollArea *waterfallScroll = nullptr;
    QLabel *infoLabel = nullptr;
    QSlider *timelineSlider = nullptr;
    QPushButton *openButton = nullptr;
    QPushButton *playButton = nullptr;
    QCheckBox *replayAudioCheckbox = nullptr;
    QComboBox *speedCombo = nullptr;
    QComboBox *replayIqSourceCombo = nullptr;
    QSlider *zoomSlider = nullptr;
    QSlider *rowHeightSlider = nullptr;
    QSlider *contrastSlider = nullptr;
    QSlider *sensitivitySlider = nullptr;
    QLabel *zoomValueLabel = nullptr;
    QLabel *rowHeightValueLabel = nullptr;
    QLabel *contrastValueLabel = nullptr;
    QLabel *sensitivityValueLabel = nullptr;
    FrequencyControl *listenFrequencyControl = nullptr;
    QSpinBox *levelMinSpin = nullptr;
    QSpinBox *levelMaxSpin = nullptr;
    QLabel *linkedIqLabel = nullptr;
    QTimer playbackTimer;
    QTimer saveSettingsTimer;
    QElapsedTimer playbackClock;

    SpectrumFrameRecording recording;
    QString recordingPath;
    QString linkedChannelIqPath;
    QString linkedFullIqPath;
    AudioProcessor *localAudioProcessor = nullptr;
    QImage waterfallImage;
    QImage scaledWaterfallCache;
    int scaledWaterfallCacheWidth = 0;
    bool scaledWaterfallDirty = true;
    std::vector<float> frequencyScratch;
    std::thread localIqWorkerThread;
    std::atomic<bool> localIqWorkerStop{false};
    int selectedFrame = 0;
    int playbackStartFrame = 0;
    qint64 playbackStartElapsedMs = 0;
    float levelMin = -140.0f;
    float levelMax = -30.0f;
    int waterfallZoomPercent = 100;
    int waterfallRowHeight = 4;
    float replayContrast = 10.0f;
    float replaySensitivity = 10.0f;
    double replayListenFrequencyHz = 0.0;
    double localIqPlaybackSampleRate = 0.0;
    double localIqPlaybackAudioSampleRate = 0.0;
    double localIqPlaybackSeekSampleRate = 0.0;
    double localIqPlaybackFeedSampleRate = 0.0;
    quint64 localIqPlaybackDataSize = 0;
    quint64 localIqPlaybackBytesRead = 0;
    quint64 localIqPlaybackStartByteOffset = 0;
    int localIqPlaybackBytesPerSample = 0;
    bool localIqPlaybackS16 = false;
    double localIqPlaybackDurationSeconds = 0.0;
    double localIqPlaybackTimelineOffsetSeconds = 0.0;
    double localIqPlaybackStartDelaySeconds = 0.0;
    bool localIqPlaybackActive = false;
    bool syncingHorizontalScroll = false;
    bool deferredRenderQueued = false;
};

#endif // SPECTRUMFRAMEREPLAYDIALOG_H
