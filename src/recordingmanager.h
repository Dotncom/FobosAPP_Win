#ifndef RECORDINGMANAGER_H
#define RECORDINGMANAGER_H

#include <QObject>
#include <QFile>
#include <QString>

#include "radiosettings.h"

class RecordingManager : public QObject {
    Q_OBJECT

public:
    enum class Mode {
        AudioWav = 0,
        ChannelIqWav = 1
    };

    explicit RecordingManager(QObject *parent = nullptr);
    ~RecordingManager();

    bool start(Mode mode, const RadioSettings &settings, QString *errorMessage = nullptr);
    void stop();
    bool isRecording() const;
    Mode mode() const;
    QString currentFilePath() const;
    void setDisplayScalePercent(double scalePercent);

    void appendAudioFrame(const QByteArray &pcmData);
    void appendIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount);

signals:
    void statusChanged(const QString &status);

private:
    bool openWaveFile(int sampleRate, int channels, int bitsPerSample, QString *errorMessage = nullptr);
    void writeWaveHeader(int sampleRate, int channels, int bitsPerSample);
    QByteArray makeMetadataChunk() const;
    void patchWaveHeader();
    QString makeRecordingPath(const QString &suffix) const;
    void updateStatus(const QString &status);

    QFile outputFile;
    Mode activeMode = Mode::AudioWav;
    RadioSettings recordingSettings;
    QString filePath;
    bool recording = false;
    bool waveOpen = false;
    int waveSampleRate = 0;
    int waveChannels = 0;
    int waveBitsPerSample = 0;
    qint64 dataSizeFieldOffset = -1;
    quint64 waveHeaderBytes = 0;
    double displayScalePercent = 100.0;
    quint64 dataBytesWritten = 0;
};

#endif // RECORDINGMANAGER_H
