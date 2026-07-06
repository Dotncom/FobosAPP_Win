#include "spectrumiqeventrecorder.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>

#include <algorithm>
#include <cmath>

namespace {

void appendFourCc(QByteArray &data, const char *text) {
    data.append(text, 4);
}

void appendLe16(QByteArray &data, quint16 value) {
    data.append(static_cast<char>(value & 0xff));
    data.append(static_cast<char>((value >> 8) & 0xff));
}

void appendLe32(QByteArray &data, quint32 value) {
    data.append(static_cast<char>(value & 0xff));
    data.append(static_cast<char>((value >> 8) & 0xff));
    data.append(static_cast<char>((value >> 16) & 0xff));
    data.append(static_cast<char>((value >> 24) & 0xff));
}

QByteArray le32Bytes(quint32 value) {
    QByteArray data;
    appendLe32(data, value);
    return data;
}

quint32 size32(quint64 value) {
    return value > 0xffffffffULL ? 0xffffffffU : static_cast<quint32>(value);
}

} // namespace

bool SpectrumIqEventRecorder::start(const QString &eventBasePath,
                                    Mode mode,
                                    const RadioSettings &settings,
                                    double sampleRate,
                                    qint64 firstFrameUtcMsValue,
                                    qint64 spectrumFirstFrameUtcMsValue,
                                    QString *errorMessage) {
    stop();
    if (eventBasePath.isEmpty()) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("empty event base path");
        }
        return false;
    }
    if (sampleRate <= 0.0 || !std::isfinite(sampleRate)) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("invalid IQ sample rate");
        }
        return false;
    }

    activeMode = mode;
    recordingSettings = settings;
    recordingSampleRate = sampleRate;
    firstFrameUtcMs = firstFrameUtcMsValue;
    lastFrameUtcMs = firstFrameUtcMsValue;
    spectrumFirstFrameUtcMs = spectrumFirstFrameUtcMsValue;
    writtenBytes = 0;
    dataSizeFieldOffset = -1;
    waveHeaderBytes = 0;
    filePath = eventBasePath + (mode == Mode::ChannelIqS16
                                    ? QStringLiteral("_channel_iq.wav")
                                    : QStringLiteral("_full_iq_s8.iq8"));
    sidecarPath = eventBasePath + (mode == Mode::ChannelIqS16
                                       ? QStringLiteral("_channel_iq.json")
                                       : QStringLiteral("_full_iq_s8.json"));

    return mode == Mode::ChannelIqS16 ? openChannelWav(errorMessage) : openFullIqRaw(errorMessage);
}

void SpectrumIqEventRecorder::stop() {
    if (!active && !outputFile.isOpen()) {
        return;
    }
    patchWavHeader();
    if (outputFile.isOpen()) {
        outputFile.flush();
        outputFile.close();
    }
    writeSidecar();
    active = false;
}

bool SpectrumIqEventRecorder::isRecording() const {
    return active;
}

QString SpectrumIqEventRecorder::currentFilePath() const {
    return filePath;
}

quint64 SpectrumIqEventRecorder::bytesWritten() const {
    return writtenBytes;
}

SpectrumIqEventRecorder::Mode SpectrumIqEventRecorder::mode() const {
    return activeMode;
}

bool SpectrumIqEventRecorder::appendFrame(const QByteArray &iqData,
                                          double sampleRate,
                                          int sampleCount,
                                          qint64 frameUtcMs) {
    Q_UNUSED(sampleCount);
    if (!active || !outputFile.isOpen() || iqData.isEmpty()) {
        return false;
    }
    if (recordingSampleRate <= 0.0 && sampleRate > 0.0) {
        recordingSampleRate = sampleRate;
    }
    if (firstFrameUtcMs <= 0 && frameUtcMs > 0) {
        firstFrameUtcMs = frameUtcMs;
    }
    if (frameUtcMs > 0) {
        lastFrameUtcMs = frameUtcMs;
    }
    const qint64 written = outputFile.write(iqData);
    if (written <= 0) {
        active = false;
        return false;
    }
    writtenBytes += static_cast<quint64>(written);
    return true;
}

bool SpectrumIqEventRecorder::openChannelWav(QString *errorMessage) {
    QDir dir = QFileInfo(filePath).absoluteDir();
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("cannot create event IQ folder");
        }
        return false;
    }
    outputFile.setFileName(filePath);
    if (!outputFile.open(QIODevice::WriteOnly)) {
        if (errorMessage) {
            *errorMessage = outputFile.errorString();
        }
        return false;
    }
    writeWavHeader();
    active = true;
    return true;
}

bool SpectrumIqEventRecorder::openFullIqRaw(QString *errorMessage) {
    QDir dir = QFileInfo(filePath).absoluteDir();
    if (!dir.exists() && !dir.mkpath(QStringLiteral("."))) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("cannot create event IQ folder");
        }
        return false;
    }
    outputFile.setFileName(filePath);
    if (!outputFile.open(QIODevice::WriteOnly)) {
        if (errorMessage) {
            *errorMessage = outputFile.errorString();
        }
        return false;
    }
    active = true;
    return true;
}

void SpectrumIqEventRecorder::writeWavHeader() {
    const int sampleRate = static_cast<int>(std::max(1.0, std::round(recordingSampleRate)));
    constexpr quint16 channels = 2;
    constexpr quint16 bitsPerSample = 16;
    const quint16 blockAlign = channels * bitsPerSample / 8;
    const quint32 byteRate = static_cast<quint32>(sampleRate * blockAlign);

    QByteArray header;
    appendFourCc(header, "RIFF");
    appendLe32(header, 0);
    appendFourCc(header, "WAVE");
    appendFourCc(header, "fmt ");
    appendLe32(header, 16);
    appendLe16(header, 1);
    appendLe16(header, channels);
    appendLe32(header, static_cast<quint32>(sampleRate));
    appendLe32(header, byteRate);
    appendLe16(header, blockAlign);
    appendLe16(header, bitsPerSample);
    appendFourCc(header, "data");
    dataSizeFieldOffset = header.size();
    appendLe32(header, 0);
    waveHeaderBytes = static_cast<quint64>(header.size());
    outputFile.write(header);
}

void SpectrumIqEventRecorder::patchWavHeader() {
    if (activeMode != Mode::ChannelIqS16 || !outputFile.isOpen()) {
        return;
    }
    const quint32 wavSampleRate =
        static_cast<quint32>(std::max(1.0, std::round(recordingSampleRate)));
    constexpr quint32 wavBlockAlign = 4;
    outputFile.flush();
    outputFile.seek(4);
    outputFile.write(le32Bytes(size32(writtenBytes + (waveHeaderBytes > 8 ? waveHeaderBytes - 8 : 0))));
    outputFile.seek(24);
    outputFile.write(le32Bytes(wavSampleRate));
    outputFile.seek(28);
    outputFile.write(le32Bytes(wavSampleRate * wavBlockAlign));
    if (dataSizeFieldOffset >= 0) {
        outputFile.seek(dataSizeFieldOffset);
        outputFile.write(le32Bytes(size32(writtenBytes)));
    }
    outputFile.flush();
}

bool SpectrumIqEventRecorder::writeSidecar() const {
    if (sidecarPath.isEmpty()) {
        return false;
    }
    QFile jsonFile(sidecarPath);
    if (!jsonFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        return false;
    }
    jsonFile.write(QJsonDocument(makeMetadataObject()).toJson(QJsonDocument::Indented));
    return true;
}

QJsonObject SpectrumIqEventRecorder::makeMetadataObject() const {
    QJsonObject root;
    root["app"] = QStringLiteral("FobosAPP");
    root["format"] = activeMode == Mode::ChannelIqS16
                         ? QStringLiteral("channel_iq_wav_s16le")
                         : QStringLiteral("full_iq_raw_s8_interleaved");
    root["recordedAtLocal"] = QDateTime::currentDateTime().toString(Qt::ISODateWithMs);
    root["filePath"] = filePath;
    root["sampleRate"] = recordingSampleRate;
    root["bytesWritten"] = static_cast<double>(writtenBytes);
    root["firstFrameUtcMs"] = static_cast<double>(firstFrameUtcMs);
    root["lastFrameUtcMs"] = static_cast<double>(lastFrameUtcMs);
    root["spectrumFirstFrameUtcMs"] = static_cast<double>(spectrumFirstFrameUtcMs);
    if (firstFrameUtcMs > 0 && spectrumFirstFrameUtcMs > 0) {
        root["spectrumOffsetMs"] = static_cast<double>(firstFrameUtcMs - spectrumFirstFrameUtcMs);
    }
    if (firstFrameUtcMs > 0 && lastFrameUtcMs > firstFrameUtcMs) {
        const double durationSeconds =
            static_cast<double>(lastFrameUtcMs - firstFrameUtcMs) / 1000.0;
        const double bytesPerIqSample =
            activeMode == Mode::ChannelIqS16 ? 4.0 : 2.0;
        const double sampleCount = static_cast<double>(writtenBytes) / bytesPerIqSample;
        root["durationUtcSeconds"] = durationSeconds;
        if (sampleCount > 0.0 && durationSeconds > 0.0) {
            root["effectiveSampleRate"] = sampleCount / durationSeconds;
        }
    }
    root["centerFrequency"] = recordingSettings.centerFrequency;
    root["actualFrequency"] = recordingSettings.actualFrequency;
    root["listeningFrequency"] = recordingSettings.listeningFrequency;
    root["sourceSampleRate"] = recordingSettings.sampleRate;
    root["inputMode"] = recordingSettings.inputMode;
    root["modulationType"] = recordingSettings.modulationType;
    return root;
}
