#ifndef DIGITALDECODER_H
#define DIGITALDECODER_H

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QtGlobal>

#include <vector>

#include "radiosettings.h"

class DigitalDecoder : public QObject {
    Q_OBJECT

public:
    explicit DigitalDecoder(QObject *parent = nullptr);

    void setEnabled(bool enabled);
    bool isEnabled() const;
    void reset();
    void configure(const RadioSettings &settings, int sampleRate = 48000);
    void processPcmFrame(const QByteArray &pcmData, const RadioSettings &settings, int sampleRate = 48000);

signals:
    void textDecoded(const QString &text);
    void statusChanged(const QString &status);

private:
    enum class RttyState {
        Idle,
        ValidateStart,
        Data,
        Stop
    };

    void configureForMode(int modulationType, int sampleRate);
    void resetRttyState();
    void resetFt8State();
    void processFt8Samples(const char *raw, int sampleCount, int sampleRate, QString &decodedText);
    void analyzeFt8Buffer(QString &decodedText, qint64 decodeSlotStartMs, qint64 bufferStartMs);
    bool detectAfskBit(float sample, bool &signalPresent);
    bool detectFskBit(float sample, bool &signalPresent);
    void advanceRttyState(bool markBit, QString &decodedText);
    QString decodeBaudotCode(unsigned int code);
    void updateStatus(const QString &status);

    bool decoderEnabled = true;
    int activeMode = -1;
    int activeSampleRate = 48000;

    double markPhase = 0.0;
    double spacePhase = 0.0;
    double markI = 0.0;
    double markQ = 0.0;
    double spaceI = 0.0;
    double spaceQ = 0.0;
    double detectorAlpha = 0.0;

    RttyState rttyState = RttyState::Idle;
    double samplesPerBit = 48000.0 / 45.45;
    double bitCountdown = 0.0;
    unsigned int rttyCode = 0;
    int rttyBitIndex = 0;
    bool previousRttyBit = true;
    bool rttyLettersShift = true;
    bool rttyLastWasNewline = false;
    bool rttyInvertPolarity = false;
    int rttyNoSignalSamples = 0;
    int rttyBadStopCount = 0;
    int rttyDecodedCount = 0;
    double fskLevel = 0.0;
    double signalQuality = 0.0;
    RadioSettings currentSettings;
    std::vector<float> ft8AudioBuffer;
    int ft8SamplesSinceAnalysis = 0;
    int ft8AnalysisCounter = 0;
    qint64 ft8LastAnalyzedSlot = -1;
    std::vector<double> recentFt8CandidateHz;
    std::vector<int> recentFt8CandidateAnalysis;
    QString lastStatus;
};

#endif // DIGITALDECODER_H
