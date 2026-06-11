#include "dmrdecoder.h"
#include "dmrvocoder.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLibrary>
#include <QMap>
#include <QMessageLogContext>
#include <QRegularExpression>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <vector>

#include "fobos_dmr_voice_backend.h"

constexpr double DMR_AUDIO_RATE = 48000.0;
constexpr double DMR_SYMBOL_RATE = 4800.0;
constexpr double TWO_PI = 6.28318530717958647692;
constexpr double DMR_CHANNEL_CUTOFF_HZ = 9500.0;
constexpr double DMR_FOURFSK_SYMBOL_FILTER_HZ = 6200.0;
constexpr double DMR_FOURFSK_DC_TRACK_HZ = 12.0;
constexpr double DMR_FOURFSK_OUTER_DEVIATION_HZ = 1944.0;
constexpr double DMR_FOURFSK_OUTPUT_OUTER_LEVEL = 0.15;
constexpr const char DMR_SYNTHETIC_MS_VOICE_SYNC[] = "133313311131311113313331";

struct ExpectedValues {
    int colorCode = -1;
    int timeslot = -1;
    int sourceId = -1;
    int targetId = -1;
};

struct ReplayOptions {
    QString filePath;
    QString jsonSummaryPath;
    QString pcmOutPath;
    QString voicePcmOutPath;
    QString syntheticIqWavOutPath;
    QString syntheticPcmWavOutPath;
    ExpectedValues expected;
    bool syntheticIqSelfTest = false;
    bool syntheticVoiceSelfTest = false;
    bool printStatus = false;
    bool printQtDebug = false;
    int syntheticIqSampleRate = 192000;
    int inputSampleRateOverride = 0;
    double fourFskOutputOuterLevel = DMR_FOURFSK_OUTPUT_OUTER_LEVEL;
};

struct WavInfo {
    int channels = 0;
    int sampleRate = 0;
    int bitsPerSample = 0;
    quint64 dataOffset = 0;
    quint64 dataSize = 0;
    QJsonObject metadata;
};

struct ReplaySummary {
    QString filePath;
    QString inputMode;
    int sampleRate = 0;
    int headerSampleRate = 0;
    int decoderSampleRate = 48000;
    int channels = 0;
    int bitsPerSample = 0;
    double fourFskOutputOuterLevel = DMR_FOURFSK_OUTPUT_OUTER_LEVEL;
    double durationSeconds = 0.0;
    double rfFrequencyHz = 0.0;
    int decodedLines = 0;
    int debugDmrLines = 0;
    int lockAcquired = 0;
    int lockLost = 0;
    int ambeFrames = 0;
    int ambePayloads = 0;
    int ambeFecCorrections = 0;
    bool vocoderAvailable = false;
    int vocoderFrames = 0;
    int vocoderErrors = 0;
    int vocoderSamples = 0;
    QMap<int, int> colorCodes;
    QMap<int, int> timeslots;
    QMap<int, int> sources;
    QMap<int, int> targets;
    QStringList lcLines;
    QStringList notableLines;

    void observeLine(const QString &line) {
        const QString trimmed = line.trimmed();
        if (trimmed.isEmpty()) {
            return;
        }

        if (trimmed.contains(QStringLiteral("[DMR"))) {
            ++debugDmrLines;
        }
        if (trimmed.contains(QStringLiteral("lock acquired"))) {
            ++lockAcquired;
        }
        if (trimmed.contains(QStringLiteral("lock lost"))) {
            ++lockLost;
        }
        if (trimmed.contains(QStringLiteral("LC72")) ||
            trimmed.contains(QStringLiteral("voice lc"))) {
            lcLines.append(trimmed);
            while (lcLines.size() > 40) {
                lcLines.removeFirst();
            }
        }
        if (trimmed.contains(QStringLiteral("LC72")) ||
            trimmed.contains(QStringLiteral("lock acquired")) ||
            trimmed.contains(QStringLiteral("strong sync candidate"))) {
            notableLines.append(trimmed);
            while (notableLines.size() > 80) {
                notableLines.removeFirst();
            }
        }

        collectInts(trimmed, QRegularExpression(QStringLiteral("\\bCC\\s+(\\d+)")), colorCodes);
        collectInts(trimmed, QRegularExpression(QStringLiteral("\\bCh\\s+(\\d+)")), timeslots);
        collectInts(trimmed, QRegularExpression(QStringLiteral("\\bTG\\s+(\\d+)")), targets);
        collectInts(trimmed, QRegularExpression(QStringLiteral("\\bSRC\\s+(\\d+)")), sources);
    }

private:
    static void collectInts(const QString &line, const QRegularExpression &rx, QMap<int, int> &histogram) {
        QRegularExpressionMatchIterator it = rx.globalMatch(line);
        while (it.hasNext()) {
            const QRegularExpressionMatch match = it.next();
            bool ok = false;
            const int value = match.captured(1).toInt(&ok);
            if (ok) {
                histogram[value] = histogram.value(value) + 1;
            }
        }
    }
};

struct IqDemodState {
    bool previousValid = false;
    float previousI = 0.0f;
    float previousQ = 0.0f;
    float lowPassI = 0.0f;
    float lowPassQ = 0.0f;
    float audioLowPass = 0.0f;
    float audioLowPass2 = 0.0f;
    float dcEstimate = 0.0f;
    float agcLevel = 0.0001f;
    float previousOutputSample = 0.0f;
    bool previousOutputValid = false;
    double resamplePhase = 0.0;
};

static ReplaySummary *gSummary = nullptr;
static bool gPrintQtDebug = false;

static void qtMessageHandler(QtMsgType type,
                             const QMessageLogContext &context,
                             const QString &message);

static quint16 readLe16(const char *p) {
    return static_cast<quint16>(static_cast<unsigned char>(p[0])) |
           static_cast<quint16>(static_cast<unsigned char>(p[1]) << 8);
}

static quint32 readLe32(const char *p) {
    return static_cast<quint32>(static_cast<unsigned char>(p[0])) |
           (static_cast<quint32>(static_cast<unsigned char>(p[1])) << 8) |
           (static_cast<quint32>(static_cast<unsigned char>(p[2])) << 16) |
           (static_cast<quint32>(static_cast<unsigned char>(p[3])) << 24);
}

static float readPcm16LeNormalized(const char *p) {
    const qint16 value = static_cast<qint16>(readLe16(p));
    return static_cast<float>(value) / 32768.0f;
}

static void appendPcm16Le(QByteArray &buffer, float sample) {
    if (!std::isfinite(sample)) {
        sample = 0.0f;
    }
    const float clamped = (std::clamp)(sample, -1.0f, 1.0f);
    const qint16 value = static_cast<qint16>(std::lrint(clamped * 32767.0f));
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
}

static void appendUInt16Le(QByteArray &buffer, quint16 value) {
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
}

static void appendUInt32Le(QByteArray &buffer, quint32 value) {
    buffer.append(static_cast<char>(value & 0xff));
    buffer.append(static_cast<char>((value >> 8) & 0xff));
    buffer.append(static_cast<char>((value >> 16) & 0xff));
    buffer.append(static_cast<char>((value >> 24) & 0xff));
}

static int jsonInt(const QJsonObject &object, const QString &key, int fallback = -1) {
    const QJsonValue value = object.value(key);
    if (value.isDouble()) {
        return value.toInt(fallback);
    }
    if (value.isString()) {
        bool ok = false;
        const int parsed = value.toString().trimmed().toInt(&ok);
        return ok ? parsed : fallback;
    }
    return fallback;
}

static double jsonDouble(const QJsonObject &object, const QString &key, double fallback = 0.0) {
    const QJsonValue value = object.value(key);
    if (value.isDouble()) {
        return value.toDouble(fallback);
    }
    if (value.isString()) {
        bool ok = false;
        const double parsed = value.toString().trimmed().toDouble(&ok);
        return ok ? parsed : fallback;
    }
    return fallback;
}

static QJsonObject mapToJson(const QMap<int, int> &map) {
    QJsonObject object;
    for (auto it = map.cbegin(); it != map.cend(); ++it) {
        object[QString::number(it.key())] = it.value();
    }
    return object;
}

static QString histogramText(const QMap<int, int> &map) {
    if (map.isEmpty()) {
        return QStringLiteral("-");
    }
    QStringList parts;
    for (auto it = map.cbegin(); it != map.cend(); ++it) {
        parts << QStringLiteral("%1:%2").arg(it.key()).arg(it.value());
    }
    return parts.join(QStringLiteral(", "));
}

static QByteArray demodulateDmrChannelIq(const QByteArray &iqBytes,
                                         int sampleRate,
                                         IqDemodState &state,
                                         double outputOuterLevel) {
    QByteArray pcm;
    if (sampleRate <= 0 || iqBytes.size() < 4) {
        return pcm;
    }

    const int iqSamples = iqBytes.size() / 4;
    pcm.reserve(static_cast<int>(std::ceil(static_cast<double>(iqSamples) *
                                           DMR_AUDIO_RATE /
                                           static_cast<double>(sampleRate))) *
                static_cast<int>(sizeof(qint16)) +
                16);

    const float channelAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * (std::min)(DMR_CHANNEL_CUTOFF_HZ, sampleRate * 0.45) /
                       static_cast<double>(sampleRate)),
        0.000001,
        1.0));
    const float fskAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * (std::min)(DMR_FOURFSK_SYMBOL_FILTER_HZ, sampleRate * 0.42) /
                       static_cast<double>(sampleRate)),
        0.000001,
        1.0));
    const float dcAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * DMR_FOURFSK_DC_TRACK_HZ /
                       static_cast<double>(sampleRate)),
        0.000001,
        0.02));
    const double outputStep = DMR_AUDIO_RATE / static_cast<double>(sampleRate);
    outputOuterLevel = (std::clamp)(outputOuterLevel, 0.05, 2.0);
    const double outputScale =
        outputOuterLevel / DMR_FOURFSK_OUTER_DEVIATION_HZ;

    for (int n = 0; n < iqSamples; ++n) {
        const char *sample = iqBytes.constData() + n * 4;
        float iValue = readPcm16LeNormalized(sample);
        float qValue = readPcm16LeNormalized(sample + 2);
        if (!std::isfinite(iValue)) {
            iValue = 0.0f;
        }
        if (!std::isfinite(qValue)) {
            qValue = 0.0f;
        }

        state.lowPassI += channelAlpha * (iValue - state.lowPassI);
        state.lowPassQ += channelAlpha * (qValue - state.lowPassQ);

        float limitedI = state.lowPassI;
        float limitedQ = state.lowPassQ;
        const float magnitude = std::sqrt(limitedI * limitedI + limitedQ * limitedQ);
        if (magnitude > 0.000001f) {
            const float invMagnitude = 1.0f / magnitude;
            limitedI *= invMagnitude;
            limitedQ *= invMagnitude;
        }

        float demodulated = 0.0f;
        if (state.previousValid) {
            const float discriminatorQ = limitedQ * state.previousI - limitedI * state.previousQ;
            const float discriminatorI = limitedI * state.previousI + limitedQ * state.previousQ;
            const float phase = std::atan2(discriminatorQ, discriminatorI);
            const float instantaneousHz =
                static_cast<float>(static_cast<double>(phase) *
                                   static_cast<double>(sampleRate) /
                                   TWO_PI);
            state.dcEstimate += dcAlpha * (instantaneousHz - state.dcEstimate);
            const float centeredHz = instantaneousHz - state.dcEstimate;
            state.audioLowPass += fskAlpha * (centeredHz - state.audioLowPass);
            state.audioLowPass2 += fskAlpha * (state.audioLowPass - state.audioLowPass2);
            demodulated = static_cast<float>(state.audioLowPass2 * outputScale);
        }
        state.previousI = limitedI;
        state.previousQ = limitedQ;
        state.previousValid = true;

        if (!state.previousOutputValid) {
            state.previousOutputSample = demodulated;
            state.previousOutputValid = true;
            continue;
        }
        const double phaseBefore = state.resamplePhase;
        state.resamplePhase += outputStep;
        while (state.resamplePhase >= 1.0) {
            const double fraction =
                outputStep > 0.0
                    ? (std::clamp)((1.0 - phaseBefore) / outputStep, 0.0, 1.0)
                    : 1.0;
            const float interpolated =
                state.previousOutputSample +
                static_cast<float>(fraction) * (demodulated - state.previousOutputSample);
            appendPcm16Le(pcm, interpolated);
            state.resamplePhase -= 1.0;
        }
        state.previousOutputSample = demodulated;
    }

    return pcm;
}

struct SyntheticDmrStream {
    std::vector<int> symbols;
    std::vector<int> voiceSyncStarts;
    int settleSymbols = 0;
};

struct FourLevelSlicer {
    bool valid = false;
    std::array<float, 4> levels = {};
    float range = 0.0f;
    float minSeparation = 0.0f;
};

struct SyntheticIqSelfTestResult {
    bool valid = false;
    int generatedSymbols = 0;
    int testedSymbols = 0;
    int symbolErrors = 0;
    int bestSamplePhase = 0;
    int bestSymbolShift = 0;
    int bestSyncScore = 0;
    double accuracy = 0.0;
    FourLevelSlicer slicer;
    QString error;
};

static int nextSyntheticDibit(quint32 &state) {
    state = state * 1664525u + 1013904223u;
    return static_cast<int>((state >> 30) & 0x03u);
}

static SyntheticDmrStream buildSyntheticDmrStream() {
    SyntheticDmrStream stream;
    stream.symbols.reserve(5200);

    const int settlePattern[] = {1, 0, 2, 3, 3, 2, 0, 1};
    stream.settleSymbols = 240;
    for (int i = 0; i < stream.settleSymbols; ++i) {
        stream.symbols.push_back(settlePattern[i % 8]);
    }

    quint32 prng = 0x4d524631u;
    constexpr int bursts = 36;
    for (int burst = 0; burst < bursts; ++burst) {
        for (int i = 0; i < 54; ++i) {
            stream.symbols.push_back(nextSyntheticDibit(prng));
        }

        if ((burst % 6) == 0) {
            stream.voiceSyncStarts.push_back(static_cast<int>(stream.symbols.size()));
            for (const char *p = DMR_SYNTHETIC_MS_VOICE_SYNC; *p; ++p) {
                stream.symbols.push_back(*p - '0');
            }
        } else {
            for (int i = 0; i < 24; ++i) {
                stream.symbols.push_back(nextSyntheticDibit(prng));
            }
        }

        for (int i = 0; i < 54; ++i) {
            stream.symbols.push_back(nextSyntheticDibit(prng));
        }
    }

    return stream;
}

static double deviationHzForDmrDibit(int dibit) {
    switch (dibit) {
    case 1:
        return -1944.0;
    case 0:
        return -648.0;
    case 2:
        return 648.0;
    case 3:
    default:
        return 1944.0;
    }
}

static QByteArray generateSyntheticDmrIq(const std::vector<int> &symbols,
                                         int sampleRate) {
    QByteArray iq;
    if (sampleRate <= 0 || symbols.empty()) {
        return iq;
    }

    const double durationSeconds =
        static_cast<double>(symbols.size()) / DMR_SYMBOL_RATE;
    const int sampleCount = static_cast<int>(
        std::ceil(durationSeconds * static_cast<double>(sampleRate)));
    if (sampleCount <= 0) {
        return iq;
    }

    iq.reserve(sampleCount * 4);
    double phase = 0.0;
    constexpr float amplitude = 0.72f;
    int lastSymbolIndex = -1;
    double phaseStep = 0.0;
    for (int sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
        const double t = static_cast<double>(sampleIndex) /
                         static_cast<double>(sampleRate);
        int symbolIndex = static_cast<int>(std::floor(t * DMR_SYMBOL_RATE));
        symbolIndex = (std::clamp)(symbolIndex,
                                   0,
                                   static_cast<int>(symbols.size()) - 1);
        if (symbolIndex != lastSymbolIndex) {
            phaseStep = TWO_PI *
                        deviationHzForDmrDibit(
                            symbols[static_cast<std::size_t>(symbolIndex)]) /
                        static_cast<double>(sampleRate);
            lastSymbolIndex = symbolIndex;
        }
        appendPcm16Le(iq, amplitude * static_cast<float>(std::cos(phase)));
        appendPcm16Le(iq, amplitude * static_cast<float>(std::sin(phase)));
        phase += phaseStep;
        if (phase > TWO_PI || phase < -TWO_PI) {
            phase = std::remainder(phase, TWO_PI);
        }
    }
    return iq;
}

static bool writePcm16Wav(const QString &path,
                          const QByteArray &pcm,
                          int sampleRate,
                          int channels,
                          QString &error) {
    if (path.isEmpty()) {
        return true;
    }
    if (sampleRate <= 0 || channels <= 0) {
        error = QStringLiteral("invalid WAV parameters");
        return false;
    }
    if (static_cast<quint64>(pcm.size()) >
        static_cast<quint64>(std::numeric_limits<quint32>::max()) - 36u) {
        error = QStringLiteral("WAV is too large for RIFF");
        return false;
    }

    QByteArray wav;
    wav.reserve(pcm.size() + 44);
    wav.append("RIFF", 4);
    appendUInt32Le(wav, static_cast<quint32>(36 + pcm.size()));
    wav.append("WAVE", 4);
    wav.append("fmt ", 4);
    appendUInt32Le(wav, 16);
    appendUInt16Le(wav, 1);
    appendUInt16Le(wav, static_cast<quint16>(channels));
    appendUInt32Le(wav, static_cast<quint32>(sampleRate));
    appendUInt32Le(wav, static_cast<quint32>(sampleRate * channels * 2));
    appendUInt16Le(wav, static_cast<quint16>(channels * 2));
    appendUInt16Le(wav, 16);
    wav.append("data", 4);
    appendUInt32Le(wav, static_cast<quint32>(pcm.size()));
    wav.append(pcm);

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        error = file.errorString();
        return false;
    }
    if (file.write(wav) != wav.size()) {
        error = file.errorString();
        return false;
    }
    return true;
}

static std::vector<float> pcm16LeToFloatVector(const QByteArray &pcm) {
    std::vector<float> samples;
    samples.reserve(pcm.size() / 2);
    for (int offset = 0; offset + 1 < pcm.size(); offset += 2) {
        samples.push_back(readPcm16LeNormalized(pcm.constData() + offset));
    }
    return samples;
}

static float averagedSelfTestSymbolSample(const std::vector<float> &samples,
                                          int centerIndex,
                                          int samplesPerSymbol) {
    if (centerIndex < 0 || centerIndex >= static_cast<int>(samples.size())) {
        return 0.0f;
    }

    const int radius = (std::max)(1, (std::min)(2, samplesPerSymbol / 5));
    double sum = 0.0;
    double weightSum = 0.0;
    for (int offset = -radius; offset <= radius; ++offset) {
        const int index = centerIndex + offset;
        if (index < 0 || index >= static_cast<int>(samples.size())) {
            continue;
        }
        const double weight = 1.0 / (1.0 + std::abs(offset));
        sum += weight * samples[static_cast<std::size_t>(index)];
        weightSum += weight;
    }
    return weightSum > 0.0
               ? static_cast<float>(sum / weightSum)
               : samples[static_cast<std::size_t>(centerIndex)];
}

static FourLevelSlicer buildFourLevelSlicer(const std::vector<float> &symbolSamples) {
    FourLevelSlicer slicer;
    std::vector<float> sorted;
    sorted.reserve(symbolSamples.size());
    for (const float sample : symbolSamples) {
        if (std::isfinite(sample)) {
            sorted.push_back(sample);
        }
    }
    if (sorted.size() < 24) {
        return slicer;
    }

    std::sort(sorted.begin(), sorted.end());
    const std::size_t n = sorted.size();
    const auto binMean = [&sorted, n](int bin) {
        std::size_t begin = static_cast<std::size_t>(bin) * n / 4;
        std::size_t end = static_cast<std::size_t>(bin + 1) * n / 4;
        begin = (std::min)(begin, n - 1);
        end = (std::max)(end, begin + 1);
        end = (std::min)(end, n);
        double sum = 0.0;
        for (std::size_t i = begin; i < end; ++i) {
            sum += sorted[i];
        }
        return static_cast<float>(sum / static_cast<double>(end - begin));
    };

    std::array<float, 4> levels = {
        binMean(0),
        binMean(1),
        binMean(2),
        binMean(3),
    };
    std::sort(levels.begin(), levels.end());

    for (int iteration = 0; iteration < 8; ++iteration) {
        std::array<double, 4> sums = {};
        std::array<int, 4> counts = {};
        for (const float sample : sorted) {
            int nearest = 0;
            float nearestDistance = std::abs(sample - levels[0]);
            for (int i = 1; i < 4; ++i) {
                const float distance = std::abs(sample - levels[static_cast<std::size_t>(i)]);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearest = i;
                }
            }
            sums[static_cast<std::size_t>(nearest)] += sample;
            ++counts[static_cast<std::size_t>(nearest)];
        }

        for (int i = 0; i < 4; ++i) {
            if (counts[static_cast<std::size_t>(i)] > 0) {
                levels[static_cast<std::size_t>(i)] =
                    static_cast<float>(sums[static_cast<std::size_t>(i)] /
                                       counts[static_cast<std::size_t>(i)]);
            } else {
                const std::size_t quantile =
                    (std::min)(n - 1, static_cast<std::size_t>((2 * i + 1) * n / 8));
                levels[static_cast<std::size_t>(i)] = sorted[quantile];
            }
        }
        std::sort(levels.begin(), levels.end());
    }

    slicer.levels = levels;
    slicer.range = levels[3] - levels[0];
    slicer.minSeparation = (std::min)(
        (std::min)(levels[1] - levels[0], levels[2] - levels[1]),
        levels[3] - levels[2]);
    slicer.valid = slicer.range >= 0.004f &&
                   slicer.minSeparation >= (std::max)(0.0005f, slicer.range * 0.035f);
    return slicer;
}

static int dibitFromFourLevel(const FourLevelSlicer &slicer, float sample) {
    int nearest = 0;
    float nearestDistance = std::abs(sample - slicer.levels[0]);
    for (int i = 1; i < 4; ++i) {
        const float distance = std::abs(sample - slicer.levels[static_cast<std::size_t>(i)]);
        if (distance < nearestDistance) {
            nearestDistance = distance;
            nearest = i;
        }
    }

    switch (nearest) {
    case 0:
        return 1;
    case 1:
        return 0;
    case 2:
        return 2;
    case 3:
    default:
        return 3;
    }
}

static SyntheticIqSelfTestResult analyzeSyntheticIqPcm(const QByteArray &pcm,
                                                       const SyntheticDmrStream &stream) {
    SyntheticIqSelfTestResult best;
    best.generatedSymbols = static_cast<int>(stream.symbols.size());

    const std::vector<float> samples = pcm16LeToFloatVector(pcm);
    const int samplesPerSymbol =
        static_cast<int>(std::lround(DMR_AUDIO_RATE / DMR_SYMBOL_RATE));
    if (samplesPerSymbol <= 0 || samples.size() < 200) {
        best.error = QStringLiteral("not enough demodulated PCM samples");
        return best;
    }

    const int compareStart = stream.settleSymbols + 24;
    const int maxPcmSymbols = static_cast<int>(samples.size()) / samplesPerSymbol;
    const int maxShift = 24;

    for (int phase = 0; phase < samplesPerSymbol; ++phase) {
        for (int shift = -maxShift; shift <= maxShift; ++shift) {
            std::vector<float> symbolSamples;
            symbolSamples.reserve(stream.symbols.size());
            std::vector<int> truth;
            truth.reserve(stream.symbols.size());

            for (int generatedIndex = compareStart;
                 generatedIndex < static_cast<int>(stream.symbols.size());
                 ++generatedIndex) {
                const int pcmSymbolIndex = generatedIndex + shift;
                if (pcmSymbolIndex < 0 || pcmSymbolIndex >= maxPcmSymbols) {
                    continue;
                }
                const int sampleIndex = pcmSymbolIndex * samplesPerSymbol + phase;
                if (sampleIndex < 0 || sampleIndex >= static_cast<int>(samples.size())) {
                    continue;
                }
                symbolSamples.push_back(averagedSelfTestSymbolSample(samples,
                                                                      sampleIndex,
                                                                      samplesPerSymbol));
                truth.push_back(stream.symbols[static_cast<std::size_t>(generatedIndex)]);
            }

            if (symbolSamples.size() < 256) {
                continue;
            }

            const FourLevelSlicer slicer = buildFourLevelSlicer(symbolSamples);
            if (!slicer.valid) {
                continue;
            }

            int errors = 0;
            for (std::size_t i = 0; i < symbolSamples.size(); ++i) {
                const int decoded = dibitFromFourLevel(slicer, symbolSamples[i]);
                if (decoded != truth[i]) {
                    ++errors;
                }
            }

            const bool prefer =
                !best.valid ||
                errors < best.symbolErrors ||
                (errors == best.symbolErrors &&
                 static_cast<int>(symbolSamples.size()) > best.testedSymbols) ||
                (errors == best.symbolErrors &&
                 static_cast<int>(symbolSamples.size()) == best.testedSymbols &&
                 std::abs(phase - samplesPerSymbol / 2) <
                     std::abs(best.bestSamplePhase - samplesPerSymbol / 2));

            if (prefer) {
                best.valid = true;
                best.testedSymbols = static_cast<int>(symbolSamples.size());
                best.symbolErrors = errors;
                best.bestSamplePhase = phase;
                best.bestSymbolShift = shift;
                best.accuracy = 1.0 - static_cast<double>(errors) /
                                          static_cast<double>(symbolSamples.size());
                best.slicer = slicer;
            }
        }
    }

    if (!best.valid) {
        best.error = QStringLiteral("no stable four-level slicer was found");
        return best;
    }

    for (const int syncStart : stream.voiceSyncStarts) {
        int score = 0;
        for (int i = 0; DMR_SYNTHETIC_MS_VOICE_SYNC[i] != '\0'; ++i) {
            const int generatedIndex = syncStart + i;
            const int pcmSymbolIndex = generatedIndex + best.bestSymbolShift;
            const int sampleIndex = pcmSymbolIndex * samplesPerSymbol + best.bestSamplePhase;
            if (sampleIndex < 0 || sampleIndex >= static_cast<int>(samples.size())) {
                continue;
            }
            const float sample = averagedSelfTestSymbolSample(samples,
                                                              sampleIndex,
                                                              samplesPerSymbol);
            const int decoded = dibitFromFourLevel(best.slicer, sample);
            if (decoded == DMR_SYNTHETIC_MS_VOICE_SYNC[i] - '0') {
                ++score;
            }
        }
        best.bestSyncScore = (std::max)(best.bestSyncScore, score);
    }

    return best;
}

static bool runSyntheticIqSelfTest(const ReplayOptions &options) {
    QTextStream out(stdout);
    QTextStream err(stderr);

    if (options.syntheticIqSampleRate <= 0) {
        err << "dmr_lab_replay: --synthetic-rate must be positive\n";
        return false;
    }

    const SyntheticDmrStream stream = buildSyntheticDmrStream();
    const QByteArray iq = generateSyntheticDmrIq(stream.symbols,
                                                options.syntheticIqSampleRate);
    IqDemodState demodState;
    const QByteArray pcm = demodulateDmrChannelIq(iq,
                                                  options.syntheticIqSampleRate,
                                                  demodState,
                                                  options.fourFskOutputOuterLevel);
    const SyntheticIqSelfTestResult result = analyzeSyntheticIqPcm(pcm, stream);

    QString writeError;
    if (!writePcm16Wav(options.syntheticIqWavOutPath,
                       iq,
                       options.syntheticIqSampleRate,
                       2,
                       writeError)) {
        err << "dmr_lab_replay: failed to write synthetic IQ WAV: "
            << writeError << '\n';
        return false;
    }
    if (!writePcm16Wav(options.syntheticPcmWavOutPath,
                       pcm,
                       static_cast<int>(DMR_AUDIO_RATE),
                       1,
                       writeError)) {
        err << "dmr_lab_replay: failed to write synthetic PCM WAV: "
            << writeError << '\n';
        return false;
    }

    out << "[synthetic IQ self-test]\n"
        << "  iq: " << options.syntheticIqSampleRate << " Hz, stereo PCM16, "
        << QString::number(static_cast<double>(iq.size()) /
                           (options.syntheticIqSampleRate * 4.0), 'f', 3)
        << " s\n"
        << "  4FSK output outer level: "
        << QString::number(options.fourFskOutputOuterLevel, 'f', 3) << '\n'
        << "  decoder input: " << static_cast<int>(DMR_AUDIO_RATE)
        << " Hz mono PCM16, " << (pcm.size() / 2) << " samples\n";

    if (!result.valid) {
        out << "  result: FAIL, " << result.error << '\n';
        out.flush();
        return false;
    }

    out << "  generated symbols: " << result.generatedSymbols << '\n'
        << "  tested symbols: " << result.testedSymbols << '\n'
        << "  symbol errors: " << result.symbolErrors
        << " (" << QString::number(result.accuracy * 100.0, 'f', 3) << "% correct)\n"
        << "  best alignment: PCM phase " << result.bestSamplePhase
        << "/10, symbol shift " << result.bestSymbolShift << '\n'
        << "  levels: "
        << QString::number(result.slicer.levels[0], 'f', 4) << ", "
        << QString::number(result.slicer.levels[1], 'f', 4) << ", "
        << QString::number(result.slicer.levels[2], 'f', 4) << ", "
        << QString::number(result.slicer.levels[3], 'f', 4) << '\n'
        << "  level range: " << QString::number(result.slicer.range, 'f', 4)
        << ", min separation: "
        << QString::number(result.slicer.minSeparation, 'f', 4) << '\n'
        << "  best MS voice sync score: " << result.bestSyncScore << "/24\n";

    if (!options.syntheticIqWavOutPath.isEmpty()) {
        out << "  wrote IQ WAV: " << options.syntheticIqWavOutPath << '\n';
    }
    if (!options.syntheticPcmWavOutPath.isEmpty()) {
        out << "  wrote PCM WAV: " << options.syntheticPcmWavOutPath << '\n';
    }

    const bool pass = result.accuracy >= 0.999 && result.bestSyncScore >= 24;
    out << "  result: " << (pass ? "PASS" : "FAIL") << '\n';
    out.flush();
    return pass;
}

struct ReplayAmbeFecDecodeResult {
    bool decoded = false;
    QString payloadHex;
    int correctedErrors = 0;
};

struct SyntheticVoiceBackend {
    using GetInfoFn = const fobos_dmr_voice_backend_info *(*)();
    using CreateFn = fobos_dmr_voice_context *(*)();
    using DestroyFn = void (*)(fobos_dmr_voice_context *);
    using ResetFn = int (*)(fobos_dmr_voice_context *);
    using LastErrorFn = const char *(*)(fobos_dmr_voice_context *);
    using EncodeCanonicalFn = int (*)(fobos_dmr_voice_context *,
                                      const std::int16_t *,
                                      std::uint8_t *);
    using DecodeCanonicalFn = int (*)(fobos_dmr_voice_context *,
                                      const std::uint8_t *,
                                      std::int16_t *,
                                      int *);

    explicit SyntheticVoiceBackend(const QString &path)
        : library(path) {}

    ~SyntheticVoiceBackend() {
        if (context && destroy) {
            destroy(context);
        }
        context = nullptr;
        library.unload();
    }

    template <typename Fn>
    bool resolve(Fn &fn, const char *name) {
        fn = reinterpret_cast<Fn>(library.resolve(name));
        return fn != nullptr;
    }

    bool load(QString &error) {
        if (!library.load()) {
            error = library.errorString();
            return false;
        }

        const bool ok =
            resolve(getInfo, "fobos_dmr_voice_get_backend_info") &&
            resolve(create, "fobos_dmr_voice_create") &&
            resolve(destroy, "fobos_dmr_voice_destroy") &&
            resolve(reset, "fobos_dmr_voice_reset") &&
            resolve(lastError, "fobos_dmr_voice_get_last_error") &&
            resolve(encodeCanonical, "fobos_dmr_voice_encode_canonical_ambe_frame72") &&
            resolve(decodeCanonical, "fobos_dmr_voice_decode_canonical_ambe_frame72");
        if (!ok) {
            error = QStringLiteral("missing required Fobos DMR voice ABI symbol");
            library.unload();
            return false;
        }

        info = getInfo();
        if (!info || info->abi_version != FOBOS_DMR_VOICE_ABI_VERSION) {
            error = QStringLiteral("incompatible Fobos DMR voice ABI");
            library.unload();
            return false;
        }
        if ((info->capability_flags & FOBOS_DMR_VOICE_CAP_ENCODE_CANONICAL_AMBE_FRAME72) == 0) {
            error = QStringLiteral("backend does not encode canonical AMBE frames");
            library.unload();
            return false;
        }

        context = create();
        if (!context) {
            error = QStringLiteral("backend context creation failed");
            library.unload();
            return false;
        }
        if (reset) {
            reset(context);
        }
        return true;
    }

    QString displayName() const {
        if (info && info->display_name) {
            return QString::fromUtf8(info->display_name);
        }
        return QFileInfo(library.fileName()).fileName();
    }

    QString lastErrorText() const {
        if (context && lastError) {
            const char *message = lastError(context);
            if (message && *message) {
                return QString::fromUtf8(message);
            }
        }
        return QString();
    }

    QLibrary library;
    const fobos_dmr_voice_backend_info *info = nullptr;
    fobos_dmr_voice_context *context = nullptr;
    GetInfoFn getInfo = nullptr;
    CreateFn create = nullptr;
    DestroyFn destroy = nullptr;
    ResetFn reset = nullptr;
    LastErrorFn lastError = nullptr;
    EncodeCanonicalFn encodeCanonical = nullptr;
    DecodeCanonicalFn decodeCanonical = nullptr;
};

static std::unique_ptr<SyntheticVoiceBackend> loadSyntheticVoiceEncoder(QString &error) {
    QStringList searchDirs;
    const auto addDir = [&searchDirs](const QString &path) {
        const QString clean = QDir::cleanPath(path);
        if (!clean.isEmpty() && !searchDirs.contains(clean)) {
            searchDirs << clean;
        }
    };

    const QDir appDir(QCoreApplication::applicationDirPath());
    const QDir currentDir(QDir::currentPath());
    addDir(appDir.absoluteFilePath(QStringLiteral("dmr_voice_backends")));
    addDir(appDir.absoluteFilePath(QStringLiteral("../fobos-dmr-voice-backend-gpl")));
    addDir(appDir.absoluteFilePath(QStringLiteral("../fobos-dmr-voice-backend-gpl/Release")));
    addDir(appDir.absoluteFilePath(QStringLiteral("../../build/fobos-dmr-voice-backend-gpl-vs/Release")));
    addDir(currentDir.absoluteFilePath(QStringLiteral("dmr_voice_backends")));
    addDir(currentDir.absoluteFilePath(QStringLiteral("release/bin/dmr_voice_backends")));
    addDir(currentDir.absoluteFilePath(QStringLiteral("build/fobos-dmr-voice-backend-gpl")));
    addDir(currentDir.absoluteFilePath(QStringLiteral("build/fobos-dmr-voice-backend-gpl/Release")));
    addDir(currentDir.absoluteFilePath(QStringLiteral("build/fobos-dmr-voice-backend-gpl-vs/Release")));

#ifdef Q_OS_WIN
    const QStringList patterns = {QStringLiteral("fobos_dmr_voice_*.dll")};
#elif defined(Q_OS_MACOS)
    const QStringList patterns = {QStringLiteral("libfobos_dmr_voice_*.dylib"),
                                  QStringLiteral("fobos_dmr_voice_*.dylib")};
#else
    const QStringList patterns = {QStringLiteral("libfobos_dmr_voice_*.so"),
                                  QStringLiteral("fobos_dmr_voice_*.so")};
#endif

    QStringList loadErrors;
    QSet<QString> seen;
    for (const QString &dirPath : searchDirs) {
        QDir dir(dirPath);
        if (!dir.exists()) {
            continue;
        }
        const QFileInfoList files = dir.entryInfoList(patterns, QDir::Files, QDir::Name);
        for (const QFileInfo &file : files) {
            const QString canonical = file.canonicalFilePath();
            if (canonical.isEmpty() || seen.contains(canonical)) {
                continue;
            }
            seen.insert(canonical);

            std::unique_ptr<SyntheticVoiceBackend> backend(new SyntheticVoiceBackend(canonical));
            QString loadError;
            if (backend->load(loadError)) {
                return backend;
            }
            loadErrors << QStringLiteral("%1: %2").arg(canonical, loadError);
        }
    }

    error = loadErrors.isEmpty()
                ? QStringLiteral("no Fobos DMR voice backend DLL was found")
                : loadErrors.join(QStringLiteral("; "));
    return nullptr;
}

static int replayHexNibbleValue(QChar ch) {
    const ushort value = ch.unicode();
    if (value >= '0' && value <= '9') {
        return static_cast<int>(value - '0');
    }
    if (value >= 'A' && value <= 'F') {
        return static_cast<int>(10 + value - 'A');
    }
    if (value >= 'a' && value <= 'f') {
        return static_cast<int>(10 + value - 'a');
    }
    return -1;
}

static QString replayBitsToHex(const bool *bits, int bitCount) {
    QString out;
    out.reserve((bitCount + 3) / 4);
    for (int i = 0; i < bitCount; i += 4) {
        int value = 0;
        for (int bit = 0; bit < 4; ++bit) {
            const int index = i + bit;
            value = (value << 1) | (index < bitCount && bits[index] ? 1 : 0);
        }
        out.append(QLatin1Char(value < 10 ? static_cast<char>('0' + value)
                                           : static_cast<char>('A' + value - 10)));
    }
    return out;
}

static bool replayHexToBits(const QString &hex, int bitCount, bool *bits) {
    if (hex.size() < (bitCount + 3) / 4) {
        return false;
    }
    for (int bitIndex = 0; bitIndex < bitCount; ++bitIndex) {
        const int nibble = replayHexNibbleValue(hex.at(bitIndex / 4));
        if (nibble < 0) {
            return false;
        }
        bits[bitIndex] = ((nibble >> (3 - (bitIndex % 4))) & 0x1) != 0;
    }
    return true;
}

static int replayPopcountBits(quint32 value) {
    int count = 0;
    while (value != 0) {
        count += static_cast<int>(value & 0x1U);
        value >>= 1;
    }
    return count;
}

static quint16 replayAmbeGolayParity(quint16 data) {
    static constexpr quint16 generator[12] = {
        0x63a, 0x31d, 0x7b4, 0x3da, 0x1ed, 0x6cc,
        0x366, 0x1b3, 0x6e3, 0x54b, 0x49f, 0x475
    };

    quint16 parity = 0;
    for (int i = 0; i < 12; ++i) {
        if ((data & (1U << (11 - i))) != 0) {
            parity ^= generator[i];
        }
    }
    return static_cast<quint16>(parity & 0x07ffU);
}

static const std::array<quint16, 2048> &replayAmbeGolaySyndromeTable() {
    static const std::array<quint16, 2048> table = []() {
        std::array<quint16, 2048> out = {};
        for (quint32 bit0 = 0; bit0 < 23; ++bit0) {
            const quint32 e0 = 1U << bit0;
            const quint16 de0 = static_cast<quint16>((e0 >> 11) & 0x0fffU);
            const quint16 pe0 = static_cast<quint16>(e0 & 0x07ffU);
            out[replayAmbeGolayParity(de0) ^ pe0] = de0;
            for (quint32 bit1 = bit0 + 1; bit1 < 23; ++bit1) {
                const quint32 e1 = e0 | (1U << bit1);
                const quint16 de1 = static_cast<quint16>((e1 >> 11) & 0x0fffU);
                const quint16 pe1 = static_cast<quint16>(e1 & 0x07ffU);
                out[replayAmbeGolayParity(de1) ^ pe1] = de1;
                for (quint32 bit2 = bit1 + 1; bit2 < 23; ++bit2) {
                    const quint32 e2 = e1 | (1U << bit2);
                    const quint16 de2 = static_cast<quint16>((e2 >> 11) & 0x0fffU);
                    const quint16 pe2 = static_cast<quint16>(e2 & 0x07ffU);
                    out[replayAmbeGolayParity(de2) ^ pe2] = de2;
                }
            }
        }
        out[0] = 0;
        return out;
    }();
    return table;
}

struct ReplayGolayDecodeResult {
    quint16 data = 0;
    int correctedErrors = 0;
};

static ReplayGolayDecodeResult replayAmbeGolayDecode2312(quint32 codeword) {
    const quint16 data = static_cast<quint16>((codeword >> 11) & 0x0fffU);
    const quint16 parity = static_cast<quint16>(codeword & 0x07ffU);
    const quint16 syndrome = static_cast<quint16>(replayAmbeGolayParity(data) ^ parity);
    const quint16 correctedData =
        static_cast<quint16>(data ^ replayAmbeGolaySyndromeTable()[syndrome]);
    return {correctedData, replayPopcountBits(static_cast<quint32>(data ^ correctedData))};
}

static std::array<bool, 24> replayAmbeC1Keystream(quint16 c0Data) {
    std::array<bool, 24> stream = {};
    quint32 previous = static_cast<quint32>(16U * (c0Data & 0x0fffU));
    for (int i = 1; i < 24; ++i) {
        previous = (173U * previous + 13849U) & 0xffffU;
        stream[static_cast<std::size_t>(i)] = (previous >> 15) != 0;
    }
    return stream;
}

static quint32 replayAmbeC1PrngMask23(quint16 c0Data) {
    const std::array<bool, 24> stream = replayAmbeC1Keystream(c0Data);
    quint32 mask = 0;
    for (int i = 1; i <= 23; ++i) {
        if (stream[static_cast<std::size_t>(i)]) {
            mask |= 1U << (23 - i);
        }
    }
    return mask;
}

static bool replayCanonicalBit(const std::array<std::uint8_t, 9> &frame, int bitIndex) {
    return ((frame[static_cast<std::size_t>(bitIndex / 8)] >>
             (7 - (bitIndex % 8))) & 0x1U) != 0;
}

static QString canonicalAmbeFrameToPayloadHex(const std::array<std::uint8_t, 9> &frame) {
    quint32 a = 0;
    for (int i = 0; i < 24; ++i) {
        if (replayCanonicalBit(frame, i)) {
            a |= 0x800000U >> i;
        }
    }
    const ReplayGolayDecodeResult c0 = replayAmbeGolayDecode2312(a >> 1);

    quint32 b = 0;
    for (int i = 0; i < 23; ++i) {
        if (replayCanonicalBit(frame, 24 + i)) {
            b |= 0x400000U >> i;
        }
    }
    const quint32 descrambledB = b ^ replayAmbeC1PrngMask23(c0.data);
    const ReplayGolayDecodeResult c1 = replayAmbeGolayDecode2312(descrambledB);

    std::array<bool, 56> payload = {};
    for (int i = 0; i < 12; ++i) {
        payload[static_cast<std::size_t>(i)] = ((c0.data >> (11 - i)) & 0x1U) != 0;
        payload[static_cast<std::size_t>(12 + i)] = ((c1.data >> (11 - i)) & 0x1U) != 0;
    }
    for (int i = 0; i < 25; ++i) {
        payload[static_cast<std::size_t>(24 + i)] = replayCanonicalBit(frame, 47 + i);
    }
    return replayBitsToHex(payload.data(), static_cast<int>(payload.size()));
}

static ReplayAmbeFecDecodeResult replayDecodeDmrAmbeFecPayload(const QString &frameHex) {
    ReplayAmbeFecDecodeResult result;
    if (frameHex.size() != 18) {
        return result;
    }

    std::array<bool, 72> frameBits = {};
    if (!replayHexToBits(frameHex, 72, frameBits.data())) {
        return result;
    }

    static constexpr int rW[36] = {
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 2,
        0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2
    };
    static constexpr int rX[36] = {
        23, 10, 22, 9, 21, 8, 20, 7, 19, 6, 18, 5,
        17, 4, 16, 3, 15, 2, 14, 1, 13, 0, 12, 10,
        11, 9, 10, 8, 9, 7, 8, 6, 7, 5, 6, 4
    };
    static constexpr int rY[36] = {
        0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 3,
        1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3,
        1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3
    };
    static constexpr int rZ[36] = {
        5, 3, 4, 2, 3, 1, 2, 0, 1, 13, 0, 12,
        22, 11, 21, 10, 20, 9, 19, 8, 18, 7, 17, 6,
        16, 5, 15, 4, 14, 3, 13, 2, 12, 1, 11, 0
    };

    std::array<std::array<bool, 24>, 4> frame = {};
    for (int i = 0; i < 36; ++i) {
        frame[static_cast<std::size_t>(rW[i])][static_cast<std::size_t>(rX[i])] =
            frameBits[static_cast<std::size_t>(2 * i)];
        frame[static_cast<std::size_t>(rY[i])][static_cast<std::size_t>(rZ[i])] =
            frameBits[static_cast<std::size_t>(2 * i + 1)];
    }

    quint32 c0Codeword = 0;
    for (int j = 0; j < 23; ++j) {
        c0Codeword |= static_cast<quint32>(frame[0][static_cast<std::size_t>(j + 1)] ? 1U : 0U) << j;
    }
    const ReplayGolayDecodeResult c0 = replayAmbeGolayDecode2312(c0Codeword);

    const std::array<bool, 24> c1Keystream = replayAmbeC1Keystream(c0.data);
    quint32 c1Codeword = 0;
    for (int j = 0; j < 23; ++j) {
        const bool descrambled =
            frame[1][static_cast<std::size_t>(j)] ^ c1Keystream[static_cast<std::size_t>(23 - j)];
        c1Codeword |= static_cast<quint32>(descrambled ? 1U : 0U) << j;
    }
    const ReplayGolayDecodeResult c1 = replayAmbeGolayDecode2312(c1Codeword);

    std::array<bool, 56> payload = {};
    for (int i = 0; i < 12; ++i) {
        payload[static_cast<std::size_t>(i)] = ((c0.data >> (11 - i)) & 0x1U) != 0;
        payload[static_cast<std::size_t>(12 + i)] = ((c1.data >> (11 - i)) & 0x1U) != 0;
    }
    for (int i = 0; i < 11; ++i) {
        payload[static_cast<std::size_t>(24 + i)] =
            frame[2][static_cast<std::size_t>(10 - i)];
    }
    for (int i = 0; i < 14; ++i) {
        payload[static_cast<std::size_t>(35 + i)] =
            frame[3][static_cast<std::size_t>(13 - i)];
    }

    result.decoded = true;
    result.payloadHex = replayBitsToHex(payload.data(), static_cast<int>(payload.size()));
    result.correctedErrors = c0.correctedErrors + c1.correctedErrors;
    return result;
}

static QString payload49ToDmrOnAirFrameHex(const QString &payloadHex) {
    std::array<bool, 49> payload = {};
    if (!replayHexToBits(payloadHex, static_cast<int>(payload.size()), payload.data())) {
        return QString();
    }

    quint16 c0Data = 0;
    quint16 c1Data = 0;
    for (int i = 0; i < 12; ++i) {
        c0Data = static_cast<quint16>((c0Data << 1) |
                                      (payload[static_cast<std::size_t>(i)] ? 1 : 0));
        c1Data = static_cast<quint16>((c1Data << 1) |
                                      (payload[static_cast<std::size_t>(12 + i)] ? 1 : 0));
    }

    std::array<std::array<bool, 24>, 4> frame = {};
    const quint32 c0Codeword =
        (static_cast<quint32>(c0Data) << 11) | replayAmbeGolayParity(c0Data);
    for (int j = 0; j < 23; ++j) {
        frame[0][static_cast<std::size_t>(j + 1)] = ((c0Codeword >> j) & 0x1U) != 0;
    }
    int c0Ones = 0;
    for (int j = 1; j < 24; ++j) {
        c0Ones += frame[0][static_cast<std::size_t>(j)] ? 1 : 0;
    }
    frame[0][0] = (c0Ones & 1) != 0;

    const quint32 c1Codeword =
        (static_cast<quint32>(c1Data) << 11) | replayAmbeGolayParity(c1Data);
    const std::array<bool, 24> c1Keystream = replayAmbeC1Keystream(c0Data);
    for (int j = 0; j < 23; ++j) {
        const bool encodedBit = ((c1Codeword >> j) & 0x1U) != 0;
        frame[1][static_cast<std::size_t>(j)] =
            encodedBit ^ c1Keystream[static_cast<std::size_t>(23 - j)];
    }

    for (int i = 0; i < 11; ++i) {
        frame[2][static_cast<std::size_t>(10 - i)] =
            payload[static_cast<std::size_t>(24 + i)];
    }
    for (int i = 0; i < 14; ++i) {
        frame[3][static_cast<std::size_t>(13 - i)] =
            payload[static_cast<std::size_t>(35 + i)];
    }

    static constexpr int rW[36] = {
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 2,
        0, 2, 0, 2, 0, 2, 0, 2, 0, 2, 0, 2
    };
    static constexpr int rX[36] = {
        23, 10, 22, 9, 21, 8, 20, 7, 19, 6, 18, 5,
        17, 4, 16, 3, 15, 2, 14, 1, 13, 0, 12, 10,
        11, 9, 10, 8, 9, 7, 8, 6, 7, 5, 6, 4
    };
    static constexpr int rY[36] = {
        0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 3,
        1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3,
        1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3
    };
    static constexpr int rZ[36] = {
        5, 3, 4, 2, 3, 1, 2, 0, 1, 13, 0, 12,
        22, 11, 21, 10, 20, 9, 19, 8, 18, 7, 17, 6,
        16, 5, 15, 4, 14, 3, 13, 2, 12, 1, 11, 0
    };

    std::array<bool, 72> rawBits = {};
    for (int i = 0; i < 36; ++i) {
        rawBits[static_cast<std::size_t>(2 * i)] =
            frame[static_cast<std::size_t>(rW[i])][static_cast<std::size_t>(rX[i])];
        rawBits[static_cast<std::size_t>(2 * i + 1)] =
            frame[static_cast<std::size_t>(rY[i])][static_cast<std::size_t>(rZ[i])];
    }
    return replayBitsToHex(rawBits.data(), static_cast<int>(rawBits.size()));
}

static int etsiDibitForBits(bool bit0, bool bit1) {
    if (!bit0 && !bit1) {
        return 2;
    }
    if (!bit0 && bit1) {
        return 3;
    }
    if (bit0 && !bit1) {
        return 0;
    }
    return 1;
}

static bool appendDmrFrameHexDibits(const QString &frameHex, std::vector<int> &dibits) {
    std::array<bool, 72> bits = {};
    if (!replayHexToBits(frameHex, static_cast<int>(bits.size()), bits.data())) {
        return false;
    }
    for (int i = 0; i < 36; ++i) {
        dibits.push_back(etsiDibitForBits(bits[static_cast<std::size_t>(2 * i)],
                                          bits[static_cast<std::size_t>(2 * i + 1)]));
    }
    return true;
}

static SyntheticDmrStream buildSyntheticDmrVoiceStream(const std::vector<QString> &onAirFrames) {
    SyntheticDmrStream stream;
    const int bursts = static_cast<int>(onAirFrames.size()) / 3;
    stream.symbols.reserve(360 + bursts * 288);

    quint32 prng = 0x44565231u;
    stream.settleSymbols = 288;
    for (int i = 0; i < stream.settleSymbols; ++i) {
        stream.symbols.push_back(nextSyntheticDibit(prng));
    }

    int nextBurstStart = static_cast<int>(stream.symbols.size());
    for (int burst = 0; burst < bursts; ++burst) {
        while (static_cast<int>(stream.symbols.size()) < nextBurstStart) {
            stream.symbols.push_back(nextSyntheticDibit(prng));
        }

        std::vector<int> payloadDibits;
        payloadDibits.reserve(108);
        for (int i = 0; i < 3; ++i) {
            appendDmrFrameHexDibits(onAirFrames[static_cast<std::size_t>(burst * 3 + i)],
                                    payloadDibits);
        }
        if (payloadDibits.size() != 108) {
            continue;
        }

        for (int i = 0; i < 54; ++i) {
            stream.symbols.push_back(payloadDibits[static_cast<std::size_t>(i)]);
        }

        const int syncStart = static_cast<int>(stream.symbols.size());
        if ((burst % 6) == 0) {
            stream.voiceSyncStarts.push_back(syncStart);
            for (const char *p = DMR_SYNTHETIC_MS_VOICE_SYNC; *p; ++p) {
                stream.symbols.push_back(*p - '0');
            }
        } else {
            for (int i = 0; i < 24; ++i) {
                stream.symbols.push_back(nextSyntheticDibit(prng));
            }
        }

        for (int i = 54; i < 108; ++i) {
            stream.symbols.push_back(payloadDibits[static_cast<std::size_t>(i)]);
        }

        nextBurstStart += 288;
    }

    for (int i = 0; i < 360; ++i) {
        stream.symbols.push_back(nextSyntheticDibit(prng));
    }
    return stream;
}

static double pcm16Rms(const QByteArray &pcm) {
    if (pcm.size() < 2) {
        return 0.0;
    }
    double sumSquares = 0.0;
    int samples = 0;
    for (int offset = 0; offset + 1 < pcm.size(); offset += 2) {
        const double sample = static_cast<double>(static_cast<qint16>(
            readLe16(pcm.constData() + offset)));
        sumSquares += sample * sample;
        ++samples;
    }
    return samples > 0 ? std::sqrt(sumSquares / static_cast<double>(samples)) : 0.0;
}

static QByteArray appendPcm8kFrameToBytes(const std::array<std::int16_t, 160> &frame) {
    QByteArray pcm;
    pcm.reserve(static_cast<int>(frame.size() * sizeof(qint16)));
    for (const std::int16_t sample : frame) {
        pcm.append(static_cast<char>(sample & 0xff));
        pcm.append(static_cast<char>((sample >> 8) & 0xff));
    }
    return pcm;
}

static bool runSyntheticVoiceSelfTest(const ReplayOptions &options) {
    QTextStream out(stdout);
    QTextStream err(stderr);

    if (options.syntheticIqSampleRate <= 0) {
        err << "dmr_lab_replay: --synthetic-rate must be positive\n";
        return false;
    }

    QString backendError;
    std::unique_ptr<SyntheticVoiceBackend> encoder = loadSyntheticVoiceEncoder(backendError);
    if (!encoder) {
        err << "dmr_lab_replay: " << backendError << '\n';
        return false;
    }

    constexpr int syntheticFrames = 51;
    std::vector<std::array<std::uint8_t, 9>> canonicalFrames;
    std::vector<QString> payloadHexes;
    std::vector<QString> onAirFrames;
    canonicalFrames.reserve(syntheticFrames);
    payloadHexes.reserve(syntheticFrames);
    onAirFrames.reserve(syntheticFrames);

    QByteArray sourcePcm8k;
    QByteArray directCanonicalPcm8k;
    int canonicalDecodeFrames = 0;
    int canonicalDecodeErrors = 0;
    int failedEncodes = 0;
    int failedCanonicalDecodes = 0;

    for (int frameIndex = 0; frameIndex < syntheticFrames; ++frameIndex) {
        std::array<std::int16_t, 160> pcmFrame = {};
        for (int i = 0; i < 160; ++i) {
            const int sampleIndex = frameIndex * 160 + i;
            const double t = static_cast<double>(sampleIndex) / 8000.0;
            const double value = 0.72 * std::sin(TWO_PI * 770.0 * t) +
                                 0.22 * std::sin(TWO_PI * 1330.0 * t);
            pcmFrame[static_cast<std::size_t>(i)] =
                static_cast<std::int16_t>(std::lrint((std::clamp)(value, -1.0, 1.0) * 15000.0));
        }
        sourcePcm8k.append(appendPcm8kFrameToBytes(pcmFrame));

        std::array<std::uint8_t, 9> canonical = {};
        const int status =
            encoder->encodeCanonical(encoder->context,
                                     pcmFrame.data(),
                                     canonical.data());
        if (status != FOBOS_DMR_VOICE_OK) {
            ++failedEncodes;
            continue;
        }

        if (encoder->decodeCanonical) {
            std::array<std::int16_t, 160> decoded = {};
            int errors = 0;
            const int decodeStatus =
                encoder->decodeCanonical(encoder->context,
                                         canonical.data(),
                                         decoded.data(),
                                         &errors);
            if (decodeStatus == FOBOS_DMR_VOICE_OK) {
                directCanonicalPcm8k.append(appendPcm8kFrameToBytes(decoded));
                ++canonicalDecodeFrames;
                canonicalDecodeErrors += errors;
            } else {
                ++failedCanonicalDecodes;
            }
        }

        const QString payloadHex = canonicalAmbeFrameToPayloadHex(canonical);
        const QString onAirFrame = payload49ToDmrOnAirFrameHex(payloadHex);
        if (onAirFrame.size() != 18) {
            continue;
        }
        canonicalFrames.push_back(canonical);
        payloadHexes.push_back(payloadHex);
        onAirFrames.push_back(onAirFrame);
    }

    int packSelfCheckFrames = 0;
    int packSelfCheckFailures = 0;
    int packSelfCheckCorrections = 0;
    for (std::size_t i = 0; i < onAirFrames.size(); ++i) {
        const ReplayAmbeFecDecodeResult decoded =
            replayDecodeDmrAmbeFecPayload(onAirFrames[i]);
        if (!decoded.decoded) {
            ++packSelfCheckFailures;
            continue;
        }
        packSelfCheckCorrections += decoded.correctedErrors;
        if (decoded.payloadHex.left(13) == payloadHexes[i].left(13)) {
            ++packSelfCheckFrames;
        } else {
            ++packSelfCheckFailures;
        }
    }

    SyntheticDmrStream stream = buildSyntheticDmrVoiceStream(onAirFrames);
    const QByteArray iq = generateSyntheticDmrIq(stream.symbols,
                                                options.syntheticIqSampleRate);
    IqDemodState demodState;
    const QByteArray pcm = demodulateDmrChannelIq(iq,
                                                  options.syntheticIqSampleRate,
                                                  demodState,
                                                  options.fourFskOutputOuterLevel);
    const SyntheticIqSelfTestResult slicerResult = analyzeSyntheticIqPcm(pcm, stream);

    QString writeError;
    if (!writePcm16Wav(options.syntheticIqWavOutPath,
                       iq,
                       options.syntheticIqSampleRate,
                       2,
                       writeError)) {
        err << "dmr_lab_replay: failed to write synthetic IQ WAV: "
            << writeError << '\n';
        return false;
    }
    if (!writePcm16Wav(options.syntheticPcmWavOutPath,
                       pcm,
                       static_cast<int>(DMR_AUDIO_RATE),
                       1,
                       writeError)) {
        err << "dmr_lab_replay: failed to write synthetic PCM WAV: "
            << writeError << '\n';
        return false;
    }

    ReplaySummary summary;
    summary.filePath = QStringLiteral("<synthetic-voice>");
    summary.inputMode = QStringLiteral("synthetic_iq_voice");
    summary.sampleRate = options.syntheticIqSampleRate;
    summary.headerSampleRate = options.syntheticIqSampleRate;
    summary.decoderSampleRate = static_cast<int>(DMR_AUDIO_RATE);
    summary.channels = 2;
    summary.bitsPerSample = 16;
    summary.fourFskOutputOuterLevel = options.fourFskOutputOuterLevel;
    summary.durationSeconds =
        options.syntheticIqSampleRate > 0
            ? static_cast<double>(iq.size()) /
                  (static_cast<double>(options.syntheticIqSampleRate) * 4.0)
            : 0.0;

    gSummary = &summary;
    gPrintQtDebug = options.printQtDebug;
    qInstallMessageHandler(qtMessageHandler);

    DmrDecoder decoder;
    DmrVocoder vocoder;
    decoder.configure(static_cast<int>(DMR_AUDIO_RATE));
    decoder.setLabHints(true,
                        options.expected.colorCode >= 0 ? options.expected.colorCode : 5,
                        options.expected.timeslot >= 1 ? options.expected.timeslot : 1,
                        options.expected.sourceId,
                        options.expected.targetId);
    summary.vocoderAvailable = vocoder.isAvailable();

    QByteArray decodedVoicePcm48k;
    QTextStream statusOut(stdout);
    const int samplesPerSymbol =
        static_cast<int>(std::lround(DMR_AUDIO_RATE / DMR_SYMBOL_RATE));
    int consumedSamples = 0;
    for (const int syncStart : stream.voiceSyncStarts) {
        const int endSymbol = syncStart + 24 + 54 + 32;
        const int endSample =
            (std::min)(pcm.size() / 2, endSymbol * samplesPerSymbol);
        if (endSample <= consumedSamples) {
            continue;
        }
        const QByteArray chunk = pcm.mid(consumedSamples * 2,
                                         (endSample - consumedSamples) * 2);
        consumedSamples = endSample;
        const DmrDecoder::Result result =
            decoder.processPcmFrame(chunk, static_cast<int>(DMR_AUDIO_RATE), 163472000.0);
        summary.ambeFrames += static_cast<int>(result.ambeFrames.size());
        summary.ambePayloads += static_cast<int>(result.ambePayloads.size());
        summary.ambeFecCorrections += result.ambeFecCorrections;

        int decodedVoiceFrames = 0;
        int voiceErrors = 0;
        const QByteArray voicePcm =
            !result.ambePayloads.empty()
                ? vocoder.decodePayloads(result.ambePayloads, &decodedVoiceFrames, &voiceErrors)
                : vocoder.decodeFrames(result.ambeFrames, &decodedVoiceFrames, &voiceErrors);
        summary.vocoderFrames += decodedVoiceFrames;
        summary.vocoderErrors += voiceErrors;
        summary.vocoderSamples += voicePcm.size() / static_cast<int>(sizeof(qint16));
        decodedVoicePcm48k.append(voicePcm);
        if (options.printStatus && result.statusChanged && !result.statusText.isEmpty()) {
            statusOut << "[status] " << result.statusText.trimmed() << '\n';
        }
        if (!result.decodedText.isEmpty()) {
            summary.decodedLines += result.decodedText.count(QLatin1Char('\n'));
        }
    }
    if (consumedSamples * 2 < pcm.size()) {
        const QByteArray tail = pcm.mid(consumedSamples * 2);
        const DmrDecoder::Result result =
            decoder.processPcmFrame(tail, static_cast<int>(DMR_AUDIO_RATE), 163472000.0);
        summary.ambeFrames += static_cast<int>(result.ambeFrames.size());
        summary.ambePayloads += static_cast<int>(result.ambePayloads.size());
        summary.ambeFecCorrections += result.ambeFecCorrections;
        int decodedVoiceFrames = 0;
        int voiceErrors = 0;
        const QByteArray voicePcm =
            !result.ambePayloads.empty()
                ? vocoder.decodePayloads(result.ambePayloads, &decodedVoiceFrames, &voiceErrors)
                : vocoder.decodeFrames(result.ambeFrames, &decodedVoiceFrames, &voiceErrors);
        summary.vocoderFrames += decodedVoiceFrames;
        summary.vocoderErrors += voiceErrors;
        summary.vocoderSamples += voicePcm.size() / static_cast<int>(sizeof(qint16));
        decodedVoicePcm48k.append(voicePcm);
    }

    QFile voicePcmOut;
    if (!options.voicePcmOutPath.isEmpty()) {
        voicePcmOut.setFileName(options.voicePcmOutPath);
        if (!voicePcmOut.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            err << "dmr_lab_replay: failed to open voice PCM dump: "
                << voicePcmOut.errorString() << '\n';
            return false;
        }
        voicePcmOut.write(decodedVoicePcm48k);
    }

    int directFrameDecodeFrames = 0;
    int directFrameDecodeErrors = 0;
    const QByteArray directFrameVoice =
        vocoder.decodeFrames(onAirFrames, &directFrameDecodeFrames, &directFrameDecodeErrors);
    std::vector<DmrAmbePayload> directPayloads;
    directPayloads.reserve(payloadHexes.size());
    for (const QString &payloadHex : payloadHexes) {
        directPayloads.push_back({payloadHex, 0});
    }
    int directPayloadDecodeFrames = 0;
    int directPayloadDecodeErrors = 0;
    const QByteArray directPayloadVoice =
        vocoder.decodePayloads(directPayloads,
                               &directPayloadDecodeFrames,
                               &directPayloadDecodeErrors);

    const bool slicerPass = slicerResult.valid &&
                            slicerResult.accuracy >= 0.999 &&
                            slicerResult.bestSyncScore >= 24;
    const bool packPass =
        packSelfCheckFrames == static_cast<int>(onAirFrames.size()) &&
        packSelfCheckFailures == 0;
    const bool decoderPass =
        summary.ambePayloads >= static_cast<int>(stream.voiceSyncStarts.size()) * 3 / 2 &&
        summary.vocoderFrames > 0;
    const bool pass = failedEncodes == 0 &&
                      !onAirFrames.empty() &&
                      packPass &&
                      slicerPass &&
                      decoderPass;

    out << "[synthetic DMR voice self-test]\n"
        << "  backend: " << encoder->displayName() << '\n'
        << "  4FSK output outer level: "
        << QString::number(options.fourFskOutputOuterLevel, 'f', 3) << '\n'
        << "  source PCM: 8000 Hz, frames " << syntheticFrames
        << ", RMS " << QString::number(pcm16Rms(sourcePcm8k), 'f', 1) << '\n'
        << "  canonical encode: frames " << canonicalFrames.size()
        << ", failed " << failedEncodes << '\n'
        << "  canonical decode check: frames " << canonicalDecodeFrames
        << ", failed " << failedCanonicalDecodes
        << ", errors " << canonicalDecodeErrors
        << ", RMS " << QString::number(pcm16Rms(directCanonicalPcm8k), 'f', 1)
        << '\n'
        << "  DMR on-air pack check: " << packSelfCheckFrames << "/"
        << onAirFrames.size() << " frames, failures " << packSelfCheckFailures
        << ", FEC corrections " << packSelfCheckCorrections << '\n'
        << "  direct vocoder frame decode: frames " << directFrameDecodeFrames
        << ", errors " << directFrameDecodeErrors
        << ", RMS " << QString::number(pcm16Rms(directFrameVoice), 'f', 1)
        << '\n'
        << "  direct vocoder payload decode: frames " << directPayloadDecodeFrames
        << ", errors " << directPayloadDecodeErrors
        << ", RMS " << QString::number(pcm16Rms(directPayloadVoice), 'f', 1)
        << '\n'
        << "  synthetic RF: " << stream.symbols.size() << " symbols, "
        << stream.voiceSyncStarts.size() << " MS voice sync bursts, "
        << QString::number(summary.durationSeconds, 'f', 3) << " s IQ\n";

    if (slicerResult.valid) {
        out << "  IQ slicer: errors " << slicerResult.symbolErrors
            << "/" << slicerResult.testedSymbols
            << ", " << QString::number(slicerResult.accuracy * 100.0, 'f', 3)
            << "% correct, sync " << slicerResult.bestSyncScore << "/24\n";
    } else {
        out << "  IQ slicer: FAIL, " << slicerResult.error << '\n';
    }

    out << "  decoder: locks " << summary.lockAcquired
        << ", AMBE frames " << summary.ambeFrames
        << ", payloads " << summary.ambePayloads
        << ", FEC corrections " << summary.ambeFecCorrections << '\n'
        << "  pipeline vocoder: available " << (summary.vocoderAvailable ? "yes" : "no")
        << ", frames " << summary.vocoderFrames
        << ", errors " << summary.vocoderErrors
        << ", samples48k " << summary.vocoderSamples
        << ", RMS " << QString::number(pcm16Rms(decodedVoicePcm48k), 'f', 1) << '\n';

    if (!options.syntheticIqWavOutPath.isEmpty()) {
        out << "  wrote IQ WAV: " << options.syntheticIqWavOutPath << '\n';
    }
    if (!options.syntheticPcmWavOutPath.isEmpty()) {
        out << "  wrote demod PCM WAV: " << options.syntheticPcmWavOutPath << '\n';
    }
    if (!options.voicePcmOutPath.isEmpty()) {
        out << "  wrote decoded voice PCM: " << options.voicePcmOutPath << '\n';
    }

    out << "  result: " << (pass ? "PASS" : "FAIL") << '\n';
    out.flush();

    qInstallMessageHandler(nullptr);
    gSummary = nullptr;
    return pass;
}

static bool parseWavInfo(const QString &path, WavInfo &info, QString &error) {
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        error = file.errorString();
        return false;
    }

    const QByteArray header = file.read(12);
    if (header.size() != 12 ||
        QByteArray(header.constData(), 4) != QByteArrayLiteral("RIFF") ||
        QByteArray(header.constData() + 8, 4) != QByteArrayLiteral("WAVE")) {
        error = QStringLiteral("not a RIFF/WAVE file");
        return false;
    }

    bool haveFmt = false;
    bool haveData = false;
    while (!file.atEnd()) {
        const QByteArray chunkHeader = file.read(8);
        if (chunkHeader.size() == 0) {
            break;
        }
        if (chunkHeader.size() != 8) {
            error = QStringLiteral("truncated WAV chunk header");
            return false;
        }

        const QByteArray chunkId(chunkHeader.constData(), 4);
        const quint32 chunkSize = readLe32(chunkHeader.constData() + 4);
        const qint64 chunkStart = file.pos();

        if (chunkId == QByteArrayLiteral("fmt ")) {
            const QByteArray fmt = file.read(static_cast<qint64>(chunkSize));
            if (fmt.size() < 16) {
                error = QStringLiteral("truncated WAV fmt chunk");
                return false;
            }
            const quint16 formatTag = readLe16(fmt.constData());
            if (formatTag != 1) {
                error = QStringLiteral("unsupported WAV format tag %1").arg(formatTag);
                return false;
            }
            info.channels = readLe16(fmt.constData() + 2);
            info.sampleRate = static_cast<int>(readLe32(fmt.constData() + 4));
            info.bitsPerSample = readLe16(fmt.constData() + 14);
            haveFmt = true;
        } else if (chunkId == QByteArrayLiteral("data")) {
            info.dataOffset = static_cast<quint64>(chunkStart);
            info.dataSize = chunkSize;
            haveData = true;
            file.seek(chunkStart + static_cast<qint64>(chunkSize) + (chunkSize & 1u));
            continue;
        } else if (chunkId == QByteArrayLiteral("fbos")) {
            const QByteArray metadataBytes = file.read(static_cast<qint64>(chunkSize));
            const QJsonDocument doc = QJsonDocument::fromJson(metadataBytes);
            if (doc.isObject()) {
                info.metadata = doc.object();
            }
        }

        const qint64 next = chunkStart + static_cast<qint64>(chunkSize) + (chunkSize & 1u);
        if (!file.seek(next)) {
            error = QStringLiteral("failed to seek to next WAV chunk");
            return false;
        }
    }

    if (!haveFmt || !haveData) {
        error = QStringLiteral("missing fmt or data chunk");
        return false;
    }
    return true;
}

static void mergeExpectedFromMetadata(ExpectedValues &expected, const QJsonObject &metadata) {
    const QJsonObject lab = metadata.value(QStringLiteral("lab")).toObject();
    if (expected.colorCode < 0) {
        expected.colorCode = jsonInt(lab, QStringLiteral("expectedColorCode"));
    }
    if (expected.timeslot < 0) {
        expected.timeslot = jsonInt(lab, QStringLiteral("expectedTimeslot"));
    }
    if (expected.sourceId < 0) {
        expected.sourceId = jsonInt(lab, QStringLiteral("expectedSourceId"));
    }
    if (expected.targetId < 0) {
        expected.targetId = jsonInt(lab, QStringLiteral("expectedTargetId"));
    }
}

static void qtMessageHandler(QtMsgType type, const QMessageLogContext &, const QString &message) {
    const QString line = message.trimmed();
    if (gSummary && line.contains(QStringLiteral("[DMR"))) {
        gSummary->observeLine(line);
    }

    const bool shouldPrint = gPrintQtDebug || type == QtWarningMsg || type == QtCriticalMsg || type == QtFatalMsg;
    if (!shouldPrint) {
        return;
    }

    QTextStream err(stderr);
    err << line << '\n';
    err.flush();
}

static void printUsage() {
    QTextStream out(stdout);
    out << "Usage: dmr_lab_replay --file recording.wav [options]\n"
        << "       dmr_lab_replay --synthetic-iq-self-test [options]\n"
        << "       dmr_lab_replay --synthetic-voice-self-test [options]\n"
        << "\n"
        << "Options:\n"
        << "  --file <path>          Mono 16-bit PCM WAV recorded from DMR audio\n"
        << "  --expect-cc <n>        Expected DMR color code\n"
        << "  --expect-ts <1|2>      Expected DMR timeslot/channel\n"
        << "  --expect-src <id>      Expected source radio ID\n"
        << "  --expect-tg <id>       Expected target/group ID\n"
        << "  --json <path>          Write machine-readable summary\n"
        << "  --pcm-out <path>       Dump decoder input PCM16LE for external tools\n"
        << "  --voice-pcm-out <path> Dump synthesized DMR voice PCM16LE, 48 kHz\n"
        << "  --synthetic-iq-self-test\n"
        << "                         Generate clean reference DMR-like 4FSK IQ and verify IQ->symbol slicing\n"
        << "  --synthetic-voice-self-test\n"
        << "                         Generate AMBE voice, pack it into DMR 4FSK IQ, and replay through the decoder\n"
        << "  --synthetic-rate <hz>  Synthetic IQ sample rate, default 192000\n"
        << "  --fourfsk-output-level <v>\n"
        << "                         FM discriminator scale for outer DMR symbols, default "
        << QString::number(DMR_FOURFSK_OUTPUT_OUTER_LEVEL, 'f', 2) << "\n"
        << "  --input-rate <hz>     Override WAV sample rate for replay timing\n"
        << "  --synthetic-iq-wav-out <path>\n"
        << "                         Write generated stereo IQ WAV\n"
        << "  --synthetic-pcm-wav-out <path>\n"
        << "                         Write demodulated 48 kHz mono PCM WAV\n"
        << "  --status              Print decoder status changes\n"
        << "  --qt-debug            Print DMR debug lines from the decoder\n"
        << "  --help                Show this help\n";
}

static bool parseArgs(const QStringList &args, ReplayOptions &options, QString &error) {
    for (int i = 1; i < args.size(); ++i) {
        const QString arg = args.at(i);
        auto requireValue = [&](const QString &name, QString &value) -> bool {
            if (i + 1 >= args.size()) {
                error = QStringLiteral("missing value for %1").arg(name);
                return false;
            }
            value = args.at(++i);
            return true;
        };
        auto requireInt = [&](const QString &name, int &value) -> bool {
            QString text;
            if (!requireValue(name, text)) {
                return false;
            }
            bool ok = false;
            const int parsed = text.toInt(&ok);
            if (!ok) {
                error = QStringLiteral("invalid integer for %1: %2").arg(name, text);
                return false;
            }
            value = parsed;
            return true;
        };
        auto requireDouble = [&](const QString &name, double &value) -> bool {
            QString text;
            if (!requireValue(name, text)) {
                return false;
            }
            bool ok = false;
            const double parsed = text.toDouble(&ok);
            if (!ok || !std::isfinite(parsed)) {
                error = QStringLiteral("invalid number for %1: %2").arg(name, text);
                return false;
            }
            value = parsed;
            return true;
        };

        if (arg == QStringLiteral("--help") || arg == QStringLiteral("-h")) {
            printUsage();
            std::exit(0);
        } else if (arg == QStringLiteral("--file") || arg == QStringLiteral("-f")) {
            if (!requireValue(arg, options.filePath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--expect-cc")) {
            if (!requireInt(arg, options.expected.colorCode)) {
                return false;
            }
        } else if (arg == QStringLiteral("--expect-ts") || arg == QStringLiteral("--expect-slot")) {
            if (!requireInt(arg, options.expected.timeslot)) {
                return false;
            }
        } else if (arg == QStringLiteral("--expect-src")) {
            if (!requireInt(arg, options.expected.sourceId)) {
                return false;
            }
        } else if (arg == QStringLiteral("--expect-tg") || arg == QStringLiteral("--expect-target")) {
            if (!requireInt(arg, options.expected.targetId)) {
                return false;
            }
        } else if (arg == QStringLiteral("--json")) {
            if (!requireValue(arg, options.jsonSummaryPath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--pcm-out")) {
            if (!requireValue(arg, options.pcmOutPath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--voice-pcm-out")) {
            if (!requireValue(arg, options.voicePcmOutPath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--synthetic-iq-self-test")) {
            options.syntheticIqSelfTest = true;
        } else if (arg == QStringLiteral("--synthetic-voice-self-test")) {
            options.syntheticVoiceSelfTest = true;
        } else if (arg == QStringLiteral("--synthetic-rate")) {
            if (!requireInt(arg, options.syntheticIqSampleRate)) {
                return false;
            }
        } else if (arg == QStringLiteral("--fourfsk-output-level")) {
            if (!requireDouble(arg, options.fourFskOutputOuterLevel)) {
                return false;
            }
        } else if (arg == QStringLiteral("--input-rate") ||
                   arg == QStringLiteral("--sample-rate-override")) {
            if (!requireInt(arg, options.inputSampleRateOverride)) {
                return false;
            }
        } else if (arg == QStringLiteral("--synthetic-iq-wav-out")) {
            if (!requireValue(arg, options.syntheticIqWavOutPath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--synthetic-pcm-wav-out")) {
            if (!requireValue(arg, options.syntheticPcmWavOutPath)) {
                return false;
            }
        } else if (arg == QStringLiteral("--status")) {
            options.printStatus = true;
        } else if (arg == QStringLiteral("--qt-debug")) {
            options.printQtDebug = true;
        } else if (arg.startsWith(QStringLiteral("--"))) {
            error = QStringLiteral("unknown option %1").arg(arg);
            return false;
        } else if (options.filePath.isEmpty()) {
            options.filePath = arg;
        } else {
            error = QStringLiteral("unexpected positional argument %1").arg(arg);
            return false;
        }
    }

    if (!options.syntheticIqSelfTest &&
        !options.syntheticVoiceSelfTest &&
        options.filePath.isEmpty()) {
        error = QStringLiteral("missing --file");
        return false;
    }
    return true;
}

static bool expectedFound(int expected, const QMap<int, int> &histogram) {
    return expected < 0 || histogram.value(expected) > 0;
}

static QJsonObject summaryToJson(const ReplaySummary &summary, const ExpectedValues &expected, bool pass) {
    QJsonObject expectedObject;
    if (expected.colorCode >= 0) {
        expectedObject[QStringLiteral("colorCode")] = expected.colorCode;
    }
    if (expected.timeslot >= 0) {
        expectedObject[QStringLiteral("timeslot")] = expected.timeslot;
    }
    if (expected.sourceId >= 0) {
        expectedObject[QStringLiteral("sourceId")] = expected.sourceId;
    }
    if (expected.targetId >= 0) {
        expectedObject[QStringLiteral("targetId")] = expected.targetId;
    }

    QJsonObject object;
    object[QStringLiteral("generatedUtc")] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
    object[QStringLiteral("file")] = summary.filePath;
    object[QStringLiteral("inputMode")] = summary.inputMode;
    object[QStringLiteral("sampleRate")] = summary.sampleRate;
    object[QStringLiteral("headerSampleRate")] = summary.headerSampleRate;
    object[QStringLiteral("decoderSampleRate")] = summary.decoderSampleRate;
    object[QStringLiteral("fourFskOutputOuterLevel")] = summary.fourFskOutputOuterLevel;
    object[QStringLiteral("channels")] = summary.channels;
    object[QStringLiteral("bitsPerSample")] = summary.bitsPerSample;
    object[QStringLiteral("durationSeconds")] = summary.durationSeconds;
    object[QStringLiteral("rfFrequencyHz")] = summary.rfFrequencyHz;
    object[QStringLiteral("decodedLines")] = summary.decodedLines;
    object[QStringLiteral("debugDmrLines")] = summary.debugDmrLines;
    object[QStringLiteral("lockAcquired")] = summary.lockAcquired;
    object[QStringLiteral("lockLost")] = summary.lockLost;
    object[QStringLiteral("ambeFrames")] = summary.ambeFrames;
    object[QStringLiteral("ambePayloads")] = summary.ambePayloads;
    object[QStringLiteral("ambeFecCorrections")] = summary.ambeFecCorrections;
    object[QStringLiteral("vocoderAvailable")] = summary.vocoderAvailable;
    object[QStringLiteral("vocoderFrames")] = summary.vocoderFrames;
    object[QStringLiteral("vocoderErrors")] = summary.vocoderErrors;
    object[QStringLiteral("vocoderSamples")] = summary.vocoderSamples;
    object[QStringLiteral("colorCodes")] = mapToJson(summary.colorCodes);
    object[QStringLiteral("timeslots")] = mapToJson(summary.timeslots);
    object[QStringLiteral("sources")] = mapToJson(summary.sources);
    object[QStringLiteral("targets")] = mapToJson(summary.targets);
    object[QStringLiteral("expected")] = expectedObject;
    object[QStringLiteral("pass")] = pass;
    object[QStringLiteral("notableLines")] = QJsonArray::fromStringList(summary.notableLines);
    object[QStringLiteral("lcLines")] = QJsonArray::fromStringList(summary.lcLines);
    return object;
}

int main(int argc, char *argv[]) {
    QCoreApplication app(argc, argv);
    ReplayOptions options;
    QString error;
    if (!parseArgs(app.arguments(), options, error)) {
        QTextStream(stderr) << "dmr_lab_replay: " << error << '\n';
        printUsage();
        return 1;
    }

    if (options.syntheticIqSelfTest) {
        return runSyntheticIqSelfTest(options) ? 0 : 2;
    }
    if (options.syntheticVoiceSelfTest) {
        return runSyntheticVoiceSelfTest(options) ? 0 : 2;
    }

    WavInfo wav;
    if (!parseWavInfo(options.filePath, wav, error)) {
        QTextStream(stderr) << "dmr_lab_replay: " << error << '\n';
        return 1;
    }
    const int headerSampleRate = wav.sampleRate;
    if (options.inputSampleRateOverride > 0) {
        wav.sampleRate = options.inputSampleRateOverride;
    }

    mergeExpectedFromMetadata(options.expected, wav.metadata);

    ReplaySummary summary;
    summary.filePath = options.filePath;
    summary.inputMode = (wav.channels == 2 && wav.bitsPerSample == 16)
                            ? QStringLiteral("channel_iq")
                            : QStringLiteral("audio");
    summary.sampleRate = wav.sampleRate;
    summary.headerSampleRate = headerSampleRate;
    summary.decoderSampleRate = summary.inputMode == QStringLiteral("channel_iq")
                                    ? static_cast<int>(DMR_AUDIO_RATE)
                                    : wav.sampleRate;
    summary.channels = wav.channels;
    summary.bitsPerSample = wav.bitsPerSample;
    summary.fourFskOutputOuterLevel = options.fourFskOutputOuterLevel;
    if (options.inputSampleRateOverride > 0 &&
        options.inputSampleRateOverride != headerSampleRate) {
        summary.notableLines.append(
            QStringLiteral("[replay] WAV sample-rate override: header %1 Hz, replay %2 Hz")
                .arg(headerSampleRate)
                .arg(options.inputSampleRateOverride));
    }
    summary.rfFrequencyHz = jsonDouble(wav.metadata, QStringLiteral("listeningFrequency"),
                                       jsonDouble(wav.metadata, QStringLiteral("centerFrequency"), 0.0));
    if (wav.sampleRate > 0 && wav.channels > 0 && wav.bitsPerSample > 0) {
        summary.durationSeconds = static_cast<double>(wav.dataSize) /
                                  (static_cast<double>(wav.sampleRate) *
                                   static_cast<double>(wav.channels) *
                                   static_cast<double>(wav.bitsPerSample / 8));
    }

    const bool audioWav = wav.channels == 1 && wav.bitsPerSample == 16;
    const bool channelIqWav = wav.channels == 2 && wav.bitsPerSample == 16;
    if (!audioWav && !channelIqWav) {
        QTextStream(stderr) << "dmr_lab_replay: supported inputs are mono PCM16 audio WAV or stereo PCM16 channel-IQ WAV. "
                            << "This file has " << wav.channels << " channel(s), "
                            << wav.bitsPerSample << " bits/sample.\n";
        return 1;
    }

    gSummary = &summary;
    gPrintQtDebug = options.printQtDebug;
    qInstallMessageHandler(qtMessageHandler);

    QFile file(options.filePath);
    if (!file.open(QIODevice::ReadOnly) ||
        !file.seek(static_cast<qint64>(wav.dataOffset))) {
        QTextStream(stderr) << "dmr_lab_replay: failed to open WAV data: " << file.errorString() << '\n';
        return 1;
    }

    QFile pcmOut;
    if (!options.pcmOutPath.isEmpty()) {
        pcmOut.setFileName(options.pcmOutPath);
        if (!pcmOut.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            QTextStream(stderr) << "dmr_lab_replay: failed to open PCM dump: "
                                << pcmOut.errorString() << '\n';
            return 1;
        }
    }

    QFile voicePcmOut;
    if (!options.voicePcmOutPath.isEmpty()) {
        voicePcmOut.setFileName(options.voicePcmOutPath);
        if (!voicePcmOut.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            QTextStream(stderr) << "dmr_lab_replay: failed to open voice PCM dump: "
                                << voicePcmOut.errorString() << '\n';
            return 1;
        }
    }

    DmrDecoder decoder;
    DmrVocoder vocoder;
    decoder.configure(summary.decoderSampleRate);
    const bool haveExpectedHints =
        options.expected.colorCode >= 0 ||
        options.expected.timeslot >= 1 ||
        options.expected.sourceId > 0 ||
        options.expected.targetId > 0;
    decoder.setLabHints(haveExpectedHints,
                        options.expected.colorCode,
                        options.expected.timeslot,
                        options.expected.sourceId,
                        options.expected.targetId);
    summary.vocoderAvailable = vocoder.isAvailable();
    const int frameSamples = audioWav
                                 ? (std::max)(480, wav.sampleRate / 10)
                                 : 4096;
    const int frameBytes = frameSamples * wav.channels * static_cast<int>(sizeof(qint16));
    quint64 bytesRemaining = wav.dataSize;
    IqDemodState iqState;

    QTextStream out(stdout);
    while (bytesRemaining > 0) {
        const int toRead =
            static_cast<int>(std::min<quint64>(static_cast<quint64>(frameBytes), bytesRemaining));
        QByteArray frame = file.read(toRead);
        if (frame.isEmpty()) {
            break;
        }
        if (audioWav && (frame.size() & 1)) {
            frame.chop(1);
        }
        if (channelIqWav && (frame.size() % 4) != 0) {
            frame.chop(frame.size() % 4);
        }
        bytesRemaining -= static_cast<quint64>(frame.size());

        QByteArray pcmFrame = frame;
        int decoderRate = wav.sampleRate;
        if (channelIqWav) {
            pcmFrame = demodulateDmrChannelIq(frame,
                                              wav.sampleRate,
                                              iqState,
                                              options.fourFskOutputOuterLevel);
            decoderRate = static_cast<int>(DMR_AUDIO_RATE);
        }
        if (pcmFrame.isEmpty()) {
            continue;
        }
        if (pcmOut.isOpen()) {
            pcmOut.write(pcmFrame);
        }

        const DmrDecoder::Result result =
            decoder.processPcmFrame(pcmFrame, decoderRate, summary.rfFrequencyHz);
        summary.ambeFrames += static_cast<int>(result.ambeFrames.size());
        summary.ambePayloads += static_cast<int>(result.ambePayloads.size());
        summary.ambeFecCorrections += result.ambeFecCorrections;
        int decodedVoiceFrames = 0;
        int voiceErrors = 0;
        const QByteArray voicePcm =
            !result.ambePayloads.empty()
                ? vocoder.decodePayloads(result.ambePayloads, &decodedVoiceFrames, &voiceErrors)
                : vocoder.decodeFrames(result.ambeFrames, &decodedVoiceFrames, &voiceErrors);
        summary.vocoderFrames += decodedVoiceFrames;
        summary.vocoderErrors += voiceErrors;
        summary.vocoderSamples += voicePcm.size() / static_cast<int>(sizeof(qint16));
        if (voicePcmOut.isOpen() && !voicePcm.isEmpty()) {
            voicePcmOut.write(voicePcm);
        }
        if (options.printStatus && result.statusChanged && !result.statusText.isEmpty()) {
            out << "[status] " << result.statusText.trimmed() << '\n';
        }
        if (!result.decodedText.isEmpty()) {
            ++summary.decodedLines;
            out << result.decodedText;
            out.flush();
        }
    }

    const bool pass =
        expectedFound(options.expected.colorCode, summary.colorCodes) &&
        expectedFound(options.expected.timeslot, summary.timeslots) &&
        expectedFound(options.expected.sourceId, summary.sources) &&
        expectedFound(options.expected.targetId, summary.targets);

    out << "\n[DMR lab summary]\n"
        << "  file: " << summary.filePath << '\n'
        << "  input: " << summary.inputMode << '\n'
        << "  wav: " << summary.sampleRate << " Hz, "
        << summary.channels << " ch, " << summary.bitsPerSample
        << " bit, " << QString::number(summary.durationSeconds, 'f', 2) << " s\n"
        << "  rf: " << QString::number(summary.rfFrequencyHz, 'f', 0) << " Hz\n"
        << "  locks: acquired " << summary.lockAcquired << ", lost " << summary.lockLost << '\n'
        << "  AMBE frames: " << summary.ambeFrames << '\n'
        << "  AMBE FEC payloads: " << summary.ambePayloads
        << ", Golay corrections: " << summary.ambeFecCorrections << '\n'
        << "  vocoder: " << (summary.vocoderAvailable ? "available" : "unavailable")
        << ", frames " << summary.vocoderFrames
        << ", samples " << summary.vocoderSamples
        << ", errors " << summary.vocoderErrors << '\n'
        << "  CC: " << histogramText(summary.colorCodes) << '\n'
        << "  Ch/TS: " << histogramText(summary.timeslots) << '\n'
        << "  SRC: " << histogramText(summary.sources) << '\n'
        << "  TG: " << histogramText(summary.targets) << '\n';

    if (options.expected.colorCode >= 0 ||
        options.expected.timeslot >= 0 ||
        options.expected.sourceId >= 0 ||
        options.expected.targetId >= 0) {
        out << "  expected: CC " << options.expected.colorCode
            << ", TS " << options.expected.timeslot
            << ", SRC " << options.expected.sourceId
            << ", TG " << options.expected.targetId
            << " => " << (pass ? "PASS" : "MISSING") << '\n';
    }
    out.flush();

    if (!options.jsonSummaryPath.isEmpty()) {
        QFile jsonFile(options.jsonSummaryPath);
        if (!jsonFile.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
            QTextStream(stderr) << "dmr_lab_replay: failed to write JSON summary: "
                                << jsonFile.errorString() << '\n';
            return 1;
        }
        const QJsonDocument doc(summaryToJson(summary, options.expected, pass));
        jsonFile.write(doc.toJson(QJsonDocument::Indented));
    }

    return pass ? 0 : 2;
}
