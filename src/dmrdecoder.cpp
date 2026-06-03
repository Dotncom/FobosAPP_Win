#include "dmrdecoder.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>

#include <QDebug>

namespace {

constexpr double DMR_SYMBOL_RATE = 4800.0;
constexpr int DMR_SYNC_SYMBOLS = 24;
constexpr int DMR_TRAFFIC_SYMBOLS = 132;
constexpr int DMR_SYMBOLS_BEFORE_SYNC = 54;
constexpr int DMR_CACH_SYMBOLS = 12;
constexpr int DMR_CANDIDATE_SYNC_SCORE = 23;
constexpr int DMR_CONFIRMED_SYNC_SCORE = 24;
constexpr int DMR_CANDIDATE_OUTER_SCORE = 14;
constexpr int DMR_CONFIRMED_OUTER_SCORE = 16;
constexpr int DMR_REPORTABLE_CANDIDATE_OUTER_SCORE = 20;
constexpr int DMR_TRUSTED_VOICE_OUTER_SCORE = 17;
constexpr int DMR_STRONG_ANCHOR_OUTER_SCORE = 18;
constexpr int DMR_LOCKED_POLARITY_OUTER_MARGIN = 3;
constexpr int DMR_MAX_BUFFER_SYMBOLS = 500;
constexpr double DMR_REPORT_INTERVAL_SECONDS = 1.0;
constexpr double DMR_LOCK_TIMEOUT_SECONDS = 1.5;
constexpr double DMR_VOICE_SYNC_MIN_GAP_SECONDS = 0.30;
constexpr int DMR_EMB_VARIANT_COUNT = 8;

const DmrDecoder::SyncPattern DMR_SYNC_PATTERNS[] = {
    {"BS Voice", "voice", "base station", "131111333113313313113313"},
    {"BS Data", "data", "base station", "313333111331131131331131"},
    {"MS Voice", "voice", "mobile station", "133313311131311113313331"},
    {"MS Data", "data", "mobile station", "311131133313133331131113"},
};

const int DMR_CACH_DEINTERLEAVE[24] = {
    0, 4, 8, 12, 14, 18, 22, 1,
    2, 3, 5, 6, 7, 9, 10, 11,
    13, 15, 16, 17, 19, 20, 21, 23
};

const int DMR_CACH_HAMMING_743[16] = {
    0, 11, 22, 29, 39, 44, 49, 58,
    69, 78, 83, 88, 98, 105, 116, 127
};

const int DMR_QR_1676_CODEWORDS[128] = {
    0x0000, 0x0273, 0x04E5, 0x0696, 0x09C9, 0x0BBA, 0x0D2C, 0x0F5F,
    0x11E2, 0x1391, 0x1507, 0x1774, 0x182B, 0x1A58, 0x1CCE, 0x1EBD,
    0x21B7, 0x23C4, 0x2552, 0x2721, 0x287E, 0x2A0D, 0x2C9B, 0x2EE8,
    0x3055, 0x3226, 0x34B0, 0x36C3, 0x399C, 0x3BEF, 0x3D79, 0x3F0A,
    0x411E, 0x436D, 0x45FB, 0x4788, 0x48D7, 0x4AA4, 0x4C32, 0x4E41,
    0x50FC, 0x528F, 0x5419, 0x566A, 0x5935, 0x5B46, 0x5DD0, 0x5FA3,
    0x60A9, 0x62DA, 0x644C, 0x663F, 0x6960, 0x6B13, 0x6D85, 0x6FF6,
    0x714B, 0x7338, 0x75AE, 0x77DD, 0x7882, 0x7AF1, 0x7C67, 0x7E14,
    0x804F, 0x823C, 0x84AA, 0x86D9, 0x8986, 0x8BF5, 0x8D63, 0x8F10,
    0x91AD, 0x93DE, 0x9548, 0x973B, 0x9864, 0x9A17, 0x9C81, 0x9EF2,
    0xA1F8, 0xA38B, 0xA51D, 0xA76E, 0xA831, 0xAA42, 0xACD4, 0xAEA7,
    0xB01A, 0xB269, 0xB4FF, 0xB68C, 0xB9D3, 0xBBA0, 0xBD36, 0xBF45,
    0xC151, 0xC322, 0xC5B4, 0xC7C7, 0xC898, 0xCAEB, 0xCC7D, 0xCE0E,
    0xD0B3, 0xD2C0, 0xD456, 0xD625, 0xD97A, 0xDB09, 0xDD9F, 0xDFEC,
    0xE0E6, 0xE295, 0xE403, 0xE670, 0xE92F, 0xEB5C, 0xEDCA, 0xEFB9,
    0xF104, 0xF377, 0xF5E1, 0xF792, 0xF8CD, 0xFABE, 0xFC28, 0xFE5B
};

qint16 readPcm16Le(const char *data) {
    qint16 value = 0;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

double syncQualityDb(int score) {
    const int errors = DMR_SYNC_SYMBOLS - score;
    const double good = static_cast<double>(score) + 0.5;
    const double bad = static_cast<double>(errors) + 0.5;
    return 10.0 * std::log10(good / bad);
}

float averagedSymbolSample(const std::deque<float> &samples, int centerIndex, int samplesPerSymbol) {
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

int scoreSyncSymbols(const std::deque<float> &samples,
                     int startIndex,
                     int samplesPerSymbol,
                     const char *pattern,
                     bool inverted,
                     int *outerScore = nullptr) {
    float minLevel = std::numeric_limits<float>::max();
    float maxLevel = std::numeric_limits<float>::lowest();
    float symbolSamples[DMR_SYNC_SYMBOLS] = {};
    for (int i = 0; i < DMR_SYNC_SYMBOLS; ++i) {
        const int sampleIndex = startIndex + i * samplesPerSymbol + samplesPerSymbol / 2;
        if (sampleIndex < 0 || sampleIndex >= static_cast<int>(samples.size())) {
            return 0;
        }
        const float sample = averagedSymbolSample(samples, sampleIndex, samplesPerSymbol);
        const float corrected = inverted ? -sample : sample;
        symbolSamples[i] = corrected;
        minLevel = (std::min)(minLevel, corrected);
        maxLevel = (std::max)(maxLevel, corrected);
    }

    if (maxLevel - minLevel < 0.004f) {
        return 0;
    }

    const float center = 0.5f * (maxLevel + minLevel);
    const float outerRatio = 0.55f;
    const float upperOuter = center + outerRatio * (maxLevel - center);
    const float lowerOuter = center + outerRatio * (minLevel - center);

    int score = 0;
    int outer = 0;
    for (int i = 0; i < DMR_SYNC_SYMBOLS; ++i) {
        if (pattern[i] == '3') {
            if (symbolSamples[i] > center) {
                ++score;
            }
            if (symbolSamples[i] > upperOuter) {
                ++outer;
            }
        } else if (pattern[i] == '1') {
            if (symbolSamples[i] < center) {
                ++score;
            }
            if (symbolSamples[i] < lowerOuter) {
                ++outer;
            }
        }
    }
    if (outerScore) {
        *outerScore = outer;
    }
    return score;
}

QString cachLcssText(int lcss) {
    switch (lcss) {
    case 0:
        return QStringLiteral("first CSBK");
    case 1:
        return QStringLiteral("first LC");
    case 2:
        return QStringLiteral("last LC");
    case 3:
        return QStringLiteral("continuation LC/CSBK");
    default:
        return QStringLiteral("unknown");
    }
}

QString dmrFrequencySuffix(double frequencyHz) {
    if (!std::isfinite(frequencyHz) || frequencyHz <= 0.0) {
        return QString();
    }
    if (frequencyHz >= 1000000.0) {
        return QStringLiteral(" @ %1 MHz").arg(frequencyHz / 1000000.0, 0, 'f', 6);
    }
    if (frequencyHz >= 1000.0) {
        return QStringLiteral(" @ %1 kHz").arg(frequencyHz / 1000.0, 0, 'f', 3);
    }
    return QStringLiteral(" @ %1 Hz").arg(frequencyHz, 0, 'f', 0);
}

bool hamming743Ok(int value) {
    return std::find(std::begin(DMR_CACH_HAMMING_743),
                     std::end(DMR_CACH_HAMMING_743),
                     value) != std::end(DMR_CACH_HAMMING_743);
}

std::array<bool, 12> golay208Parity(const std::array<bool, 8> &bits) {
    std::array<bool, 12> p = {};
    p[0] = bits[1] ^ bits[4] ^ bits[5] ^ bits[6] ^ bits[7];
    p[1] = bits[1] ^ bits[2] ^ bits[4];
    p[2] = bits[0] ^ bits[2] ^ bits[3] ^ bits[5];
    p[3] = bits[0] ^ bits[1] ^ bits[3] ^ bits[4] ^ bits[6];
    p[4] = bits[0] ^ bits[1] ^ bits[2] ^ bits[4] ^ bits[5] ^ bits[7];
    p[5] = bits[0] ^ bits[2] ^ bits[3] ^ bits[4] ^ bits[7];
    p[6] = bits[3] ^ bits[6] ^ bits[7];
    p[7] = bits[0] ^ bits[1] ^ bits[5] ^ bits[6];
    p[8] = bits[0] ^ bits[1] ^ bits[2] ^ bits[6] ^ bits[7];
    p[9] = bits[2] ^ bits[3] ^ bits[4] ^ bits[5] ^ bits[6];
    p[10] = bits[0] ^ bits[3] ^ bits[4] ^ bits[5] ^ bits[6] ^ bits[7];
    p[11] = bits[1] ^ bits[2] ^ bits[3] ^ bits[5] ^ bits[7];
    return p;
}

std::array<bool, 20> golay208Codeword(int data) {
    std::array<bool, 20> bits = {};
    for (int i = 0; i < 8; ++i) {
        bits[i] = ((data >> (7 - i)) & 0x01) != 0;
    }
    std::array<bool, 8> dataBits = {};
    std::copy(bits.begin(), bits.begin() + 8, dataBits.begin());
    const std::array<bool, 12> parity = golay208Parity(dataBits);
    for (int i = 0; i < 12; ++i) {
        bits[8 + i] = parity[i];
    }
    return bits;
}

int bitDistance(const std::array<bool, 20> &a, const std::array<bool, 20> &b) {
    int errors = 0;
    for (int i = 0; i < 20; ++i) {
        if (a[i] != b[i]) {
            ++errors;
        }
    }
    return errors;
}

QString dmrDataTypeName(int dataType) {
    switch (dataType) {
    case 0:
        return QStringLiteral("PI Header");
    case 1:
        return QStringLiteral("Voice LC Header");
    case 2:
        return QStringLiteral("Terminator with LC");
    case 3:
        return QStringLiteral("CSBK");
    case 4:
        return QStringLiteral("MBC Header");
    case 5:
        return QStringLiteral("MBC Continuation");
    case 6:
        return QStringLiteral("Data Header");
    case 7:
        return QStringLiteral("Rate 1/2 Data");
    case 8:
        return QStringLiteral("Rate 3/4 Data");
    case 9:
        return QStringLiteral("Idle");
    case 10:
        return QStringLiteral("Rate 1 Data");
    case 11:
        return QStringLiteral("Unified Single Block");
    default:
        return QStringLiteral("Reserved");
    }
}

bool qr1676Decode(const std::array<bool, 16> &receivedBits, int &data, int &errors) {
    const auto popcountInt = [](int value) {
        int count = 0;
        while (value != 0) {
            count += value & 0x01;
            value >>= 1;
        }
        return count;
    };

    int received = 0;
    for (int i = 0; i < 16; ++i) {
        received = (received << 1) | (receivedBits[i] ? 1 : 0);
    }

    int bestData = 0;
    int bestErrors = 17;
    for (int candidateData = 0; candidateData < 128; ++candidateData) {
        const int code16 = DMR_QR_1676_CODEWORDS[candidateData] & 0xffff;
        const int delta = received ^ code16;
        const int distance = popcountInt(delta);
        if (distance < bestErrors) {
            bestErrors = distance;
            bestData = candidateData;
        }
    }

    if (bestErrors > 2) {
        return false;
    }

    data = bestData;
    errors = bestErrors;
    return true;
}

QString embVariantName(int variant) {
    switch (variant) {
    case 0:
        return QStringLiteral("direct");
    case 1:
        return QStringLiteral("reverse16");
    case 2:
        return QStringLiteral("swap8");
    case 3:
        return QStringLiteral("reverse8");
    case 4:
        return QStringLiteral("swap+reverse8");
    case 5:
        return QStringLiteral("swapDibitBits");
    case 6:
        return QStringLiteral("reverseDibits");
    case 7:
        return QStringLiteral("reverseDibits+swapBits");
    default:
        return QStringLiteral("unknown");
    }
}

std::array<bool, 16> embBitsVariant(const std::array<bool, 16> &bits, int variant) {
    std::array<bool, 16> out = {};
    switch (variant) {
    case 1:
        for (int i = 0; i < 16; ++i) {
            out[i] = bits[15 - i];
        }
        break;
    case 2:
        for (int i = 0; i < 16; ++i) {
            out[i] = bits[(i + 8) & 0x0f];
        }
        break;
    case 3:
        for (int i = 0; i < 8; ++i) {
            out[i] = bits[7 - i];
            out[8 + i] = bits[15 - i];
        }
        break;
    case 4:
        for (int i = 0; i < 8; ++i) {
            out[i] = bits[15 - i];
            out[8 + i] = bits[7 - i];
        }
        break;
    case 5:
        for (int i = 0; i < 8; ++i) {
            out[i * 2] = bits[i * 2 + 1];
            out[i * 2 + 1] = bits[i * 2];
        }
        break;
    case 6:
        for (int i = 0; i < 8; ++i) {
            out[i * 2] = bits[(7 - i) * 2];
            out[i * 2 + 1] = bits[(7 - i) * 2 + 1];
        }
        break;
    case 7:
        for (int i = 0; i < 8; ++i) {
            out[i * 2] = bits[(7 - i) * 2 + 1];
            out[i * 2 + 1] = bits[(7 - i) * 2];
        }
        break;
    case 0:
    default:
        out = bits;
        break;
    }
    return out;
}

void appendDibitBits(int dibit, bool etsiMap, bool *rawBits, int bitIndex) {
    bool bit0 = false;
    bool bit1 = false;
    if (etsiMap) {
        switch (dibit) {
        case 3:
            bit0 = false;
            bit1 = true;
            break;
        case 2:
            bit0 = false;
            bit1 = false;
            break;
        case 0:
            bit0 = true;
            bit1 = false;
            break;
        case 1:
        default:
            bit0 = true;
            bit1 = true;
            break;
        }
    } else {
        bit0 = (dibit & 0x02) != 0;
        bit1 = (dibit & 0x01) != 0;
    }
    rawBits[bitIndex] = bit0;
    rawBits[bitIndex + 1] = bit1;
}

QString bitsToHex(const bool *bits, int bitCount) {
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

int hexNibbleValue(QChar ch) {
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

QString embeddedLc72HexFromBptcRaw(const QString &emb128) {
    if (emb128.size() != 32) {
        return QString();
    }

    std::array<bool, 128> txBits = {};
    for (int hexIndex = 0; hexIndex < emb128.size(); ++hexIndex) {
        const int value = hexNibbleValue(emb128.at(hexIndex));
        if (value < 0) {
            return QString();
        }
        for (int bit = 0; bit < 4; ++bit) {
            txBits[static_cast<std::size_t>(hexIndex * 4 + bit)] =
                ((value >> (3 - bit)) & 0x1) != 0;
        }
    }

    bool matrix[8][16] = {};
    for (int txIndex = 0; txIndex < 128; ++txIndex) {
        const int column = txIndex / 8;
        const int row = txIndex % 8;
        matrix[row][column] = txBits[static_cast<std::size_t>(txIndex)];
    }

    bool lcBits[72] = {};
    int out = 0;
    for (int row = 0; row < 6; ++row) {
        for (int column = 0; column < 11; ++column) {
            lcBits[out++] = matrix[row][column];
        }
    }
    for (int column = 0; column < 6; ++column) {
        lcBits[out++] = matrix[6][column];
    }

    return bitsToHex(lcBits, static_cast<int>(std::size(lcBits)));
}

} // namespace

void DmrDecoder::reset() {
    sampleBuffer.clear();
    pendingVoiceEmb.clear();
    voiceEmbeddedFrames.clear();
    voiceLcSequences.clear();
    voiceLcRawSinceReport.clear();
    dcEstimate = 0.0;
    averageMagnitude = 0.0;
    totalSamples = 0;
    lastReportSample = 0;
    lastAcceptedSyncSample = 0;
    haveAcceptedSync = false;
    haveLock = false;
    lockedPolarityInverted = false;
    lockedPhase = 0;
    lockedRfFrequencyHz = 0.0;
    confirmedSyncsSinceReport = 0;
    candidateSyncsSinceReport = 0;
    polarityFlipsSinceReport = 0;
    bestScoreSinceReport = 0;
    bestPhaseSinceReport = 0;
    bestQualitySinceReport = 0.0;
    bestPolarityInvertedSinceReport = false;
    lastConfirmedSyncSample = 0;
    spacingSamplesCount = 0;
    spacingMsSum = 0.0;
    spacingMsMin = 0.0;
    spacingMsMax = 0.0;
    std::fill(std::begin(phaseHistogram), std::end(phaseHistogram), 0);
    std::fill(std::begin(intervalHistogram), std::end(intervalHistogram), 0);
    cachOkSinceReport = 0;
    cachStrictOkSinceReport = 0;
    cachFailSinceReport = 0;
    std::fill(std::begin(cachChannelHistogram), std::end(cachChannelHistogram), 0);
    std::fill(std::begin(cachLcssHistogram), std::end(cachLcssHistogram), 0);
    std::fill(std::begin(cachTimingOffsetHistogram), std::end(cachTimingOffsetHistogram), 0);
    cachMapEtsiCount = 0;
    cachMapLegacyCount = 0;
    slotTypeOkSinceReport = 0;
    slotTypeFailSinceReport = 0;
    std::fill(std::begin(colorCodeHistogram), std::end(colorCodeHistogram), 0);
    std::fill(std::begin(dataTypeHistogram), std::end(dataTypeHistogram), 0);
    slotTypeCorrectedErrors = 0;
    embOkSinceReport = 0;
    embAnchorsSinceReport = 0;
    embWeakSinceReport = 0;
    embFailSinceReport = 0;
    std::fill(std::begin(embColorCodeHistogram), std::end(embColorCodeHistogram), 0);
    std::fill(std::begin(embLcssHistogram), std::end(embLcssHistogram), 0);
    embPrivacyCount = 0;
    embCorrectedErrors = 0;
    std::fill(std::begin(embVariantHistogram), std::end(embVariantHistogram), 0);
    std::fill(std::begin(embWeakVariantHistogram), std::end(embWeakVariantHistogram), 0);
    std::fill(std::begin(embErrorHistogram), std::end(embErrorHistogram), 0);
    std::fill(std::begin(embWeakErrorHistogram), std::end(embWeakErrorHistogram), 0);
    std::fill(std::begin(embWeakColorCodeHistogram), std::end(embWeakColorCodeHistogram), 0);
    std::fill(std::begin(embTimingOffsetHistogram), std::end(embTimingOffsetHistogram), 0);
    std::fill(std::begin(embWeakTimingOffsetHistogram), std::end(embWeakTimingOffsetHistogram), 0);
    lastVoiceEmbAnchorSample = 0;
    lastCachText.clear();
    lastPatternName.clear();
    lockedPatternName.clear();
    lastStatus.clear();
}

void DmrDecoder::configure(int sampleRate) {
    if (sampleRate <= 0) {
        sampleRate = 48000;
    }
    const int newSamplesPerSymbol =
        (std::max)(1, static_cast<int>(std::lround(sampleRate / DMR_SYMBOL_RATE)));
    if (activeSampleRate == sampleRate && samplesPerSymbol == newSamplesPerSymbol) {
        return;
    }

    activeSampleRate = sampleRate;
    samplesPerSymbol = newSamplesPerSymbol;
    reset();
}

DmrDecoder::Result DmrDecoder::processPcmFrame(const QByteArray &pcmData, int sampleRate, double rfFrequencyHz) {
    configure(sampleRate);
    const QString frequencySuffix = dmrFrequencySuffix(rfFrequencyHz);
    const auto lockedFrequencySuffix = [this, rfFrequencyHz]() {
        return dmrFrequencySuffix(lockedRfFrequencyHz > 0.0 ? lockedRfFrequencyHz : rfFrequencyHz);
    };

    Result result;
    if (pcmData.size() < static_cast<int>(sizeof(qint16))) {
        result.statusText = idleStatus();
        result.statusChanged = result.statusText != lastStatus;
        lastStatus = result.statusText;
        return result;
    }

    const int appendedSamples = pcmData.size() / static_cast<int>(sizeof(qint16));
    appendSamples(pcmData);
    processPendingVoiceEmb();
    SyncHit hit;
    const quint64 syncSpanSamples = static_cast<quint64>(DMR_SYNC_SYMBOLS * samplesPerSymbol);
    quint64 minimumSample = totalSamples > static_cast<quint64>(appendedSamples) + syncSpanSamples
                                ? totalSamples - static_cast<quint64>(appendedSamples) - syncSpanSamples
                                : 0;
    quint64 acceptedSyncGapSamples = syncSpanSamples;
    if (haveLock && lockedPatternName.contains(QStringLiteral("Voice"))) {
        acceptedSyncGapSamples =
            static_cast<quint64>(std::lround(DMR_VOICE_SYNC_MIN_GAP_SECONDS * activeSampleRate));
    }
    if (haveAcceptedSync) {
        minimumSample = (std::max)(minimumSample, lastAcceptedSyncSample + acceptedSyncGapSamples);
    }
    if (!findBestSync(hit, minimumSample)) {
        result.statusText = idleStatus();
        result.statusChanged = result.statusText != lastStatus;
        lastStatus = result.statusText;
        const quint64 lockTimeoutSamples =
            static_cast<quint64>(std::lround(DMR_LOCK_TIMEOUT_SECONDS * activeSampleRate));
        if (haveLock &&
            lastConfirmedSyncSample > 0 &&
            totalSamples >= lastConfirmedSyncSample + lockTimeoutSamples) {
            const double ageMs =
                1000.0 * static_cast<double>(totalSamples - lastConfirmedSyncSample) /
                static_cast<double>(activeSampleRate);
            result.decodedText =
                QStringLiteral("[DMR%1] lock lost: %2, last confirmed sync %3 ms ago\n")
                    .arg(lockedFrequencySuffix())
                    .arg(lockedPatternName)
                    .arg(ageMs, 0, 'f', 0);
            qDebug().noquote() << result.decodedText.trimmed();
            haveLock = false;
            lockedPatternName.clear();
            lockedRfFrequencyHz = 0.0;
            confirmedSyncsSinceReport = 0;
            candidateSyncsSinceReport = 0;
            polarityFlipsSinceReport = 0;
            bestScoreSinceReport = 0;
            bestPhaseSinceReport = 0;
            bestQualitySinceReport = 0.0;
            spacingSamplesCount = 0;
            spacingMsSum = 0.0;
            spacingMsMin = 0.0;
            spacingMsMax = 0.0;
            std::fill(std::begin(phaseHistogram), std::end(phaseHistogram), 0);
            std::fill(std::begin(intervalHistogram), std::end(intervalHistogram), 0);
            cachOkSinceReport = 0;
            cachStrictOkSinceReport = 0;
            cachFailSinceReport = 0;
            std::fill(std::begin(cachChannelHistogram), std::end(cachChannelHistogram), 0);
            std::fill(std::begin(cachLcssHistogram), std::end(cachLcssHistogram), 0);
            std::fill(std::begin(cachTimingOffsetHistogram), std::end(cachTimingOffsetHistogram), 0);
            cachMapEtsiCount = 0;
            cachMapLegacyCount = 0;
            slotTypeOkSinceReport = 0;
            slotTypeFailSinceReport = 0;
            std::fill(std::begin(colorCodeHistogram), std::end(colorCodeHistogram), 0);
            std::fill(std::begin(dataTypeHistogram), std::end(dataTypeHistogram), 0);
            slotTypeCorrectedErrors = 0;
            embOkSinceReport = 0;
            embAnchorsSinceReport = 0;
            embWeakSinceReport = 0;
            embFailSinceReport = 0;
            std::fill(std::begin(embColorCodeHistogram), std::end(embColorCodeHistogram), 0);
            std::fill(std::begin(embLcssHistogram), std::end(embLcssHistogram), 0);
            embPrivacyCount = 0;
            embCorrectedErrors = 0;
            lastVoiceEmbAnchorSample = 0;
            pendingVoiceEmb.clear();
            voiceEmbeddedFrames.clear();
            voiceLcSequences.clear();
            voiceLcRawSinceReport.clear();
            lastCachText.clear();
        }
        return result;
    }

    result.statusText = formatHit(hit);
    result.statusChanged = result.statusText != lastStatus;
    lastStatus = result.statusText;

    if (hit.score < DMR_CONFIRMED_SYNC_SCORE ||
        hit.outerScore < DMR_CONFIRMED_OUTER_SCORE) {
        ++candidateSyncsSinceReport;
        const quint64 reportIntervalSamples =
            static_cast<quint64>(std::lround(DMR_REPORT_INTERVAL_SECONDS * activeSampleRate));
        const bool reportDue =
            lastReportSample == 0 ||
            hit.absoluteSample >= lastReportSample + reportIntervalSamples;
        if (reportDue &&
            hit.score >= DMR_CANDIDATE_SYNC_SCORE &&
            hit.outerScore >= DMR_REPORTABLE_CANDIDATE_OUTER_SCORE) {
            lastReportSample = hit.absoluteSample;
            const QString patternName = QString::fromLatin1(hit.pattern->name);
            result.decodedText =
                QStringLiteral("[DMR%1] strong sync candidate: %2, sync %3/24, outer %4/24, phase %5, polarity %6, quality %7 dB\n")
                    .arg(frequencySuffix)
                    .arg(patternName)
                    .arg(hit.score)
                    .arg(hit.outerScore)
                    .arg(hit.phase)
                    .arg(hit.inverted ? QStringLiteral("inverted") : QStringLiteral("normal"))
                    .arg(hit.qualityDb, 0, 'f', 1);
            qDebug().noquote() << result.decodedText.trimmed();
        }
        return result;
    }

    const CachInfo strictCach = decodeCachBeforeSync(hit, false);
    const CachInfo cach = strictCach.decoded ? strictCach : decodeCachBeforeSync(hit, true);
    const bool baseStationVoice =
        hit.pattern &&
        std::strcmp(hit.pattern->source, "base station") == 0 &&
        std::strcmp(hit.pattern->kind, "voice") == 0;
    if (baseStationVoice &&
        hit.outerScore < DMR_TRUSTED_VOICE_OUTER_SCORE &&
        !cach.decoded) {
        ++candidateSyncsSinceReport;
        return result;
    }

    haveAcceptedSync = true;
    lastAcceptedSyncSample = hit.absoluteSample;

    const QString patternName = QString::fromLatin1(hit.pattern->name);
    const bool lockFrequencyChanged =
        lockedRfFrequencyHz > 0.0 &&
        rfFrequencyHz > 0.0 &&
        std::abs(lockedRfFrequencyHz - rfFrequencyHz) > 100.0;
    const bool polarityChangedWithinLock =
        haveLock &&
        lockedPatternName == patternName &&
        lockedPolarityInverted != hit.inverted &&
        !lockFrequencyChanged;
    const bool newLock = !haveLock ||
                         lockedPatternName != patternName ||
                         lockFrequencyChanged;
    if (newLock) {
        confirmedSyncsSinceReport = 0;
        candidateSyncsSinceReport = 0;
        polarityFlipsSinceReport = 0;
        bestScoreSinceReport = 0;
        bestPhaseSinceReport = 0;
        bestQualitySinceReport = 0.0;
        spacingSamplesCount = 0;
        spacingMsSum = 0.0;
        spacingMsMin = 0.0;
        spacingMsMax = 0.0;
        std::fill(std::begin(phaseHistogram), std::end(phaseHistogram), 0);
        std::fill(std::begin(intervalHistogram), std::end(intervalHistogram), 0);
        cachOkSinceReport = 0;
        cachStrictOkSinceReport = 0;
        cachFailSinceReport = 0;
        std::fill(std::begin(cachChannelHistogram), std::end(cachChannelHistogram), 0);
        std::fill(std::begin(cachLcssHistogram), std::end(cachLcssHistogram), 0);
        std::fill(std::begin(cachTimingOffsetHistogram), std::end(cachTimingOffsetHistogram), 0);
        cachMapEtsiCount = 0;
        cachMapLegacyCount = 0;
        slotTypeOkSinceReport = 0;
        slotTypeFailSinceReport = 0;
        std::fill(std::begin(colorCodeHistogram), std::end(colorCodeHistogram), 0);
        std::fill(std::begin(dataTypeHistogram), std::end(dataTypeHistogram), 0);
        slotTypeCorrectedErrors = 0;
        embOkSinceReport = 0;
        embAnchorsSinceReport = 0;
        embWeakSinceReport = 0;
        embFailSinceReport = 0;
        std::fill(std::begin(embColorCodeHistogram), std::end(embColorCodeHistogram), 0);
        std::fill(std::begin(embLcssHistogram), std::end(embLcssHistogram), 0);
        embPrivacyCount = 0;
        embCorrectedErrors = 0;
        lastVoiceEmbAnchorSample = 0;
        pendingVoiceEmb.clear();
        voiceEmbeddedFrames.clear();
        voiceLcSequences.clear();
        voiceLcRawSinceReport.clear();
        lastCachText.clear();
    }
    if (polarityChangedWithinLock) {
        ++polarityFlipsSinceReport;
    }
    haveLock = true;
    lockedPatternName = patternName;
    if (newLock) {
        lockedPolarityInverted = hit.inverted;
        lockedRfFrequencyHz = rfFrequencyHz;
    }
    lastPatternName = patternName;

    if (lastConfirmedSyncSample > 0 && hit.absoluteSample > lastConfirmedSyncSample) {
        const double spacingMs =
            1000.0 * static_cast<double>(hit.absoluteSample - lastConfirmedSyncSample) /
            static_cast<double>(activeSampleRate);
        if (spacingMs >= 20.0 && spacingMs <= 400.0) {
            spacingMsSum += spacingMs;
            ++spacingSamplesCount;
            const int gridStep = (std::clamp)(static_cast<int>(std::lround(spacingMs / 30.0)), 1, 12);
            ++intervalHistogram[gridStep - 1];
            if (spacingSamplesCount == 1) {
                spacingMsMin = spacingMs;
                spacingMsMax = spacingMs;
            } else {
                spacingMsMin = (std::min)(spacingMsMin, spacingMs);
                spacingMsMax = (std::max)(spacingMsMax, spacingMs);
            }
        }
    }
    lastConfirmedSyncSample = hit.absoluteSample;

    ++confirmedSyncsSinceReport;
    if (hit.phase >= 0 && hit.phase < static_cast<int>(std::size(phaseHistogram))) {
        ++phaseHistogram[hit.phase];
    }
    if (cach.decoded) {
        ++cachOkSinceReport;
        if (strictCach.decoded) {
            ++cachStrictOkSinceReport;
        }
        if (cach.channel >= 1 && cach.channel <= 2) {
            ++cachChannelHistogram[cach.channel - 1];
        }
        if (cach.lcss >= 0 && cach.lcss < static_cast<int>(std::size(cachLcssHistogram))) {
            ++cachLcssHistogram[cach.lcss];
        }
        const int timingOffsetIndex = cach.timingOffset + 2;
        if (timingOffsetIndex >= 0 && timingOffsetIndex < static_cast<int>(std::size(cachTimingOffsetHistogram))) {
            ++cachTimingOffsetHistogram[timingOffsetIndex];
        }
        if (cach.mapName == QStringLiteral("ETSI")) {
            ++cachMapEtsiCount;
        } else {
            ++cachMapLegacyCount;
        }
        lastCachText =
            QStringLiteral("CACH AT=%1 Ch %2 %3")
                .arg(cach.accessType ? 1 : 0)
                .arg(cach.channel)
                .arg(cachLcssText(cach.lcss));
    } else if (hit.pattern && std::strcmp(hit.pattern->source, "base station") == 0) {
        ++cachFailSinceReport;
    }

    const SlotTypeInfo slotType = decodeSlotTypeAroundSync(hit);
    if (slotType.decoded) {
        qDebug() << "[DMR] slot type"
                 << hit.pattern->name
                 << "cc" << slotType.colorCode
                 << "dataType" << slotType.dataTypeName
                 << "errors" << slotType.correctedErrors
                 << "timing" << slotType.timingOffset
                 << "slicer" << slotType.slicerRatio
                 << "map" << slotType.mapName;
        ++slotTypeOkSinceReport;
        if (slotType.colorCode >= 0 && slotType.colorCode < static_cast<int>(std::size(colorCodeHistogram))) {
            ++colorCodeHistogram[slotType.colorCode];
        }
        if (slotType.dataType >= 0 && slotType.dataType < static_cast<int>(std::size(dataTypeHistogram))) {
            ++dataTypeHistogram[slotType.dataType];
        }
        slotTypeCorrectedErrors += slotType.correctedErrors;
    } else if (hit.pattern && std::strcmp(hit.pattern->kind, "data") == 0) {
        ++slotTypeFailSinceReport;
    }

    if (hit.pattern && std::strcmp(hit.pattern->kind, "voice") == 0) {
        scheduleVoiceEmbBursts(hit, cach);
    }
    if (hit.score > bestScoreSinceReport || hit.qualityDb > bestQualitySinceReport) {
        bestScoreSinceReport = hit.score;
        bestPhaseSinceReport = hit.phase;
        bestQualitySinceReport = hit.qualityDb;
        bestPolarityInvertedSinceReport = hit.inverted;
    }

    const quint64 reportIntervalSamples =
        static_cast<quint64>(std::lround(DMR_REPORT_INTERVAL_SECONDS * activeSampleRate));
    const bool reportDue =
        lastReportSample == 0 ||
        hit.absoluteSample >= lastReportSample + reportIntervalSamples;

    if (newLock) {
        lastReportSample = hit.absoluteSample;
        result.decodedText =
            QStringLiteral("[DMR%1] lock acquired: %2, sync %3/24, outer %4/24, phase %5, polarity %6, quality %7 dB\n")
                .arg(frequencySuffix)
                .arg(patternName)
                .arg(hit.score)
                .arg(hit.outerScore)
                .arg(hit.phase)
                .arg(hit.inverted ? QStringLiteral("inverted") : QStringLiteral("normal"))
                .arg(hit.qualityDb, 0, 'f', 1);
    } else if (reportDue) {
        int dominantPhase = 0;
        int dominantPhaseCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(phaseHistogram)); ++i) {
            if (phaseHistogram[i] > dominantPhaseCount) {
                dominantPhaseCount = phaseHistogram[i];
                dominantPhase = i;
            }
        }
        lockedPhase = dominantPhase;

        int dominantIntervalBin = 0;
        int dominantIntervalCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(intervalHistogram)); ++i) {
            if (intervalHistogram[i] > dominantIntervalCount) {
                dominantIntervalCount = intervalHistogram[i];
                dominantIntervalBin = i + 1;
            }
        }

        int dominantCachChannel = 0;
        int dominantCachChannelCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(cachChannelHistogram)); ++i) {
            if (cachChannelHistogram[i] > dominantCachChannelCount) {
                dominantCachChannelCount = cachChannelHistogram[i];
                dominantCachChannel = i + 1;
            }
        }
        int dominantLcss = -1;
        int dominantLcssCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(cachLcssHistogram)); ++i) {
            if (cachLcssHistogram[i] > dominantLcssCount) {
                dominantLcssCount = cachLcssHistogram[i];
                dominantLcss = i;
            }
        }
        int dominantCachTimingOffset = 0;
        int dominantCachTimingOffsetCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(cachTimingOffsetHistogram)); ++i) {
            if (cachTimingOffsetHistogram[i] > dominantCachTimingOffsetCount) {
                dominantCachTimingOffsetCount = cachTimingOffsetHistogram[i];
                dominantCachTimingOffset = i - 2;
            }
        }
        const QString timingOffsetText =
            dominantCachTimingOffset > 0
                ? QStringLiteral("+%1").arg(dominantCachTimingOffset)
                : QString::number(dominantCachTimingOffset);
        const QString cachMapText =
            cachMapEtsiCount >= cachMapLegacyCount
                ? QStringLiteral("ETSI %1/%2").arg(cachMapEtsiCount).arg(cachOkSinceReport)
                : QStringLiteral("legacy %1/%2").arg(cachMapLegacyCount).arg(cachOkSinceReport);

        int dominantColorCode = -1;
        int dominantColorCodeCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(colorCodeHistogram)); ++i) {
            if (colorCodeHistogram[i] > dominantColorCodeCount) {
                dominantColorCodeCount = colorCodeHistogram[i];
                dominantColorCode = i;
            }
        }
        int dominantDataType = -1;
        int dominantDataTypeCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(dataTypeHistogram)); ++i) {
            if (dataTypeHistogram[i] > dominantDataTypeCount) {
                dominantDataTypeCount = dataTypeHistogram[i];
                dominantDataType = i;
            }
        }
        const QString slotTypeText =
            slotTypeOkSinceReport > 0
                ? QStringLiteral(", SlotType %1/%2 ok, CC %3, %4, Golay corrected %5")
                      .arg(slotTypeOkSinceReport)
                      .arg(slotTypeOkSinceReport + slotTypeFailSinceReport)
                      .arg(dominantColorCode >= 0 ? QString::number(dominantColorCode) : QStringLiteral("?"))
                      .arg(dominantDataType >= 0 ? dmrDataTypeName(dominantDataType) : QStringLiteral("Data Type ?"))
                      .arg(slotTypeCorrectedErrors)
                : (std::strcmp(hit.pattern->kind, "voice") == 0
                       ? QStringLiteral(", SlotType/CC pending on voice sync")
                       : (slotTypeFailSinceReport > 0
                              ? QStringLiteral(", SlotType 0/%1 ok").arg(slotTypeFailSinceReport)
                              : QString()));

        int dominantEmbColorCode = -1;
        int dominantEmbColorCodeCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embColorCodeHistogram)); ++i) {
            if (embColorCodeHistogram[i] > dominantEmbColorCodeCount) {
                dominantEmbColorCodeCount = embColorCodeHistogram[i];
                dominantEmbColorCode = i;
            }
        }
        int dominantEmbLcss = -1;
        int dominantEmbLcssCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embLcssHistogram)); ++i) {
            if (embLcssHistogram[i] > dominantEmbLcssCount) {
                dominantEmbLcssCount = embLcssHistogram[i];
                dominantEmbLcss = i;
            }
        }
        int dominantEmbVariant = -1;
        int dominantEmbVariantCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embVariantHistogram)); ++i) {
            if (embVariantHistogram[i] > dominantEmbVariantCount) {
                dominantEmbVariantCount = embVariantHistogram[i];
                dominantEmbVariant = i;
            }
        }
        int dominantWeakEmbVariant = -1;
        int dominantWeakEmbVariantCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embWeakVariantHistogram)); ++i) {
            if (embWeakVariantHistogram[i] > dominantWeakEmbVariantCount) {
                dominantWeakEmbVariantCount = embWeakVariantHistogram[i];
                dominantWeakEmbVariant = i;
            }
        }
        int dominantWeakEmbColorCode = -1;
        int dominantWeakEmbColorCodeCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embWeakColorCodeHistogram)); ++i) {
            if (embWeakColorCodeHistogram[i] > dominantWeakEmbColorCodeCount) {
                dominantWeakEmbColorCodeCount = embWeakColorCodeHistogram[i];
                dominantWeakEmbColorCode = i;
            }
        }
        int dominantEmbTimingOffset = 0;
        int dominantEmbTimingOffsetCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embTimingOffsetHistogram)); ++i) {
            if (embTimingOffsetHistogram[i] > dominantEmbTimingOffsetCount) {
                dominantEmbTimingOffsetCount = embTimingOffsetHistogram[i];
                dominantEmbTimingOffset = i - 2;
            }
        }
        int dominantWeakEmbTimingOffset = 0;
        int dominantWeakEmbTimingOffsetCount = 0;
        for (int i = 0; i < static_cast<int>(std::size(embWeakTimingOffsetHistogram)); ++i) {
            if (embWeakTimingOffsetHistogram[i] > dominantWeakEmbTimingOffsetCount) {
                dominantWeakEmbTimingOffsetCount = embWeakTimingOffsetHistogram[i];
                dominantWeakEmbTimingOffset = i - 2;
            }
        }
        const auto signedOffsetText = [](int offset) {
            return offset > 0 ? QStringLiteral("+%1").arg(offset) : QString::number(offset);
        };
        const QString embCcText =
            (dominantEmbColorCode >= 0 &&
             dominantEmbColorCodeCount >= 3 &&
             dominantEmbColorCodeCount * 2 >= embOkSinceReport)
                ? QString::number(dominantEmbColorCode)
                : (dominantEmbColorCode >= 0
                       ? QStringLiteral("%1? %2/%3")
                             .arg(dominantEmbColorCode)
                             .arg(dominantEmbColorCodeCount)
                             .arg(embOkSinceReport)
                       : QStringLiteral("?"));
        const QString weakEmbCcText =
            dominantWeakEmbColorCode >= 0
                ? QStringLiteral(", weak CC %1 %2/%3")
                      .arg(dominantWeakEmbColorCode)
                      .arg(dominantWeakEmbColorCodeCount)
                      .arg(embWeakSinceReport)
                : QString();
        const QString embVariantText =
            dominantEmbVariant >= 0
                ? QStringLiteral(", map %1 %2/%3")
                      .arg(embVariantName(dominantEmbVariant))
                      .arg(dominantEmbVariantCount)
                      .arg(embOkSinceReport)
                : QString();
        const QString weakEmbVariantText =
            dominantWeakEmbVariant >= 0
                ? QStringLiteral(", weak map %1 %2/%3")
                      .arg(embVariantName(dominantWeakEmbVariant))
                      .arg(dominantWeakEmbVariantCount)
                      .arg(embWeakSinceReport)
                : QString();
        const QString embErrorText =
            embOkSinceReport > 0
                ? QStringLiteral(", QR e0/e1 %1/%2, timing %3 (%4/%5)")
                      .arg(embErrorHistogram[0])
                      .arg(embErrorHistogram[1])
                      .arg(signedOffsetText(dominantEmbTimingOffset))
                      .arg(dominantEmbTimingOffsetCount)
                      .arg(embOkSinceReport)
                : QString();
        const QString weakEmbErrorText =
            embWeakSinceReport > 0
                ? QStringLiteral(", weak QR e2 %1/%2, timing %3 (%4/%2)")
                      .arg(embWeakErrorHistogram[2])
                      .arg(embWeakSinceReport)
                      .arg(signedOffsetText(dominantWeakEmbTimingOffset))
                      .arg(dominantWeakEmbTimingOffsetCount)
                : QString();
        const QString embText =
            embOkSinceReport > 0
                ? QStringLiteral(", EMB anchors %1, corrected %2/%3, weak %4, CC %5, PI %6/%2, %7, QR corrected %8%9%10")
                      .arg(embAnchorsSinceReport)
                      .arg(embOkSinceReport)
                      .arg(embOkSinceReport + embWeakSinceReport + embFailSinceReport)
                      .arg(embWeakSinceReport)
                      .arg(embCcText)
                      .arg(embPrivacyCount)
                      .arg(dominantEmbLcss >= 0 ? cachLcssText(dominantEmbLcss) : QStringLiteral("LCSS ?"))
                      .arg(embCorrectedErrors)
                      .arg(embVariantText)
                      .arg(embErrorText)
                : (embWeakSinceReport > 0
                       ? QStringLiteral(", EMB anchors %1, corrected 0/%2, weak %3%4%5%6")
                             .arg(embAnchorsSinceReport)
                             .arg(embWeakSinceReport + embFailSinceReport)
                             .arg(embWeakSinceReport)
                             .arg(weakEmbCcText)
                             .arg(weakEmbVariantText)
                             .arg(weakEmbErrorText)
                : (embFailSinceReport > 0
                       ? QStringLiteral(", EMB anchors %1, 0/%2 ok")
                             .arg(embAnchorsSinceReport)
                             .arg(embFailSinceReport)
                       : QString()));
        const QString polarityFlipText =
            polarityFlipsSinceReport > 0
                ? QStringLiteral(", polarity flips %1").arg(polarityFlipsSinceReport)
                : QString();
        const QString voiceLcRawText = voiceLcRawSummaryText();

        lastReportSample = hit.absoluteSample;
        const QString intervalText =
            spacingSamplesCount > 0
                ? QStringLiteral(", interval %1 ms avg (%2-%3), burst step ~%4 ms (%5/%6)")
                      .arg(spacingMsSum / static_cast<double>(spacingSamplesCount), 0, 'f', 1)
                      .arg(spacingMsMin, 0, 'f', 1)
                      .arg(spacingMsMax, 0, 'f', 1)
                      .arg(dominantIntervalBin * 30)
                      .arg(dominantIntervalCount)
                      .arg(spacingSamplesCount)
                : QString();
        const QString cachText =
            cachOkSinceReport > 0
                ? QStringLiteral(", CACH strict %1/%3, search %2/%3 ok, Ch %4, %5, timing %6 (%7/%2), map %8")
                      .arg(cachStrictOkSinceReport)
                      .arg(cachOkSinceReport)
                      .arg(cachOkSinceReport + cachFailSinceReport)
                      .arg(dominantCachChannel > 0 ? QString::number(dominantCachChannel) : QStringLiteral("?"))
                      .arg(dominantLcss >= 0 ? cachLcssText(dominantLcss) : QStringLiteral("TACT"))
                      .arg(timingOffsetText)
                      .arg(dominantCachTimingOffsetCount)
                      .arg(cachMapText)
                : (cachFailSinceReport > 0
                       ? QStringLiteral(", CACH 0/%1 ok").arg(cachFailSinceReport)
                       : QString());
        const bool shortBurstReport =
            confirmedSyncsSinceReport <= 1 &&
            spacingSamplesCount == 0 &&
            embOkSinceReport == 0;
        if (shortBurstReport) {
            const QString embBriefText =
                embWeakSinceReport > 0
                    ? QStringLiteral(", EMB weak only %1/%2")
                          .arg(embWeakSinceReport)
                          .arg(embWeakSinceReport + embFailSinceReport)
                    : (embFailSinceReport > 0
                           ? QStringLiteral(", EMB 0/%1 ok").arg(embFailSinceReport)
                           : QString());
            result.decodedText =
                QStringLiteral("[DMR%1] short %2 burst: %3 accepted sync, %4 candidates, phase %5, polarity %6, best %7/24 %8 dB%9%10%11%12\n")
                    .arg(lockedFrequencySuffix())
                    .arg(patternName)
                    .arg(confirmedSyncsSinceReport)
                    .arg(candidateSyncsSinceReport)
                    .arg(dominantPhase)
                    .arg(bestPolarityInvertedSinceReport ? QStringLiteral("inverted") : QStringLiteral("normal"))
                    .arg(bestScoreSinceReport)
                    .arg(bestQualitySinceReport, 0, 'f', 1)
                    .arg(polarityFlipText)
                    .arg(cachText)
                    .arg(embBriefText)
                    .arg(voiceLcRawText);
        } else {
            result.decodedText =
                QStringLiteral("[DMR%1] %2: %3 accepted syncs, %4 candidates, dominant phase %5, locked phase %6, polarity %7, best %8/24 %9 dB%10%11%12%13%14%15\n")
                    .arg(lockedFrequencySuffix())
                    .arg(patternName)
                    .arg(confirmedSyncsSinceReport)
                    .arg(candidateSyncsSinceReport)
                    .arg(dominantPhase)
                    .arg(lockedPhase)
                    .arg(bestPolarityInvertedSinceReport ? QStringLiteral("inverted") : QStringLiteral("normal"))
                    .arg(bestScoreSinceReport)
                    .arg(bestQualitySinceReport, 0, 'f', 1)
                    .arg(intervalText)
                    .arg(cachText)
                    .arg(slotTypeText)
                    .arg(embText)
                    .arg(voiceLcRawText)
                    .arg(polarityFlipText);
        }
    }

    if (!result.decodedText.isEmpty()) {
        qDebug().noquote() << result.decodedText.trimmed();
        qDebug() << "[DMR] sync"
                 << hit.pattern->name
                 << "score" << hit.score
                 << "outer" << hit.outerScore
                 << "errors" << hit.errors
                 << "phase" << hit.phase
                 << "inverted" << hit.inverted
                 << "qualityDb" << hit.qualityDb;
        confirmedSyncsSinceReport = 0;
        candidateSyncsSinceReport = 0;
        polarityFlipsSinceReport = 0;
        bestScoreSinceReport = 0;
        bestPhaseSinceReport = 0;
        bestQualitySinceReport = 0.0;
        spacingSamplesCount = 0;
        spacingMsSum = 0.0;
        spacingMsMin = 0.0;
        spacingMsMax = 0.0;
        std::fill(std::begin(phaseHistogram), std::end(phaseHistogram), 0);
        std::fill(std::begin(intervalHistogram), std::end(intervalHistogram), 0);
        cachOkSinceReport = 0;
        cachStrictOkSinceReport = 0;
        cachFailSinceReport = 0;
        std::fill(std::begin(cachChannelHistogram), std::end(cachChannelHistogram), 0);
        std::fill(std::begin(cachLcssHistogram), std::end(cachLcssHistogram), 0);
        std::fill(std::begin(cachTimingOffsetHistogram), std::end(cachTimingOffsetHistogram), 0);
        cachMapEtsiCount = 0;
        cachMapLegacyCount = 0;
        slotTypeOkSinceReport = 0;
        slotTypeFailSinceReport = 0;
        std::fill(std::begin(colorCodeHistogram), std::end(colorCodeHistogram), 0);
        std::fill(std::begin(dataTypeHistogram), std::end(dataTypeHistogram), 0);
        slotTypeCorrectedErrors = 0;
        embOkSinceReport = 0;
        embAnchorsSinceReport = 0;
        embWeakSinceReport = 0;
        embFailSinceReport = 0;
        std::fill(std::begin(embColorCodeHistogram), std::end(embColorCodeHistogram), 0);
        std::fill(std::begin(embLcssHistogram), std::end(embLcssHistogram), 0);
        embPrivacyCount = 0;
        embCorrectedErrors = 0;
        std::fill(std::begin(embVariantHistogram), std::end(embVariantHistogram), 0);
        std::fill(std::begin(embWeakVariantHistogram), std::end(embWeakVariantHistogram), 0);
        std::fill(std::begin(embErrorHistogram), std::end(embErrorHistogram), 0);
        std::fill(std::begin(embWeakErrorHistogram), std::end(embWeakErrorHistogram), 0);
        std::fill(std::begin(embWeakColorCodeHistogram), std::end(embWeakColorCodeHistogram), 0);
        std::fill(std::begin(embTimingOffsetHistogram), std::end(embTimingOffsetHistogram), 0);
        std::fill(std::begin(embWeakTimingOffsetHistogram), std::end(embWeakTimingOffsetHistogram), 0);
        voiceLcRawSinceReport.clear();
    }

    return result;
}

QString DmrDecoder::voiceLcRawSummaryText() const {
    if (voiceLcRawSinceReport.empty()) {
        return QString();
    }

    QString dominantRaw;
    int dominantCount = 0;
    for (const QString &candidate : voiceLcRawSinceReport) {
        int count = 0;
        for (const QString &other : voiceLcRawSinceReport) {
            if (other == candidate) {
                ++count;
            }
        }
        if (count > dominantCount) {
            dominantRaw = candidate;
            dominantCount = count;
        }
    }

    return QStringLiteral(", LC raw %1 (%2/%3)")
        .arg(dominantRaw)
        .arg(dominantCount)
        .arg(static_cast<int>(voiceLcRawSinceReport.size())) +
        (embeddedLc72HexFromBptcRaw(dominantRaw).isEmpty()
             ? QString()
             : QStringLiteral(", LC72 %1").arg(embeddedLc72HexFromBptcRaw(dominantRaw)));
}

void DmrDecoder::appendSamples(const QByteArray &pcmData) {
    const int sampleCount = pcmData.size() / static_cast<int>(sizeof(qint16));
    const char *raw = pcmData.constData();
    sampleBuffer.resize(sampleBuffer.size() + static_cast<std::size_t>(sampleCount));

    for (int i = 0; i < sampleCount; ++i) {
        const float sample = readPcm16Le(raw + i * static_cast<int>(sizeof(qint16))) / 32768.0f;
        dcEstimate += 0.00035 * (sample - dcEstimate);
        const float centered = static_cast<float>(sample - dcEstimate);
        averageMagnitude += 0.0006 * (std::abs(centered) - averageMagnitude);
        sampleBuffer[static_cast<std::size_t>(sampleBuffer.size() - sampleCount + i)] = centered;
    }

    totalSamples += static_cast<quint64>(sampleCount);
    const std::size_t maxSamples =
        static_cast<std::size_t>(DMR_MAX_BUFFER_SYMBOLS * samplesPerSymbol + DMR_SYNC_SYMBOLS * samplesPerSymbol);
    while (sampleBuffer.size() > maxSamples) {
        sampleBuffer.pop_front();
    }
}

bool DmrDecoder::findBestSync(SyncHit &hit, quint64 minimumAbsoluteSample) const {
    if (sampleBuffer.size() < static_cast<std::size_t>(DMR_SYNC_SYMBOLS * samplesPerSymbol) ||
        averageMagnitude < 0.002) {
        return false;
    }

    SyncHit best;
    const int symbolCapacity = static_cast<int>(sampleBuffer.size()) / samplesPerSymbol;
    if (symbolCapacity < DMR_SYNC_SYMBOLS) {
        return false;
    }

    const quint64 bufferStartSample = totalSamples > sampleBuffer.size()
                                          ? totalSamples - static_cast<quint64>(sampleBuffer.size())
                                          : 0;
    for (int phase = 0; phase < samplesPerSymbol; ++phase) {
        const int maxStart = (static_cast<int>(sampleBuffer.size()) - phase) / samplesPerSymbol - DMR_SYNC_SYMBOLS;
        if (maxStart < 0) {
            continue;
        }
        for (int symbolStart = 0; symbolStart <= maxStart; ++symbolStart) {
            const quint64 absoluteSample =
                bufferStartSample + static_cast<quint64>(symbolStart * samplesPerSymbol + phase);
            if (absoluteSample < minimumAbsoluteSample) {
                continue;
            }
            for (const SyncPattern &pattern : DMR_SYNC_PATTERNS) {
                for (bool inverted : {false, true}) {
                    const int startIndex = phase + symbolStart * samplesPerSymbol;
                    int outerScore = 0;
                    const int score = scoreSyncSymbols(sampleBuffer,
                                                       startIndex,
                                                       samplesPerSymbol,
                                                       pattern.symbols,
                                                       inverted,
                                                       &outerScore);
                    if (score < DMR_CANDIDATE_SYNC_SCORE ||
                        outerScore < DMR_CANDIDATE_OUTER_SCORE) {
                        continue;
                    }

                    SyncHit candidate;
                    candidate.pattern = &pattern;
                    candidate.score = score;
                    candidate.errors = DMR_SYNC_SYMBOLS - score;
                    candidate.outerScore = outerScore;
                    candidate.phase = phase;
                    candidate.symbolIndex = symbolStart;
                    candidate.inverted = inverted;
                    candidate.qualityDb = syncQualityDb(score);
                    candidate.absoluteSample = absoluteSample;
                    const bool candidateVoice = std::strcmp(candidate.pattern->kind, "voice") == 0;
                    const bool bestVoice = best.pattern && std::strcmp(best.pattern->kind, "voice") == 0;
                    const bool candidateMatchesLockedPolarity =
                        haveLock &&
                        lockedPatternName == QString::fromLatin1(candidate.pattern->name) &&
                        candidate.inverted == lockedPolarityInverted;
                    const bool bestMatchesLockedPolarity =
                        haveLock &&
                        best.pattern &&
                        lockedPatternName == QString::fromLatin1(best.pattern->name) &&
                        best.inverted == lockedPolarityInverted;
                    const bool lockedPolarityWithinMargin =
                        best.pattern &&
                        candidate.score == best.score &&
                        candidateVoice == bestVoice &&
                        candidateMatchesLockedPolarity &&
                        !bestMatchesLockedPolarity &&
                        candidate.outerScore + DMR_LOCKED_POLARITY_OUTER_MARGIN >= best.outerScore;
                    const bool outerScoreWins =
                        best.pattern &&
                        candidate.score == best.score &&
                        candidate.outerScore > best.outerScore &&
                        !(bestMatchesLockedPolarity &&
                          !candidateMatchesLockedPolarity &&
                          best.outerScore + DMR_LOCKED_POLARITY_OUTER_MARGIN >= candidate.outerScore);
                    const bool preferCandidate =
                        !best.pattern ||
                        candidate.score > best.score ||
                        lockedPolarityWithinMargin ||
                        outerScoreWins ||
                        (candidate.score == best.score &&
                         candidate.outerScore == best.outerScore &&
                        candidateVoice &&
                        !bestVoice) ||
                        (candidate.score == best.score &&
                         candidate.outerScore == best.outerScore &&
                         candidateVoice == bestVoice &&
                         candidateMatchesLockedPolarity &&
                         !bestMatchesLockedPolarity) ||
                        (candidate.score == best.score &&
                         candidate.outerScore == best.outerScore &&
                         candidateVoice == bestVoice &&
                         !candidate.inverted &&
                         best.inverted) ||
                        (candidate.score == best.score &&
                         candidate.outerScore == best.outerScore &&
                         candidateVoice == bestVoice &&
                         candidate.inverted == best.inverted &&
                         candidate.absoluteSample > best.absoluteSample);

                    if (preferCandidate) {
                        best = candidate;
                    }
                }
            }
        }
    }

    if (!best.pattern || best.score < DMR_CANDIDATE_SYNC_SCORE) {
        return false;
    }

    hit = best;
    return true;
}

DmrDecoder::CachInfo DmrDecoder::decodeCachBeforeSync(const SyncHit &hit, bool allowTimingSearch) const {
    CachInfo info;
    if (!hit.pattern || std::strcmp(hit.pattern->source, "base station") != 0) {
        return info;
    }

    const int syncSampleIndex = hit.phase + hit.symbolIndex * samplesPerSymbol;
    const int cachStartSampleIndex =
        syncSampleIndex - (DMR_SYMBOLS_BEFORE_SYNC + DMR_CACH_SYMBOLS) * samplesPerSymbol;
    const int cachEndSampleIndex =
        cachStartSampleIndex + (DMR_CACH_SYMBOLS - 1) * samplesPerSymbol + samplesPerSymbol / 2;
    if (cachStartSampleIndex < 0 ||
        cachEndSampleIndex < 0 ||
        cachEndSampleIndex >= static_cast<int>(sampleBuffer.size())) {
        return info;
    }

    float minSync = std::numeric_limits<float>::max();
    float maxSync = std::numeric_limits<float>::lowest();
    for (int i = 0; i < DMR_SYNC_SYMBOLS; ++i) {
        const int sampleIndex =
            syncSampleIndex + i * samplesPerSymbol + samplesPerSymbol / 2;
        if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
            return info;
        }
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        minSync = (std::min)(minSync, sample);
        maxSync = (std::max)(maxSync, sample);
    }
    if (maxSync - minSync < 0.004f) {
        return info;
    }

    const float center = 0.5f * (maxSync + minSync);
    const auto sampleToDibit = [&](float sample, float thresholdRatio) -> int {
        const float upperMid = center + thresholdRatio * (maxSync - center);
        const float lowerMid = center + thresholdRatio * (minSync - center);
        const float corrected = hit.inverted ? -sample : sample;
        const float correctedCenter = hit.inverted ? -center : center;
        const float correctedUpper = hit.inverted ? -lowerMid : upperMid;
        const float correctedLower = hit.inverted ? -upperMid : lowerMid;
        if (corrected > correctedCenter) {
            return corrected > correctedUpper ? 3 : 2;
        }
        return corrected < correctedLower ? 1 : 0;
    };

    std::array<int, 5> timingOffsets = {
        samplesPerSymbol / 2,
        (std::max)(0, samplesPerSymbol / 2 - 1),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 1),
        (std::max)(0, samplesPerSymbol / 2 - 2),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 2)
    };
    if (!allowTimingSearch) {
        timingOffsets = {
            samplesPerSymbol / 2,
            samplesPerSymbol / 2,
            samplesPerSymbol / 2,
            samplesPerSymbol / 2,
            samplesPerSymbol / 2
        };
    }

    const std::array<float, 3> slicerRatios = {0.625f, 0.55f, 0.70f};
    const std::array<bool, 2> mapVariants = {true, false};

    for (int timingOffset : timingOffsets) {
        for (float slicerRatio : slicerRatios) {
            for (bool etsiMap : mapVariants) {
                bool rawBits[24] = {};
                for (int i = 0; i < DMR_CACH_SYMBOLS; ++i) {
                    const int sampleIndex =
                        cachStartSampleIndex + i * samplesPerSymbol + timingOffset;
                    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
                        return info;
                    }
                    const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
                    const int dibit =
                        (std::clamp)(sampleToDibit(sample, slicerRatio),
                                     0,
                                     3);
                    appendDibitBits(dibit, etsiMap, rawBits, i * 2);
                }

                bool dataBits[24] = {};
                for (int i = 0; i < 24; ++i) {
                    dataBits[i] = rawBits[DMR_CACH_DEINTERLEAVE[i]];
                }

                int tact = 0;
                for (int i = 0; i < 7; ++i) {
                    tact = (tact << 1) | (dataBits[i] ? 1 : 0);
                }
                if (!hamming743Ok(tact)) {
                    continue;
                }

                info.decoded = true;
                info.accessType = dataBits[0];
                info.channel = dataBits[1] ? 2 : 1;
                info.lcss = (dataBits[2] ? 2 : 0) | (dataBits[3] ? 1 : 0);
                info.timingOffset = timingOffset - samplesPerSymbol / 2;
                info.slicerRatio = slicerRatio;
                info.mapName = etsiMap ? QStringLiteral("ETSI") : QStringLiteral("legacy");
                QString payload;
                payload.reserve(17);
                for (int i = 7; i < 24; ++i) {
                    payload.append(dataBits[i] ? QLatin1Char('1') : QLatin1Char('0'));
                }
                info.payloadBits = payload;
                return info;
            }
        }
    }
    return info;
}

DmrDecoder::SlotTypeInfo DmrDecoder::decodeSlotTypeAroundSync(const SyncHit &hit) const {
    SlotTypeInfo info;
    if (!hit.pattern || std::strcmp(hit.pattern->kind, "data") != 0) {
        return info;
    }

    const int syncSampleIndex = hit.phase + hit.symbolIndex * samplesPerSymbol;
    const int firstSlotTypeSampleIndex = syncSampleIndex - 5 * samplesPerSymbol;
    const int secondSlotTypeSampleIndex = syncSampleIndex + DMR_SYNC_SYMBOLS * samplesPerSymbol;
    const int lastSampleIndex = secondSlotTypeSampleIndex + 4 * samplesPerSymbol + samplesPerSymbol / 2;
    if (firstSlotTypeSampleIndex < 0 ||
        secondSlotTypeSampleIndex < 0 ||
        lastSampleIndex >= static_cast<int>(sampleBuffer.size())) {
        return info;
    }

    float minSync = std::numeric_limits<float>::max();
    float maxSync = std::numeric_limits<float>::lowest();
    for (int i = 0; i < DMR_SYNC_SYMBOLS; ++i) {
        const int sampleIndex =
            syncSampleIndex + i * samplesPerSymbol + samplesPerSymbol / 2;
        if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
            return info;
        }
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        minSync = (std::min)(minSync, sample);
        maxSync = (std::max)(maxSync, sample);
    }
    if (maxSync - minSync < 0.004f) {
        return info;
    }

    const float center = 0.5f * (maxSync + minSync);
    const auto sampleToDibit = [&](float sample, float thresholdRatio) -> int {
        const float upperMid = center + thresholdRatio * (maxSync - center);
        const float lowerMid = center + thresholdRatio * (minSync - center);
        const float corrected = hit.inverted ? -sample : sample;
        const float correctedCenter = hit.inverted ? -center : center;
        const float correctedUpper = hit.inverted ? -lowerMid : upperMid;
        const float correctedLower = hit.inverted ? -upperMid : lowerMid;
        if (corrected > correctedCenter) {
            return corrected > correctedUpper ? 3 : 2;
        }
        return corrected < correctedLower ? 1 : 0;
    };

    int bestData = 0;
    int bestErrors = 21;
    int bestTimingOffset = 0;
    int bestTimingDistance = 99;
    int bestMapPenalty = 99;
    int bestSlicerPenalty = 99;
    float bestSlicerRatio = 0.0f;
    QString bestMapName;
    const std::array<int, 5> timingOffsets = {
        samplesPerSymbol / 2,
        (std::max)(0, samplesPerSymbol / 2 - 1),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 1),
        (std::max)(0, samplesPerSymbol / 2 - 2),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 2)
    };
    const std::array<float, 3> slicerRatios = {0.625f, 0.55f, 0.70f};
    const std::array<bool, 2> mapVariants = {true, false};

    for (int timingOffset : timingOffsets) {
        const int relativeTimingOffset = timingOffset - samplesPerSymbol / 2;
        const int timingDistance = std::abs(relativeTimingOffset);
        for (int slicerIndex = 0; slicerIndex < static_cast<int>(slicerRatios.size()); ++slicerIndex) {
            const float slicerRatio = slicerRatios[static_cast<std::size_t>(slicerIndex)];
            for (bool etsiMap : mapVariants) {
                bool slotBitsRaw[20] = {};
                bool inRange = true;
                for (int i = 0; i < 5; ++i) {
                    const int sampleIndex = firstSlotTypeSampleIndex + i * samplesPerSymbol + timingOffset;
                    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
                        inRange = false;
                        break;
                    }
                    const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
                    const int dibit =
                        (std::clamp)(sampleToDibit(sample, slicerRatio),
                                     0,
                                     3);
                    appendDibitBits(dibit, etsiMap, slotBitsRaw, i * 2);
                }
                if (!inRange) {
                    continue;
                }
                for (int i = 0; i < 5; ++i) {
                    const int sampleIndex = secondSlotTypeSampleIndex + i * samplesPerSymbol + timingOffset;
                    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
                        inRange = false;
                        break;
                    }
                    const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
                    const int dibit =
                        (std::clamp)(sampleToDibit(sample, slicerRatio),
                                     0,
                                     3);
                    appendDibitBits(dibit, etsiMap, slotBitsRaw, 10 + i * 2);
                }
                if (!inRange) {
                    continue;
                }

                std::array<bool, 20> received = {};
                std::copy(std::begin(slotBitsRaw), std::end(slotBitsRaw), received.begin());

                int localBestData = 0;
                int localBestErrors = 21;
                for (int data = 0; data < 256; ++data) {
                    const std::array<bool, 20> candidate = golay208Codeword(data);
                    const int errors = bitDistance(received, candidate);
                    if (errors < localBestErrors) {
                        localBestErrors = errors;
                        localBestData = data;
                    }
                }

                const int mapPenalty = etsiMap ? 0 : 1;
                const int slicerPenalty = slicerIndex == 0 ? 0 : 1;
                const bool prefer =
                    localBestErrors < bestErrors ||
                    (localBestErrors == bestErrors && timingDistance < bestTimingDistance) ||
                    (localBestErrors == bestErrors &&
                     timingDistance == bestTimingDistance &&
                     mapPenalty < bestMapPenalty) ||
                    (localBestErrors == bestErrors &&
                     timingDistance == bestTimingDistance &&
                     mapPenalty == bestMapPenalty &&
                     slicerPenalty < bestSlicerPenalty);
                if (prefer) {
                    bestErrors = localBestErrors;
                    bestData = localBestData;
                    bestTimingOffset = relativeTimingOffset;
                    bestTimingDistance = timingDistance;
                    bestMapPenalty = mapPenalty;
                    bestSlicerPenalty = slicerPenalty;
                    bestSlicerRatio = slicerRatio;
                    bestMapName = etsiMap ? QStringLiteral("ETSI") : QStringLiteral("legacy");
                }
            }
        }
    }

    if (bestErrors > 3) {
        return info;
    }

    info.decoded = true;
    info.colorCode = (bestData >> 4) & 0x0f;
    info.dataType = bestData & 0x0f;
    info.correctedErrors = bestErrors;
    info.dataTypeName = dmrDataTypeName(info.dataType);
    info.timingOffset = bestTimingOffset;
    info.slicerRatio = bestSlicerRatio;
    info.mapName = bestMapName;
    return info;
}

bool DmrDecoder::measureSyncLevels(const SyncHit &hit, float &minLevel, float &maxLevel) const {
    const int syncSampleIndex = hit.phase + hit.symbolIndex * samplesPerSymbol;
    minLevel = std::numeric_limits<float>::max();
    maxLevel = std::numeric_limits<float>::lowest();
    for (int i = 0; i < DMR_SYNC_SYMBOLS; ++i) {
        const int sampleIndex =
            syncSampleIndex + i * samplesPerSymbol + samplesPerSymbol / 2;
        if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
            return false;
        }
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        minLevel = (std::min)(minLevel, sample);
        maxLevel = (std::max)(maxLevel, sample);
    }
    return maxLevel - minLevel >= 0.004f;
}

void DmrDecoder::scheduleVoiceEmbBursts(const SyncHit &hit, const CachInfo &anchorCach) {
    if (!hit.pattern ||
        std::strcmp(hit.pattern->kind, "voice") != 0 ||
        hit.score < DMR_CONFIRMED_SYNC_SCORE) {
        return;
    }
    const bool baseStation = std::strcmp(hit.pattern->source, "base station") == 0;
    if (hit.outerScore < DMR_STRONG_ANCHOR_OUTER_SCORE && !(baseStation && anchorCach.decoded)) {
        return;
    }

    const quint64 minAnchorIntervalSamples =
        static_cast<quint64>(std::lround(0.32 * activeSampleRate));
    if (lastVoiceEmbAnchorSample > 0 &&
        hit.absoluteSample < lastVoiceEmbAnchorSample + minAnchorIntervalSamples) {
        return;
    }

    float minLevel = 0.0f;
    float maxLevel = 0.0f;
    if (!measureSyncLevels(hit, minLevel, maxLevel)) {
        return;
    }

    lastVoiceEmbAnchorSample = hit.absoluteSample;
    ++embAnchorsSinceReport;

    constexpr int voiceEmbBurstsAfterSync = 5;
    constexpr int sameTimeslotStepSymbols = 288;
    for (int i = 1; i <= voiceEmbBurstsAfterSync; ++i) {
        PendingEmb pending;
        pending.anchorSample = hit.absoluteSample;
        pending.absoluteSample =
            hit.absoluteSample +
            static_cast<quint64>(i * sameTimeslotStepSymbols * samplesPerSymbol);
        pending.burstIndex = i;
        pending.inverted = hit.inverted;
        pending.minLevel = minLevel;
        pending.maxLevel = maxLevel;

        bool duplicate = false;
        for (const PendingEmb &existing : pendingVoiceEmb) {
            const quint64 distance =
                existing.absoluteSample > pending.absoluteSample
                    ? existing.absoluteSample - pending.absoluteSample
                    : pending.absoluteSample - existing.absoluteSample;
            if (distance < static_cast<quint64>(samplesPerSymbol * 4)) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            pendingVoiceEmb.push_back(pending);
        }
    }

    while (pendingVoiceEmb.size() > 48) {
        pendingVoiceEmb.pop_front();
    }
}

void DmrDecoder::processPendingVoiceEmb() {
    const quint64 centralFieldSpanSamples =
        static_cast<quint64>(DMR_SYNC_SYMBOLS * samplesPerSymbol);
    const quint64 bufferStartSample = totalSamples > sampleBuffer.size()
                                          ? totalSamples - static_cast<quint64>(sampleBuffer.size())
                                          : 0;

    while (!pendingVoiceEmb.empty()) {
        const PendingEmb pending = pendingVoiceEmb.front();
        if (totalSamples < pending.absoluteSample + centralFieldSpanSamples) {
            break;
        }
        pendingVoiceEmb.pop_front();

        if (pending.absoluteSample < bufferStartSample) {
            ++embFailSinceReport;
            continue;
        }

        const EmbInfo emb = decodeVoiceEmbAt(pending);
        if (!emb.decoded) {
            ++embFailSinceReport;
            continue;
        }
        const VoiceEmbeddedBits fragment =
            decodeVoiceEmbeddedFragmentAt(pending, emb.inverted, emb.timingOffset);
        const bool reliableEmbForLc =
            fragment.decoded &&
            emb.colorCode >= 0 &&
            emb.correctedErrors <= 1;
        if (reliableEmbForLc) {
            recordVoiceEmbeddedFragment(pending, emb, fragment);
        }

        if (emb.correctedErrors <= 1) {
            ++embOkSinceReport;
            if (emb.colorCode >= 0 && emb.colorCode < static_cast<int>(std::size(embColorCodeHistogram))) {
                ++embColorCodeHistogram[emb.colorCode];
            }
            if (emb.lcss >= 0 && emb.lcss < static_cast<int>(std::size(embLcssHistogram))) {
                ++embLcssHistogram[emb.lcss];
            }
            if (emb.privacyIndicator) {
                ++embPrivacyCount;
            }
            embCorrectedErrors += emb.correctedErrors;
            if (emb.variantIndex >= 0 && emb.variantIndex < static_cast<int>(std::size(embVariantHistogram))) {
                ++embVariantHistogram[emb.variantIndex];
            }
            if (emb.correctedErrors >= 0 && emb.correctedErrors < static_cast<int>(std::size(embErrorHistogram))) {
                ++embErrorHistogram[emb.correctedErrors];
            }
            const int timingIndex = emb.timingOffset + 2;
            if (timingIndex >= 0 && timingIndex < static_cast<int>(std::size(embTimingOffsetHistogram))) {
                ++embTimingOffsetHistogram[timingIndex];
            }
        } else {
            ++embWeakSinceReport;
            if (emb.variantIndex >= 0 && emb.variantIndex < static_cast<int>(std::size(embWeakVariantHistogram))) {
                ++embWeakVariantHistogram[emb.variantIndex];
            }
            if (emb.correctedErrors >= 0 && emb.correctedErrors < static_cast<int>(std::size(embWeakErrorHistogram))) {
                ++embWeakErrorHistogram[emb.correctedErrors];
            }
            if (emb.colorCode >= 0 && emb.colorCode < static_cast<int>(std::size(embWeakColorCodeHistogram))) {
                ++embWeakColorCodeHistogram[emb.colorCode];
            }
            const int timingIndex = emb.timingOffset + 2;
            if (timingIndex >= 0 && timingIndex < static_cast<int>(std::size(embWeakTimingOffsetHistogram))) {
                ++embWeakTimingOffsetHistogram[timingIndex];
            }
        }
    }
}

void DmrDecoder::recordVoiceEmbeddedFragment(const PendingEmb &pending,
                                             const EmbInfo &emb,
                                             const VoiceEmbeddedBits &fragment) {
    if (!fragment.decoded || pending.burstIndex < 1 || pending.burstIndex > 5) {
        return;
    }

    const double anchorMs = activeSampleRate > 0
                                ? static_cast<double>(pending.anchorSample) * 1000.0 /
                                      static_cast<double>(activeSampleRate)
                                : 0.0;
    qDebug() << "[DMR] voice emb"
             << "anchorMs" << anchorMs
             << "burst" << pending.burstIndex
             << "cc" << emb.colorCode
             << "lcss" << cachLcssText(emb.lcss)
             << "qrErrors" << emb.correctedErrors
             << "polarity" << (emb.inverted ? "inverted" : "normal")
             << "timing" << emb.timingOffset
             << "variant" << embVariantName(emb.variantIndex)
             << "emb32" << fragment.hex;

    recordVoiceLcSequenceFragment(pending, emb, fragment);

    auto frameIt = std::find_if(voiceEmbeddedFrames.begin(),
                                voiceEmbeddedFrames.end(),
                                [&](const VoiceEmbeddedFrame &frame) {
                                    return frame.anchorSample == pending.anchorSample;
                                });
    if (frameIt == voiceEmbeddedFrames.end()) {
        VoiceEmbeddedFrame frame;
        frame.anchorSample = pending.anchorSample;
        voiceEmbeddedFrames.push_back(frame);
        frameIt = std::prev(voiceEmbeddedFrames.end());
    }

    VoiceEmbeddedFrame &frame = *frameIt;
    const int burst = pending.burstIndex;
    frame.present[static_cast<std::size_t>(burst)] = true;
    frame.colorCode[static_cast<std::size_t>(burst)] = emb.colorCode;
    frame.correctedErrors[static_cast<std::size_t>(burst)] = emb.correctedErrors;
    frame.lcss[static_cast<std::size_t>(burst)] = emb.lcss;
    frame.timing[static_cast<std::size_t>(burst)] = emb.timingOffset;
    frame.emb32[static_cast<std::size_t>(burst)] = fragment.hex;

    const bool hasEmbeddedLc =
        frame.present[1] &&
        frame.present[2] &&
        frame.present[3] &&
        frame.present[4] &&
        frame.lcss[1] == 1 &&
        frame.lcss[2] == 3 &&
        frame.lcss[3] == 3 &&
        frame.lcss[4] == 2;
    const bool embeddedLcColorCodeConsistent =
        frame.colorCode[1] == frame.colorCode[2] &&
        frame.colorCode[1] == frame.colorCode[3] &&
        frame.colorCode[1] == frame.colorCode[4];
    if (hasEmbeddedLc && !frame.reportedLc && embeddedLcColorCodeConsistent) {
        const QString emb128 =
            frame.emb32[1] +
            frame.emb32[2] +
            frame.emb32[3] +
            frame.emb32[4];
        const QString lc72 = embeddedLc72HexFromBptcRaw(emb128);
        qDebug() << "[DMR] voice lc raw"
                 << "anchorMs" << anchorMs
                 << "b1" << frame.emb32[1]
                 << "b2" << frame.emb32[2]
                 << "b3" << frame.emb32[3]
                 << "b4" << frame.emb32[4]
                 << "cc"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.colorCode[1])
                        .arg(frame.colorCode[2])
                        .arg(frame.colorCode[3])
                        .arg(frame.colorCode[4])
                 << "qrErrors"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.correctedErrors[1])
                        .arg(frame.correctedErrors[2])
                        .arg(frame.correctedErrors[3])
                        .arg(frame.correctedErrors[4])
                 << "timing"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.timing[1])
                        .arg(frame.timing[2])
                        .arg(frame.timing[3])
                        .arg(frame.timing[4])
                 << "emb128" << emb128
                 << "lc72" << lc72;
        frame.reportedLc = true;
    } else if (hasEmbeddedLc && !frame.reportedLc) {
        qDebug() << "[DMR] voice lc candidate rejected"
                 << "anchorMs" << anchorMs
                 << "reason" << "mixed color code"
                 << "cc"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.colorCode[1])
                        .arg(frame.colorCode[2])
                        .arg(frame.colorCode[3])
                        .arg(frame.colorCode[4])
                 << "qrErrors"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.correctedErrors[1])
                        .arg(frame.correctedErrors[2])
                        .arg(frame.correctedErrors[3])
                        .arg(frame.correctedErrors[4])
                 << "timing"
                 << QStringLiteral("%1/%2/%3/%4")
                        .arg(frame.timing[1])
                        .arg(frame.timing[2])
                        .arg(frame.timing[3])
                        .arg(frame.timing[4]);
        frame.reportedLc = true;
    }

    while (voiceEmbeddedFrames.size() > 16) {
        voiceEmbeddedFrames.pop_front();
    }
}

void DmrDecoder::recordVoiceLcSequenceFragment(const PendingEmb &pending,
                                               const EmbInfo &emb,
                                               const VoiceEmbeddedBits &fragment) {
    if (!fragment.decoded || emb.colorCode < 0 || emb.lcss < 1 || emb.lcss > 3) {
        return;
    }

    const quint64 maxSequenceSpanSamples =
        static_cast<quint64>(std::lround(0.30 * activeSampleRate));
    const quint64 minFragmentGapSamples =
        static_cast<quint64>(std::lround(0.035 * activeSampleRate));
    const quint64 maxFragmentGapSamples =
        static_cast<quint64>(std::lround(0.095 * activeSampleRate));
    voiceLcSequences.erase(
        std::remove_if(voiceLcSequences.begin(),
                       voiceLcSequences.end(),
                       [&](const VoiceLcSequence &sequence) {
                           return pending.absoluteSample > sequence.startSample + maxSequenceSpanSamples;
                       }),
        voiceLcSequences.end());

    const auto storeStage = [&](VoiceLcSequence &sequence, int stage) {
        sequence.emb32[static_cast<std::size_t>(stage)] = fragment.hex;
        sequence.burstIndex[static_cast<std::size_t>(stage)] = pending.burstIndex;
        sequence.correctedErrors[static_cast<std::size_t>(stage)] = emb.correctedErrors;
        sequence.lcss[static_cast<std::size_t>(stage)] = emb.lcss;
        sequence.timing[static_cast<std::size_t>(stage)] = emb.timingOffset;
        sequence.lastSample = pending.absoluteSample;
    };

    if (emb.lcss == 1) {
        if (pending.burstIndex != 1) {
            return;
        }

        VoiceLcSequence sequence;
        sequence.colorCode = emb.colorCode;
        sequence.nextStage = 1;
        sequence.startSample = pending.absoluteSample;
        sequence.lastSample = pending.absoluteSample;
        storeStage(sequence, 0);
        voiceLcSequences.push_back(sequence);
        while (voiceLcSequences.size() > 32) {
            voiceLcSequences.pop_front();
        }
        return;
    }

    bool completedSequence = false;
    if (emb.lcss == 2 || emb.lcss == 3) {
        for (auto it = voiceLcSequences.begin(); it != voiceLcSequences.end();) {
            VoiceLcSequence &sequence = *it;
            const bool sampleAfterLast = pending.absoluteSample > sequence.lastSample;
            const quint64 fragmentGap =
                sampleAfterLast ? pending.absoluteSample - sequence.lastSample : 0;
            if (sequence.colorCode == emb.colorCode &&
                sequence.nextStage == 3 &&
                pending.burstIndex == 4 &&
                sampleAfterLast &&
                fragmentGap >= minFragmentGapSamples &&
                fragmentGap <= maxFragmentGapSamples &&
                pending.absoluteSample <= sequence.startSample + maxSequenceSpanSamples) {
                storeStage(sequence, 3);
                reportVoiceLcSequence(sequence);
                it = voiceLcSequences.erase(it);
                completedSequence = true;
            } else {
                ++it;
            }
        }
    }
    if (completedSequence) {
        return;
    }

    if (emb.lcss == 3) {
        for (VoiceLcSequence &sequence : voiceLcSequences) {
            const int expectedBurst = sequence.nextStage + 1;
            const bool sampleAfterLast = pending.absoluteSample > sequence.lastSample;
            const quint64 fragmentGap =
                sampleAfterLast ? pending.absoluteSample - sequence.lastSample : 0;
            if (sequence.colorCode != emb.colorCode ||
                sequence.nextStage < 1 ||
                sequence.nextStage > 2 ||
                pending.burstIndex != expectedBurst ||
                !sampleAfterLast ||
                fragmentGap < minFragmentGapSamples ||
                fragmentGap > maxFragmentGapSamples ||
                pending.absoluteSample > sequence.startSample + maxSequenceSpanSamples) {
                continue;
            }
            storeStage(sequence, sequence.nextStage);
            ++sequence.nextStage;
        }
        return;
    }
}

void DmrDecoder::reportVoiceLcSequence(const VoiceLcSequence &sequence) {
    const QString emb128 =
        sequence.emb32[0] +
        sequence.emb32[1] +
        sequence.emb32[2] +
        sequence.emb32[3];
    if (emb128.size() != 32) {
        return;
    }

    const QString lc72 = embeddedLc72HexFromBptcRaw(emb128);
    const double startMs = activeSampleRate > 0
                               ? static_cast<double>(sequence.startSample) * 1000.0 /
                                     static_cast<double>(activeSampleRate)
                               : 0.0;
    const double spanMs = activeSampleRate > 0
                              ? static_cast<double>(sequence.lastSample - sequence.startSample) *
                                    1000.0 / static_cast<double>(activeSampleRate)
                              : 0.0;

    qDebug() << "[DMR] voice lc stream"
             << "startMs" << startMs
             << "spanMs" << spanMs
             << "cc" << sequence.colorCode
             << "bursts"
             << QStringLiteral("%1/%2/%3/%4")
                    .arg(sequence.burstIndex[0])
                    .arg(sequence.burstIndex[1])
                    .arg(sequence.burstIndex[2])
                    .arg(sequence.burstIndex[3])
             << "qrErrors"
             << QStringLiteral("%1/%2/%3/%4")
                    .arg(sequence.correctedErrors[0])
                    .arg(sequence.correctedErrors[1])
                    .arg(sequence.correctedErrors[2])
                    .arg(sequence.correctedErrors[3])
             << "lcss"
             << QStringLiteral("%1/%2/%3/%4")
                    .arg(sequence.lcss[0])
                    .arg(sequence.lcss[1])
                    .arg(sequence.lcss[2])
                    .arg(sequence.lcss[3])
             << "timing"
             << QStringLiteral("%1/%2/%3/%4")
                    .arg(sequence.timing[0])
                    .arg(sequence.timing[1])
                    .arg(sequence.timing[2])
                    .arg(sequence.timing[3])
             << "emb128" << emb128
             << "lc72" << lc72;

    voiceLcRawSinceReport.push_back(emb128);
    if (voiceLcRawSinceReport.size() > 64) {
        voiceLcRawSinceReport.erase(voiceLcRawSinceReport.begin());
    }
}

DmrDecoder::VoicePayloadBits DmrDecoder::decodeVoicePayloadAt(const PendingEmb &pending,
                                                              bool inverted,
                                                              int timingOffset) const {
    VoicePayloadBits payload;
    const quint64 bufferStartSample = totalSamples > sampleBuffer.size()
                                          ? totalSamples - static_cast<quint64>(sampleBuffer.size())
                                          : 0;
    if (pending.absoluteSample < bufferStartSample) {
        return payload;
    }

    const int centralStartIndex =
        static_cast<int>(pending.absoluteSample - bufferStartSample);
    const int sampleOffset = (std::clamp)(samplesPerSymbol / 2 + timingOffset,
                                          0,
                                          (std::max)(0, samplesPerSymbol - 1));
    const int firstPayloadStart = centralStartIndex - DMR_SYMBOLS_BEFORE_SYNC * samplesPerSymbol;
    const int secondPayloadStart = centralStartIndex + DMR_SYNC_SYMBOLS * samplesPerSymbol;
    const int lastPayloadSample =
        secondPayloadStart + (DMR_SYMBOLS_BEFORE_SYNC - 1) * samplesPerSymbol + sampleOffset;
    if (firstPayloadStart + sampleOffset < 0 ||
        lastPayloadSample >= static_cast<int>(sampleBuffer.size())) {
        return payload;
    }

    const float center = 0.5f * (pending.maxLevel + pending.minLevel);
    const float thresholdRatio = 0.625f;
    const float upperMid = center + thresholdRatio * (pending.maxLevel - center);
    const float lowerMid = center + thresholdRatio * (pending.minLevel - center);
    const auto sampleToDibit = [&](float sample) -> int {
        const float corrected = inverted ? -sample : sample;
        const float correctedCenter = inverted ? -center : center;
        const float correctedUpper = inverted ? -lowerMid : upperMid;
        const float correctedLower = inverted ? -upperMid : lowerMid;
        if (corrected > correctedCenter) {
            return corrected > correctedUpper ? 3 : 2;
        }
        return corrected < correctedLower ? 1 : 0;
    };

    bool leftBits[DMR_SYMBOLS_BEFORE_SYNC * 2] = {};
    bool rightBits[DMR_SYMBOLS_BEFORE_SYNC * 2] = {};
    bool combinedBits[DMR_SYMBOLS_BEFORE_SYNC * 4] = {};
    for (int i = 0; i < DMR_SYMBOLS_BEFORE_SYNC; ++i) {
        const int sampleIndex = firstPayloadStart + i * samplesPerSymbol + sampleOffset;
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        const int dibit = (std::clamp)(sampleToDibit(sample), 0, 3);
        appendDibitBits(dibit, true, leftBits, i * 2);
        appendDibitBits(dibit, true, combinedBits, i * 2);
    }
    for (int i = 0; i < DMR_SYMBOLS_BEFORE_SYNC; ++i) {
        const int sampleIndex = secondPayloadStart + i * samplesPerSymbol + sampleOffset;
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        const int dibit = (std::clamp)(sampleToDibit(sample), 0, 3);
        appendDibitBits(dibit, true, rightBits, i * 2);
        appendDibitBits(dibit, true, combinedBits, DMR_SYMBOLS_BEFORE_SYNC * 2 + i * 2);
    }

    payload.decoded = true;
    payload.leftHex = bitsToHex(leftBits, DMR_SYMBOLS_BEFORE_SYNC * 2);
    payload.rightHex = bitsToHex(rightBits, DMR_SYMBOLS_BEFORE_SYNC * 2);
    payload.combinedHex = bitsToHex(combinedBits, DMR_SYMBOLS_BEFORE_SYNC * 4);
    return payload;
}

DmrDecoder::VoiceEmbeddedBits DmrDecoder::decodeVoiceEmbeddedFragmentAt(const PendingEmb &pending,
                                                                        bool inverted,
                                                                        int timingOffset) const {
    VoiceEmbeddedBits fragment;
    const quint64 bufferStartSample = totalSamples > sampleBuffer.size()
                                          ? totalSamples - static_cast<quint64>(sampleBuffer.size())
                                          : 0;
    if (pending.absoluteSample < bufferStartSample) {
        return fragment;
    }

    const int centralStartIndex =
        static_cast<int>(pending.absoluteSample - bufferStartSample);
    const int sampleOffset = (std::clamp)(samplesPerSymbol / 2 + timingOffset,
                                          0,
                                          (std::max)(0, samplesPerSymbol - 1));
    const int firstFragmentSymbol = 4;
    const int fragmentSymbols = 16;
    const int lastFragmentSample =
        centralStartIndex + (firstFragmentSymbol + fragmentSymbols - 1) * samplesPerSymbol +
        sampleOffset;
    if (centralStartIndex + firstFragmentSymbol * samplesPerSymbol + sampleOffset < 0 ||
        lastFragmentSample >= static_cast<int>(sampleBuffer.size())) {
        return fragment;
    }

    const float center = 0.5f * (pending.maxLevel + pending.minLevel);
    const float thresholdRatio = 0.625f;
    const float upperMid = center + thresholdRatio * (pending.maxLevel - center);
    const float lowerMid = center + thresholdRatio * (pending.minLevel - center);
    const auto sampleToDibit = [&](float sample) -> int {
        const float corrected = inverted ? -sample : sample;
        const float correctedCenter = inverted ? -center : center;
        const float correctedUpper = inverted ? -lowerMid : upperMid;
        const float correctedLower = inverted ? -upperMid : lowerMid;
        if (corrected > correctedCenter) {
            return corrected > correctedUpper ? 3 : 2;
        }
        return corrected < correctedLower ? 1 : 0;
    };

    bool bits[fragmentSymbols * 2] = {};
    for (int i = 0; i < fragmentSymbols; ++i) {
        const int symbol = firstFragmentSymbol + i;
        const int sampleIndex = centralStartIndex + symbol * samplesPerSymbol + sampleOffset;
        const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
        const int dibit = (std::clamp)(sampleToDibit(sample), 0, 3);
        appendDibitBits(dibit, true, bits, i * 2);
    }

    fragment.decoded = true;
    fragment.hex = bitsToHex(bits, fragmentSymbols * 2);
    return fragment;
}

DmrDecoder::EmbInfo DmrDecoder::decodeVoiceEmbAt(const PendingEmb &pending) const {
    EmbInfo info;
    const quint64 bufferStartSample = totalSamples > sampleBuffer.size()
                                          ? totalSamples - static_cast<quint64>(sampleBuffer.size())
                                          : 0;
    if (pending.absoluteSample < bufferStartSample) {
        return info;
    }

    const int centralStartIndex =
        static_cast<int>(pending.absoluteSample - bufferStartSample);
    const int lastEmbSampleIndex =
        centralStartIndex + 23 * samplesPerSymbol + samplesPerSymbol / 2;
    if (centralStartIndex < 0 || lastEmbSampleIndex >= static_cast<int>(sampleBuffer.size())) {
        return info;
    }

    const float center = 0.5f * (pending.maxLevel + pending.minLevel);
    const auto sampleToDibit = [&](float sample, float thresholdRatio, bool inverted) -> int {
        const float upperMid = center + thresholdRatio * (pending.maxLevel - center);
        const float lowerMid = center + thresholdRatio * (pending.minLevel - center);
        const float corrected = inverted ? -sample : sample;
        const float correctedCenter = inverted ? -center : center;
        const float correctedUpper = inverted ? -lowerMid : upperMid;
        const float correctedLower = inverted ? -upperMid : lowerMid;
        if (corrected > correctedCenter) {
            return corrected > correctedUpper ? 3 : 2;
        }
        return corrected < correctedLower ? 1 : 0;
    };

    int bestData = 0;
    int bestErrors = 17;
    int bestVariant = -1;
    int bestTimingOffset = 0;
    int bestTimingDistance = 99;
    int bestPolarityPenalty = 99;
    bool bestInverted = pending.inverted;
    int fallbackData = 0;
    int fallbackErrors = 17;
    int fallbackVariant = -1;
    int fallbackTimingOffset = 0;
    int fallbackTimingDistance = 99;
    int fallbackPolarityPenalty = 99;
    int fallbackVariantPenalty = 99;
    bool fallbackInverted = pending.inverted;
    const std::array<int, 5> timingOffsets = {
        samplesPerSymbol / 2,
        (std::max)(0, samplesPerSymbol / 2 - 1),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 1),
        (std::max)(0, samplesPerSymbol / 2 - 2),
        (std::min)(samplesPerSymbol - 1, samplesPerSymbol / 2 + 2)
    };
    const std::array<float, 3> slicerRatios = {0.625f, 0.55f, 0.70f};
    const std::array<bool, 2> polarityVariants = {pending.inverted, !pending.inverted};
    for (bool candidateInverted : polarityVariants) {
        const int polarityPenalty = candidateInverted == pending.inverted ? 0 : 1;
        for (int timingOffset : timingOffsets) {
            const int relativeTimingOffset = timingOffset - samplesPerSymbol / 2;
            const int timingDistance = std::abs(relativeTimingOffset);
            for (float slicerRatio : slicerRatios) {
                bool embRawBits[16] = {};
                bool inRange = true;
                for (int i = 0; i < 4; ++i) {
                    const int sampleIndex = centralStartIndex + i * samplesPerSymbol + timingOffset;
                    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
                        inRange = false;
                        break;
                    }
                    const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
                    const int dibit =
                        (std::clamp)(sampleToDibit(sample, slicerRatio, candidateInverted),
                                     0,
                                     3);
                    appendDibitBits(dibit, true, embRawBits, i * 2);
                }
                if (!inRange) {
                    continue;
                }
                for (int i = 0; i < 4; ++i) {
                    const int sampleIndex = centralStartIndex + (20 + i) * samplesPerSymbol + timingOffset;
                    if (sampleIndex < 0 || sampleIndex >= static_cast<int>(sampleBuffer.size())) {
                        inRange = false;
                        break;
                    }
                    const float sample = averagedSymbolSample(sampleBuffer, sampleIndex, samplesPerSymbol);
                    const int dibit =
                        (std::clamp)(sampleToDibit(sample, slicerRatio, candidateInverted),
                                     0,
                                     3);
                    appendDibitBits(dibit, true, embRawBits, 8 + i * 2);
                }
                if (!inRange) {
                    continue;
                }

                std::array<bool, 16> embBits = {};
                std::copy(std::begin(embRawBits), std::end(embRawBits), embBits.begin());

                for (int variant = 0; variant < DMR_EMB_VARIANT_COUNT; ++variant) {
                    int data = 0;
                    int errors = 0;
                    const std::array<bool, 16> candidateBits = embBitsVariant(embBits, variant);
                    if (!qr1676Decode(candidateBits, data, errors)) {
                        continue;
                    }
                    const int variantPenalty = variant == 0 ? 0 : (variant == 1 ? 1 : 2);
                    const bool preferFallback =
                        errors < fallbackErrors ||
                        (errors == fallbackErrors && timingDistance < fallbackTimingDistance) ||
                        (errors == fallbackErrors &&
                         timingDistance == fallbackTimingDistance &&
                         polarityPenalty < fallbackPolarityPenalty) ||
                        (errors == fallbackErrors &&
                         timingDistance == fallbackTimingDistance &&
                         polarityPenalty == fallbackPolarityPenalty &&
                         variantPenalty < fallbackVariantPenalty);
                    if (preferFallback) {
                        fallbackErrors = errors;
                        fallbackData = data;
                        fallbackVariant = variant;
                        fallbackTimingOffset = relativeTimingOffset;
                        fallbackTimingDistance = timingDistance;
                        fallbackPolarityPenalty = polarityPenalty;
                        fallbackVariantPenalty = variantPenalty;
                        fallbackInverted = candidateInverted;
                    }
                    if (variant == 0) {
                        const bool preferDirect =
                        errors < bestErrors ||
                            (errors == bestErrors && timingDistance < bestTimingDistance) ||
                            (errors == bestErrors &&
                             timingDistance == bestTimingDistance &&
                             polarityPenalty < bestPolarityPenalty);
                        if (preferDirect) {
                            bestErrors = errors;
                            bestData = data;
                            bestVariant = variant;
                            bestTimingOffset = relativeTimingOffset;
                            bestTimingDistance = timingDistance;
                            bestPolarityPenalty = polarityPenalty;
                            bestInverted = candidateInverted;
                        }
                    }
                }
            }
            if (bestErrors == 0 && bestTimingDistance == 0 && bestPolarityPenalty == 0) {
                break;
            }
        }
        if (bestErrors == 0 && bestTimingDistance == 0 && bestPolarityPenalty == 0) {
            break;
        }
    }

    if (bestVariant < 0) {
        if (fallbackVariant < 0) {
            return info;
        }
        bestData = fallbackData;
        bestErrors = fallbackErrors;
        bestVariant = fallbackVariant;
        bestTimingOffset = fallbackTimingOffset;
        bestTimingDistance = fallbackTimingDistance;
        bestPolarityPenalty = fallbackPolarityPenalty;
        bestInverted = fallbackInverted;
    } else if (bestErrors > 1 && fallbackVariant >= 0 && fallbackErrors < bestErrors) {
        bestData = fallbackData;
        bestErrors = fallbackErrors;
        bestVariant = fallbackVariant;
        bestTimingOffset = fallbackTimingOffset;
        bestTimingDistance = fallbackTimingDistance;
        bestPolarityPenalty = fallbackPolarityPenalty;
        bestInverted = fallbackInverted;
    }

    if (bestVariant != 0 && bestErrors < 2) {
        bestErrors = 2;
    }

    info.decoded = true;
    info.colorCode = (bestData >> 3) & 0x0f;
    info.privacyIndicator = ((bestData >> 2) & 0x01) != 0;
    info.lcss = bestData & 0x03;
    info.correctedErrors = bestErrors;
    info.variantIndex = bestVariant;
    info.timingOffset = bestTimingOffset;
    info.inverted = bestInverted;
    return info;
}

QString DmrDecoder::formatHit(const SyncHit &hit) const {
    if (!hit.pattern) {
        return idleStatus();
    }

    const bool confirmed =
        hit.score >= DMR_CONFIRMED_SYNC_SCORE &&
        hit.outerScore >= DMR_CONFIRMED_OUTER_SCORE;
    return QStringLiteral("DMR %1: %2, %3, sync %4/24, outer %5/24, phase %6, %7, CC/slot pending")
        .arg(confirmed ? QStringLiteral("lock") : QStringLiteral("candidate"))
        .arg(QString::fromLatin1(hit.pattern->source))
        .arg(QString::fromLatin1(hit.pattern->kind))
        .arg(hit.score)
        .arg(hit.outerScore)
        .arg(hit.phase)
        .arg(hit.inverted ? QStringLiteral("inverted") : QStringLiteral("normal"));
}

QString DmrDecoder::idleStatus() const {
    return averageMagnitude < 0.002
               ? QStringLiteral("DMR monitor: waiting for discriminator signal")
               : QStringLiteral("DMR monitor: searching sync");
}
