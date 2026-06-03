#ifndef DMRDECODER_H
#define DMRDECODER_H

#include <QByteArray>
#include <QString>

#include <array>
#include <deque>
#include <vector>

class DmrDecoder {
public:
    struct Result {
        QString decodedText;
        QString statusText;
        bool statusChanged = false;
    };

    void reset();
    void configure(int sampleRate);
    Result processPcmFrame(const QByteArray &pcmData, int sampleRate, double rfFrequencyHz = 0.0);

    struct SyncPattern {
        const char *name = "";
        const char *kind = "";
        const char *source = "";
        const char *symbols = "";
    };

private:
    struct SyncHit {
        const SyncPattern *pattern = nullptr;
        int score = 0;
        int errors = 24;
        int outerScore = 0;
        int phase = 0;
        int symbolIndex = 0;
        bool inverted = false;
        double qualityDb = 0.0;
        quint64 absoluteSample = 0;
    };

    struct CachInfo {
        bool decoded = false;
        bool accessType = false;
        int channel = 0;
        int lcss = 0;
        int timingOffset = 0;
        double slicerRatio = 0.0;
        QString mapName;
        QString payloadBits;
    };

    struct SlotTypeInfo {
        bool decoded = false;
        int colorCode = -1;
        int dataType = -1;
        int correctedErrors = 0;
        QString dataTypeName;
        int timingOffset = 0;
        double slicerRatio = 0.0;
        QString mapName;
    };

    struct EmbInfo {
        bool decoded = false;
        int colorCode = -1;
        bool privacyIndicator = false;
        int lcss = 0;
        int correctedErrors = 0;
        int variantIndex = -1;
        int timingOffset = 0;
        bool inverted = false;
    };

    struct PendingEmb {
        quint64 anchorSample = 0;
        quint64 absoluteSample = 0;
        int burstIndex = 0;
        bool inverted = false;
        float minLevel = 0.0f;
        float maxLevel = 0.0f;
    };

    struct VoiceEmbeddedBits {
        bool decoded = false;
        QString hex;
    };

    struct VoicePayloadBits {
        bool decoded = false;
        QString leftHex;
        QString rightHex;
        QString combinedHex;
    };

    struct VoiceEmbeddedFrame {
        quint64 anchorSample = 0;
        bool reportedLc = false;
        std::array<bool, 6> present = {};
        std::array<int, 6> colorCode = {};
        std::array<int, 6> correctedErrors = {};
        std::array<int, 6> lcss = {};
        std::array<int, 6> timing = {};
        std::array<QString, 6> emb32 = {};
    };

    struct VoiceLcSequence {
        int colorCode = -1;
        int nextStage = 0;
        quint64 startSample = 0;
        quint64 lastSample = 0;
        std::array<QString, 4> emb32 = {};
        std::array<int, 4> burstIndex = {};
        std::array<int, 4> correctedErrors = {};
        std::array<int, 4> lcss = {};
        std::array<int, 4> timing = {};
    };

    void appendSamples(const QByteArray &pcmData);
    bool findBestSync(SyncHit &hit, quint64 minimumAbsoluteSample) const;
    CachInfo decodeCachBeforeSync(const SyncHit &hit, bool allowTimingSearch = false) const;
    SlotTypeInfo decodeSlotTypeAroundSync(const SyncHit &hit) const;
    void scheduleVoiceEmbBursts(const SyncHit &hit, const CachInfo &anchorCach);
    void processPendingVoiceEmb();
    bool measureSyncLevels(const SyncHit &hit, float &minLevel, float &maxLevel) const;
    EmbInfo decodeVoiceEmbAt(const PendingEmb &pending) const;
    VoiceEmbeddedBits decodeVoiceEmbeddedFragmentAt(const PendingEmb &pending, bool inverted, int timingOffset) const;
    VoicePayloadBits decodeVoicePayloadAt(const PendingEmb &pending, bool inverted, int timingOffset) const;
    void recordVoiceEmbeddedFragment(const PendingEmb &pending,
                                     const EmbInfo &emb,
                                     const VoiceEmbeddedBits &fragment);
    void recordVoiceLcSequenceFragment(const PendingEmb &pending,
                                       const EmbInfo &emb,
                                       const VoiceEmbeddedBits &fragment);
    void reportVoiceLcSequence(const VoiceLcSequence &sequence);
    QString voiceLcRawSummaryText() const;
    QString formatHit(const SyncHit &hit) const;
    QString idleStatus() const;

    int activeSampleRate = 48000;
    int samplesPerSymbol = 10;
    double dcEstimate = 0.0;
    double averageMagnitude = 0.0;
    quint64 totalSamples = 0;
    quint64 lastReportSample = 0;
    quint64 lastAcceptedSyncSample = 0;
    bool haveAcceptedSync = false;
    bool haveLock = false;
    bool lockedPolarityInverted = false;
    int lockedPhase = 0;
    double lockedRfFrequencyHz = 0.0;
    int confirmedSyncsSinceReport = 0;
    int candidateSyncsSinceReport = 0;
    int polarityFlipsSinceReport = 0;
    int bestScoreSinceReport = 0;
    int bestPhaseSinceReport = 0;
    double bestQualitySinceReport = 0.0;
    bool bestPolarityInvertedSinceReport = false;
    quint64 lastConfirmedSyncSample = 0;
    int spacingSamplesCount = 0;
    double spacingMsSum = 0.0;
    double spacingMsMin = 0.0;
    double spacingMsMax = 0.0;
    int phaseHistogram[16] = {};
    int intervalHistogram[12] = {};
    int cachOkSinceReport = 0;
    int cachStrictOkSinceReport = 0;
    int cachFailSinceReport = 0;
    int cachChannelHistogram[2] = {};
    int cachLcssHistogram[4] = {};
    int cachTimingOffsetHistogram[5] = {};
    int cachMapEtsiCount = 0;
    int cachMapLegacyCount = 0;
    int slotTypeOkSinceReport = 0;
    int slotTypeFailSinceReport = 0;
    int colorCodeHistogram[16] = {};
    int dataTypeHistogram[16] = {};
    int slotTypeCorrectedErrors = 0;
    int embOkSinceReport = 0;
    int embAnchorsSinceReport = 0;
    int embWeakSinceReport = 0;
    int embFailSinceReport = 0;
    int embColorCodeHistogram[16] = {};
    int embLcssHistogram[4] = {};
    int embPrivacyCount = 0;
    int embCorrectedErrors = 0;
    int embVariantHistogram[8] = {};
    int embWeakVariantHistogram[8] = {};
    int embErrorHistogram[3] = {};
    int embWeakErrorHistogram[3] = {};
    int embWeakColorCodeHistogram[16] = {};
    int embTimingOffsetHistogram[5] = {};
    int embWeakTimingOffsetHistogram[5] = {};
    QString lastCachText;
    QString lastStatus;
    QString lastPatternName;
    QString lockedPatternName;
    quint64 lastVoiceEmbAnchorSample = 0;
    std::deque<float> sampleBuffer;
    std::deque<PendingEmb> pendingVoiceEmb;
    std::deque<VoiceEmbeddedFrame> voiceEmbeddedFrames;
    std::deque<VoiceLcSequence> voiceLcSequences;
    std::vector<QString> voiceLcRawSinceReport;
};

#endif // DMRDECODER_H
