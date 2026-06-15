#include "iqbuffer.h"
#include "diagnosticlogging.h"

#include <deque>
#include <algorithm>
#include <cmath>
#include <mutex>

#include <QDebug>

namespace {

constexpr std::size_t MAX_QUEUED_BLOCKS = 256;
constexpr std::size_t MAX_QUEUED_FLOATS = 16 * 1024 * 1024;
constexpr std::size_t MIN_LIVE_QUEUED_FLOATS = 128 * 1024;
constexpr double MAX_LIVE_QUEUED_SECONDS = 1.25;
constexpr std::size_t MAX_SNAPSHOT_FLOATS = 16 * 1024 * 1024;

std::mutex g_iqMutex;
std::vector<float> g_iqSnapshot(MAX_SNAPSHOT_FLOATS);
std::size_t g_iqSnapshotStart = 0;
std::size_t g_iqSnapshotSize = 0;
std::deque<std::vector<float>> g_iqBlocks;
std::deque<std::uint64_t> g_iqBlockSequences;
std::size_t g_iqQueuedFloatCount = 0;
std::uint64_t g_iqSequence = 0;
std::uint64_t g_iqEpoch = 1;
double g_sampleRateEstimate = 0.0;
IqBuffer::BlockMetadata g_latestMetadata;
std::uint64_t g_traceEpoch = 0;
int g_tracePublishRemaining = 0;
int g_traceRejectRemaining = 0;
int g_traceSnapshotRemaining = 0;
int g_tracePopRemaining = 0;

bool traceMatchesCurrentEpoch() {
    return fobosVerboseLoggingEnabled() &&
           g_traceEpoch != 0 &&
           g_iqEpoch == g_traceEpoch;
}

bool traceMatchesPublishEpoch(std::uint64_t expectedEpoch) {
    return fobosVerboseLoggingEnabled() &&
           g_traceEpoch != 0 &&
           (g_iqEpoch == g_traceEpoch || expectedEpoch == g_traceEpoch);
}

void logTraceState(const char *event) {
    if (!fobosVerboseLoggingEnabled()) {
        return;
    }
    qDebug() << "[IqBufferTrace]" << event
             << "epoch" << static_cast<qulonglong>(g_iqEpoch)
             << "traceEpoch" << static_cast<qulonglong>(g_traceEpoch)
             << "sequence" << static_cast<qulonglong>(g_iqSequence)
             << "snapshotStart" << g_iqSnapshotStart
             << "snapshotSize" << g_iqSnapshotSize
             << "queuedBlocks" << g_iqBlocks.size()
             << "queuedFloats" << g_iqQueuedFloatCount
             << "sampleRateEstimate" << g_sampleRateEstimate;
}

void appendToSnapshot(const float *samples, std::size_t floatCount) {
    if (floatCount >= MAX_SNAPSHOT_FLOATS) {
        std::copy(samples + (floatCount - MAX_SNAPSHOT_FLOATS), samples + floatCount, g_iqSnapshot.begin());
        g_iqSnapshotStart = 0;
        g_iqSnapshotSize = MAX_SNAPSHOT_FLOATS;
        return;
    }

    if (g_iqSnapshotSize + floatCount > MAX_SNAPSHOT_FLOATS) {
        const std::size_t dropCount = g_iqSnapshotSize + floatCount - MAX_SNAPSHOT_FLOATS;
        g_iqSnapshotStart = (g_iqSnapshotStart + dropCount) % MAX_SNAPSHOT_FLOATS;
        g_iqSnapshotSize -= dropCount;
    }

    std::size_t writePos = (g_iqSnapshotStart + g_iqSnapshotSize) % MAX_SNAPSHOT_FLOATS;
    const std::size_t firstCopy = (std::min)(floatCount, MAX_SNAPSHOT_FLOATS - writePos);
    std::copy(samples, samples + firstCopy, g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(writePos));
    if (firstCopy < floatCount) {
        std::copy(samples + firstCopy, samples + floatCount, g_iqSnapshot.begin());
    }
    g_iqSnapshotSize += floatCount;
}

bool copySnapshotTail(std::vector<float> &out,
                      std::size_t maxFloatCount,
                      std::uint64_t *sequence,
                      IqBuffer::BlockMetadata *metadata,
                      const char *source) {
    const bool shouldTrace = traceMatchesCurrentEpoch() &&
                             g_traceSnapshotRemaining > 0;
    if (g_iqSnapshotSize == 0 || maxFloatCount == 0) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
        }
        if (metadata) {
            *metadata = g_latestMetadata;
        }
        if (shouldTrace) {
            --g_traceSnapshotRemaining;
            qDebug() << "[IqBufferTrace]" << source
                     << "empty"
                     << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                     << "sequence" << static_cast<qulonglong>(g_iqSequence)
                     << "maxFloatCount" << maxFloatCount
                     << "snapshotStart" << g_iqSnapshotStart
                     << "snapshotSize" << g_iqSnapshotSize
                     << "queuedBlocks" << g_iqBlocks.size()
                     << "queuedFloats" << g_iqQueuedFloatCount;
        }
        return false;
    }

    std::size_t copyCount = (std::min)(g_iqSnapshotSize, maxFloatCount);
    copyCount -= copyCount % 2U;
    if (copyCount == 0) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
        }
        if (metadata) {
            *metadata = g_latestMetadata;
        }
        if (shouldTrace) {
            --g_traceSnapshotRemaining;
            qDebug() << "[IqBufferTrace]" << source
                     << "zero-copy"
                     << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                     << "sequence" << static_cast<qulonglong>(g_iqSequence)
                     << "maxFloatCount" << maxFloatCount
                     << "snapshotStart" << g_iqSnapshotStart
                     << "snapshotSize" << g_iqSnapshotSize;
        }
        return false;
    }

    const std::size_t tailOffset = g_iqSnapshotSize - copyCount;
    const std::size_t readStart = (g_iqSnapshotStart + tailOffset) % MAX_SNAPSHOT_FLOATS;
    out.resize(copyCount);
    const std::size_t firstCopy = (std::min)(copyCount, MAX_SNAPSHOT_FLOATS - readStart);
    if (shouldTrace) {
        --g_traceSnapshotRemaining;
        qDebug() << "[IqBufferTrace]" << source
                 << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                 << "sequence" << static_cast<qulonglong>(g_iqSequence)
                 << "maxFloatCount" << maxFloatCount
                 << "snapshotStart" << g_iqSnapshotStart
                 << "snapshotSize" << g_iqSnapshotSize
                 << "tailOffset" << tailOffset
                 << "readStart" << readStart
                 << "copyCount" << copyCount
                 << "firstCopy" << firstCopy
                 << "queuedBlocks" << g_iqBlocks.size()
                 << "queuedFloats" << g_iqQueuedFloatCount;
    }
    std::copy(g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(readStart),
              g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(readStart + firstCopy),
              out.begin());
    if (firstCopy < copyCount) {
        std::copy(g_iqSnapshot.begin(),
                  g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(copyCount - firstCopy),
                  out.begin() + static_cast<std::ptrdiff_t>(firstCopy));
    }
    if (sequence) {
        *sequence = g_iqSequence;
    }
    if (metadata) {
        *metadata = g_latestMetadata;
    }
    return true;
}

std::size_t maxQueuedFloatsForLiveAudio() {
    if (g_sampleRateEstimate <= 0.0 || !std::isfinite(g_sampleRateEstimate)) {
        return MAX_QUEUED_FLOATS;
    }

    const double targetFloats = g_sampleRateEstimate * 2.0 * MAX_LIVE_QUEUED_SECONDS;
    if (targetFloats <= 0.0) {
        return MAX_QUEUED_FLOATS;
    }

    return (std::clamp)(static_cast<std::size_t>(targetFloats),
                        MIN_LIVE_QUEUED_FLOATS,
                        MAX_QUEUED_FLOATS);
}

} // namespace

namespace IqBuffer {

bool publish(const float *samples,
             std::size_t floatCount,
             bool queueBlock,
             bool updateSnapshot,
             std::uint64_t expectedEpoch,
             const BlockMetadata *metadata) {
    if (!samples || floatCount == 0) {
        return false;
    }

    std::vector<float> block;
    if (queueBlock) {
        block.assign(samples, samples + floatCount);
    }

    std::lock_guard<std::mutex> lock(g_iqMutex);
    if (expectedEpoch != 0 && expectedEpoch != g_iqEpoch) {
        if (traceMatchesPublishEpoch(expectedEpoch) && g_traceRejectRemaining > 0) {
            --g_traceRejectRemaining;
            qDebug() << "[IqBufferTrace] publish rejected"
                     << "expectedEpoch" << static_cast<qulonglong>(expectedEpoch)
                     << "currentEpoch" << static_cast<qulonglong>(g_iqEpoch)
                     << "floatCount" << floatCount
                     << "queueBlock" << queueBlock
                     << "updateSnapshot" << updateSnapshot
                     << "sequence" << static_cast<qulonglong>(g_iqSequence)
                     << "snapshotStart" << g_iqSnapshotStart
                     << "snapshotSize" << g_iqSnapshotSize
                     << "queuedBlocks" << g_iqBlocks.size()
                     << "queuedFloats" << g_iqQueuedFloatCount;
        }
        return false;
    }

    if (updateSnapshot) {
        appendToSnapshot(samples, floatCount);
        ++g_iqSequence;
        g_latestMetadata = metadata ? *metadata : BlockMetadata();
        g_latestMetadata.sequence = g_iqSequence;
        g_latestMetadata.floatCount = floatCount;
    }

    if (!queueBlock) {
        g_iqBlocks.clear();
        g_iqBlockSequences.clear();
        g_iqQueuedFloatCount = 0;
        if (traceMatchesPublishEpoch(expectedEpoch) && g_tracePublishRemaining > 0) {
            --g_tracePublishRemaining;
            qDebug() << "[IqBufferTrace] publish accepted"
                     << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                     << "expectedEpoch" << static_cast<qulonglong>(expectedEpoch)
                     << "sequence" << static_cast<qulonglong>(g_iqSequence)
                     << "floatCount" << floatCount
                     << "queueBlock" << queueBlock
                     << "updateSnapshot" << updateSnapshot
                     << "snapshotStart" << g_iqSnapshotStart
                     << "snapshotSize" << g_iqSnapshotSize
                     << "queuedBlocks" << g_iqBlocks.size()
                     << "queuedFloats" << g_iqQueuedFloatCount;
        }
        return true;
    }

    g_iqQueuedFloatCount += block.size();
    g_iqBlocks.push_back(std::move(block));
    g_iqBlockSequences.push_back(g_iqSequence);
    const std::size_t maxQueuedFloats = maxQueuedFloatsForLiveAudio();
    while (g_iqBlocks.size() > MAX_QUEUED_BLOCKS || g_iqQueuedFloatCount > maxQueuedFloats) {
        g_iqQueuedFloatCount -= g_iqBlocks.front().size();
        g_iqBlocks.pop_front();
        g_iqBlockSequences.pop_front();
    }
    if (traceMatchesPublishEpoch(expectedEpoch) && g_tracePublishRemaining > 0) {
        --g_tracePublishRemaining;
        qDebug() << "[IqBufferTrace] publish accepted"
                 << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                 << "expectedEpoch" << static_cast<qulonglong>(expectedEpoch)
                 << "sequence" << static_cast<qulonglong>(g_iqSequence)
                 << "floatCount" << floatCount
                 << "queueBlock" << queueBlock
                 << "updateSnapshot" << updateSnapshot
                 << "snapshotStart" << g_iqSnapshotStart
                 << "snapshotSize" << g_iqSnapshotSize
                 << "queuedBlocks" << g_iqBlocks.size()
                 << "queuedFloats" << g_iqQueuedFloatCount
                 << "maxQueuedFloats" << maxQueuedFloats;
    }
    return true;
}

bool snapshot(std::vector<float> &out, std::uint64_t *sequence, BlockMetadata *metadata) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return copySnapshotTail(out, g_iqSnapshotSize, sequence, metadata, "snapshot");
}

bool snapshotRecent(std::vector<float> &out,
                    std::size_t maxFloatCount,
                    std::uint64_t *sequence,
                    BlockMetadata *metadata) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return copySnapshotTail(out, maxFloatCount, sequence, metadata, "snapshotRecent");
}

bool popBlock(std::vector<float> &out, std::uint64_t *sequence) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    const bool shouldTrace = traceMatchesCurrentEpoch() && g_tracePopRemaining > 0;
    if (g_iqBlocks.empty()) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
        }
        if (shouldTrace) {
            --g_tracePopRemaining;
            qDebug() << "[IqBufferTrace] popBlock empty"
                     << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                     << "sequence" << static_cast<qulonglong>(g_iqSequence)
                     << "snapshotStart" << g_iqSnapshotStart
                     << "snapshotSize" << g_iqSnapshotSize
                     << "queuedBlocks" << g_iqBlocks.size()
                     << "queuedFloats" << g_iqQueuedFloatCount;
        }
        return false;
    }

    const std::uint64_t blockSequence =
        g_iqBlockSequences.empty() ? g_iqSequence : g_iqBlockSequences.front();
    out = std::move(g_iqBlocks.front());
    g_iqQueuedFloatCount -= out.size();
    g_iqBlocks.pop_front();

    if (sequence) {
        *sequence = blockSequence;
    }
    if (!g_iqBlockSequences.empty()) {
        g_iqBlockSequences.pop_front();
    }
    if (shouldTrace) {
        --g_tracePopRemaining;
        qDebug() << "[IqBufferTrace] popBlock"
                 << "epoch" << static_cast<qulonglong>(g_iqEpoch)
                 << "blockSequence" << static_cast<qulonglong>(blockSequence)
                 << "blockFloats" << out.size()
                 << "snapshotStart" << g_iqSnapshotStart
                 << "snapshotSize" << g_iqSnapshotSize
                 << "remainingBlocks" << g_iqBlocks.size()
                 << "remainingFloats" << g_iqQueuedFloatCount;
    }
    return true;
}

void clear(std::uint64_t epoch) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    const std::uint64_t previousEpoch = g_iqEpoch;
    const std::uint64_t previousSequence = g_iqSequence;
    const std::size_t previousSnapshotStart = g_iqSnapshotStart;
    const std::size_t previousSnapshotSize = g_iqSnapshotSize;
    const std::size_t previousQueuedBlocks = g_iqBlocks.size();
    const std::size_t previousQueuedFloats = g_iqQueuedFloatCount;
    if (epoch != 0) {
        g_iqEpoch = epoch;
    }
    g_iqSnapshotStart = 0;
    g_iqSnapshotSize = 0;
    g_iqBlocks.clear();
    g_iqBlockSequences.clear();
    g_iqQueuedFloatCount = 0;
    g_latestMetadata = BlockMetadata();
    ++g_iqSequence;
    if (epoch != 0 && traceMatchesCurrentEpoch()) {
        qDebug() << "[IqBufferTrace] clear"
                 << "requestedEpoch" << static_cast<qulonglong>(epoch)
                 << "previousEpoch" << static_cast<qulonglong>(previousEpoch)
                 << "currentEpoch" << static_cast<qulonglong>(g_iqEpoch)
                 << "sequenceBefore" << static_cast<qulonglong>(previousSequence)
                 << "sequenceAfter" << static_cast<qulonglong>(g_iqSequence)
                 << "previousSnapshotStart" << previousSnapshotStart
                 << "previousSnapshotSize" << previousSnapshotSize
                 << "previousQueuedBlocks" << previousQueuedBlocks
                 << "previousQueuedFloats" << previousQueuedFloats;
    }
}

std::size_t size() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return g_iqSnapshotSize;
}

std::size_t queuedBlocks() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return g_iqBlocks.size();
}

std::size_t queuedFloatCount() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return g_iqQueuedFloatCount;
}

Stats stats() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    Stats result;
    result.epoch = g_iqEpoch;
    result.sequence = g_iqSequence;
    result.snapshotStart = g_iqSnapshotStart;
    result.snapshotSize = g_iqSnapshotSize;
    result.queuedBlocks = g_iqBlocks.size();
    result.queuedFloatCount = g_iqQueuedFloatCount;
    result.sampleRateEstimate = g_sampleRateEstimate;
    return result;
}

void armRetuneTrace(std::uint64_t epoch,
                    int publishLogs,
                    int snapshotLogs,
                    int popLogs,
                    int rejectLogs) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    if (epoch == 0 || !fobosVerboseLoggingEnabled()) {
        g_traceEpoch = 0;
        g_tracePublishRemaining = 0;
        g_traceSnapshotRemaining = 0;
        g_tracePopRemaining = 0;
        g_traceRejectRemaining = 0;
        return;
    }

    g_traceEpoch = epoch;
    g_tracePublishRemaining = (std::max)(0, publishLogs);
    g_traceSnapshotRemaining = (std::max)(0, snapshotLogs);
    g_tracePopRemaining = (std::max)(0, popLogs);
    g_traceRejectRemaining = (std::max)(0, rejectLogs);
    logTraceState("armed");
}

void setSampleRateEstimate(double sampleRate) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    g_sampleRateEstimate = sampleRate;
}

double sampleRateEstimate() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return g_sampleRateEstimate;
}

} // namespace IqBuffer
