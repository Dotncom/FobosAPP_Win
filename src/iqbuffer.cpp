#include "iqbuffer.h"

#include <deque>
#include <algorithm>
#include <cmath>
#include <mutex>

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
double g_sampleRateEstimate = 0.0;

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
                      std::uint64_t *sequence) {
    if (g_iqSnapshotSize == 0 || maxFloatCount == 0) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
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
        return false;
    }

    const std::size_t tailOffset = g_iqSnapshotSize - copyCount;
    const std::size_t readStart = (g_iqSnapshotStart + tailOffset) % MAX_SNAPSHOT_FLOATS;
    out.resize(copyCount);
    const std::size_t firstCopy = (std::min)(copyCount, MAX_SNAPSHOT_FLOATS - readStart);
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

void publish(const float *samples, std::size_t floatCount, bool queueBlock, bool updateSnapshot) {
    if (!samples || floatCount == 0) {
        return;
    }

    std::vector<float> block;
    if (queueBlock) {
        block.assign(samples, samples + floatCount);
    }

    std::lock_guard<std::mutex> lock(g_iqMutex);
    if (updateSnapshot) {
        appendToSnapshot(samples, floatCount);
        ++g_iqSequence;
    }

    if (!queueBlock) {
        g_iqBlocks.clear();
        g_iqBlockSequences.clear();
        g_iqQueuedFloatCount = 0;
        return;
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
}

bool snapshot(std::vector<float> &out, std::uint64_t *sequence) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return copySnapshotTail(out, g_iqSnapshotSize, sequence);
}

bool snapshotRecent(std::vector<float> &out,
                    std::size_t maxFloatCount,
                    std::uint64_t *sequence) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return copySnapshotTail(out, maxFloatCount, sequence);
}

bool popBlock(std::vector<float> &out, std::uint64_t *sequence) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    if (g_iqBlocks.empty()) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
        }
        return false;
    }

    out = std::move(g_iqBlocks.front());
    g_iqQueuedFloatCount -= out.size();
    g_iqBlocks.pop_front();

    if (sequence) {
        *sequence = g_iqBlockSequences.front();
    }
    g_iqBlockSequences.pop_front();
    return true;
}

void clear() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    g_iqSnapshotStart = 0;
    g_iqSnapshotSize = 0;
    g_iqBlocks.clear();
    g_iqBlockSequences.clear();
    g_iqQueuedFloatCount = 0;
    ++g_iqSequence;
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

void setSampleRateEstimate(double sampleRate) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    g_sampleRateEstimate = sampleRate;
}

double sampleRateEstimate() {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    return g_sampleRateEstimate;
}

} // namespace IqBuffer
