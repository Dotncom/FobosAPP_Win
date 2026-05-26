#include "iqbuffer.h"

#include <deque>
#include <algorithm>
#include <mutex>

namespace {

constexpr std::size_t MAX_QUEUED_BLOCKS = 96;
constexpr std::size_t MAX_QUEUED_FLOATS = 4 * 1024 * 1024;
constexpr std::size_t MAX_SNAPSHOT_FLOATS = 524288 * 2;

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

} // namespace

namespace IqBuffer {

void publish(const float *samples, std::size_t floatCount, bool queueBlock) {
    if (!samples || floatCount == 0) {
        return;
    }

    std::vector<float> block;
    if (queueBlock) {
        block.assign(samples, samples + floatCount);
    }

    std::lock_guard<std::mutex> lock(g_iqMutex);
    appendToSnapshot(samples, floatCount);
    ++g_iqSequence;

    if (!queueBlock) {
        g_iqBlocks.clear();
        g_iqBlockSequences.clear();
        g_iqQueuedFloatCount = 0;
        return;
    }

    g_iqQueuedFloatCount += block.size();
    g_iqBlocks.push_back(std::move(block));
    g_iqBlockSequences.push_back(g_iqSequence);
    while (g_iqBlocks.size() > MAX_QUEUED_BLOCKS || g_iqQueuedFloatCount > MAX_QUEUED_FLOATS) {
        g_iqQueuedFloatCount -= g_iqBlocks.front().size();
        g_iqBlocks.pop_front();
        g_iqBlockSequences.pop_front();
    }
}

bool snapshot(std::vector<float> &out, std::uint64_t *sequence) {
    std::lock_guard<std::mutex> lock(g_iqMutex);
    if (g_iqSnapshotSize == 0) {
        out.clear();
        if (sequence) {
            *sequence = g_iqSequence;
        }
        return false;
    }

    out.resize(g_iqSnapshotSize);
    const std::size_t firstCopy = (std::min)(g_iqSnapshotSize, MAX_SNAPSHOT_FLOATS - g_iqSnapshotStart);
    std::copy(g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(g_iqSnapshotStart),
              g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(g_iqSnapshotStart + firstCopy),
              out.begin());
    if (firstCopy < g_iqSnapshotSize) {
        std::copy(g_iqSnapshot.begin(),
                  g_iqSnapshot.begin() + static_cast<std::ptrdiff_t>(g_iqSnapshotSize - firstCopy),
                  out.begin() + static_cast<std::ptrdiff_t>(firstCopy));
    }
    if (sequence) {
        *sequence = g_iqSequence;
    }
    return true;
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
