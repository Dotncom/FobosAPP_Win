#ifndef IQBUFFER_H
#define IQBUFFER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace IqBuffer {

struct Stats {
    std::uint64_t epoch = 0;
    std::uint64_t sequence = 0;
    std::size_t snapshotStart = 0;
    std::size_t snapshotSize = 0;
    std::size_t queuedBlocks = 0;
    std::size_t queuedFloatCount = 0;
    double sampleRateEstimate = 0.0;
};

bool publish(const float *samples,
             std::size_t floatCount,
             bool queueBlock = true,
             bool updateSnapshot = true,
             std::uint64_t expectedEpoch = 0);
bool snapshot(std::vector<float> &out, std::uint64_t *sequence = nullptr);
bool snapshotRecent(std::vector<float> &out,
                    std::size_t maxFloatCount,
                    std::uint64_t *sequence = nullptr);
bool popBlock(std::vector<float> &out, std::uint64_t *sequence = nullptr);
void clear(std::uint64_t epoch = 0);
std::size_t size();
std::size_t queuedBlocks();
std::size_t queuedFloatCount();
Stats stats();
void armRetuneTrace(std::uint64_t epoch,
                    int publishLogs = 12,
                    int snapshotLogs = 8,
                    int popLogs = 12,
                    int rejectLogs = 8);
void setSampleRateEstimate(double sampleRate);
double sampleRateEstimate();

} // namespace IqBuffer

#endif // IQBUFFER_H
