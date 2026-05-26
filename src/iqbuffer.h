#ifndef IQBUFFER_H
#define IQBUFFER_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace IqBuffer {

void publish(const float *samples, std::size_t floatCount, bool queueBlock = true);
bool snapshot(std::vector<float> &out, std::uint64_t *sequence = nullptr);
bool popBlock(std::vector<float> &out, std::uint64_t *sequence = nullptr);
void clear();
std::size_t size();
std::size_t queuedBlocks();
std::size_t queuedFloatCount();
void setSampleRateEstimate(double sampleRate);
double sampleRateEstimate();

} // namespace IqBuffer

#endif // IQBUFFER_H
