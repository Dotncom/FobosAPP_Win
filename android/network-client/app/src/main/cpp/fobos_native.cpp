#include <jni.h>

#include <android/log.h>
#include <errno.h>
#include <linux/usbdevice_fs.h>
#include <cmath>
#include <cstdint>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {

constexpr const char *kLogTag = "FobosNativeUsb";
constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr int kAudioSampleRate = 48000;
constexpr bool kEnableNativeAudioFir = false;
constexpr int kInputRf = 0;
constexpr int kInputHfCombined = 1;
constexpr int kInputHf1 = 2;
constexpr int kInputHf2 = 3;
constexpr int kInputHfNoiseCancel = 4;
constexpr int kModAm = 0;
constexpr int kModNfm = 1;
constexpr int kModSam = 2;
constexpr int kModUsb = 3;
constexpr int kModLsb = 4;
constexpr int kModDsb = 5;
constexpr int kModCw = 6;
constexpr int kModWfm = 7;
constexpr int kModDmr = 17;

struct NativeAudioState {
    double mixerPhase = 0.0;
    double rawDcI = 0.0;
    double rawDcQ = 0.0;
    double channelSumI = 0.0;
    double channelSumQ = 0.0;
    int decimationCount = 0;
    std::vector<double> firTaps;
    std::vector<double> firRingI;
    std::vector<double> firRingQ;
    int firRingIndex = 0;
    int firConfiguredDecimation = 0;
    int firConfiguredModulation = -1;
    double firConfiguredSampleRate = 0.0;
    double firConfiguredBandwidth = 0.0;
    double channelLowPassI = 0.0;
    double channelLowPassQ = 0.0;
    double demodLowPass = 0.0;
    double deemphasis = 0.0;
    double fmLastI = 0.0;
    double fmLastQ = 0.0;
    bool fmPhaseValid = false;
    double audioDc = 0.0;
    double agc = 0.001;
    double resamplePhase = 0.0;
    long long debugLastNanos = 0;
};

NativeAudioState gAudioState;
std::mutex gAudioMutex;

long long nowNanos() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long long>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

uint16_t readLe16(const jbyte *data, int offset) {
    const auto *bytes = reinterpret_cast<const uint8_t *>(data);
    return static_cast<uint16_t>(bytes[offset]) |
            static_cast<uint16_t>(bytes[offset + 1] << 8);
}

bool isDirectInput(int inputMode) {
    return inputMode != kInputRf;
}

double clamp01(double value) {
    if (!std::isfinite(value)) {
        return 0.0;
    }
    return std::max(0.0, std::min(1.0, value));
}

double audioTargetChannelRate(int modulationType, double bandwidth) {
    switch (modulationType) {
        case kModWfm:
            return std::max(384000.0, std::min(768000.0, bandwidth * 3.0));
        case kModDmr:
            return 96000.0;
        case kModNfm:
            return std::max(240000.0, std::min(384000.0, bandwidth * 4.0));
        default:
            return 192000.0;
    }
}

double audioChannelCutoffForMode(int modulationType, double bandwidth) {
    switch (modulationType) {
        case kModWfm:
            return std::min(140000.0, std::max(80000.0, bandwidth * 0.55));
        case kModNfm:
            return std::min(25000.0, std::max(6000.0, bandwidth * 0.55));
        case kModDmr:
            return std::min(9500.0, std::max(6000.0, bandwidth * 0.75));
        case kModUsb:
        case kModLsb:
            return std::min(3600.0, std::max(700.0, bandwidth * 0.95));
        case kModCw:
            return std::min(1500.0, std::max(250.0, bandwidth * 0.5));
        case kModDsb:
        case kModSam:
            return std::min(12000.0, std::max(1000.0, bandwidth * 0.45));
        case kModAm:
        default:
            return std::min(10000.0, std::max(1000.0, bandwidth * 0.45));
    }
}

double audioDemodCutoffForMode(int modulationType, double bandwidth) {
    switch (modulationType) {
        case kModWfm:
            return 15000.0;
        case kModNfm:
            return 3000.0;
        case kModDmr:
            return 6000.0;
        case kModUsb:
        case kModLsb:
            return std::min(3600.0, std::max(700.0, bandwidth * 0.95));
        case kModCw:
            return 1200.0;
        case kModDsb:
        case kModSam:
            return std::min(6500.0, std::max(1000.0, bandwidth * 0.45));
        case kModAm:
        default:
            return std::min(6000.0, std::max(1000.0, bandwidth * 0.45));
    }
}

int audioChannelDecimationFactor(double sampleRate, int modulationType, double bandwidth) {
    const double targetRate = audioTargetChannelRate(modulationType, bandwidth);
    int factor = std::max(1, static_cast<int>(std::floor(sampleRate / std::max(1.0, targetRate))));
    if (sampleRate > 1000000.0) {
        int minimum = 2;
        if (modulationType == kModNfm || modulationType == kModDmr) {
            minimum = 3;
        }
        factor = std::max(minimum, factor);
    }
    return factor;
}

int audioInputStride(double sampleRate, int modulationType) {
    (void) sampleRate;
    (void) modulationType;
    return 1;
}

int audioFirTapCount(int decimationFactor, int modulationType) {
    int taps;
    switch (modulationType) {
        case kModWfm:
            taps = decimationFactor * 4 + 1;
            taps = std::max(41, std::min(81, taps));
            break;
        case kModNfm:
        case kModDmr:
        case kModUsb:
        case kModLsb:
        case kModCw:
            taps = decimationFactor * 5 + 1;
            taps = std::max(49, std::min(97, taps));
            break;
        default:
            taps = decimationFactor * 4 + 1;
            taps = std::max(33, std::min(81, taps));
            break;
    }
    return (taps & 1) == 0 ? taps + 1 : taps;
}

void ensureAudioFirConfigured(NativeAudioState &state,
                              int decimationFactor,
                              double sampleRate,
                              int modulationType,
                              double bandwidth) {
    const int normalizedDecimation = std::max(1, decimationFactor);
    if (!state.firTaps.empty() &&
            state.firConfiguredDecimation == normalizedDecimation &&
            state.firConfiguredModulation == modulationType &&
            std::abs(state.firConfiguredSampleRate - sampleRate) < 1.0 &&
            std::abs(state.firConfiguredBandwidth - bandwidth) < 1.0) {
        return;
    }

    const int tapCount = audioFirTapCount(normalizedDecimation, modulationType);
    const double outputNyquist = sampleRate / (2.0 * normalizedDecimation);
    double cutoff = std::min(audioChannelCutoffForMode(modulationType, bandwidth) * 1.15,
                             outputNyquist * 0.82);
    cutoff = std::max(500.0, std::min(cutoff, sampleRate * 0.45));
    const double normalizedCutoff = cutoff / sampleRate;
    const int center = tapCount / 2;

    std::vector<double> taps(static_cast<size_t>(tapCount));
    double sum = 0.0;
    for (int i = 0; i < tapCount; ++i) {
        const int m = i - center;
        const double sinc = m == 0
                ? 2.0 * normalizedCutoff
                : std::sin(2.0 * kPi * normalizedCutoff * m) / (kPi * m);
        const double window = 0.42
                - 0.5 * std::cos((2.0 * kPi * i) / (tapCount - 1))
                + 0.08 * std::cos((4.0 * kPi * i) / (tapCount - 1));
        taps[static_cast<size_t>(i)] = sinc * window;
        sum += taps[static_cast<size_t>(i)];
    }
    if (std::abs(sum) > 1.0e-12) {
        for (double &tap : taps) {
            tap /= sum;
        }
    }

    state.firTaps = std::move(taps);
    state.firRingI.assign(static_cast<size_t>(tapCount), 0.0);
    state.firRingQ.assign(static_cast<size_t>(tapCount), 0.0);
    state.firRingIndex = 0;
    state.decimationCount = 0;
    state.firConfiguredDecimation = normalizedDecimation;
    state.firConfiguredModulation = modulationType;
    state.firConfiguredSampleRate = sampleRate;
    state.firConfiguredBandwidth = bandwidth;
}

void pushAudioFirSample(NativeAudioState &state, double sampleI, double sampleQ) {
    if (state.firRingI.empty()) {
        return;
    }
    state.firRingIndex = (state.firRingIndex + 1) % static_cast<int>(state.firRingI.size());
    state.firRingI[static_cast<size_t>(state.firRingIndex)] = sampleI;
    state.firRingQ[static_cast<size_t>(state.firRingIndex)] = sampleQ;
}

void updateAudioFirOutput(NativeAudioState &state, double &outputI, double &outputQ) {
    if (state.firTaps.empty() || state.firRingI.size() != state.firTaps.size()) {
        outputI = 0.0;
        outputQ = 0.0;
        return;
    }
    double sumI = 0.0;
    double sumQ = 0.0;
    int index = state.firRingIndex;
    const int ringLength = static_cast<int>(state.firRingI.size());
    for (size_t tap = 0; tap < state.firTaps.size(); ++tap) {
        const double coefficient = state.firTaps[tap];
        sumI += state.firRingI[static_cast<size_t>(index)] * coefficient;
        sumQ += state.firRingQ[static_cast<size_t>(index)] * coefficient;
        --index;
        if (index < 0) {
            index = ringLength - 1;
        }
    }
    outputI = sumI;
    outputQ = sumQ;
}

int rawFobosSampleOffset(int sample,
                         int usableBytes,
                         bool agileSwappedHalves,
                         int agileFirstOffset,
                         int agileSecondOffset) {
    const int directOffset = sample * 4;
    if (!agileSwappedHalves) {
        return directOffset;
    }
    constexpr int pairBytes = 0x8000;
    constexpr int halfBytes = 0x4000;
    constexpr int pairSamples = pairBytes / 4;
    constexpr int halfSamples = halfBytes / 4;
    const int pair = sample / pairSamples;
    const int withinPair = sample - pair * pairSamples;
    const int pairBase = pair * pairBytes;
    if (pairBase + pairBytes > usableBytes) {
        return directOffset;
    }
    if (withinPair < halfSamples) {
        return pairBase + agileFirstOffset + withinPair * 4;
    }
    return pairBase + agileSecondOffset + (withinPair - halfSamples) * 4;
}

double demodulateChannelSample(NativeAudioState &state,
                               double channelI,
                               double channelQ,
                               int modulationType,
                               double channelRate) {
    switch (modulationType) {
        case kModNfm:
        case kModWfm:
        case kModDmr: {
            const double magnitude = std::hypot(channelI, channelQ);
            if (magnitude < 1.0e-9) {
                return 0.0;
            }
            channelI /= magnitude;
            channelQ /= magnitude;
            if (!state.fmPhaseValid) {
                state.fmLastI = channelI;
                state.fmLastQ = channelQ;
                state.fmPhaseValid = true;
                return 0.0;
            }
            const double cross = state.fmLastI * channelQ - state.fmLastQ * channelI;
            const double dot = state.fmLastI * channelI + state.fmLastQ * channelQ;
            const double diff = std::atan2(cross, dot);
            state.fmLastI = channelI;
            state.fmLastQ = channelQ;
            if (modulationType == kModDmr) {
                return diff * 10.0;
            }
            const double deviationHz = modulationType == kModWfm ? 75000.0 : 5000.0;
            const double rate = std::max(static_cast<double>(kAudioSampleRate), channelRate);
            return diff * rate / (2.0 * kPi * deviationHz);
        }
        case kModAm:
        case kModSam:
            return std::hypot(channelI, channelQ);
        case kModUsb:
        case kModLsb:
        case kModDsb:
        case kModCw:
            return channelI;
        default:
            return 0.0;
    }
}

double applyDeemphasis(NativeAudioState &state, double sample, int modulationType, double channelRate) {
    if (modulationType != kModWfm || channelRate <= 0.0) {
        return sample;
    }
    constexpr double tauSeconds = 50.0e-6;
    const double alpha = clamp01(1.0 - std::exp(-1.0 / (tauSeconds * channelRate)));
    state.deemphasis += alpha * (sample - state.deemphasis);
    return state.deemphasis;
}

double normalizeAudioSample(NativeAudioState &state, double demodulated, int modulationType) {
    const bool digital = modulationType == kModDmr;
    const bool fm = modulationType == kModWfm || modulationType == kModNfm;
    const double dcRate = digital ? 0.0001 : (fm ? 0.0008 : 0.0005);
    state.audioDc += (demodulated - state.audioDc) * dcRate;
    const double acSample = demodulated - state.audioDc;
    const double absSample = std::abs(acSample);
    const double agcRate = absSample > state.agc ? (fm ? 0.006 : 0.04) : (fm ? 0.00008 : 0.0002);
    state.agc += (absSample - state.agc) * agcRate;
    state.agc = std::max(state.agc, 0.0001);
    const double target = digital ? 0.25 :
            (modulationType == kModWfm ? 0.30 : (modulationType == kModNfm ? 0.30 : 0.28));
    const double normalized = acSample * (target / state.agc);
    return digital ? normalized : std::tanh(normalized * (fm ? 0.55 : 1.0));
}

void resetAudioStateLocked() {
    gAudioState = NativeAudioState{};
}

void disableAudioFir(NativeAudioState &state) {
    if (state.firTaps.empty() &&
            state.firRingI.empty() &&
            state.firRingQ.empty()) {
        return;
    }
    state.firTaps.clear();
    state.firRingI.clear();
    state.firRingQ.clear();
    state.firRingIndex = 0;
    state.firConfiguredDecimation = 0;
    state.firConfiguredModulation = -1;
    state.firConfiguredSampleRate = 0.0;
    state.firConfiguredBandwidth = 0.0;
}

std::string formatResult(long long bytesRead,
                         long long reads,
                         long long errors,
                         int lastError,
                         long long elapsedNanos,
                         int endpoint,
                         int transferBytes,
                         const char *mode) {
    const double seconds = std::max(0.001, elapsedNanos / 1000000000.0);
    const double mbps = (bytesRead * 8.0) / (seconds * 1000000.0);
    const double msps = (bytesRead / 4.0) / (seconds * 1000000.0);
    const double readsPerSecond = reads / seconds;

    char buffer[384];
    snprintf(buffer,
             sizeof(buffer),
             "native USB %s bulk: %.1f Mb/s, %.2f MS/s, %.0f reads/s, bytes %lld, reads %lld, errors %lld, last errno %d (%s), ep 0x%02x, transfer %d",
             mode,
             mbps,
             msps,
             readsPerSecond,
             bytesRead,
             reads,
             errors,
             lastError,
             lastError != 0 ? strerror(lastError) : "none",
             endpoint & 0xff,
             transferBytes);
    return std::string(buffer);
}

struct NativeTransfer {
    std::unique_ptr<unsigned char[]> data;
    usbdevfs_urb urb{};
};

std::string runAsyncBulkBenchmark(int fd,
                                  int endpointAddress,
                                  int transferBytes,
                                  int durationMs,
                                  int queueDepth) {
    const int normalizedQueueDepth = std::max(1, std::min(64, queueDepth));
    std::vector<std::unique_ptr<NativeTransfer>> transfers;
    transfers.reserve(normalizedQueueDepth);

    long long bytesRead = 0;
    long long reads = 0;
    long long errors = 0;
    int lastError = 0;

    for (int i = 0; i < normalizedQueueDepth; ++i) {
        auto transfer = std::make_unique<NativeTransfer>();
        transfer->data.reset(new (std::nothrow) unsigned char[transferBytes]);
        if (!transfer->data) {
            return "native USB async bulk: allocation failed";
        }
        transfer->urb.type = USBDEVFS_URB_TYPE_BULK;
        transfer->urb.endpoint = static_cast<unsigned char>(endpointAddress & 0xff);
        transfer->urb.buffer = transfer->data.get();
        transfer->urb.buffer_length = transferBytes;
        transfer->urb.usercontext = transfer.get();
        if (ioctl(fd, USBDEVFS_SUBMITURB, &transfer->urb) != 0) {
            lastError = errno;
            ++errors;
            return formatResult(bytesRead,
                                reads,
                                errors,
                                lastError,
                                1,
                                endpointAddress,
                                transferBytes,
                                "async submit-failed");
        }
        transfers.emplace_back(std::move(transfer));
    }

    const long long start = nowNanos();
    const long long deadline = start + static_cast<long long>(durationMs) * 1000000LL;
    while (nowNanos() < deadline) {
        usbdevfs_urb *completed = nullptr;
        if (ioctl(fd, USBDEVFS_REAPURB, &completed) != 0) {
            lastError = errno;
            ++errors;
            if (lastError == EINTR || lastError == EAGAIN) {
                continue;
            }
            break;
        }
        if (completed == nullptr) {
            continue;
        }
        if (completed->status == 0 && completed->actual_length > 0) {
            bytesRead += completed->actual_length;
            ++reads;
        } else {
            lastError = completed->status != 0 ? -completed->status : errno;
            ++errors;
        }
        completed->actual_length = 0;
        completed->status = 0;
        if (nowNanos() < deadline &&
                ioctl(fd, USBDEVFS_SUBMITURB, completed) != 0) {
            lastError = errno;
            ++errors;
            break;
        }
    }

    for (auto &transfer : transfers) {
        ioctl(fd, USBDEVFS_DISCARDURB, &transfer->urb);
    }
    for (;;) {
        usbdevfs_urb *completed = nullptr;
        if (ioctl(fd, USBDEVFS_REAPURBNDELAY, &completed) != 0) {
            break;
        }
    }

    const long long elapsed = nowNanos() - start;
    std::string result = formatResult(bytesRead,
                                      reads,
                                      errors,
                                      lastError,
                                      elapsed,
                                      endpointAddress,
                                      transferBytes,
                                      "async");
    result += ", q ";
    result += std::to_string(normalizedQueueDepth);
    return result;
}

std::string runSyncBulkBenchmark(int fd,
                                 int endpointAddress,
                                 int transferBytes,
                                 int durationMs) {
    std::unique_ptr<unsigned char[]> data(new (std::nothrow) unsigned char[transferBytes]);
    if (!data) {
        return "native USB sync bulk: allocation failed";
    }

    long long bytesRead = 0;
    long long reads = 0;
    long long errors = 0;
    int lastError = 0;
    const long long start = nowNanos();
    const long long deadline = start + static_cast<long long>(durationMs) * 1000000LL;

    while (nowNanos() < deadline) {
        usbdevfs_bulktransfer transfer{};
        transfer.ep = static_cast<unsigned int>(endpointAddress & 0xff);
        transfer.len = static_cast<unsigned int>(transferBytes);
        transfer.timeout = 1000;
        transfer.data = data.get();

        int result = ioctl(fd, USBDEVFS_BULK, &transfer);
        if (result > 0) {
            bytesRead += result;
            ++reads;
            continue;
        }
        if (result == 0) {
            ++reads;
            continue;
        }
        lastError = errno;
        ++errors;
        __android_log_print(ANDROID_LOG_WARN,
                            kLogTag,
                            "USBDEVFS_BULK failed errno=%d %s",
                            lastError,
                            strerror(lastError));
        if (lastError == EINTR || lastError == ETIMEDOUT) {
            continue;
        }
        break;
    }

    const long long elapsed = nowNanos() - start;
    return formatResult(bytesRead,
                        reads,
                        errors,
                        lastError,
                        elapsed,
                        endpointAddress,
                        transferBytes,
                        "sync");
}

} // namespace

extern "C" JNIEXPORT jstring JNICALL
Java_com_fobosapp_networkclient_NativeUsbBridge_nativeBulkBenchmark(JNIEnv *env,
                                                                    jclass,
                                                                    jint fd,
                                                                    jint endpointAddress,
                                                                    jint transferBytes,
                                                                    jint durationMs,
                                                                    jint queueDepth) {
    if (fd < 0) {
        return env->NewStringUTF("native USB bulk: invalid fd");
    }

    const int normalizedTransfer = std::max(4096, std::min(1024 * 1024, static_cast<int>(transferBytes)));
    const int normalizedDuration = std::max(100, std::min(10000, static_cast<int>(durationMs)));
    if (queueDepth <= 0) {
        std::string syncSummary = runSyncBulkBenchmark(fd,
                                                       endpointAddress,
                                                       normalizedTransfer,
                                                       normalizedDuration);
        __android_log_print(ANDROID_LOG_INFO, kLogTag, "%s", syncSummary.c_str());
        return env->NewStringUTF(syncSummary.c_str());
    }
    std::string summary = runAsyncBulkBenchmark(fd,
                                                endpointAddress,
                                                normalizedTransfer,
                                                normalizedDuration,
                                                static_cast<int>(queueDepth));
    if (summary.find("submit-failed") != std::string::npos) {
        summary += "\n";
        summary += runSyncBulkBenchmark(fd,
                                        endpointAddress,
                                        normalizedTransfer,
                                        normalizedDuration);
    }
    __android_log_print(ANDROID_LOG_INFO, kLogTag, "%s", summary.c_str());
    return env->NewStringUTF(summary.c_str());
}

extern "C" JNIEXPORT void JNICALL
Java_com_fobosapp_networkclient_NativeUsbBridge_nativeResetAudioDemodState(JNIEnv *,
                                                                           jclass) {
    std::lock_guard<std::mutex> lock(gAudioMutex);
    resetAudioStateLocked();
}

extern "C" JNIEXPORT jbyteArray JNICALL
Java_com_fobosapp_networkclient_NativeUsbBridge_nativeAudioFromRawFobosIq(JNIEnv *env,
                                                                          jclass,
                                                                          jbyteArray bufferArray,
                                                                          jint bytesRead,
                                                                          jdouble centerFrequency,
                                                                          jdouble listeningFrequency,
                                                                          jdouble sampleRate,
                                                                          jdouble timingSampleRate,
                                                                          jint inputMode,
                                                                          jint modulationType,
                                                                          jdouble bandwidth,
                                                                          jboolean activeAgileApi,
                                                                          jintArray statsArray) {
    if (bufferArray == nullptr || sampleRate <= kAudioSampleRate) {
        return env->NewByteArray(0);
    }
    const jsize inputLength = env->GetArrayLength(bufferArray);
    const int usableBytes = std::max(0, std::min(static_cast<int>(bytesRead),
                                                static_cast<int>(inputLength))) & ~0x3;
    const int complexSamples = usableBytes / 4;
    if (complexSamples <= 2) {
        return env->NewByteArray(0);
    }

    jboolean isCopy = JNI_FALSE;
    jbyte *data = env->GetByteArrayElements(bufferArray, &isCopy);
    if (data == nullptr) {
        return env->NewByteArray(0);
    }

    std::vector<jbyte> pcm;
    int pcmPeak = 0;
    int decimatedSamples = 0;
    double rawPeak = 0.0;
    double channelPeak = 0.0;
    const double tuneOffsetHz = isDirectInput(inputMode)
            ? listeningFrequency
            : listeningFrequency - centerFrequency;

    {
        std::lock_guard<std::mutex> lock(gAudioMutex);
        NativeAudioState &state = gAudioState;
        const int inputStride = audioInputStride(sampleRate, modulationType);
        const double effectiveSampleRate = sampleRate / std::max(1, inputStride);
        const int processedSamplesEstimate = (complexSamples + inputStride - 1) / inputStride;
        pcm.reserve(static_cast<size_t>(std::ceil((processedSamplesEstimate / effectiveSampleRate) *
                                                 kAudioSampleRate) + 16) * 2);
        const int decimationFactor = audioChannelDecimationFactor(effectiveSampleRate,
                                                                  modulationType,
                                                                  bandwidth);
        const double channelRate = effectiveSampleRate / std::max(1, decimationFactor);
        const double playbackSampleRate = std::max(kAudioSampleRate * 1.25,
                                                   std::min(static_cast<double>(sampleRate),
                                                            timingSampleRate > 0.0
                                                                    ? static_cast<double>(timingSampleRate)
                                                                    : static_cast<double>(sampleRate))) /
                std::max(1, inputStride);
        const double channelCutoff = std::min(audioChannelCutoffForMode(modulationType, bandwidth),
                                             channelRate * 0.45);
        const double channelAlpha = clamp01(1.0 - std::exp(-2.0 * kPi * channelCutoff / channelRate));
        const double demodCutoff = std::min(audioDemodCutoffForMode(modulationType, bandwidth),
                                           channelRate * 0.45);
        const double demodAlpha = clamp01(1.0 - std::exp(-2.0 * kPi * demodCutoff / channelRate));
        const double outputStep = (kAudioSampleRate * std::max(1, decimationFactor)) / playbackSampleRate;
        const double phaseStep = -2.0 * kPi * tuneOffsetHz / effectiveSampleRate;
        double oscillatorI = std::cos(state.mixerPhase);
        double oscillatorQ = std::sin(state.mixerPhase);
        const double stepI = std::cos(phaseStep);
        const double stepQ = std::sin(phaseStep);
        if (kEnableNativeAudioFir) {
            ensureAudioFirConfigured(state,
                                     decimationFactor,
                                     effectiveSampleRate,
                                     modulationType,
                                     bandwidth);
        } else {
            disableAudioFir(state);
        }

        const bool directSampling = isDirectInput(inputMode);
        const uint16_t p0 = readLe16(data, 0);
        const uint16_t p1 = usableBytes >= 4 ? readLe16(data, 2) : 0;
        const bool rawSwapIq = (p0 & 0x8000) != 0 && (p1 & 0x8000) != 0;
        const bool swapIq = directSampling || !rawSwapIq;
        bool agileSwappedHalves = false;
        int agileFirstOffset = 0;
        int agileSecondOffset = 0x4000;
        if (activeAgileApi == JNI_TRUE) {
            agileSwappedHalves = (p0 & 0x4000) != 0 &&
                    (p1 & 0x4000) != 0 &&
                    usableBytes >= 0x8000;
            agileFirstOffset = p0 & p1 & 0x4000;
            agileSecondOffset = agileFirstOffset ^ 0x4000;
        }

        constexpr double rawScale = 1.0 / 8192.0;
        constexpr double dcRate = 0.0015;
        for (int sample = 0; sample < complexSamples; sample += inputStride) {
            const int pos = rawFobosSampleOffset(sample,
                                                 usableBytes,
                                                 agileSwappedHalves,
                                                 agileFirstOffset,
                                                 agileSecondOffset);
            if (pos < 0 || pos + 3 >= usableBytes) {
                continue;
            }
            const int first = readLe16(data, pos);
            const int second = readLe16(data, pos + 2);
            const double real = static_cast<double>((swapIq ? second : first) & 0x3fff);
            const double imag = static_cast<double>((swapIq ? first : second) & 0x3fff);
            state.rawDcI += dcRate * (real - state.rawDcI);
            state.rawDcQ += dcRate * (imag - state.rawDcQ);
            double iSample = (real - state.rawDcI) * rawScale;
            double qSample = (imag - state.rawDcQ) * rawScale;
            rawPeak = std::max(rawPeak, std::max(std::abs(iSample), std::abs(qSample)));

            if (inputMode == kInputHfCombined) {
                if (tuneOffsetHz < 0.0) {
                    qSample = 0.0;
                } else {
                    iSample = qSample;
                    qSample = 0.0;
                }
            } else if (inputMode == kInputHf1 || inputMode == kInputHfNoiseCancel) {
                qSample = 0.0;
            } else if (inputMode == kInputHf2) {
                iSample = qSample;
                qSample = 0.0;
            }

            const double mixedI = iSample * oscillatorI - qSample * oscillatorQ;
            const double mixedQ = iSample * oscillatorQ + qSample * oscillatorI;
            state.channelSumI += mixedI;
            state.channelSumQ += mixedQ;
            if (kEnableNativeAudioFir) {
                pushAudioFirSample(state, mixedI, mixedQ);
            }
            ++state.decimationCount;

            const double nextOscillatorI = oscillatorI * stepI - oscillatorQ * stepQ;
            oscillatorQ = oscillatorI * stepQ + oscillatorQ * stepI;
            oscillatorI = nextOscillatorI;
            if ((sample & 0x1ff) == 0) {
                const double magnitude = oscillatorI * oscillatorI + oscillatorQ * oscillatorQ;
                if (magnitude > 0.0) {
                    const double scale = 1.0 / std::sqrt(magnitude);
                    oscillatorI *= scale;
                    oscillatorQ *= scale;
                }
            }

            if (state.decimationCount < decimationFactor) {
                continue;
            }

            double channelI = 0.0;
            double channelQ = 0.0;
            if (kEnableNativeAudioFir && !state.firTaps.empty()) {
                updateAudioFirOutput(state, channelI, channelQ);
            } else {
                channelI = state.channelSumI / state.decimationCount;
                channelQ = state.channelSumQ / state.decimationCount;
            }
            state.channelSumI = 0.0;
            state.channelSumQ = 0.0;
            state.decimationCount = 0;
            ++decimatedSamples;
            channelPeak = std::max(channelPeak, std::hypot(channelI, channelQ));

            state.channelLowPassI += channelAlpha * (channelI - state.channelLowPassI);
            state.channelLowPassQ += channelAlpha * (channelQ - state.channelLowPassQ);
            double demodulated = demodulateChannelSample(state,
                                                         state.channelLowPassI,
                                                         state.channelLowPassQ,
                                                         modulationType,
                                                         channelRate);
            demodulated = applyDeemphasis(state, demodulated, modulationType, channelRate);
            state.demodLowPass += demodAlpha * (demodulated - state.demodLowPass);
            demodulated = state.demodLowPass;

            state.resamplePhase += outputStep;
            while (state.resamplePhase >= 1.0) {
                state.resamplePhase -= 1.0;
                const double audioValue = normalizeAudioSample(state, demodulated, modulationType);
                const double clipped = std::max(-1.0, std::min(1.0, audioValue));
                const int pcmValue = static_cast<int>(std::lround(clipped * 32767.0));
                pcmPeak = std::max(pcmPeak, std::abs(pcmValue));
                pcm.push_back(static_cast<jbyte>(pcmValue & 0xff));
                pcm.push_back(static_cast<jbyte>((pcmValue >> 8) & 0xff));
            }
        }

        state.mixerPhase = std::atan2(oscillatorQ, oscillatorI);
        const long long now = nowNanos();
        if (now - state.debugLastNanos > 1000000000LL) {
            state.debugLastNanos = now;
            __android_log_print(ANDROID_LOG_DEBUG,
                                kLogTag,
                                "native audio raw inIq=%d decimated=%d out=%zu sr=%.3fMHz play=%.3fMHz off=%.0fHz dec=%d chRate=%.0f taps=%zu rawPeak=%.3f chPeak=%.5f pcmPeak=%d mod=%d",
                                complexSamples,
                                decimatedSamples,
                                pcm.size() / 2,
                                effectiveSampleRate / 1000000.0,
                                playbackSampleRate / 1000000.0,
                                tuneOffsetHz,
                                decimationFactor,
                                channelRate,
                                state.firTaps.size(),
                                rawPeak,
                                channelPeak,
                                pcmPeak,
                                static_cast<int>(modulationType));
        }

        if (statsArray != nullptr && env->GetArrayLength(statsArray) >= 3) {
            jint stats[3] = {
                    static_cast<jint>(pcmPeak),
                    static_cast<jint>(pcm.size() / 2),
                    static_cast<jint>(std::lround(tuneOffsetHz))
            };
            env->SetIntArrayRegion(statsArray, 0, 3, stats);
        }
    }

    env->ReleaseByteArrayElements(bufferArray, data, JNI_ABORT);
    jbyteArray result = env->NewByteArray(static_cast<jsize>(pcm.size()));
    if (result != nullptr && !pcm.empty()) {
        env->SetByteArrayRegion(result, 0, static_cast<jsize>(pcm.size()), pcm.data());
    }
    return result;
}
