#ifndef M_PI
#define M_PI_F 3.1415927f
#endif

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif


#include "audioprocessor.h"
#include "iqbuffer.h"
#include "diagnosticlogging.h"

#pragma comment(lib, "winmm.lib")

namespace {
constexpr double AUDIO_OUTPUT_RATE = 48000.0;
constexpr double TWO_PI = 6.28318530717958647692;
constexpr double CW_TONE_HZ = 700.0;
constexpr double AM_CHANNEL_RATE = 192000.0;
constexpr double FM_MIN_CHANNEL_RATE = 240000.0;
constexpr double FM_MAX_CHANNEL_RATE = 768000.0;
constexpr double FM_DEEMPHASIS_SECONDS = 50e-6;
constexpr double NFM_DEEMPHASIS_SECONDS = 300e-6;
constexpr size_t MAX_AUDIO_BUFFER_SAMPLES = BUFFER_SIZE * 10;
constexpr double SSB_LOW_CUT_HZ = 250.0;
constexpr double SSB_MAX_AUDIO_HZ = 3600.0;
constexpr double SAM_LOCK_RANGE_HZ = 1500.0;

double clampDouble(double value, double low, double high) {
    return (std::max)(low, (std::min)(value, high));
}

double wrapRadians(double phase) {
    return std::remainder(phase, TWO_PI);
}

double channelCutoffForMode(int modulationType, double bandwidth) {
    switch (modulationType) {
    case MOD_WFM:
        return (std::min)(140000.0, (std::max)(80000.0, bandwidth * 0.55));
    case MOD_NFM:
        return (std::min)(25000.0, (std::max)(6000.0, bandwidth * 0.55));
    case MOD_RTTY:
    case MOD_FSK:
        return (std::min)(12000.0, (std::max)(2500.0, bandwidth * 0.55));
    case MOD_USB:
    case MOD_LSB:
    case MOD_FT8:
    case MOD_PSK:
        return (std::min)(SSB_MAX_AUDIO_HZ, (std::max)(700.0, bandwidth * 0.95));
    case MOD_CW:
        return (std::min)(1500.0, (std::max)(250.0, bandwidth * 0.5));
    case MOD_DSB:
    case MOD_SAM:
        return (std::min)(12000.0, (std::max)(1000.0, bandwidth * 0.45));
    case MOD_AM:
    default:
        return (std::min)(10000.0, (std::max)(1000.0, bandwidth * 0.45));
    }
}

double demodAudioCutoffForMode(int modulationType, double bandwidth) {
    switch (modulationType) {
    case MOD_WFM:
        return 15000.0;
    case MOD_NFM:
        return 3500.0;
    case MOD_RTTY:
    case MOD_FSK:
        return 5000.0;
    case MOD_USB:
    case MOD_LSB:
    case MOD_FT8:
    case MOD_PSK:
        return (std::min)(SSB_MAX_AUDIO_HZ, (std::max)(700.0, bandwidth * 0.95));
    case MOD_CW:
        return 1200.0;
    case MOD_DSB:
    case MOD_SAM:
        return (std::min)(10000.0, (std::max)(1000.0, bandwidth * 0.45));
    case MOD_AM:
    default:
        return (std::min)(10000.0, (std::max)(1000.0, bandwidth * 0.45));
    }
}

double targetChannelRate(int modulationType, double bandwidth) {
    if (modulationType == MOD_WFM) {
        return clampDouble((std::max)(384000.0, bandwidth * 3.0), 384000.0, FM_MAX_CHANNEL_RATE);
    }
    if (modulationType == MOD_NFM || modulationType == MOD_RTTY || modulationType == MOD_FSK) {
        return clampDouble((std::max)(FM_MIN_CHANNEL_RATE, bandwidth * 4.0),
                           FM_MIN_CHANNEL_RATE,
                           384000.0);
    }
    return AM_CHANNEL_RATE;
}

int channelDecimationFactor(double inputRate, int modulationType, double bandwidth) {
    const double targetRate = targetChannelRate(modulationType, bandwidth);
    return (std::max)(1, static_cast<int>(std::floor(inputRate / targetRate)));
}

}

AudioProcessor::AudioProcessor(QObject *parent) : QObject(parent), running(false), workerThread(nullptr), hWaveOut(nullptr) {
    ZeroMemory(&format, sizeof(WAVEFORMATEX));
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
    format.nSamplesPerSec = 48000;
    format.wBitsPerSample = 16;
    format.nBlockAlign = (format.nChannels * format.wBitsPerSample) / 8;
    format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;
    format.cbSize = 0;

    for (int i = 0; i < NUM_BUFFERS; i++) {
        waveBuffers[i].resize(BUFFER_SIZE);
        ZeroMemory(&waveHdrs[i], sizeof(WAVEHDR));
        waveHdrs[i].lpData = reinterpret_cast<LPSTR>(waveBuffers[i].data());
        waveHdrs[i].dwBufferLength = BUFFER_SIZE * sizeof(short);
        bufferReady[i] = true;
    }
}


AudioProcessor::~AudioProcessor() {
    stopDemodulation();
    closeAudioDevice();
}

void AudioProcessor::configure(const RadioSettings &settings) {
    std::lock_guard<std::mutex> lock(settingsMutex);
    audioSettings = settings;
    selectedAudioDeviceID = settings.audioDeviceId;
}

void AudioProcessor::setAudioDevice(int deviceID) {
    qDebug() << "start setAudioDevice";
    {
        std::lock_guard<std::mutex> lock(settingsMutex);
        selectedAudioDeviceID = deviceID;
        audioSettings.audioDeviceId = deviceID;
    }

    const bool wasRunning = running.load();
    if (wasRunning) {
        stopDemodulation();
        startDemodulation();
        return;
    }

    if (hWaveOut && openedAudioDeviceID != deviceID) {
        closeAudioDevice();
    }
    qDebug() << "Audio device selected. It will be opened when audio starts.";
}

void AudioProcessor::setLocalPlaybackEnabled(bool enabled) {
    const bool wasEnabled = localPlaybackEnabled.exchange(enabled);
    if (wasEnabled == enabled) {
        return;
    }
    qDebug() << "[Audio] local waveOut playback" << (enabled ? "enabled" : "disabled");
    if (!enabled) {
        closeAudioDevice();
    }
}

void AudioProcessor::startDemodulation() {
    bool expected = false;
    if (!running.compare_exchange_strong(expected, true)) {
        qDebug() << "Audio demodulation is already running.";
        return;
    }

    {
        std::lock_guard<std::mutex> lock(audioMutex);
        audioBuffer.clear();
        audioBufferReadOffset = 0;
        for (int i = 0; i < NUM_BUFFERS; ++i) {
            bufferReady[i] = true;
        }
    }
    resetDemodulatorState();

    if (localPlaybackEnabled.load()) {
        const int audioDeviceId = currentSettingsSnapshot().audioDeviceId;
        if (!openAudioDevice(audioDeviceId)) {
            closeAudioDevice();
            qDebug() << "Failed to open local audio output; demodulation will continue without waveOut playback.";
        }
    } else {
        qDebug() << "[Audio] local audio output disabled; demodulation will continue for streaming.";
    }
    // Запуск SDR-потока
    sdrWorker = std::thread(&AudioProcessor::SDRThread, this);

    // Запуск потока вывода аудио
    audioWorker = std::thread(&AudioProcessor::startAudioOutput, this);
}



void AudioProcessor::stopDemodulation() {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation enter"
                 << "running" << running.load()
                 << "hWaveOut" << hWaveOut
                 << "sdrJoinable" << sdrWorker.joinable()
                 << "audioJoinable" << audioWorker.joinable();
    }
    const bool wasRunning = running.exchange(false);
    cv.notify_all();

    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation joining workers" << "wasRunning" << wasRunning;
    }
    joinWorkerThreads();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation workers joined";
    }
    if (wasRunning) {
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[Audio] stopDemodulation resetting audio device";
        }
        pauseAudioDevice();
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[Audio] stopDemodulation audio device reset";
        }
    }
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation closing audio device";
    }
    closeAudioDevice();
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation audio device closed";
    }

    {
        std::lock_guard<std::mutex> lock(audioMutex);
        audioBuffer.clear();
        audioBufferReadOffset = 0;
        for (int i = 0; i < NUM_BUFFERS; ++i) {
            bufferReady[i] = true;
        }
    }
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[Audio] stopDemodulation exit";
    }
}

bool AudioProcessor::openAudioDevice(int deviceID) {
    if (hWaveOut && waveHeadersPrepared && openedAudioDeviceID == deviceID) {
        audioDeviceClosing = false;
        return true;
    }

    if (hWaveOut) {
        closeAudioDevice();
    }

    std::lock_guard<std::mutex> lock(waveOutMutex);
    audioDeviceClosing = false;
    MMRESULT result = waveOutOpen(&hWaveOut, deviceID, &format,
                                  reinterpret_cast<DWORD_PTR>(&AudioProcessor::WaveOutCallback),
                                  reinterpret_cast<DWORD_PTR>(this),
                                  CALLBACK_FUNCTION);

    if (result != MMSYSERR_NOERROR) {
        hWaveOut = nullptr;
        return false;
    }

    openedAudioDeviceID = deviceID;
    prepareWaveHeaders();
    return waveHeadersPrepared;
}

void AudioProcessor::closeAudioDevice() {
    std::lock_guard<std::mutex> lock(waveOutMutex);
    if (!hWaveOut) {
        return;
    }

    audioDeviceClosing = true;
    waveOutReset(hWaveOut);
    unprepareWaveHeaders();
    waveOutClose(hWaveOut);
    hWaveOut = nullptr;
    openedAudioDeviceID = -1;
}

void AudioProcessor::pauseAudioDevice() {
    std::lock_guard<std::mutex> lock(waveOutMutex);
    if (!hWaveOut) {
        return;
    }

    audioDeviceClosing = true;
    waveOutReset(hWaveOut);
    for (int i = 0; i < NUM_BUFFERS; ++i) {
        bufferReady[i] = true;
    }
    cv.notify_all();
}

void AudioProcessor::prepareWaveHeaders() {
    waveHeadersPrepared = false;

    for (int i = 0; i < NUM_BUFFERS; ++i) {
        ZeroMemory(&waveHdrs[i], sizeof(WAVEHDR));
        waveHdrs[i].lpData = reinterpret_cast<LPSTR>(waveBuffers[i].data());
        waveHdrs[i].dwBufferLength = BUFFER_SIZE * sizeof(short);
        bufferReady[i] = true;

        MMRESULT result = waveOutPrepareHeader(hWaveOut, &waveHdrs[i], sizeof(WAVEHDR));
        if (result != MMSYSERR_NOERROR) {
            qDebug() << "waveOutPrepareHeader failed!";
            unprepareWaveHeaders();
            return;
        }
    }

    waveHeadersPrepared = true;
}

void AudioProcessor::unprepareWaveHeaders() {
    for (int i = 0; i < NUM_BUFFERS; ++i) {
        if (waveHdrs[i].dwFlags & WHDR_PREPARED) {
            waveOutUnprepareHeader(hWaveOut, &waveHdrs[i], sizeof(WAVEHDR));
        }
        bufferReady[i] = true;
    }
    waveHeadersPrepared = false;
}

void AudioProcessor::joinWorkerThreads() {
    if (sdrWorker.joinable() && sdrWorker.get_id() != std::this_thread::get_id()) {
        sdrWorker.join();
    }
    if (audioWorker.joinable() && audioWorker.get_id() != std::this_thread::get_id()) {
        audioWorker.join();
    }
}

void AudioProcessor::resetDemodulatorState() {
    ncoPhase = 0.0;
    audioResamplePhase = 0.0;
    amLowPassState = std::complex<float>(0.0f, 0.0f);
    sidebandLowPassStates.fill(std::complex<float>(0.0f, 0.0f));
    channelDecimationSum = std::complex<float>(0.0f, 0.0f);
    channelDecimationCount = 0;
    fmPreviousSample = std::complex<float>(1.0f, 0.0f);
    fmPreviousValid = false;
    fmDeemphasisState = 0.0f;
    demodAudioLowPassState = 0.0f;
    samCarrierPhase = 0.0;
    samCarrierFrequency = 0.0;
    sidebandFilterPhase = 0.0;
    cwTonePhase = 0.0;
    amDcEstimate = 0.0f;
    amAgcLevel = 0.05f;
}

size_t AudioProcessor::queuedAudioSamples() const {
    return audioBuffer.size() > audioBufferReadOffset
               ? audioBuffer.size() - audioBufferReadOffset
               : 0;
}

void AudioProcessor::discardAudioSamples(size_t count) {
    audioBufferReadOffset += (std::min)(count, queuedAudioSamples());
    compactAudioBufferIfNeeded();
}

void AudioProcessor::compactAudioBufferIfNeeded() {
    if (audioBufferReadOffset == 0) {
        return;
    }

    if (audioBufferReadOffset >= audioBuffer.size()) {
        audioBuffer.clear();
        audioBufferReadOffset = 0;
        return;
    }

    if (audioBufferReadOffset >= BUFFER_SIZE * 4 && audioBufferReadOffset >= audioBuffer.size() / 2) {
        audioBuffer.erase(audioBuffer.begin(), audioBuffer.begin() + static_cast<std::ptrdiff_t>(audioBufferReadOffset));
        audioBufferReadOffset = 0;
    }
}

RadioSettings AudioProcessor::currentSettingsSnapshot() const {
    std::lock_guard<std::mutex> lock(settingsMutex);
    return audioSettings;
}

void AudioProcessor::processDemodulatorBlock(const std::vector<float>& iqBlock, std::vector<short>& audioSamples, const RadioSettings &settings) {
    audioSamples.clear();

    const double rfInputRate = settings.sampleRate;
    const double audioTimingRate = rfInputRate;

    if (iqBlock.size() < 2 || rfInputRate <= 0.0 || audioTimingRate <= 0.0) {
        return;
    }

    const int modulationType = settings.modulationType;
    const int inputMode = settings.inputMode;
    const size_t iqSamples = iqBlock.size() / 2;
    const double fShift = settings.listeningFrequency - settings.centerFrequency;
    const double phaseIncrement = -TWO_PI * fShift / rfInputRate;
    const double bandwidth = settings.bandwidth;
    const int decimationFactor = channelDecimationFactor(rfInputRate, modulationType, bandwidth);
    const double rfChannelRate = rfInputRate / decimationFactor;
    const double audioTimingChannelRate = audioTimingRate / decimationFactor;
    const double outputStep = AUDIO_OUTPUT_RATE / audioTimingChannelRate;

    double cutoff = channelCutoffForMode(modulationType, bandwidth);
    cutoff = (std::min)(cutoff, rfChannelRate * 0.45);
    const float lowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * cutoff / rfChannelRate),
        0.000001,
        1.0
        ));
    const double demodAudioCutoff = (std::min)(demodAudioCutoffForMode(modulationType, bandwidth),
                                               AUDIO_OUTPUT_RATE * 0.45);
    const float demodAudioLowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * demodAudioCutoff / AUDIO_OUTPUT_RATE),
        0.000001,
        1.0
        ));

    const bool lowerSideband = isLowerSidebandMode(modulationType);
    const double sidebandHighCut = (std::min)(cutoff, rfChannelRate * 0.45);
    const double sidebandLowCut = (std::min)(SSB_LOW_CUT_HZ, sidebandHighCut * 0.5);
    const double sidebandCenter = (sidebandLowCut + sidebandHighCut) * 0.5;
    const double sidebandHalfWidth = (std::max)(50.0, (sidebandHighCut - sidebandLowCut) * 0.5);
    const double sidebandShiftIncrement = (lowerSideband ? TWO_PI : -TWO_PI) * sidebandCenter / rfChannelRate;
    const float sidebandLowPassAlpha = static_cast<float>((std::clamp)(
        1.0 - std::exp(-TWO_PI * sidebandHalfWidth / rfChannelRate),
        0.000001,
        1.0
        ));

    const size_t reserveCount = static_cast<size_t>((iqSamples / decimationFactor + 1) * outputStep) + 8;
    audioSamples.reserve(reserveCount);

    float rotI = static_cast<float>(std::cos(ncoPhase));
    float rotQ = static_cast<float>(std::sin(ncoPhase));
    const float rotStepI = static_cast<float>(std::cos(phaseIncrement));
    const float rotStepQ = static_cast<float>(std::sin(phaseIncrement));
    float channelSumI = std::real(channelDecimationSum);
    float channelSumQ = std::imag(channelDecimationSum);
    int decimationCount = channelDecimationCount;
    float lowPassI = std::real(amLowPassState);
    float lowPassQ = std::imag(amLowPassState);
    float fmPrevI = std::real(fmPreviousSample);
    float fmPrevQ = std::imag(fmPreviousSample);
    std::array<std::complex<float>, 4> sidebandStates = sidebandLowPassStates;
    double sidebandPhase = sidebandFilterPhase;
    float audioLowPass = demodAudioLowPassState;
    double samPhase = samCarrierPhase;
    double samFrequency = samCarrierFrequency;
    const double samMaxFrequency = TWO_PI * SAM_LOCK_RANGE_HZ / rfChannelRate;
    const double fmDeemphasisSeconds = modulationType == MOD_NFM ? NFM_DEEMPHASIS_SECONDS : FM_DEEMPHASIS_SECONDS;
    const float fmDeemphasisAlpha = static_cast<float>(
        1.0 - std::exp(-1.0 / ((std::max)(1.0, rfChannelRate) * fmDeemphasisSeconds))
        );

    for (size_t n = 0; n < iqSamples; ++n) {
        float iSample = iqBlock[2 * n];
        float qSample = iqBlock[2 * n + 1];
        if (!std::isfinite(iSample)) {
            iSample = 0.0f;
        }
        if (!std::isfinite(qSample)) {
            qSample = 0.0f;
        }
        if (inputMode == 2) {
            qSample = 0.0f;
        } else if (inputMode == 3) {
            iSample = qSample;
            qSample = 0.0f;
        }
        const float mixedI = iSample * rotI - qSample * rotQ;
        const float mixedQ = iSample * rotQ + qSample * rotI;
        channelSumI += mixedI;
        channelSumQ += mixedQ;
        ++decimationCount;

        const float nextRotI = rotI * rotStepI - rotQ * rotStepQ;
        const float nextRotQ = rotI * rotStepQ + rotQ * rotStepI;
        rotI = nextRotI;
        rotQ = nextRotQ;

        if ((n & 4095) == 4095) {
            const float norm = std::sqrt(rotI * rotI + rotQ * rotQ);
            if (norm > 0.0f) {
                rotI /= norm;
                rotQ /= norm;
            }
        }

        if (decimationCount < decimationFactor) {
            continue;
        }

        const float invDecimationCount = 1.0f / static_cast<float>(decimationCount);
        const float channelI = channelSumI * invDecimationCount;
        const float channelQ = channelSumQ * invDecimationCount;
        channelSumI = 0.0f;
        channelSumQ = 0.0f;
        decimationCount = 0;

        lowPassI += lowPassAlpha * (channelI - lowPassI);
        lowPassQ += lowPassAlpha * (channelQ - lowPassQ);

        const std::complex<float> filteredChannel(lowPassI, lowPassQ);
        const float envelope = std::sqrt(lowPassI * lowPassI + lowPassQ * lowPassQ);
        float demodulatedSample = 0.0f;

        switch (modulationType) {
        case MOD_NFM:
        case MOD_WFM:
        case MOD_RTTY:
        case MOD_FSK: {
            float limitedI = lowPassI;
            float limitedQ = lowPassQ;
            const float magnitude = std::sqrt(limitedI * limitedI + limitedQ * limitedQ);
            if (magnitude > 0.000001f) {
                const float invMagnitude = 1.0f / magnitude;
                limitedI *= invMagnitude;
                limitedQ *= invMagnitude;
            }
            if (fmPreviousValid) {
                const float discriminatorQ = limitedQ * fmPrevI - limitedI * fmPrevQ;
                const float discriminatorI = limitedI * fmPrevI + limitedQ * fmPrevQ;
                demodulatedSample = std::atan2(discriminatorQ, discriminatorI);
            }
            fmPrevI = limitedI;
            fmPrevQ = limitedQ;
            fmPreviousValid = true;
            fmDeemphasisState += fmDeemphasisAlpha * (demodulatedSample - fmDeemphasisState);
            demodulatedSample = fmDeemphasisState * (modulationType == MOD_WFM ? 2.5f : 8.0f);
            break;
        }
        case MOD_USB:
        case MOD_FT8:
        case MOD_PSK:
        case MOD_LSB: {
            const float oscI = static_cast<float>(std::cos(sidebandPhase));
            const float oscQ = static_cast<float>(std::sin(sidebandPhase));
            const std::complex<float> shiftOscillator(oscI, oscQ);
            std::complex<float> shifted = filteredChannel * shiftOscillator;
            for (std::complex<float> &state : sidebandStates) {
                state += sidebandLowPassAlpha * (shifted - state);
                shifted = state;
            }
            const std::complex<float> restored = shifted * std::conj(shiftOscillator);
            demodulatedSample = std::real(restored);
            sidebandPhase = wrapRadians(sidebandPhase + sidebandShiftIncrement);
            break;
        }
        case MOD_SAM: {
            if (envelope > 0.000001f) {
                const float phaseError = std::atan2(lowPassQ, lowPassI);
                samFrequency = (std::clamp)(samFrequency + 0.00002 * phaseError,
                                            -samMaxFrequency,
                                            samMaxFrequency);
                samPhase = wrapRadians(samPhase + samFrequency + 0.02 * phaseError);
            }
            const float carrierI = static_cast<float>(std::cos(samPhase));
            const float carrierQ = static_cast<float>(std::sin(samPhase));
            demodulatedSample = lowPassI * carrierI + lowPassQ * carrierQ;
            break;
        }
        case MOD_DSB:
            demodulatedSample = lowPassI;
            break;
        case MOD_CW: {
            const float bfoI = static_cast<float>(std::cos(cwTonePhase));
            const float bfoQ = static_cast<float>(std::sin(cwTonePhase));
            demodulatedSample = lowPassI * bfoI - lowPassQ * bfoQ;
            cwTonePhase += TWO_PI * CW_TONE_HZ / rfChannelRate;
            if (cwTonePhase > M_PI_F || cwTonePhase < -M_PI_F) {
                cwTonePhase = std::remainder(cwTonePhase, TWO_PI);
            }
            break;
        }
        case MOD_AM:
        default:
            demodulatedSample = envelope;
            break;
        }

        audioLowPass += demodAudioLowPassAlpha * (demodulatedSample - audioLowPass);
        demodulatedSample = audioLowPass;

        audioResamplePhase += outputStep;
        while (audioResamplePhase >= 1.0) {
            audioResamplePhase -= 1.0;

            amDcEstimate += 0.0005f * (demodulatedSample - amDcEstimate);
            const float acSample = demodulatedSample - amDcEstimate;
            const float absSample = std::fabs(acSample);
            const float agcCoeff = absSample > amAgcLevel ? 0.01f : 0.0002f;
            amAgcLevel += agcCoeff * (absSample - amAgcLevel);
            amAgcLevel = (std::max)(amAgcLevel, 0.0001f);

            const float normalized = std::tanh(acSample * (0.35f / amAgcLevel));
            const float finiteSample = std::isfinite(normalized) ? normalized : 0.0f;
            const float clamped = (std::clamp)(finiteSample, -1.0f, 1.0f);
            audioSamples.push_back(static_cast<short>(clamped * 32767.0f));
        }
    }

    channelDecimationSum = std::complex<float>(channelSumI, channelSumQ);
    channelDecimationCount = decimationCount;
    amLowPassState = std::complex<float>(lowPassI, lowPassQ);
    sidebandLowPassStates = sidebandStates;
    fmPreviousSample = std::complex<float>(fmPrevI, fmPrevQ);
    demodAudioLowPassState = audioLowPass;
    samCarrierPhase = wrapRadians(samPhase);
    samCarrierFrequency = samFrequency;
    sidebandFilterPhase = wrapRadians(sidebandPhase);
    ncoPhase = std::remainder(ncoPhase + phaseIncrement * static_cast<double>(iqSamples), TWO_PI);
}

void AudioProcessor::SDRThread() {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "SDRThread started (Real IQ processing)";
    }

    std::vector<float> iqBlock;
    std::vector<short> audioSamples;
    int activeModulationType = currentSettingsSnapshot().modulationType;
    uint64_t audioBlockCounter = 0;

    while (running) {
        const RadioSettings settings = currentSettingsSnapshot();
        if (activeModulationType != settings.modulationType) {
            activeModulationType = settings.modulationType;
            resetDemodulatorState();
        }

        if (!IqBuffer::popBlock(iqBlock) || settings.sampleRate <= 0.0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        const auto demodStart = std::chrono::steady_clock::now();
        processDemodulatorBlock(iqBlock, audioSamples, settings);
        const auto demodEnd = std::chrono::steady_clock::now();
        const double demodMs = std::chrono::duration<double, std::milli>(demodEnd - demodStart).count();
        if (audioSamples.empty()) {
            continue;
        }

        if (fobosVerboseLoggingEnabled() &&
            (audioBlockCounter < 5 || (audioBlockCounter % 100) == 0)) {
            qDebug() << "[Audio] demod block"
                     << "blockCounter" << audioBlockCounter
                     << "iqFloats" << iqBlock.size()
                     << "configuredRate" << settings.sampleRate
                     << "estimatedRate" << IqBuffer::sampleRateEstimate()
                     << "audioSamples" << audioSamples.size()
                     << "demodMs" << demodMs;
        }
        ++audioBlockCounter;

        {
            std::lock_guard<std::mutex> lock(audioMutex);
            const size_t queuedSamples = queuedAudioSamples();
            const size_t overflow = queuedSamples + audioSamples.size() > MAX_AUDIO_BUFFER_SAMPLES
                                        ? queuedSamples + audioSamples.size() - MAX_AUDIO_BUFFER_SAMPLES
                                        : 0;
            if (overflow > 0) {
                discardAudioSamples(overflow);
            }
            audioBuffer.insert(audioBuffer.end(), audioSamples.begin(), audioSamples.end());
            compactAudioBufferIfNeeded();
        }
        cv.notify_one();
    }

    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "SDRThread finished";
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CALLBACK AudioProcessor::WaveOutCallback(HWAVEOUT hwo, UINT uMsg, DWORD_PTR dwInstance,
                                              DWORD_PTR dwParam1, DWORD_PTR dwParam2) {
    if (uMsg == WOM_DONE) {
        //qDebug() << "WaveOutCallback: WOM_DONE called";

        AudioProcessor* self = reinterpret_cast<AudioProcessor*>(dwInstance);
        if (!self || self->audioDeviceClosing.load()) {
            return;
        }
        WAVEHDR* hdr = reinterpret_cast<WAVEHDR*>(dwParam1);

        for (int i = 0; i < NUM_BUFFERS; ++i) {
            if (hdr == &self->waveHdrs[i]) {
                self->bufferReady[i] = true;
                self->cv.notify_one();
                break;
            }
        }
    }
}



void AudioProcessor::startAudioOutput() {
    while (running.load()) {
        std::unique_lock<std::mutex> lock(audioMutex);
        cv.wait(lock, [this] {
            if (!running.load()) {
                return true;
            }
            if (queuedAudioSamples() < static_cast<size_t>(BUFFER_SIZE)) {
                return false;
            }
            for (int i = 0; i < NUM_BUFFERS; ++i) {
                if (bufferReady[i]) return true;
            }
            return false;
        });

        if (!running.load()) break;
        for (int i = 0; i < NUM_BUFFERS; ++i) {
            if (!bufferReady[i]) continue;

            if (queuedAudioSamples() < static_cast<size_t>(BUFFER_SIZE)) continue;

            const size_t samplesToCopy = (std::min)(queuedAudioSamples(), static_cast<size_t>(BUFFER_SIZE));
            if (samplesToCopy > 0) {
                std::copy(audioBuffer.begin() + static_cast<std::ptrdiff_t>(audioBufferReadOffset),
                          audioBuffer.begin() + static_cast<std::ptrdiff_t>(audioBufferReadOffset + samplesToCopy),
                          waveBuffers[i].begin());
                discardAudioSamples(samplesToCopy);
            }

            if (samplesToCopy < static_cast<size_t>(BUFFER_SIZE)) {
                std::fill(waveBuffers[i].begin() + samplesToCopy, waveBuffers[i].end(), 0);
            }

            const QByteArray pcmFrame(reinterpret_cast<const char *>(waveBuffers[i].data()),
                                      static_cast<int>(waveBuffers[i].size() * sizeof(short)));
            emit audioFrameReady(pcmFrame);

            // Заполняем буфер
            bufferReady[i] = false;
            MMRESULT result = MMSYSERR_NOERROR;
            {
                std::lock_guard<std::mutex> waveLock(waveOutMutex);
                if (!hWaveOut || audioDeviceClosing.load()) {
                    bufferReady[i] = true;
                    continue;
                }
                result = waveOutWrite(hWaveOut, &waveHdrs[i], sizeof(WAVEHDR));
            }
            if (result != MMSYSERR_NOERROR) {
                qDebug() << "waveOutWrite failed!";
            }

        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void AudioProcessor::decimateIQ(
    const std::vector<std::complex<float>>& inputIQ,
    std::vector<std::complex<float>>& outputIQ,
    int decimationFactor
    ) {
    outputIQ.clear();
    size_t numSamples = inputIQ.size();

    for (size_t i = 0; i < numSamples; i += decimationFactor) {
        std::complex<float> sum(0.0f, 0.0f);
        int count = 0;

        for (size_t j = 0; j < decimationFactor && (i + j) < numSamples; ++j) {
            sum += inputIQ[i + j];
            count++;
        }

        outputIQ.push_back(sum / static_cast<float>(count));
    }
}



void AudioProcessor::downsampleAudio(
    const std::vector<float>& input,
    std::vector<float>& output,
    int downsampleFactor
    ) {
    output.clear();
    size_t numSamples = input.size();

    for (size_t i = 0; i < numSamples; i += downsampleFactor) {
        float sum = 0.0f;
        int count = 0;

        for (size_t j = 0; j < downsampleFactor && (i + j) < numSamples; ++j) {
            sum += input[i + j];
            count++;
        }

        output.push_back(sum / count);
    }
}


void AudioProcessor::demodulateAM(
    const std::vector<std::complex<float>>& decimatedIQ,
    std::vector<float>& demodulatedData
    ) {
    size_t numSamples = decimatedIQ.size();
    demodulatedData.resize(numSamples);

    for (size_t i = 0; i < numSamples; ++i) {
        demodulatedData[i] = std::abs(decimatedIQ[i]);  // sqrt(I*I + Q*Q)
    }
}


void AudioProcessor::demodulateFM(const std::vector<float>& iqData, std::vector<float>& demodulatedData, float& lastPhase) {
    size_t numSamples = iqData.size() / 2;
    demodulatedData.resize(numSamples);
    for (size_t i = 0; i < numSamples; ++i) {
        float I = iqData[2 * i];
        float Q = iqData[2 * i + 1];
        float phase = std::atan2(Q, I);
        float deltaPhase = phase - lastPhase;
        if (deltaPhase > M_PI_F) deltaPhase -= 2.0f * M_PI_F;
        if (deltaPhase < -M_PI_F) deltaPhase += 2.0f * M_PI_F;
        lastPhase = phase;
        demodulatedData[i] = deltaPhase;
    }
}

inline float fastAtan2(float y, float x) {
    float abs_y = fabsf(y);
    float r, angle;
    if (x == 0.0f && y == 0.0f) { return 0.0f; }
    if (x >= 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = M_PI_4 - M_PI_4 * r; // M_PI_4 ??? p/4
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 3 * M_PI_4 - M_PI_4 * r;
    }
    return y < 0.0f ? -angle : angle;
}


std::vector<float> AudioProcessor::demodulateSSB(const std::vector<float>& iqData, double frequency, double globalBandwidth, double sampleRate) {
    std::vector<float> demodulatedData;
    const float PI = 3.14159265358979323846f;
    float prevPhase = 0.0f;
    for (size_t i = 0; i < iqData.size(); i += 2) {
        float I = iqData[i];
        float Q = iqData[i + 1];
        float phase = atan2(Q, I);
        float phaseDiff = phase - prevPhase;
        if (phaseDiff > PI) phaseDiff -= 2 * PI;
        if (phaseDiff < -PI) phaseDiff += 2 * PI;
        demodulatedData.push_back(phaseDiff * sampleRate / (2 * PI));
        prevPhase = phase;
    }
    return demodulatedData;
}

std::vector<float> AudioProcessor::demodulateFSK(const std::vector<float>& iqData, double frequency, double globalBandwidth, double sampleRate) {
    std::vector<float> demodulatedData;
    const float PI = 3.14159265358979323846f;
    float prevPhase = 0.0f;
    float prevFreq = 0.0f;
    float bandwidth = globalBandwidth / 2.0;
    for (size_t i = 0; i < iqData.size(); i += 2) {
        float I = iqData[i];
        float Q = iqData[i + 1]; 
        float phase = atan2(Q, I);
        float phaseDiff = phase - prevPhase;
        if (phaseDiff > PI) phaseDiff -= 2 * PI;
        if (phaseDiff < -PI) phaseDiff += 2 * PI;
        float freq = phaseDiff * sampleRate / (2 * PI);
        if (std::abs(freq - prevFreq) > bandwidth) {
            float bit = (freq > prevFreq) ? 1.0f : 0.0f;
            demodulatedData.push_back(bit);
        }
        prevPhase = phase;
        prevFreq = freq;
    }
    return demodulatedData;
}
