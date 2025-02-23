#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_4
#define M_PI_4 0.78539816339744830962
#endif

#include "audioprocessor.h"

#pragma comment(lib, "winmm.lib")

#define DEFAULT_BUFS_COUNT 32
extern int DEFAULT_BUF_LEN;
extern double globalFrequency;
extern float* iqData;
extern double listeningFrequency;
double audioSamplerate = 44100;
extern double globalBandwidth;
extern double globalSampleRate;
extern int globalModulationType;
extern int globalMode;
extern int fftLength;
float lastPhase = 0.0f;
const int bufferSize = 4800;
extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;

std::vector<WAVEHDR> waveHeaders;
std::condition_variable cv;
std::mutex cvMutex;

std::vector<float> filteredData;
std::vector<float> demodulatedData;
std::vector<float> lowPassFilteredData;
std::vector<float> resampledData(480, 0);
std::vector<short> audioData;
std::vector<short> audioBuffer;  // Общий буфер для аудиоданных
std::vector<short> localBuffer;

std::mutex audioMutex;           // Защита от одновременного доступа
std::mutex audio1Mutex;


AudioProcessor::AudioProcessor(QObject *parent) : QObject(parent), running(false), workerThread(nullptr), hWaveOut(nullptr) {
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 2;
    format.nSamplesPerSec = 48000;
    format.wBitsPerSample = 16;
    format.nBlockAlign = (format.nChannels * format.wBitsPerSample) / 8;
    format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;
    if (waveOutOpen(&hWaveOut, WAVE_MAPPER, &format, 0, 0, CALLBACK_NULL) != MMSYSERR_NOERROR) {
        std::cerr << "Failed to open audio output!" << std::endl;
        hWaveOut = nullptr;
    }
     //udpSocket = new QUdpSocket(this);
    //update1Timer = new QTimer(this);
    //update1Timer->setInterval(500);
    //connect(update1Timer, &QTimer::timeout, this, &AudioProcessor::processAudioData);
}

AudioProcessor::~AudioProcessor() {
        stopDemodulation();
    if (hWaveOut) {
        waveOutClose(hWaveOut);
    }
}

void AudioProcessor::setAudioDevice(int deviceID) {
    qDebug() << "start setAudioDevice";
    if (hWaveOut) {
        waveOutReset(hWaveOut);
        waveOutClose(hWaveOut);
    }

    WAVEFORMATEX waveFormat;
    waveFormat.wFormatTag = WAVE_FORMAT_PCM;
    waveFormat.nChannels = 2;
    waveFormat.nSamplesPerSec = 48000;
    waveFormat.wBitsPerSample = 16;
    waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
    waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
    waveFormat.cbSize = 0;
    MMRESULT result = waveOutOpen(&hWaveOut, deviceID, &waveFormat, 0, 0, CALLBACK_NULL);
    if (result != MMSYSERR_NOERROR) {
        qDebug() << "Error opening audio device!";
    } else {
        qDebug() << "Audio device switched successfully!";
    }
}

void AudioProcessor::startDemodulation() {
    running = true;
    std::thread sdrThread(&AudioProcessor::SDRThread, this);
    std::thread audioThread(&AudioProcessor::AudioThread, this);
    sdrThread.detach();
    audioThread.detach();
}

void AudioProcessor::stopDemodulation() {
    running = false;
}

void AudioProcessor::demodulateAM(const std::vector<float>& testIQ_AM, std::vector<float>& demodulatedData) {
    size_t numSamples = testIQ_AM.size() / 2; // Количество пар I/Q
    demodulatedData.resize(numSamples);

    for (size_t i = 0; i < numSamples; ++i) {
        float I = testIQ_AM[2 * i];
        float Q = testIQ_AM[2 * i + 1];
        demodulatedData[i] = std::sqrt(I * I + Q * Q);
    }

}


bool AudioProcessor::loadWAV(const std::string& filename, int& sampleRate, int& numChannels) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Ошибка: не удалось открыть файл " << filename << std::endl;
        return false;
    }

    char riffHeader[4];
    file.read(riffHeader, 4);
    if (std::string(riffHeader, 4) != "RIFF") {
        std::cerr << "Ошибка: Файл не является WAV" << std::endl;
        return false;
    }

    file.seekg(22, std::ios::beg);
    file.read(reinterpret_cast<char*>(&numChannels), sizeof(numChannels));

    file.seekg(24, std::ios::beg);
    file.read(reinterpret_cast<char*>(&sampleRate), sizeof(sampleRate));

    file.seekg(34, std::ios::beg);
    uint16_t bitsPerSample;
    file.read(reinterpret_cast<char*>(&bitsPerSample), sizeof(bitsPerSample));
    if (bitsPerSample != 16) {
        std::cerr << "Ошибка: Только 16-битные WAV поддерживаются" << std::endl;
        return false;
    }

    file.seekg(0, std::ios::beg);
    char chunkId[4];
    uint32_t chunkSize;
    while (file.read(chunkId, 4)) {
        file.read(reinterpret_cast<char*>(&chunkSize), sizeof(chunkSize));
        if (std::string(chunkId, 4) == "data") {
            break;
        }
        file.seekg(chunkSize, std::ios::cur);
    }

    return true;  // WAV-файл загружен успешно
}


void AudioProcessor::SDRThread() {
    qDebug() << "SDRThread started";
    auto nextTimePoint = std::chrono::steady_clock::now();

    while (running) {
        std::vector<float> tempFilteredData;
        std::vector<float> tempDemodulatedData;
        std::vector<short> tempAudioData(480);

        // **1. Фильтруем IQ-данные**
        filterIQData(iqData, globalFrequency, globalSampleRate, listeningFrequency, globalBandwidth, tempFilteredData);
        if (tempFilteredData.empty()) {
            qDebug() << "[SDRThread] Warning: tempFilteredData is empty!";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // **2. Выбираем метод демодуляции**
        switch (globalModulationType) {
        case 0:  // AM
            demodulateAM(tempFilteredData, tempDemodulatedData);
            break;
        case 1:  // FM
            tempDemodulatedData = demodulateFM(tempFilteredData, lastPhase);
            break;
        case 2:  // SSB
            tempDemodulatedData = demodulateSSB(tempFilteredData, listeningFrequency, globalBandwidth, audioSamplerate);
            break;
        case 9:  // FSK
            tempDemodulatedData = demodulateFSK(tempFilteredData, listeningFrequency, globalBandwidth, audioSamplerate);
            break;
        default:
            qWarning() << "Unknown modulation type.";
            continue;
        }

        // **3. Применяем нормализацию и низкочастотный фильтр**
        normalizeAudio(tempDemodulatedData);
        applyLowPassFilter(tempDemodulatedData, lowPassFilteredData, 3000, audioSamplerate);  // Фильтр 3 кГц

        // **4. Пересэмплируем под аудиовывод**
        resampleAudio(lowPassFilteredData, tempAudioData);

        // **5. Записываем в общий буфер**
        {
            std::lock_guard<std::mutex> lock(audioMutex);
            if (audioBuffer.size() >= 4800) {
                audioBuffer.erase(audioBuffer.begin(), audioBuffer.begin() + 4800);
            }
            audioBuffer.insert(audioBuffer.end(), tempAudioData.begin(), tempAudioData.end());
        }
        cv.notify_one();

        nextTimePoint += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(nextTimePoint);
    }
}






void AudioProcessor::AudioThread() {
    qDebug() << "AudioThread started";
    auto nextTimePoint = std::chrono::steady_clock::now();

    while (running) {
        std::vector<short> leftBuffer(480);
        std::vector<short> rightBuffer(480);

        {
            std::unique_lock<std::mutex> lock(audioMutex);
            cv.wait(lock, [this] { return audioBuffer.size() >= 960 || !running; });
            if (!running) break;

            std::copy(audioBuffer.begin(), audioBuffer.begin() + 480, leftBuffer.begin());
            std::copy(audioBuffer.begin(), audioBuffer.begin() + 480, rightBuffer.begin());
            audioBuffer.erase(audioBuffer.begin(), audioBuffer.begin() + 480);
        }

        playAudio(leftBuffer, rightBuffer);

        nextTimePoint += std::chrono::milliseconds(10);
        std::this_thread::sleep_until(nextTimePoint);
    }
}


void AudioProcessor::normalizeAudio(std::vector<float>& audioData) {
    if (audioData.empty()) return;

    float maxVal = *std::max_element(audioData.begin(), audioData.end());
    float minVal = *std::min_element(audioData.begin(), audioData.end());

    float range = maxVal - minVal;
    if (range == 0) return;

    float scaleFactor = 2.0f / range;

    for (float& sample : audioData) {
        sample = (sample - minVal) * scaleFactor - 1.0f;
        sample = std::clamp(sample, -1.0f, 1.0f);
    }
}

void AudioProcessor::resampleAudio(const std::vector<float>& input, std::vector<short>& output) {
    if (input.empty()) return;

    std::vector<float> resampledData(480);
    double step = static_cast<double>(input.size()) / 480.0;

    for (size_t i = 0; i < 480; i++) {
        double srcIndex = i * step;
        size_t index1 = static_cast<size_t>(srcIndex);
        size_t index2 = (index1 + 1 < input.size()) ? (index1 + 1) : (input.size() - 1);
        float sample = input[index1] + (input[index2] - input[index1]) * (srcIndex - index1);
        resampledData[i] = sample;
    }

    output.resize(480);
    for (size_t i = 0; i < 480; i++) {
        output[i] = static_cast<short>(resampledData[i] * 32767);
    }
}


void AudioProcessor::playAudio(const std::vector<short>& leftChannel, const std::vector<short>& rightChannel) {
    if (!hWaveOut) return;
    if (leftChannel.size() != rightChannel.size()) return;

    std::vector<short> stereoBuffer(leftChannel.size() * 2);
    for (size_t i = 0; i < leftChannel.size(); i++) {
        stereoBuffer[2 * i] = leftChannel[i];
        stereoBuffer[2 * i + 1] = rightChannel[i];
    }

    WAVEHDR waveHeader = {};
    waveHeader.lpData = reinterpret_cast<LPSTR>(stereoBuffer.data());
    waveHeader.dwBufferLength = static_cast<DWORD>(stereoBuffer.size() * sizeof(short));
    waveHeader.dwFlags = 0;

    MMRESULT result = waveOutPrepareHeader(hWaveOut, &waveHeader, sizeof(WAVEHDR));
    if (result != MMSYSERR_NOERROR) {
        std::cerr << "Ошибка подготовки аудиобуфера" << std::endl;
        return;
    }

    result = waveOutWrite(hWaveOut, &waveHeader, sizeof(WAVEHDR));
    if (result != MMSYSERR_NOERROR) {
        std::cerr << "Ошибка воспроизведения аудио" << std::endl;
        return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}




void AudioProcessor::filterIQData(float* iqData, double centerFrequency, double globalSampleRate,
                                  double listeningFrequency, double globalBandwidth,
                                  std::vector<float>& filteredData) {
    filteredData.clear();
    double minFreq = listeningFrequency - globalBandwidth / 2.0;
    double maxFreq = listeningFrequency + globalBandwidth / 2.0;
    double freqStep = globalSampleRate / fftLength;
    double freqStep2 = globalSampleRate / fftLength/2;
    for (size_t i = 0; i < static_cast<size_t>(fftLength); i += 2) {
        double freq;
        if (globalMode == 0 || globalMode == 1) {
            freq = (centerFrequency - globalSampleRate / 2.0) + (i * freqStep);
        } else {
            freq = centerFrequency + (i * freqStep2);
        }
        if (freq >= minFreq && freq <= maxFreq) {
            filteredData.push_back(iqData[i]);      // I-компонента
            filteredData.push_back(iqData[i + 1]);  // Q-компонента
        }
    }
    if (filteredData.empty()) {
        qDebug() << "[filterIQData] Warning: No data selected! Check min/max frequency and center frequency.";
    }
}


void AudioProcessor::applyLowPassFilter(const std::vector<float>& demodulatedData, std::vector<float>& lowPassFilteredData, float cutoffFreq, float sampleRate) {
    const int filterOrder = 100;
    std::vector<float> filterCoeffs(filterOrder);
    float normCutoff = cutoffFreq / sampleRate;
    for (int i = 0; i < filterOrder; ++i) {
        if (i == filterOrder / 2) {
            filterCoeffs[i] = 2 * normCutoff;
        } else {
            float sinc = std::sin(2 * M_PI * normCutoff * (i - filterOrder / 2)) / (M_PI * (i - filterOrder / 2));
            filterCoeffs[i] = sinc * (0.54 - 0.46 * std::cos(2 * M_PI * i / (filterOrder - 1)));
        }
    }
    lowPassFilteredData.resize(demodulatedData.size());
    for (size_t i = 0; i < demodulatedData.size(); ++i) {
        lowPassFilteredData[i] = 0;
        for (int j = 0; j < filterOrder; ++j) {
            if (i >= j) {
                lowPassFilteredData[i] += demodulatedData[i - j] * filterCoeffs[j];
            }
        }
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

std::vector<float> AudioProcessor::demodulateFM(const std::vector<float>& localBuffer, float& lastPhase) {
    std::vector<float> demodulatedData(localBuffer.size() / 2);
    const float PI_F = static_cast<float>(M_PI);
    const float TWO_PI_F = 2.0f * PI_F;
    const float scalingFactor = 1.0f / 32768.0f;  // Нормализация входных данных
    float maxPhaseDiff = 0.0f;  // Для нормализации громкости
    for (size_t i = 0; i < demodulatedData.size(); ++i) {
        float I = localBuffer[2 * i] * scalingFactor;
        float Q = localBuffer[2 * i + 1] * scalingFactor;
        float phase = fastAtan2(Q, I);
        float deltaPhase = phase - lastPhase;
        if (deltaPhase > PI_F) {
            deltaPhase -= TWO_PI_F;
        } else if (deltaPhase < -PI_F) {
            deltaPhase += TWO_PI_F;
        }
        lastPhase = phase;
        demodulatedData[i] = deltaPhase;
        if (std::abs(deltaPhase) > maxPhaseDiff) {
            maxPhaseDiff = std::abs(deltaPhase);
        }

    }
    if (maxPhaseDiff > 0.1f) {
        for (float& sample : demodulatedData) {
            sample /= maxPhaseDiff;
        }
    }
    applyDeemphasisFilter(demodulatedData, audioSamplerate);
    return demodulatedData;
}

 


void AudioProcessor::applyDeemphasisFilter(std::vector<float>& lowPassFilteredData, float sampleRate) {
    const float deemphasisTau = 50e-6f;
    float deemDt = 1.0f / sampleRate;
    float alpha = deemDt / (deemphasisTau + deemDt);
    float prevOutput = 0.0f;
    for (size_t i = 0; i < lowPassFilteredData.size(); ++i) {
        float input = lowPassFilteredData[i];
        float output = alpha * input + (1 - alpha) * prevOutput;
        lowPassFilteredData[i] = output;
        prevOutput = output;
    }
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
