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
double audioSamplerate = 48000;
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
std::vector<float> resampledData(96, 0);
std::vector<short> audioData;
std::vector<short> audioBuffer;  // Общий буфер для аудиоданных
std::vector<short> localBuffer;

std::mutex audioMutex;           // Защита от одновременного доступа
std::mutex audio1Mutex;

AudioProcessor::AudioProcessor(QObject *parent) : QObject(parent), running(false), workerThread(nullptr), hWaveOut(nullptr) {
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
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
    waveFormat.nChannels = 1;
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

void AudioProcessor::SDRThread() {
    qDebug() << "SDRThread started";
    auto nextTimePoint = std::chrono::steady_clock::now();

    while (running) {
       // auto start = std::chrono::high_resolution_clock::now(); // Старт таймера
        std::vector<float> tempFilteredData;
        std::vector<float> tempDemodulatedData;
        std::vector<short> tempAudioData(96);

        // 1. Извлекаем данные из IQ-буфера
        filterIQData(iqData, globalFrequency, globalSampleRate, listeningFrequency, globalBandwidth, tempFilteredData);
        if (tempFilteredData.empty()) {
            qDebug() << "[SDRThread] Warning: tempFilteredData is empty!";
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // 2. Демодуляция в зависимости от типа модуляции
        switch (globalModulationType) {
        case 0: demodulateAM(tempFilteredData, tempDemodulatedData); break;
        case 1: tempDemodulatedData = demodulateFM(tempFilteredData, lastPhase); break;
        case 2: tempDemodulatedData = demodulateSSB(tempFilteredData, listeningFrequency, globalBandwidth, audioSamplerate); break;
        case 9: tempDemodulatedData = demodulateFSK(tempFilteredData, listeningFrequency, globalBandwidth, audioSamplerate); break;
        default: qWarning() << "Unknown modulation type."; continue;
        }
        // 3. Нормализация данных
        normalizeAudio(tempDemodulatedData);
        // 4. Приведение к 480 сэмплам
        resampleAudio(tempDemodulatedData, tempAudioData);
        // 5. Запись в аудиобуфер
        {
            std::lock_guard<std::mutex> lock(audioMutex);
            if (audioBuffer.size() >= 48000) {
                audioBuffer.erase(audioBuffer.begin(), audioBuffer.begin() + 48000);
            }
            audioBuffer.insert(audioBuffer.end(), tempAudioData.begin(), tempAudioData.end());
        }
        cv.notify_one();
        //auto end = std::chrono::high_resolution_clock::now(); // Конец таймера
        //std::cout << "[filterIQData] Выполнено за: "
                 // << std::chrono::duration<double, std::micro>(end - start).count()
                  //<< " мкс" << std::endl; // Выводим в микросекундах (мкс)
        nextTimePoint += std::chrono::milliseconds(2);
        std::this_thread::sleep_until(nextTimePoint);
    }
}


void AudioProcessor::AudioThread() {
    qDebug() << "AudioThread started";
    auto nextTimePoint = std::chrono::steady_clock::now();
    while (running) {
        std::vector<short> tempBuffer;
        {
            std::unique_lock<std::mutex> lock(audioMutex);
            cv.wait(lock, [this] { return audioBuffer.size() >= 4800 || !running; });
            if (!running) break;

            tempBuffer.assign(audioBuffer.begin(), audioBuffer.begin() + 4800);
            audioBuffer.erase(audioBuffer.begin(), audioBuffer.begin() + 4800);
        }
        playAudio(tempBuffer);
        nextTimePoint += std::chrono::milliseconds(100);
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

    std::vector<float> resampledData(96);
    double step = static_cast<double>(input.size()) / 96.0;

    for (size_t i = 0; i < 96; i++) {
        double srcIndex = i * step;
        size_t index1 = static_cast<size_t>(srcIndex);
        size_t index2 = (index1 + 1 < input.size()) ? (index1 + 1) : (input.size() - 1);
        float sample = input[index1] + (input[index2] - input[index1]) * (srcIndex - index1);
        resampledData[i] = sample;
    }

    output.resize(96);
    for (size_t i = 0; i < 96; i++) {
        output[i] = static_cast<short>(resampledData[i] * 32767);
    }
}


void AudioProcessor::playAudio(const std::vector<short>& audioData) {
    if (!hWaveOut) return;
    while (!waveHeaders.empty()) {
        WAVEHDR& hdr = waveHeaders.front();
        if ((hdr.dwFlags & WHDR_DONE) == 0) break;  // Если еще проигрывается, не трогаем
        waveOutUnprepareHeader(hWaveOut, &hdr, sizeof(WAVEHDR));
        waveHeaders.erase(waveHeaders.begin());
    }
    WAVEHDR waveHeader = {};
    waveHeaders.push_back(waveHeader);
    WAVEHDR &hdr = waveHeaders.back();
    hdr.lpData = (LPSTR)audioData.data();
    hdr.dwBufferLength = static_cast<DWORD>(audioData.size() * sizeof(short));
    hdr.dwFlags = 0;
    while (waveOutUnprepareHeader(hWaveOut, &hdr, sizeof(WAVEHDR)) == WAVERR_STILLPLAYING) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (waveOutPrepareHeader(hWaveOut, &hdr, sizeof(WAVEHDR)) == MMSYSERR_NOERROR) {
        waveOutWrite(hWaveOut, &hdr, sizeof(WAVEHDR));
    }
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


void AudioProcessor::demodulateAM(const std::vector<float>& filteredData, std::vector<float>& demodulatedData) {
    size_t numSamples = filteredData.size() / 2; // Количество пар I/Q
    demodulatedData.resize(numSamples);         // Устанавливаем правильный размер

    for (size_t i = 0; i < numSamples; ++i) {
        float I = filteredData[2 * i];     // Реальная часть
        float Q = filteredData[2 * i + 1]; // Мнимая часть
        demodulatedData[i] = std::sqrt(I * I + Q * Q);
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
