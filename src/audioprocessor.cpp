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
double audioSamplerate = 8096.0;
extern double globalBandwidth;
extern double globalSampleRate;
extern int globalModulationType;
extern int globalMode;
extern int fftLength;
float lastPhase = 0.0f;
const int bufferSize = 9600;

std::vector<float> filteredData;
std::vector<float> demodulatedData;
std::vector<float> lowPassFilteredData;
std::vector<float> resampledData;
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
    while (running) {
        //std::vector<short> audioData;       
        //processAudioData(audioData);  // Получаем новую порцию звука
        //qDebug() << "SDRThread got data";
        //if (!audioData.empty()) {
        //std::lock_guard<std::mutex> lock(audioMutex);
        filterIQData(iqData, globalFrequency, globalSampleRate, listeningFrequency, globalBandwidth, filteredData);
            if (audioBuffer.size() < bufferSize * filteredData.size() + 1) {
            audioBuffer.insert(audioBuffer.end(), filteredData.begin(), filteredData.end());  // Добавляем в общий буфер
            //qDebug() << "Размер audioBuffer:" << audioBuffer.size();
            }
        //if (audioData.empty()) {
        //qDebug() << "data empty";
        //}
        //qDebug() << "SDRThread sent data";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Небольшая задержка
        //qDebug() << "SDRThread stop sleep";
        //}
    }
}

void AudioProcessor::AudioThread() {
    qDebug() << "AudioThread started";
    auto nextTimePoint = std::chrono::steady_clock::now(); // Начальное время

    while (running) {
        {
            //qDebug() << "Размер audioBuffer:" << audioBuffer.size();
            //std::lock_guard<std::mutex> lock(audio1Mutex);
            if (audioBuffer.size() >= bufferSize * filteredData.size()) {  // Если накопили достаточно данных
                localBuffer.assign(audioBuffer.begin(), audioBuffer.begin() + bufferSize * filteredData.size());
                audioBuffer.erase(audioBuffer.begin(), audioBuffer.end());
                qDebug() << "Размер localBuffer:" << localBuffer.size();
                processAudioData();
                //qDebug() << "buffer done";
            }
            //if (audioBuffer.size() < bufferSize) {
            //qDebug() << "buffer not done";
             //}
        }

        //if (!localBuffer.empty()) {


           // playAudio(localBuffer);  // Отправляем в воспроизведение
        //qDebug() << "played";
        //}

        // Ждём точные 500 мс перед следующим циклом
        nextTimePoint += std::chrono::milliseconds(200);
        std::this_thread::sleep_until(nextTimePoint);
    }
}

 
//void AudioProcessor::processAudioData(std::vector<short>& output) {
void AudioProcessor::processAudioData() {

    //QMutexLocker locker(&mutex);
    //qDebug() << "processAudioData ok";

    //qDebug() << "filterIQData ok";
        switch (globalModulationType) {
            case 0: // AM
                //lowPassFilteredData = demodulateAM(localBuffer);
                resampledData = demodulateAM(localBuffer);
                //qDebug() << "AM ok";
                //applyLowPassFilter(demodulatedData, lowPassFilteredData, 12000,  audioSamplerate);
                //resampleToAudioRate(lowPassFilteredData, resampledData, globalBandwidth, audioSamplerate);
                //qDebug() << "Размер audiodata:" << lowPassFilteredData.size();
                //qDebug() << "Размер audiodata:" << resampledData.size();
                //qDebug() << "Размер globalBandwidth:" << globalBandwidth;
                //qDebug() << "Размер audioSamplerate:" << audioSamplerate;
                //qDebug() << "Размер globalSamplerate:" << globalSampleRate;
                break;
            case 1: // FM
                lowPassFilteredData = std::move(demodulateFM(filteredData, lastPhase));
                //applyLowPassFilter(demodulatedData, lowPassFilteredData, 12000, globalSampleRate);
                //applyDeemphasisFilter(lowPassFilteredData, audioSamplerate);
                resampleToAudioRate(lowPassFilteredData, resampledData, globalBandwidth, audioSamplerate);
                //qDebug() << "FM ok";
                break;
            case 2: // SSB
                demodulatedData = demodulateSSB(filteredData, listeningFrequency, globalBandwidth, audioSamplerate);
                applyLowPassFilter(demodulatedData, lowPassFilteredData, 12000, globalSampleRate);
                resampleToAudioRate(lowPassFilteredData, resampledData, globalSampleRate, audioSamplerate);
                qDebug() << "SSB ok";
                break;
            case 9: // FSK
                demodulatedData = demodulateFSK(filteredData, listeningFrequency, globalBandwidth, audioSamplerate);
                applyLowPassFilter(demodulatedData, lowPassFilteredData, 12000, globalSampleRate);
                resampleToAudioRate(lowPassFilteredData, resampledData, globalSampleRate, audioSamplerate);
                qDebug() << "FSK ok";
                break;
            default:
                qWarning() << "Unknown modulation type.";
                break;
        }
            audioData.resize(resampledData.size());
            for (size_t i = 0; i < resampledData.size(); i++) {
                audioData[i] = static_cast<short>(std::clamp(resampledData[i], -1.0f, 1.0f) * 32767);
            }
            playAudio(audioData);
            //output = audioData;

}

void AudioProcessor::playAudio(const std::vector<short>& audioData) {
    if (!hWaveOut) return;
    WAVEHDR waveHeader = {};
    waveHeader.lpData = (LPSTR)audioData.data();
    waveHeader.dwBufferLength = static_cast<DWORD>(audioData.size()) * sizeof(short);
    waveHeader.dwFlags = 0;
    if (waveOutPrepareHeader(hWaveOut, &waveHeader, sizeof(WAVEHDR)) == MMSYSERR_NOERROR) {
        waveOutWrite(hWaveOut, &waveHeader, sizeof(WAVEHDR));
        while ((waveHeader.dwFlags & WHDR_DONE) == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        waveOutUnprepareHeader(hWaveOut, &waveHeader, sizeof(WAVEHDR));
    }
}

void AudioProcessor::filterIQData(float* iqData, double centerFrequency, double globalSampleRate, double listeningFrequency, double globalBandwidth, std::vector<float>& filteredData) {
    filteredData.clear();
    double minFreq = listeningFrequency - globalBandwidth / 2;
    double maxFreq = listeningFrequency + globalBandwidth / 2;
    double freqStep = globalSampleRate / fftLength;
    for (size_t i = 0; i < static_cast<size_t>(fftLength); i += 1)
     {
		if (globalMode == 0 || globalMode == 1){
        double freq = (centerFrequency - globalSampleRate / 2) + (i / 2) * freqStep;
        if (freq >= minFreq && freq <= maxFreq) {
            filteredData.push_back(iqData[i]);      // I
            filteredData.push_back(iqData[i + 1]);  // Q
        }
	} else {
		double freq = centerFrequency + (i / 2) * freqStep;
        if (freq >= minFreq && freq <= maxFreq) {
            filteredData.push_back(iqData[i]);      // I
            filteredData.push_back(iqData[i + 1]);  // Q
        }
	}
    }
}

std::vector<float> AudioProcessor::demodulateAM(const std::vector<short>& filteredData) {
    std::vector<float> demodulatedData(filteredData.size() / 2);
    for (size_t i = 0; i < demodulatedData.size(); ++i) {
        float I = filteredData[2 * i];
        float Q = filteredData[2 * i + 1];
        demodulatedData[i] = std::sqrt(I * I + Q * Q);
    }
    return demodulatedData;
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

void AudioProcessor::resampleToAudioRate(std::vector<float>& lowPassFilteredData, std::vector<float>& resampledData, double sourceRate, double targetRate) {
    if (lowPassFilteredData.empty() || sourceRate <= 0 || targetRate <= 0) {
        std::cerr << "Invalid parameters in resampleToAudioRate" << std::endl;
        return;
    }
    double ratio = sourceRate / targetRate;
    size_t newSize = static_cast<size_t>(lowPassFilteredData.size() * ratio);
    resampledData.resize(newSize);
    for (size_t i = 0; i < newSize; ++i) {
        double srcIndex = i / ratio;
        size_t index1 = static_cast<size_t>(srcIndex);
        size_t index2 = index1 + 1;
        if (index2 >= lowPassFilteredData.size()) {
            resampledData[i] = lowPassFilteredData[index1];
        } else {
            double frac = srcIndex - index1;
            resampledData[i] = (1 - frac) * lowPassFilteredData[index1] + frac * lowPassFilteredData[index2];
        }
    }
    //lowPassFilteredData = std::move(resampledData);
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

std::vector<float> AudioProcessor::demodulateFM(const std::vector<float>& filteredData, float& lastPhase) {
    std::vector<float> demodulatedData(filteredData.size() / 2);
    for (size_t i = 0; i < demodulatedData.size(); ++i) {
        float I = filteredData[2 * i];
        float Q = filteredData[2 * i + 1];
        float phase = fastAtan2(Q, I);
        float deltaPhase = phase - lastPhase;
        const float PI_F = static_cast<float>(M_PI);

        if (deltaPhase > PI_F) {
            deltaPhase -= 2 * PI_F;
        } else if (deltaPhase < -PI_F) {
            deltaPhase += 2 * PI_F;
        }

        lastPhase = phase;
        demodulatedData[i] = deltaPhase;
    }
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
