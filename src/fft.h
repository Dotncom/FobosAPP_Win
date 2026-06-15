#ifndef FFT_H
#define FFT_H

#include <fftw3.h>
#include <vector>
//#include <main.h>
#include <cmath>
#include <QVector>
#include <thread>
#include <mutex>
#include <complex>
#include <QObject>
#include <QThread>
#include <QWaitCondition>
#include <algorithm>
#include <QtConcurrent/QtConcurrent>
#include "iqbuffer.h"
#include "radiosettings.h"

extern int DEFAULT_BUF_LEN;

extern float* iqData;
extern int globalMode;
extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;
extern int fftLength;
extern double currentScale;
extern double minFrequency;
extern double maxFrequency;
extern double globalFrequency;
extern double globalSampleRate;

class FFTResult : public QObject {
    Q_OBJECT
public:
    explicit FFTResult(QObject *parent = nullptr);
    ~FFTResult();
    bool storeFFTResults(const RadioSettings &settings,
                         std::vector<float> &outFrequencies,
                         std::vector<float> &outMagnitudes,
                         std::vector<float> *outReferenceMagnitudes = nullptr,
                         IqBuffer::BlockMetadata *outMetadata = nullptr);
    void storeFFTResults();
    void resetHfNoiseCancelState();
    std::mutex fftMutex;
	void performFFTInThread();
private:
    bool ensurePlan(int length);
    void releasePlan();
    fftwf_complex *fftIn;
    fftwf_complex *fftOut;
    fftwf_plan plan;
    int planLength;
    std::vector<std::complex<float>> hfNoiseCancelBins;
    std::vector<std::complex<float>> hfNoiseCancelCrossPower;
    std::vector<float> hfNoiseCancelMainPower;
    std::vector<float> hfNoiseCancelRefPower;
};


#endif // FFT_H
