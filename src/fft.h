#ifndef FFT_H
#define FFT_H

#include <fftw3.h>
#include <vector>
//#include <main.h>
#include <cmath>
#include <QVector>
#include <thread>
#include <mutex>
#include <QObject>
#include <QThread>
#include <QWaitCondition>
#include <algorithm>
#include <QtConcurrent/QtConcurrent>

extern int DEFAULT_BUF_LEN;

extern float* iqData;
extern int globalMode;
extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;
extern int fftLength;
extern int currentScale;
extern double minFrequency;
extern double maxFrequency;
extern double globalFrequency;
extern double globalSampleRate;

class FFTResult : public QObject {
    Q_OBJECT
public:
    explicit FFTResult(QObject *parent = nullptr);
    ~FFTResult();
    void storeFFTResults();
    std::mutex fftMutex;
	void performFFTInThread();
private:
    fftwf_complex *fftIn;
    fftwf_complex *fftOut;
    fftwf_plan plan;
};


#endif // FFT_H
