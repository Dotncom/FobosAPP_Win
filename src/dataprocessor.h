#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#include <fobos.h>
#include <QObject>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <cmath>
#include <QDebug>
#include <cstring>
#include <QVector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <QElapsedTimer>
//#include <main.h>

extern float* dataq;
extern float* iqData;
extern fobos_dev_t *device;
extern int globalMode;
extern int actualBufLength;
extern int DEFAULT_BUF_LEN;


class DataProcessor : public QThread {
    Q_OBJECT
public:
    explicit DataProcessor(fobos_dev_t* dev, QObject *parent = nullptr);
    ~DataProcessor();
    void handleData(float *buf, uint32_t buf_length);
    void run() override;
    void stop();
//signals:
    //void dataReady(); 

private:
    bool running;
    bool isReading; 

};

#endif // DATAPROCESSOR_H
