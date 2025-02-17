#include "dataprocessor.h"

#define DEFAULT_BUFS_COUNT 32

extern int fftLength;
extern int DEFAULT_BUF_LEN;
size_t iqDataSize = fftLength;
size_t dataqSize = DEFAULT_BUF_LEN/8;
extern float* iqData;
extern float* dataq;
extern fobos_dev_t *device;
extern int globalMode;
int actualBufLength = 32768;
extern bool syncWariable;


DataProcessor::DataProcessor(fobos_dev_t* dev, QObject *parent)
    : QThread(parent), running(false) {
}
    
DataProcessor::~DataProcessor() {
    stop();
    wait();
}

void DataProcessor::run() {
    qDebug() << "Starting read operation based on globalMode...";
    running = true;
    iqData = new float[iqDataSize];
    dataq = new float[dataqSize];
    if (syncWariable == false) {
        while (running) {
            int ret = fobos_rx_read_async(device, [](float *buf, uint32_t buf_length, void *ctx) {
                auto *processor = static_cast<DataProcessor*>(ctx);
				processor->handleData(buf, buf_length);
              },
              this,
              DEFAULT_BUFS_COUNT,
              DEFAULT_BUF_LEN);
            if (ret != FOBOS_ERR_OK) {
                qDebug() << "Failed to start async read, error code:" << ret;
                running = false;  // Ensure to stop the loop
                break;
            } else {
                qDebug() << "Async read started successfully.";
                exec();
            }
        }

    } else if (syncWariable == true) {
            int ret = fobos_rx_start_sync(device, DEFAULT_BUF_LEN);
            if (ret != FOBOS_ERR_OK) {
                qDebug() << "Failed to start sync mode, error code:" << ret;
                running = false;  // Ensure to stop the loop
                
            }		
        while (running) {
            uint32_t actual_buf_length;
            ret = fobos_rx_read_sync(device, iqData, &actual_buf_length);
            //qDebug() << "Buffer length is " << actualBufLength;
            if (ret != FOBOS_ERR_OK) {
                qDebug() << "Failed to read sync data, error code:" << ret;
                    running = false;  // Ensure to stop the loop
                    break;
            }

            actualBufLength = actual_buf_length;
            //emit dataReady();
        }
    }  
}

void DataProcessor::handleData(float *buf, uint32_t buf_length) {
        if (buf_length < dataqSize) {
        qDebug() << "Buffer length mismatch in handleData!";
        return;
    }
    iqData = buf;

    //emit dataReady();
}
 

void DataProcessor::stop() {
    if (running) {
        running = false;
        if (syncWariable == false) {
            fobos_rx_cancel_async(device);
        } else if (syncWariable == true) {
            fobos_rx_stop_sync(device);
        }
        qDebug() << "Attempting to stop DataProcessor.";
        QThread::quit();
        if (!QThread::wait(5000)) { // Timeout for waiting
            qDebug() << "Error: DataProcessor thread did not quit within timeout.";
        } else {
            qDebug() << "DataProcessor thread successfully quit.";
        }
        qDebug() << "DataProcessor stopped";
    }
}

