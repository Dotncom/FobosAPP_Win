#ifndef SOAPYSDRBACKEND_H
#define SOAPYSDRBACKEND_H

#include <QString>
#include <QVector>

#include <cstdint>

struct SoapySdrDeviceInfo {
    int nativeIndex = 0;
    QString label;
    QString driver;
    QString hardware;
    QString serial;
};

bool soapySdrLibraryAvailable(QString *loadedPath = nullptr, QString *errorMessage = nullptr);
QString soapySdrLastErrorMessage();
QVector<SoapySdrDeviceInfo> enumerateSoapySdrDevices();

int openSoapySdrDeviceSafely(void **dev, int nativeIndex);
int closeSoapySdrDeviceSafely(void *dev);
int setSoapySdrSampleRateSafely(void *dev, double sampleRateHz);
int setSoapySdrCenterFrequencySafely(void *dev, double frequencyHz);
int setSoapySdrBandwidthSafely(void *dev, double bandwidthHz);
void *setupSoapySdrRxStreamSafely(void *dev);
int activateSoapySdrStreamSafely(void *dev, void *stream);
int readSoapySdrStreamSafely(void *dev,
                             void *stream,
                             float *buffer,
                             uint32_t sampleCount,
                             long timeoutUs);
int deactivateSoapySdrStreamSafely(void *dev, void *stream);
int closeSoapySdrStreamSafely(void *dev, void *stream);

#endif // SOAPYSDRBACKEND_H
