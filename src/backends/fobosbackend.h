#ifndef FOBOSBACKEND_H
#define FOBOSBACKEND_H

#include "receiverbackend.h"

#include <fobos.h>
#include <fobos_sdr.h>

#include <QString>

#include <cstdint>

extern fobos_dev_t *device;
extern fobos_sdr_dev_t *agileDevice;
extern FobosApiKind activeFobosApiKind;

const char *fobosApiKindName(FobosApiKind kind);
QString fobosApiDisplayName(FobosApiKind kind);
void *activeFobosDevice();
bool hasActiveFobosDevice();
ReceiverBackendFeatures fobosBackendFeatures(FobosApiKind kind);
ReceiverStreamDescriptor makeFobosStreamDescriptor(void *nativeDevice,
                                                   FobosApiKind apiKind,
                                                   bool syncReader,
                                                   double sampleRateHz,
                                                   double centerFrequencyHz,
                                                   bool queueAudioBlocks,
                                                   bool publishIqSnapshot,
                                                   bool emitIqFrames,
                                                   bool agileScanEnabled,
                                                   const QVector<double> &agileScanFrequenciesHz = QVector<double>());

int getFobosStandardApiInfoSafely(char *libVersion, char *driverVersion);
int getFobosAgileApiInfoSafely(char *libVersion, char *driverVersion);
int getFobosStandardDeviceCountSafely();
int getFobosAgileDeviceCountSafely();
int openFobosDeviceSafely(fobos_dev_t **dev, uint32_t index);
int openFobosAgileDeviceSafely(fobos_sdr_dev_t **dev, uint32_t index);
int getFobosBoardInfoSafely(fobos_dev_t *dev,
                            char *hwRevision,
                            char *firmwareVersion,
                            char *manufacturer,
                            char *product,
                            char *serial);
int getFobosAgileBoardInfoSafely(fobos_sdr_dev_t *dev,
                                 char *hwRevision,
                                 char *firmwareVersion,
                                 char *manufacturer,
                                 char *product,
                                 char *serial);
int getFobosSampleRatesSafely(fobos_dev_t *dev, double *values, unsigned int *count);
int getFobosAgileSampleRatesSafely(fobos_sdr_dev_t *dev, double *values, unsigned int *count);
void logFobosApiInfo();
int closeFobosDeviceSafely(fobos_dev_t *dev);
int resetFobosDeviceSafely(fobos_dev_t *dev);
int setFobosClockSourceSafely(fobos_dev_t *dev, unsigned int value);
int setFobosDirectSamplingSafely(fobos_dev_t *dev, unsigned int enabled);
int setFobosSampleRateSafely(fobos_dev_t *dev, double value, double *actual);
int setFobosFrequencySafely(fobos_dev_t *dev, double value, double *actual);
int setFobosLnaGainSafely(fobos_dev_t *dev, unsigned int value);
int setFobosVgaGainSafely(fobos_dev_t *dev, unsigned int value);
int setFobosGpoSafely(fobos_dev_t *dev, uint8_t value);
int closeFobosAgileDeviceSafely(fobos_sdr_dev_t *dev);
int resetFobosAgileDeviceSafely(fobos_sdr_dev_t *dev);
int setFobosAgileClockSourceSafely(fobos_sdr_dev_t *dev, int value);
int setFobosAgileDirectSamplingSafely(fobos_sdr_dev_t *dev, unsigned int enabled);
int setFobosAgileSampleRateSafely(fobos_sdr_dev_t *dev, double value);
int setFobosAgileFrequencySafely(fobos_sdr_dev_t *dev, double value);
int setFobosAgileLnaGainSafely(fobos_sdr_dev_t *dev, unsigned int value);
int setFobosAgileVgaGainSafely(fobos_sdr_dev_t *dev, unsigned int value);
int setFobosAgileAutoBandwidthSafely(fobos_sdr_dev_t *dev, double value);
int setFobosAgileGpoSafely(fobos_sdr_dev_t *dev, uint8_t value);
int startFobosAgileScanSafely(fobos_sdr_dev_t *dev, double *frequencies, unsigned int count);
int stopFobosAgileScanSafely(fobos_sdr_dev_t *dev);
int isFobosAgileScanningSafely(fobos_sdr_dev_t *dev);
int getFobosAgileScanIndexSafely(fobos_sdr_dev_t *dev);
int startFobosSyncSafely(fobos_dev_t *dev, uint32_t blockSamples);
int startFobosAgileSyncSafely(fobos_sdr_dev_t *dev, uint32_t blockSamples);
int readFobosSyncSafely(fobos_dev_t *dev, float *buf, uint32_t *actualBufLength);
int readFobosAgileSyncSafely(fobos_sdr_dev_t *dev, float *buf, uint32_t *actualBufLength);
int stopFobosSyncSafely(fobos_dev_t *dev);
int stopFobosAgileSyncSafely(fobos_sdr_dev_t *dev);
int cancelFobosAsyncSafely(fobos_dev_t *dev);
int cancelFobosAgileAsyncSafely(fobos_sdr_dev_t *dev);
int readFobosAsyncSafely(fobos_dev_t *dev,
                         void (*callback)(float *buf, uint32_t bufLength, void *ctx),
                         void *ctx,
                         uint32_t bufferCount,
                         uint32_t blockSamples);
int readFobosAgileAsyncSafely(fobos_sdr_dev_t *dev,
                              void (*callback)(float *buf, uint32_t bufLength, fobos_sdr_dev_t *dev, void *ctx),
                              void *ctx,
                              uint32_t bufferCount,
                              uint32_t blockSamples);
int setActiveClockSourceSafely(int value);
int setActiveDirectSamplingSafely(unsigned int enabled);
int setActiveSampleRateSafely(double value, double *actual);
int setActiveFrequencySafely(double value, double *actual);
int setActiveLnaGainSafely(unsigned int value);
int setActiveVgaGainSafely(unsigned int value);
int setActiveGpoSafely(uint8_t value);

#endif // FOBOSBACKEND_H
