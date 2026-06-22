#ifndef RECEIVERDEVICEUTILS_H
#define RECEIVERDEVICEUTILS_H

#include <QString>

int rtlSdrNativeComboValue(int nativeIndex);
bool isRtlSdrNativeComboValue(int value);
int rtlSdrNativeIndexFromComboValue(int value);
QString rtlSdrNativeDeviceLabel(int nativeIndex, const QString &name);
bool isKnownRtlSampleRate(double value);

int bladeRfNativeComboValue(int nativeIndex);
bool isBladeRfNativeComboValue(int value);
int bladeRfNativeIndexFromComboValue(int value);

#endif // RECEIVERDEVICEUTILS_H
