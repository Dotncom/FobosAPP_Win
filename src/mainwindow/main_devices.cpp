#include "main.h"
#include "appconstants.h"
#include "appruntimeutils.h"
#include "bladerfbackend.h"
#include "receiverbackendregistry.h"
#include "receiverdeviceutils.h"

#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>
#include <QSignalBlocker>

#include <algorithm>
#include <cmath>
void YourClassName::buildLocalReceiverDeviceChoices(QStringList &labels, QVector<int> &values) const {
    labels.clear();
    values.clear();

    for (int i = 0; i < availableFobosDevices.size(); ++i) {
        labels << availableFobosDevices.at(i).label;
        values << i;
    }
    if (hasActiveFobosDevice()) {
        const int activeLogicalIndex = openedDeviceIndex >= 0
                                           ? openedDeviceIndex
                                           : pendingSettings.deviceIndex;
        bool activeAlreadyListed = false;
        for (int value : values) {
            if (value == activeLogicalIndex) {
                activeAlreadyListed = true;
                break;
            }
        }
        if (!activeAlreadyListed) {
            labels << QStringLiteral("[%1] active device #%2")
                          .arg(fobosApiDisplayName(activeFobosApiKind))
                          .arg(openedNativeDeviceIndex >= 0
                                   ? openedNativeDeviceIndex
                                   : std::max(0, activeLogicalIndex));
            values << activeLogicalIndex;
        }
    }

    const QVector<RtlSdrDeviceInfo> rtlDevices = enumerateRtlSdrDevices();
    if (rtlDevices.isEmpty()) {
        labels << QStringLiteral("RTL-SDR native auto (rtlsdr.dll)");
        values << rtlSdrNativeComboValue(0);
    } else {
        for (const RtlSdrDeviceInfo &rtlInfo : rtlDevices) {
            labels << rtlSdrNativeDeviceLabel(rtlInfo.nativeIndex, rtlInfo.name);
            values << rtlSdrNativeComboValue(rtlInfo.nativeIndex);
        }
    }

    labels << QStringLiteral("RTL-SDR via rtl_tcp (127.0.0.1:1234)");
    values << RTL_TCP_DEVICE_INDEX;

    const QVector<BladeRfDeviceInfo> bladeRfDevices = enumerateBladeRfDevices();
    if (bladeRfDevices.isEmpty()) {
        labels << QStringLiteral("bladeRF native auto (bladeRF.dll)");
        values << bladeRfNativeComboValue(0);
    } else {
        for (const BladeRfDeviceInfo &bladeInfo : bladeRfDevices) {
            labels << bladeInfo.label;
            values << bladeRfNativeComboValue(bladeInfo.nativeIndex);
        }
    }

    labels << QStringLiteral("SoapySDR auto (SoapySDR.dll)");
    values << SOAPY_SDR_DEVICE_INDEX;

    if (labels.isEmpty()) {
        labels << uiText(QStringLiteral("no_fobos_devices_detected"),
                         QStringLiteral("No Fobos devices detected"));
        values << 0;
    }
}

void YourClassName::rebuildReceiverDeviceCombo() {
    if (!comboBox) {
        return;
    }

    QStringList labels;
    QVector<int> values;
    if (isNetworkClientMode() && remoteReceiverDeviceListValid) {
        const int count = std::min(remoteReceiverDeviceLabels.size(), remoteReceiverDeviceValues.size());
        for (int i = 0; i < count; ++i) {
            labels << QStringLiteral("[Server] %1").arg(remoteReceiverDeviceLabels.at(i));
            values << remoteReceiverComboValue(remoteReceiverDeviceValues.at(i));
        }
    } else {
        if (!isNetworkClientMode() &&
            !deviceOpened &&
            !(processor && processor->isRunning()) &&
            availableFobosDevices.isEmpty()) {
            refreshFobosDeviceList();
        }
        buildLocalReceiverDeviceChoices(labels, values);
    }

    if (labels.size() != values.size()) {
        labels.clear();
        values.clear();
    }
    if (labels.isEmpty()) {
        labels << (isNetworkClientMode()
                       ? QStringLiteral("Waiting for server receiver list...")
                       : uiText(QStringLiteral("no_fobos_devices_detected"),
                                QStringLiteral("No Fobos devices detected")));
        values << pendingSettings.deviceIndex;
    }

    QSignalBlocker blocker(comboBox);
    comboBox->clear();
    for (int i = 0; i < labels.size(); ++i) {
        comboBox->addItem(labels.at(i), values.at(i));
    }

    const int selectedComboValue =
        isNetworkClientMode() && remoteReceiverDeviceListValid
            ? remoteReceiverComboValue(pendingSettings.deviceIndex)
            : pendingSettings.deviceIndex;
    int selectedIndex = comboBox->findData(selectedComboValue);
    if (selectedIndex < 0 &&
        pendingSettings.deviceIndex >= 0 &&
        pendingSettings.deviceIndex < comboBox->count()) {
        selectedIndex = pendingSettings.deviceIndex;
    }
    if (selectedIndex < 0 && comboBox->count() > 0) {
        selectedIndex = 0;
    }
    if (selectedIndex >= 0) {
        comboBox->setCurrentIndex(selectedIndex);
        bool ok = false;
        const int selectedDevice = comboBox->itemData(selectedIndex).toInt(&ok);
        if (ok) {
            pendingSettings.deviceIndex = receiverDeviceIndexFromComboValue(selectedDevice);
        }
    }
}

QJsonArray YourClassName::receiverDeviceListToJson() const {
    QStringList labels;
    QVector<int> values;

    for (int i = 0; i < availableFobosDevices.size(); ++i) {
        labels << availableFobosDevices.at(i).label;
        values << i;
    }
    if (hasActiveFobosDevice()) {
        const int activeLogicalIndex = openedDeviceIndex >= 0
                                           ? openedDeviceIndex
                                           : pendingSettings.deviceIndex;
        bool activeAlreadyListed = false;
        for (int value : values) {
            if (value == activeLogicalIndex) {
                activeAlreadyListed = true;
                break;
            }
        }
        if (!activeAlreadyListed) {
            labels << QStringLiteral("[%1] active device #%2")
                          .arg(fobosApiDisplayName(activeFobosApiKind))
                          .arg(openedNativeDeviceIndex >= 0
                                   ? openedNativeDeviceIndex
                                   : std::max(0, activeLogicalIndex));
            values << activeLogicalIndex;
        }
    }

    labels << QStringLiteral("RTL-SDR native auto (server)");
    values << rtlSdrNativeComboValue(0);
    labels << QStringLiteral("RTL-SDR via rtl_tcp (server 127.0.0.1:1234)");
    values << RTL_TCP_DEVICE_INDEX;
    labels << QStringLiteral("bladeRF native auto (server)");
    values << bladeRfNativeComboValue(0);
    labels << QStringLiteral("SoapySDR auto (server)");
    values << SOAPY_SDR_DEVICE_INDEX;

    QJsonArray array;
    const int count = std::min(labels.size(), values.size());
    for (int i = 0; i < count; ++i) {
        QJsonObject object;
        object["label"] = labels.at(i);
        object["deviceIndex"] = values.at(i);
        array.append(object);
    }
    return array;
}

bool YourClassName::applyReceiverDeviceListFromJson(const QJsonArray &devices) {
    QStringList labels;
    QVector<int> values;
    for (const QJsonValue &value : devices) {
        const QJsonObject object = value.toObject();
        const QString label = object.value(QStringLiteral("label")).toString().trimmed();
        if (label.isEmpty()) {
            continue;
        }
        labels << label;
        values << object.value(QStringLiteral("deviceIndex")).toInt(values.size());
    }

    remoteReceiverDeviceLabels = labels;
    remoteReceiverDeviceValues = values;
    remoteReceiverDeviceListValid = !labels.isEmpty() && labels.size() == values.size();
    rebuildReceiverDeviceCombo();
    return remoteReceiverDeviceListValid;
}

void YourClassName::clearRemoteReceiverDeviceList() {
    remoteReceiverDeviceLabels.clear();
    remoteReceiverDeviceValues.clear();
    remoteReceiverDeviceListValid = false;
    rebuildReceiverDeviceCombo();
}

QString YourClassName::formatFobosDeviceLabel(const FobosDeviceInfo &info) const {
    const QString serial = info.serial.isEmpty() ? QStringLiteral("unknown SN") : info.serial;
    const QString hw = info.hardwareRevision.isEmpty() ? QStringLiteral("?") : info.hardwareRevision;
    const QString fw = info.firmwareVersion.isEmpty() ? QStringLiteral("?") : info.firmwareVersion;
    return QString("[%1] #%2  SN %3  HW %4  FW %5")
        .arg(fobosApiDisplayName(info.apiKind))
        .arg(info.nativeIndex)
        .arg(serial)
        .arg(hw)
        .arg(fw);
}

void YourClassName::refreshFobosDeviceList(bool recoverUsb) {
    if (recoverUsb) {
        qDebug() << "[FobosDevices] USB recovery refresh requested"
                 << "runState" << static_cast<int>(runState)
                 << "deviceOpened" << deviceOpened
                 << "processorRunning" << (processor && processor->isRunning())
                 << "device" << activeFobosDevice();

        if (isRunningOrTransitioning() || (processor && processor->isRunning())) {
            qDebug() << "[FobosDevices] USB recovery skipped because Fobos is active or transitioning";
        } else {
            bool resetIssued = false;

            if (device) {
                qDebug() << "[FobosDevices] resetting idle standard session" << device;
                const int resetResult = resetFobosDeviceSafely(device);
                qDebug() << "[FobosDevices] idle standard reset result" << resetResult;
                device = nullptr;
                resetIssued = true;
            }
            if (agileDevice) {
                if (agileScanRunning) {
                    const int stopResult = stopFobosAgileScanSafely(agileDevice);
                    qDebug() << "[FobosDevices] idle agile scan stop before reset result" << stopResult;
                    agileScanRunning = false;
                    activeAgileScanFrequencies.clear();
                }
                qDebug() << "[FobosDevices] resetting idle agile session" << agileDevice;
                const int resetResult = resetFobosAgileDeviceSafely(agileDevice);
                qDebug() << "[FobosDevices] idle agile reset result" << resetResult;
                agileDevice = nullptr;
                resetIssued = true;
            }

            if (resetIssued) {
                deviceOpened = false;
                openedDeviceIndex = -1;
                openedNativeDeviceIndex = -1;
                appliedSampleRate = 0.0;
                appliedHardwareSettings = RadioSettings{};
                hardwareSettingsApplied = false;
                sampleRateReopenRequired = false;
                fobosCloseKnownUnsafe = false;
                activeFobosApiKind = FobosApiKind::Standard;
                openedDeviceApiKind = FobosApiKind::Standard;
                QThread::msleep(700);
            }

            const int standardResetCount = getFobosStandardDeviceCountSafely();
            for (int i = 0; i < standardResetCount; ++i) {
                fobos_dev_t *resetDevice = nullptr;
                const int openResult = openFobosDeviceSafely(&resetDevice, static_cast<uint32_t>(i));
                if (openResult != FOBOS_ERR_OK || !resetDevice) {
                    qDebug() << "[FobosDevices] standard recovery open failed"
                             << "index" << i << "result" << openResult;
                    continue;
                }
                const int resetResult = resetFobosDeviceSafely(resetDevice);
                qDebug() << "[FobosDevices] standard recovery reset"
                         << "index" << i << "result" << resetResult;
                resetIssued = true;
            }

            const int agileResetCount = getFobosAgileDeviceCountSafely();
            for (int i = 0; i < agileResetCount; ++i) {
                fobos_sdr_dev_t *resetDevice = nullptr;
                const int openResult = openFobosAgileDeviceSafely(&resetDevice, static_cast<uint32_t>(i));
                if (openResult != FOBOS_ERR_OK || !resetDevice) {
                    qDebug() << "[FobosDevices] agile recovery open failed"
                             << "index" << i << "result" << openResult;
                    continue;
                }
                const int resetResult = resetFobosAgileDeviceSafely(resetDevice);
                qDebug() << "[FobosDevices] agile recovery reset"
                         << "index" << i << "result" << resetResult;
                resetIssued = true;
            }

            if (resetIssued) {
                qDebug() << "[FobosDevices] waiting after USB recovery reset before enumeration";
                QThread::msleep(1200);
            } else {
                qDebug() << "[FobosDevices] no Fobos handles were available for USB recovery reset";
            }
        }
    }

    availableFobosDevices.clear();

    auto addDevice = [this](const FobosDeviceInfo &info) {
        if (!info.serial.isEmpty()) {
            for (int i = 0; i < availableFobosDevices.size(); ++i) {
                FobosDeviceInfo &existing = availableFobosDevices[i];
                if (existing.serial != info.serial) {
                    continue;
                }
                const bool preferAgile =
                    info.apiKind == FobosApiKind::Agile &&
                    (firmwareLooksAgile(info.firmwareVersion) ||
                     firmwareLooksAgile(existing.firmwareVersion));
                if (preferAgile) {
                    existing = info;
                }
                return;
            }
        }
        availableFobosDevices.append(info);
    };

    const int standardCount = getFobosStandardDeviceCountSafely();
    for (int i = 0; i < standardCount; ++i) {
        fobos_dev_t *infoDevice = nullptr;
        const int openResult = openFobosDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] standard open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] standard board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Standard;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    const int agileCount = getFobosAgileDeviceCountSafely();
    for (int i = 0; i < agileCount; ++i) {
        fobos_sdr_dev_t *infoDevice = nullptr;
        const int openResult = openFobosAgileDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] agile open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosAgileBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosAgileDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] agile board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Agile;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    qDebug() << "[FobosDevices] refreshed"
             << "standardCount" << standardCount
             << "agileCount" << agileCount
             << "usable" << availableFobosDevices.size()
             << "rtlNativeLazy" << true;
    updateAgileScanControls();
}

QStringList YourClassName::getFobosDevices() {
    if (!deviceOpened && !(processor && processor->isRunning())) {
        refreshFobosDeviceList();
    }

    QStringList deviceList;
    QVector<int> values;
    buildLocalReceiverDeviceChoices(deviceList, values);
    return deviceList;
}

YourClassName::FobosDeviceInfo YourClassName::selectedFobosDeviceInfo() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    if (selected >= 0 && selected < availableFobosDevices.size()) {
        return availableFobosDevices[selected];
    }

    FobosDeviceInfo fallback;
    fallback.apiKind = FobosApiKind::Standard;
    fallback.nativeIndex = std::max(0, selected);
    fallback.label = QString("Standard device #%1").arg(fallback.nativeIndex);
    return fallback;
}

bool YourClassName::isRtlTcpSelected() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return selected == RTL_TCP_DEVICE_INDEX;
}

bool YourClassName::isRtlSdrNativeSelected() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return isRtlSdrNativeComboValue(selected);
}

bool YourClassName::isSoapySdrSelected() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return selected == SOAPY_SDR_DEVICE_INDEX;
}

bool YourClassName::isBladeRfNativeSelected() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return isBladeRfNativeComboValue(selected);
}

int YourClassName::selectedRtlSdrNativeIndex() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return isRtlSdrNativeComboValue(selected) ? rtlSdrNativeIndexFromComboValue(selected) : 0;
}

int YourClassName::selectedBladeRfNativeIndex() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = receiverDeviceIndexFromComboValue(value);
        }
    }
    return isBladeRfNativeComboValue(selected) ? bladeRfNativeIndexFromComboValue(selected) : 0;
}

bool YourClassName::isRtlBackendSelected() const {
    return isRtlTcpSelected() || isRtlSdrNativeSelected();
}

bool YourClassName::isExternalReceiverBackendSelected() const {
    return isRtlBackendSelected() || isSoapySdrSelected() || isBladeRfNativeSelected();
}

bool YourClassName::normalizeRtlSdrSettings() {
    if (!isRtlBackendSelected()) {
        return false;
    }

    if (!isKnownRtlSampleRate(pendingSettings.sampleRate)) {
        qDebug() << "[RTL-TCP] replacing non-RTL sample rate"
                 << pendingSettings.sampleRate
                 << "with" << RTL_TCP_SAFE_SAMPLE_RATE;
        pendingSettings.sampleRate = RTL_TCP_SAFE_SAMPLE_RATE;
        if (sampleBox) {
            QSignalBlocker blocker(sampleBox);
            const int index = sampleBox->findData(pendingSettings.sampleRate);
            if (index >= 0) {
                sampleBox->setCurrentIndex(index);
            }
        }
        settingRange();
        return true;
    }
    return false;
}

ReceiverStreamDescriptor YourClassName::makeRtlTcpStreamDescriptor(bool queueAudioBlocks,
                                                                   bool publishIqSnapshot,
                                                                   bool emitIqFrames) const {
    ReceiverStreamDescriptor stream;
    stream.kind = ReceiverBackendStreamKind::RtlTcp;
    stream.backendId = QStringLiteral("rtl-tcp");
    stream.backendName = QStringLiteral("RTL-SDR via rtl_tcp");
    stream.nativeDevice = nullptr;
    stream.sampleRateHz = pendingSettings.sampleRate;
    stream.centerFrequencyHz = pendingSettings.centerFrequency;
    stream.rtlTcpHost = QString::fromLatin1(RTL_TCP_DEFAULT_HOST);
    stream.rtlTcpPort = RTL_TCP_DEFAULT_PORT;
    stream.rtlTcpAgc = pendingSettings.rtlAgc;
    stream.rtlTcpTunerGainTenthsDb =
        pendingSettings.rtlAgc ? -1 : (std::clamp)(pendingSettings.rtlTunerGainTenthsDb, 0, 496);
    stream.syncReader = false;
    stream.queueAudioBlocks = queueAudioBlocks;
    stream.publishIqSnapshot = publishIqSnapshot;
    stream.emitIqFrames = emitIqFrames;
    stream.agileScanEnabled = false;
    return stream;
}

ReceiverStreamDescriptor YourClassName::makeRtlSdrNativeStreamDescriptor(bool queueAudioBlocks,
                                                                        bool publishIqSnapshot,
                                                                        bool emitIqFrames) const {
    ReceiverStreamDescriptor stream;
    stream.kind = ReceiverBackendStreamKind::RtlSdrNative;
    stream.backendId = QStringLiteral("rtl-sdr-native");
    stream.backendName = QStringLiteral("RTL-SDR native");
    stream.nativeDevice = nullptr;
    stream.sampleRateHz = pendingSettings.sampleRate;
    stream.centerFrequencyHz = pendingSettings.centerFrequency;
    stream.rtlSdrNativeDeviceIndex = selectedRtlSdrNativeIndex();
    stream.rtlTcpAgc = pendingSettings.rtlAgc;
    stream.rtlTcpTunerGainTenthsDb =
        pendingSettings.rtlAgc ? -1 : (std::clamp)(pendingSettings.rtlTunerGainTenthsDb, 0, 496);
    stream.syncReader = false;
    stream.queueAudioBlocks = queueAudioBlocks;
    stream.publishIqSnapshot = publishIqSnapshot;
    stream.emitIqFrames = emitIqFrames;
    stream.agileScanEnabled = false;
    return stream;
}

ReceiverStreamDescriptor YourClassName::makeSoapySdrStreamDescriptor(bool queueAudioBlocks,
                                                                     bool publishIqSnapshot,
                                                                     bool emitIqFrames) const {
    ReceiverStreamDescriptor stream;
    stream.kind = ReceiverBackendStreamKind::SoapySdr;
    stream.backendId = QStringLiteral("soapy-sdr");
    stream.backendName = QStringLiteral("SoapySDR");
    stream.nativeDevice = nullptr;
    stream.sampleRateHz = pendingSettings.sampleRate;
    stream.centerFrequencyHz = pendingSettings.centerFrequency;
    stream.soapySdrDeviceIndex = 0;
    stream.syncReader = false;
    stream.queueAudioBlocks = queueAudioBlocks;
    stream.publishIqSnapshot = publishIqSnapshot;
    stream.emitIqFrames = emitIqFrames;
    stream.agileScanEnabled = false;
    return stream;
}

ReceiverStreamDescriptor YourClassName::makeBladeRfNativeStreamDescriptor(bool queueAudioBlocks,
                                                                         bool publishIqSnapshot,
                                                                         bool emitIqFrames) const {
    ReceiverStreamDescriptor stream;
    stream.kind = ReceiverBackendStreamKind::BladeRfNative;
    stream.backendId = QStringLiteral("bladerf-native");
    stream.backendName = QStringLiteral("bladeRF native");
    stream.nativeDevice = nullptr;
    stream.sampleRateHz = pendingSettings.sampleRate;
    stream.centerFrequencyHz = pendingSettings.centerFrequency;
    stream.bladeRfNativeDeviceIndex = selectedBladeRfNativeIndex();
    stream.syncReader = false;
    stream.queueAudioBlocks = queueAudioBlocks;
    stream.publishIqSnapshot = publishIqSnapshot;
    stream.emitIqFrames = emitIqFrames;
    stream.agileScanEnabled = false;
    return stream;
}

void YourClassName::listFobosDevices() {
    const bool canRefreshUsbList =
        !hasActiveFobosDevice() &&
        !deviceOpened &&
        !(processor && processor->isRunning()) &&
        !isRunningOrTransitioning();
    if (canRefreshUsbList) {
        refreshFobosDeviceList();
    }

    QVector<FobosDeviceInfo> devicesForReport = availableFobosDevices;
    if (devicesForReport.isEmpty() && hasActiveFobosDevice()) {
        FobosDeviceInfo activeInfo;
        activeInfo.apiKind = activeFobosApiKind;
        activeInfo.nativeIndex = openedNativeDeviceIndex >= 0
                                     ? openedNativeDeviceIndex
                                     : std::max(0, pendingSettings.deviceIndex);
        activeInfo.label = QStringLiteral("[%1] active device #%2")
                               .arg(fobosApiDisplayName(activeInfo.apiKind))
                               .arg(activeInfo.nativeIndex);
        devicesForReport.append(activeInfo);
    }

    char standardLib[256] = {};
    char standardDriver[256] = {};
    char agileLib[256] = {};
    char agileDriver[256] = {};
    getFobosStandardApiInfoSafely(standardLib, standardDriver);
    getFobosAgileApiInfoSafely(agileLib, agileDriver);

    QString deviceInfo = QStringLiteral("%1 %2 (%3)\n%4 %5 (%6)\n\n%7 %8\n")
                             .arg(uiText(QStringLiteral("standard_api"), QStringLiteral("Standard API:")))
                             .arg(standardLib)
                             .arg(standardDriver)
                             .arg(uiText(QStringLiteral("agile_api"), QStringLiteral("Agile API:")))
                             .arg(agileLib)
                             .arg(agileDriver)
                             .arg(uiText(QStringLiteral("detected_devices"), QStringLiteral("Detected devices:")))
                             .arg(devicesForReport.size());
    if (!canRefreshUsbList) {
        deviceInfo += uiText(QStringLiteral("detected_devices_cached"),
                             QStringLiteral("Receiver is active; showing cached/active device information.\n"));
    }
    for (int i = 0; i < devicesForReport.size(); ++i) {
        const FobosDeviceInfo &info = devicesForReport[i];
        deviceInfo += QStringLiteral("%1. %2\n    %3 %4\n    %5 %6\n")
                          .arg(i)
                          .arg(info.label)
                          .arg(uiText(QStringLiteral("manufacturer"), QStringLiteral("manufacturer:")))
                          .arg(info.manufacturer)
                          .arg(uiText(QStringLiteral("product"), QStringLiteral("product:")))
                          .arg(info.product);
    }
    QMessageBox::information(this,
                             uiText(QStringLiteral("fobos_devices"), QStringLiteral("Fobos Devices")),
                             deviceInfo);
}
