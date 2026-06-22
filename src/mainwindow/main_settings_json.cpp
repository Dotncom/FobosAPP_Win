#include "main.h"
#include "appconstants.h"
#include "appsettingsutils.h"
#include "diagnosticlogging.h"
#include "dmrbackendpaths.h"
#include "dmrprivacyutils.h"
#include "gnssqthhelpers.h"
#include "gnssserialutils.h"
#include "modulationutils.h"
#include "presethelpers.h"
#include "receiverdeviceutils.h"
#include "scanvisualutils.h"
#include "tuningutils.h"

#include <QDebug>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSignalBlocker>

#include <algorithm>
#include <cmath>

extern bool secondGraph;
extern bool syncWariable;
extern float sensitivity;
extern float contrast;
extern bool colorf;
QJsonObject YourClassName::settingsToJson() const {
    QJsonObject settings;
    settings["deviceIndex"] = pendingSettings.deviceIndex;
    settings["clockSource"] = pendingSettings.clockSource;
    settings["inputMode"] = pendingSettings.inputMode;
    settings["centerFrequency"] = pendingSettings.centerFrequency;
    settings["actualFrequency"] = pendingSettings.actualFrequency;
    settings["listeningFrequency"] = pendingSettings.listeningFrequency;
    settings["sampleRate"] = pendingSettings.sampleRate;
    settings["bandwidth"] = pendingSettings.bandwidth;
    settings["modulationType"] = pendingSettings.modulationType;
    settings["fftLength"] = pendingSettings.fftLength;
    settings["lnaGain"] = pendingSettings.lnaGain;
    settings["vgaGain"] = pendingSettings.vgaGain;
    settings["rtlAgc"] = pendingSettings.rtlAgc;
    settings["rtlTunerGainTenthsDb"] = pendingSettings.rtlTunerGainTenthsDb;
    settings["audioLowPassHz"] = pendingSettings.audioLowPassHz;
    settings["audioHighPassHz"] = pendingSettings.audioHighPassHz;
    settings["hfNoiseCancelDepth"] = pendingSettings.hfNoiseCancelDepth;
    settings["hfNoiseCancelRefGainDb"] = pendingSettings.hfNoiseCancelRefGainDb;
    settings["hfNoiseCancelRefDelayNs"] = pendingSettings.hfNoiseCancelRefDelayNs;
    settings["hfNoiseCancelRefTiltDb"] = pendingSettings.hfNoiseCancelRefTiltDb;
    settings["hfNoiseCancelFreeze"] = pendingSettings.hfNoiseCancelFreeze;
    settings["audioEnabled"] = pendingSettings.audioEnabled;
    settings["syncEnabled"] = false;
    settings["gpoValue"] = static_cast<int>(pendingSettings.gpoValue);
    settings["dmrBasebandSampleRate"] = normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    settings["dmrAmbeLayout"] = normalizedDmrAmbeLayout(pendingSettings.dmrAmbeLayout);
    settings["dmrManualTimingEnabled"] = pendingSettings.dmrManualTimingEnabled;
    settings["dmrManualTimingOffset"] = pendingSettings.dmrManualTimingOffset;
    settings["dmrSlicerRatio"] = pendingSettings.dmrSlicerRatio;
    settings["dmrAdaptiveSlicer"] = pendingSettings.dmrAdaptiveSlicer;
    settings["dmrPrivacyMode"] = pendingSettings.dmrPrivacyMode;
    settings["dmrPrivacyKeyId"] = pendingSettings.dmrPrivacyKeyId;
    settings["dmrPrivacyKeyHex"] = pendingSettings.dmrPrivacyKeyHex;
    settings["dmrPrivacyForwardToBackends"] = pendingSettings.dmrPrivacyForwardToBackends;
    settings["dmrPrivacyVariant"] = pendingSettings.dmrPrivacyVariant;
    settings["dmrPrivacyLayout"] = pendingSettings.dmrPrivacyLayout;
    settings["dmrPrivacyFrameOffset"] = pendingSettings.dmrPrivacyFrameOffset;
    settings["scalePercent"] = currentScale;
    settings["agileScanEnabled"] = agileScanEnabled;
    settings["agileScanAutoStepSampleRate"] = agileScanAutoStepSampleRate;
    settings["scanVisualMode"] = normalizedScanVisualMode(scanVisualMode);
    settings["agileScanRangesMhz"] = agileScanRangesMhz;
    settings["agileScanStepMhz"] = agileScanStepMhz;
    settings["scanListeningLockEnabled"] = scanListeningLockEnabled;
    settings["standardScanEnabled"] = standardScanEnabled;
    settings["standardScanCentersMhz"] = standardScanCentersMhz;
    settings["standardScanDwellMs"] = standardScanDwellMs;
    settings["standardScanSettleMs"] = standardScanSettleMs;
    settings["standardScanRangeStartMhz"] = standardScanRangeStartMhz;
    settings["standardScanRangeEndMhz"] = standardScanRangeEndMhz;
    settings["listeningScanEnabled"] = listeningScanEnabled;
    settings["listeningScanTargetsMhz"] = listeningScanTargetsMhz;
    settings["listeningScanDwellMs"] = listeningScanDwellMs;
    settings["listeningScanSettleMs"] = listeningScanSettleMs;
    settings["scanMeasurementEnabled"] = scanMeasurementEnabled;
    settings["scanMeasurementBinMhz"] = scanMeasurementBinMhz;
    settings["scanMeasurementUpdateIntervalMs"] = scanMeasurementUpdateIntervalMs;
    settings["qthLatitude"] = qthLatitude;
    settings["qthLongitude"] = qthLongitude;
    settings["qthPositionVisible"] = qthPositionVisible;
    settings["qthSource"] = qthSource;
    settings["gnssSerialPortName"] = gnssSerialPortName;
    settings["gnssSerialBaud"] = gnssSerialBaud;
    settings["gnssPositionPolicy"] = normalizedGnssPositionPolicy(gnssPositionPolicy);
    settings["gnssUbxAutoEnable"] = gnssUbxAutoEnable;
    settings["gnssTimeZoneOffsetMinutes"] = gnssTimeZoneOffsetMinutes;
    settings["gnssSatelliteTableVisible"] = gnssSatelliteTableVisible;
    settings["qthTileDirectory"] = qthTileDirectory;
    settings["qthMapLayer"] = qthMapLayer;
    settings["qthMapZoom"] = qthMapZoom;
    settings["qthGridPrecision"] = qthGridPrecision;
    settings["qthMapOverlayMode"] = qthMapOverlayMode;
    settings["qthOnlineProviderId"] = qthOnlineProviderId;
    settings["qthOnlineTileUrlTemplate"] = qthOnlineTileUrlTemplate;
    settings["qthOnlineAttribution"] = qthOnlineAttribution;
    settings["qthOnlineApiKey"] = qthOnlineApiKey;
    settings["qthOnlineNoDiskCache"] = qthOnlineNoDiskCache;
    settings["gnssSystemId"] = gnssSystemId;
    settings["gnssMonitorEnabled"] = gnssMonitorEnabled;
    settings["gnssUseGps"] = gnssUseGps;
    settings["gnssUseGlonass"] = gnssUseGlonass;
    settings["gnssUseGalileo"] = gnssUseGalileo;
    settings["gnssUseBeidou"] = gnssUseBeidou;
    settings["gnssUseQzss"] = gnssUseQzss;
    settings["gnssUseSbas"] = gnssUseSbas;
    settings["gnssUseOther"] = gnssUseOther;
    QJsonArray gnssDisabledSatellites;
    QStringList disabledSatelliteKeys = gnssDisabledSatelliteKeys.values();
    disabledSatelliteKeys.sort();
    for (const QString &key : disabledSatelliteKeys) {
        if (!key.trimmed().isEmpty()) {
            gnssDisabledSatellites.append(key);
        }
    }
    settings["gnssDisabledSatellites"] = gnssDisabledSatellites;
    settings["gnssAcquisitionIntegrationMs"] = gnssAcquisitionIntegrationMs;
    settings["gnssChannelFilterCutoffHz"] = gnssChannelFilterCutoffHz;
    settings["gnssDopplerSpanHz"] = gnssDopplerSpanHz;
    settings["gnssDopplerStepHz"] = gnssDopplerStepHz;
    settings["gnssContinuousAcquisitionEnabled"] = gnssContinuousAcquisitionEnabled;
    settings["gnssContinuousAcquisitionIntervalMs"] = gnssContinuousAcquisitionIntervalMs;
    QJsonArray qthMarkers;
    for (const qth::UserMarker &marker : qthUserMarkers) {
        if (!qth::isValidLatitude(marker.latitude) || !qth::isValidLongitude(marker.longitude)) {
            continue;
        }
        QJsonObject object;
        object["number"] = marker.number;
        object["name"] = marker.name.trimmed();
        object["description"] = marker.description.trimmed();
        object["latitude"] = marker.latitude;
        object["longitude"] = marker.longitude;
        qthMarkers.append(object);
    }
    settings["qthMarkers"] = qthMarkers;
    settings["spectrumUpdateIntervalMs"] = spectrumUpdateIntervalMs;
    settings["waterfallRowsPerFrame"] = waterfallRowsPerFrame;
    settings["experimentalGpuWaterfall"] = experimentalGpuWaterfall;
    settings["spurSuppressionEnabled"] = spurSuppressionEnabled;
    QJsonArray spurMask;
    for (const SpurMaskEntry &entry : spurMaskEntries) {
        if (!std::isfinite(entry.offsetHz) ||
            !std::isfinite(entry.widthHz) ||
            entry.widthHz <= 0.0) {
            continue;
        }
        QJsonObject object;
        object["offsetHz"] = entry.offsetHz;
        object["widthHz"] = entry.widthHz;
        object["prominenceDb"] = entry.prominenceDb;
        object["hits"] = entry.hits;
        spurMask.append(object);
    }
    settings["spurMask"] = spurMask;
    return settings;
}

void YourClassName::applySettingsFromJson(const QJsonObject &settingsJson, bool normalizeAfterApply) {
    auto readInt = [&settingsJson](const char *key, int currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toInt(currentValue) : currentValue;
    };
    auto readDouble = [&settingsJson](const char *key, double currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toDouble(currentValue) : currentValue;
    };
    auto readBool = [&settingsJson](const char *key, bool currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toBool(currentValue) : currentValue;
    };

    pendingSettings.deviceIndex = readInt("deviceIndex", pendingSettings.deviceIndex);
    pendingSettings.clockSource = readInt("clockSource", pendingSettings.clockSource);
    pendingSettings.inputMode = (std::clamp)(readInt("inputMode", pendingSettings.inputMode),
                                             static_cast<int>(INPUT_RF),
                                             static_cast<int>(INPUT_HF_NOISE_CANCEL));
    pendingSettings.centerFrequency = readDouble("centerFrequency", pendingSettings.centerFrequency);
    pendingSettings.actualFrequency = readDouble("actualFrequency", pendingSettings.actualFrequency);
    pendingSettings.listeningFrequency = readDouble("listeningFrequency", pendingSettings.listeningFrequency);
    pendingSettings.sampleRate = readDouble("sampleRate", pendingSettings.sampleRate);
    pendingSettings.bandwidth = readDouble("bandwidth", pendingSettings.bandwidth);
    pendingSettings.modulationType = readInt("modulationType", pendingSettings.modulationType);
    pendingSettings.fftLength = readInt("fftLength", pendingSettings.fftLength);
    pendingSettings.lnaGain = readInt("lnaGain", pendingSettings.lnaGain);
    pendingSettings.vgaGain = readInt("vgaGain", pendingSettings.vgaGain);
    pendingSettings.rtlAgc = readBool("rtlAgc", pendingSettings.rtlAgc);
    pendingSettings.rtlTunerGainTenthsDb =
        (std::clamp)(readInt("rtlTunerGainTenthsDb", pendingSettings.rtlTunerGainTenthsDb), 0, 496);
    pendingSettings.audioLowPassHz = clampAudioLowPassHz(readDouble("audioLowPassHz", pendingSettings.audioLowPassHz));
    pendingSettings.audioHighPassHz = clampAudioHighPassHz(readDouble("audioHighPassHz", pendingSettings.audioHighPassHz));
    pendingSettings.hfNoiseCancelDepth = clampHfNoiseCancelDepth(readDouble("hfNoiseCancelDepth", pendingSettings.hfNoiseCancelDepth));
    pendingSettings.hfNoiseCancelRefGainDb =
        clampHfNoiseCancelRefGainDb(readDouble("hfNoiseCancelRefGainDb", pendingSettings.hfNoiseCancelRefGainDb));
    pendingSettings.hfNoiseCancelRefDelayNs =
        clampHfNoiseCancelRefDelayNs(readDouble("hfNoiseCancelRefDelayNs", pendingSettings.hfNoiseCancelRefDelayNs));
    pendingSettings.hfNoiseCancelRefTiltDb =
        clampHfNoiseCancelRefTiltDb(readDouble("hfNoiseCancelRefTiltDb", pendingSettings.hfNoiseCancelRefTiltDb));
    pendingSettings.hfNoiseCancelFreeze = readBool("hfNoiseCancelFreeze", pendingSettings.hfNoiseCancelFreeze);
    pendingSettings.audioEnabled = readBool("audioEnabled", pendingSettings.audioEnabled);
    pendingSettings.syncEnabled = false;
    pendingSettings.gpoValue = static_cast<std::uint8_t>(readInt("gpoValue", pendingSettings.gpoValue));
    pendingSettings.dmrBasebandSampleRate =
        normalizedDmrBasebandSampleRate(readInt("dmrBasebandSampleRate",
                                                pendingSettings.dmrBasebandSampleRate));
    pendingSettings.dmrAmbeLayout =
        normalizedDmrAmbeLayout(readInt("dmrAmbeLayout", pendingSettings.dmrAmbeLayout));
    pendingSettings.dmrManualTimingEnabled =
        readBool("dmrManualTimingEnabled", pendingSettings.dmrManualTimingEnabled);
    pendingSettings.dmrManualTimingOffset =
        (std::clamp)(readInt("dmrManualTimingOffset", pendingSettings.dmrManualTimingOffset),
                     -80,
                     80);
    pendingSettings.dmrSlicerRatio =
        (std::clamp)(readDouble("dmrSlicerRatio", pendingSettings.dmrSlicerRatio),
                     0.45,
                     0.80);
    pendingSettings.dmrAdaptiveSlicer =
        readBool("dmrAdaptiveSlicer", pendingSettings.dmrAdaptiveSlicer);
    pendingSettings.dmrPrivacyMode =
        normalizedDmrPrivacyMode(readInt("dmrPrivacyMode", pendingSettings.dmrPrivacyMode));
    pendingSettings.dmrPrivacyKeyId =
        (std::clamp)(readInt("dmrPrivacyKeyId", pendingSettings.dmrPrivacyKeyId), 0, 255);
    pendingSettings.dmrPrivacyKeyHex =
        normalizedDmrPrivacyKeyHex(settingsJson.value(QStringLiteral("dmrPrivacyKeyHex"))
                                       .toString(pendingSettings.dmrPrivacyKeyHex));
    pendingSettings.dmrPrivacyForwardToBackends =
        readBool("dmrPrivacyForwardToBackends", pendingSettings.dmrPrivacyForwardToBackends);
    pendingSettings.dmrPrivacyVariant =
        settingsJson.value(QStringLiteral("dmrPrivacyVariant"))
            .toString(pendingSettings.dmrPrivacyVariant)
            .trimmed()
            .toLower();
    if (pendingSettings.dmrPrivacyVariant.isEmpty()) {
        pendingSettings.dmrPrivacyVariant = QStringLiteral("dmra");
    }
    pendingSettings.dmrPrivacyLayout =
        settingsJson.value(QStringLiteral("dmrPrivacyLayout"))
            .toString(pendingSettings.dmrPrivacyLayout)
            .trimmed()
            .toLower();
    if (pendingSettings.dmrPrivacyLayout.isEmpty()) {
        pendingSettings.dmrPrivacyLayout = QStringLiteral("normal");
    }
    pendingSettings.dmrPrivacyFrameOffset =
        (std::clamp)(readInt("dmrPrivacyFrameOffset", pendingSettings.dmrPrivacyFrameOffset), 0, 17);
    currentScale = readDouble("scalePercent", currentScale);
    agileScanEnabled = readBool("agileScanEnabled", agileScanEnabled);
    agileScanAutoStepSampleRate =
        readBool("agileScanAutoStepSampleRate", agileScanAutoStepSampleRate);
    agileScanRangesMhz = settingsJson.value("agileScanRangesMhz").toString(agileScanRangesMhz).trimmed();
    agileScanStepMhz = (std::clamp)(readDouble("agileScanStepMhz", agileScanStepMhz),
                                    AGILE_SCAN_MIN_STEP_MHZ,
                                    AGILE_SCAN_MAX_STEP_MHZ);
    applyAgileScanAutoStep(false);
    scanVisualMode = normalizedScanVisualMode(readInt("scanVisualMode", scanVisualMode));
    scanListeningLockEnabled = readBool("scanListeningLockEnabled", scanListeningLockEnabled);
    standardScanEnabled = readBool("standardScanEnabled", standardScanEnabled);
    standardScanCentersMhz =
        settingsJson.value("standardScanCentersMhz").toString(standardScanCentersMhz).trimmed();
    standardScanDwellMs = (std::clamp)(readInt("standardScanDwellMs", standardScanDwellMs),
                                       STANDARD_SCAN_MIN_DWELL_MS,
                                       STANDARD_SCAN_MAX_DWELL_MS);
    standardScanSettleMs = (std::clamp)(readInt("standardScanSettleMs", standardScanSettleMs),
                                        STANDARD_SCAN_MIN_SETTLE_MS,
                                        STANDARD_SCAN_MAX_SETTLE_MS);
    standardScanRangeStartMhz =
        settingsJson.value("standardScanRangeStartMhz").toString(standardScanRangeStartMhz).trimmed();
    standardScanRangeEndMhz =
        settingsJson.value("standardScanRangeEndMhz").toString(standardScanRangeEndMhz).trimmed();
    listeningScanEnabled = readBool("listeningScanEnabled", listeningScanEnabled);
    listeningScanTargetsMhz =
        settingsJson.value("listeningScanTargetsMhz").toString(listeningScanTargetsMhz).trimmed();
    listeningScanDwellMs = (std::clamp)(readInt("listeningScanDwellMs", listeningScanDwellMs),
                                        LISTENING_SCAN_MIN_DWELL_MS,
                                        LISTENING_SCAN_MAX_DWELL_MS);
    listeningScanSettleMs = (std::clamp)(readInt("listeningScanSettleMs", listeningScanSettleMs),
                                         LISTENING_SCAN_MIN_SETTLE_MS,
                                         LISTENING_SCAN_MAX_SETTLE_MS);
    const bool previousScanMeasurementEnabled = scanMeasurementEnabled;
    const double previousScanMeasurementBinMhz = scanMeasurementBinMhz;
    scanMeasurementEnabled = readBool("scanMeasurementEnabled", scanMeasurementEnabled);
    scanMeasurementBinMhz = (std::clamp)(readDouble("scanMeasurementBinMhz", scanMeasurementBinMhz),
                                         SCAN_MEASUREMENT_MIN_BIN_MHZ,
                                         SCAN_MEASUREMENT_MAX_BIN_MHZ);
    scanMeasurementUpdateIntervalMs =
        (std::clamp)(readInt("scanMeasurementUpdateIntervalMs", scanMeasurementUpdateIntervalMs),
                     SCAN_MEASUREMENT_MIN_UPDATE_MS,
                     SCAN_MEASUREMENT_MAX_UPDATE_MS);
    if (previousScanMeasurementEnabled != scanMeasurementEnabled ||
        std::abs(previousScanMeasurementBinMhz - scanMeasurementBinMhz) > 0.000001) {
        clearScanMeasurement();
    }
    qthLatitude = (std::clamp)(readDouble("qthLatitude", qthLatitude), -90.0, 90.0);
    qthLongitude = (std::clamp)(readDouble("qthLongitude", qthLongitude), -180.0, 180.0);
    qthPositionVisible = settingsJson.value("qthPositionVisible").toBool(qthPositionVisible);
    qthSource = settingsJson.value("qthSource").toString(qthSource).trimmed();
    if (qthSource.isEmpty()) {
        qthSource = QStringLiteral("manual");
    }
    if (qthSource == QStringLiteral("nmea")) {
        qthPositionVisible = false;
    }
    gnssSerialPortName = settingsJson.value("gnssSerialPortName").toString(gnssSerialPortName).trimmed();
    gnssSerialBaud = (std::clamp)(readInt("gnssSerialBaud", gnssSerialBaud), 1200, 921600);
    gnssPositionPolicy =
        normalizedGnssPositionPolicy(settingsJson.value("gnssPositionPolicy").toString(gnssPositionPolicy));
    gnssUbxAutoEnable = readBool("gnssUbxAutoEnable", gnssUbxAutoEnable);
    gnssTimeZoneOffsetMinutes = (std::clamp)(readInt("gnssTimeZoneOffsetMinutes", gnssTimeZoneOffsetMinutes),
                                             -12 * 60,
                                             100000);
    gnssSatelliteTableVisible = readBool("gnssSatelliteTableVisible", gnssSatelliteTableVisible);
    qthTileDirectory = settingsJson.value("qthTileDirectory").toString(qthTileDirectory).trimmed();
    qthMapLayer = (std::clamp)(readInt("qthMapLayer", qthMapLayer), 0, 2);
    qthMapZoom = (std::clamp)(readInt("qthMapZoom", qthMapZoom), 0, 19);
    qthOnlineProviderId =
        settingsJson.value("qthOnlineProviderId").toString(qthOnlineProviderId).trimmed();
    if (qthOnlineProviderId.isEmpty()) {
        qthOnlineProviderId = QStringLiteral("custom");
    }
    qthOnlineTileUrlTemplate =
        settingsJson.value("qthOnlineTileUrlTemplate").toString(qthOnlineTileUrlTemplate).trimmed();
    qthOnlineAttribution =
        settingsJson.value("qthOnlineAttribution").toString(qthOnlineAttribution).trimmed();
    qthOnlineApiKey =
        settingsJson.value("qthOnlineApiKey").toString(qthOnlineApiKey).trimmed();
    qthOnlineNoDiskCache = readBool("qthOnlineNoDiskCache", qthOnlineNoDiskCache);
    gnssSystemId =
        gnssSystemPreset(settingsJson.value("gnssSystemId").toString(gnssSystemId).trimmed()).id;
    gnssMonitorEnabled = readBool("gnssMonitorEnabled", gnssMonitorEnabled);
    gnssUseGps = readBool("gnssUseGps", gnssUseGps);
    gnssUseGlonass = readBool("gnssUseGlonass", gnssUseGlonass);
    gnssUseGalileo = readBool("gnssUseGalileo", gnssUseGalileo);
    gnssUseBeidou = readBool("gnssUseBeidou", gnssUseBeidou);
    gnssUseQzss = readBool("gnssUseQzss", gnssUseQzss);
    gnssUseSbas = readBool("gnssUseSbas", gnssUseSbas);
    gnssUseOther = readBool("gnssUseOther", gnssUseOther);
    if (settingsJson.contains(QStringLiteral("gnssDisabledSatellites"))) {
        gnssDisabledSatelliteKeys.clear();
        const QJsonArray disabledSatellites = settingsJson.value(QStringLiteral("gnssDisabledSatellites")).toArray();
        for (const QJsonValue &value : disabledSatellites) {
            const QString key = value.toString().trimmed();
            if (!key.isEmpty()) {
                gnssDisabledSatelliteKeys.insert(key);
            }
        }
        for (auto it = gnssNmeaSatelliteEnabled.begin(); it != gnssNmeaSatelliteEnabled.end(); ++it) {
            it.value() = !gnssDisabledSatelliteKeys.contains(it.key());
        }
    }
    gnssAcquisitionIntegrationMs =
        (std::clamp)(readInt("gnssAcquisitionIntegrationMs", gnssAcquisitionIntegrationMs),
                     GNSS_ACQUISITION_MIN_INTEGRATION_MS,
                     GNSS_ACQUISITION_MAX_INTEGRATION_MS);
    gnssChannelFilterCutoffHz =
        (std::clamp)(readDouble("gnssChannelFilterCutoffHz", gnssChannelFilterCutoffHz),
                     GNSS_CHANNEL_FILTER_MIN_HZ,
                     GNSS_CHANNEL_FILTER_MAX_HZ);
    gnssDopplerSpanHz =
        (std::clamp)(readInt("gnssDopplerSpanHz", gnssDopplerSpanHz), 1000, 50000);
    gnssDopplerStepHz =
        (std::clamp)(readInt("gnssDopplerStepHz", gnssDopplerStepHz), 250, 5000);
    gnssContinuousAcquisitionEnabled =
        readBool("gnssContinuousAcquisitionEnabled", gnssContinuousAcquisitionEnabled);
    gnssContinuousAcquisitionIntervalMs =
        (std::clamp)(readInt("gnssContinuousAcquisitionIntervalMs",
                             gnssContinuousAcquisitionIntervalMs),
                     GNSS_CONTINUOUS_ACQUISITION_MIN_INTERVAL_MS,
                     GNSS_CONTINUOUS_ACQUISITION_MAX_INTERVAL_MS);
    qthGridPrecision = readInt("qthGridPrecision", qthGridPrecision);
    if (qthGridPrecision <= 2) {
        qthGridPrecision = 2;
    } else if (qthGridPrecision <= 4) {
        qthGridPrecision = 4;
    } else {
        qthGridPrecision = 6;
    }
    qthMapOverlayMode = (std::clamp)(readInt("qthMapOverlayMode", qthMapOverlayMode), 0, 3);
    if (settingsJson.contains(QStringLiteral("qthMarkers"))) {
        QVector<qth::UserMarker> nextMarkers;
        QVector<int> usedNumbers;
        const QJsonArray markers = settingsJson.value(QStringLiteral("qthMarkers")).toArray();
        for (const QJsonValue &value : markers) {
            const QJsonObject object = value.toObject();
            qth::UserMarker marker;
            marker.number = object.value(QStringLiteral("number")).toInt(0);
            marker.name = object.value(QStringLiteral("name")).toString().trimmed().left(80);
            marker.description = object.value(QStringLiteral("description")).toString().trimmed().left(512);
            marker.latitude = object.value(QStringLiteral("latitude")).toDouble(std::numeric_limits<double>::quiet_NaN());
            marker.longitude = object.value(QStringLiteral("longitude")).toDouble(std::numeric_limits<double>::quiet_NaN());
            if (marker.number <= 0 ||
                usedNumbers.contains(marker.number) ||
                !qth::isValidLatitude(marker.latitude) ||
                !qth::isValidLongitude(marker.longitude)) {
                continue;
            }
            if (marker.name.isEmpty()) {
                marker.name = qth::maidenheadLocator(marker.latitude, marker.longitude, 6);
            }
            usedNumbers.append(marker.number);
            nextMarkers.append(marker);
        }
        qthUserMarkers = nextMarkers;
    }
    spectrumUpdateIntervalMs = (std::clamp)(readInt("spectrumUpdateIntervalMs", spectrumUpdateIntervalMs),
                                            SPECTRUM_UPDATE_AUTO_MS,
                                            SPECTRUM_UPDATE_MAX_MS);
    if (spectrumUpdateIntervalMs > 0 && spectrumUpdateIntervalMs < SPECTRUM_UPDATE_MIN_MS) {
        spectrumUpdateIntervalMs = SPECTRUM_UPDATE_MIN_MS;
    }
    waterfallRowsPerFrame = (std::clamp)(readInt("waterfallRowsPerFrame", waterfallRowsPerFrame),
                                         WATERFALL_ROWS_PER_FRAME_MIN,
                                         WATERFALL_ROWS_PER_FRAME_MAX);
    if (waterfallWidget) {
        waterfallWidget->setRowsPerFrame(waterfallRowsPerFrame);
    }
    experimentalGpuWaterfall = readBool("experimentalGpuWaterfall", experimentalGpuWaterfall);
    if (waterfallWidget) {
        waterfallWidget->setRenderBackend(experimentalGpuWaterfall
                                              ? MyWaterfallWidget::RenderBackend::GpuPrepared
                                              : MyWaterfallWidget::RenderBackend::CpuTexture);
    }
    {
        bool adjusted = false;
        QString standardScanError;
        const QVector<double> normalized =
            parseStandardScanCentersMhz(standardScanCentersMhz,
                                        pendingSettings.sampleRate,
                                        0,
                                        &standardScanError,
                                        &adjusted);
        if (adjusted && standardScanError.isEmpty() && !normalized.isEmpty()) {
            standardScanCentersMhz = formatMhzList(normalized);
        }
    }
    spurSuppressionEnabled = readBool("spurSuppressionEnabled", spurSuppressionEnabled);
    if (settingsJson.contains(QStringLiteral("spurMask"))) {
        QVector<SpurMaskEntry> nextMask;
        const QJsonArray array = settingsJson.value(QStringLiteral("spurMask")).toArray();
        for (const QJsonValue &value : array) {
            const QJsonObject object = value.toObject();
            SpurMaskEntry entry;
            entry.offsetHz = object.value(QStringLiteral("offsetHz")).toDouble(std::numeric_limits<double>::quiet_NaN());
            entry.widthHz = object.value(QStringLiteral("widthHz")).toDouble(SPUR_MIN_MASK_WIDTH_HZ);
            entry.prominenceDb = static_cast<float>(object.value(QStringLiteral("prominenceDb")).toDouble(0.0));
            entry.hits = object.value(QStringLiteral("hits")).toInt(0);
            if (std::isfinite(entry.offsetHz) &&
                std::isfinite(entry.widthHz) &&
                entry.widthHz > 0.0) {
                nextMask.append(entry);
            }
        }
        spurMaskEntries = nextMask;
    }
    if (spurSuppressionCheckbox) {
        QSignalBlocker blocker(spurSuppressionCheckbox);
        spurSuppressionCheckbox->setChecked(spurSuppressionEnabled);
    }
    updateSpurSuppressionStatus();
    if (normalizeAfterApply) {
        normalizeTuning(pendingSettings);
    }
}
