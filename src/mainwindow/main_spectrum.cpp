#include "main.h"
#include "appconstants.h"
#include "appruntimeutils.h"
#include "diagnosticlogging.h"
#include "iqbuffer.h"
#include "receiverdeviceutils.h"
#include "scanvisualutils.h"
#include "spectrumfftworker.h"
#include "tuningutils.h"

#include <QDebug>
#include <QDateTime>
#include <QElapsedTimer>
#include <QSignalBlocker>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

extern std::vector<float> fftMagnitudes;
extern std::vector<float> fftFrequencies;
extern int fftLength;
extern double currentScale;
extern bool secondGraph;
extern bool syncWariable;
extern float sensitivity;
extern float contrast;
extern bool colorf;
void YourClassName::onfftLengthEntered() {
    const int newFftLength = fftComboBox->currentText().toInt();
    applyFftLengthChange(newFftLength, true);
}

bool YourClassName::applyFftLengthChange(int newFftLength, bool notifyRemote) {
    if (newFftLength <= 0) {
        qDebug() << "Invalid FFT length selected.";
        return false;
    }

    if (pendingSettings.fftLength == newFftLength && fftResult) {
        return true;
    }

    qDebug() << "[FFT] applying FFT length"
             << "previous" << pendingSettings.fftLength
             << "new" << newFftLength
             << "state" << runStateName(runState)
             << "networkMode" << static_cast<int>(networkMode)
             << "processingMode" << static_cast<int>(networkProcessingMode);

    pendingSettings.fftLength = newFftLength;
    publishSettingsToGlobals();
    updateSpectrumTimerInterval();
    fftResult = std::make_unique<FFTResult>();
    if (spectrumFftWorker) {
        spectrumFftWorker->resetHfNoiseCancelState();
    }

    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }

    if (notifyRemote && isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }

    return true;
}

void YourClassName::populateSampleRates() {
    if (!sampleBox) {
        return;
    }

    QSignalBlocker sampleBoxBlocker(sampleBox);
    if (isSoapySdrSelected()) {
        sampleBox->clear();
        const QVector<double> soapyRates = {
            1000000.0,
            1024000.0,
            1536000.0,
            2048000.0,
            2400000.0,
            3200000.0,
            8000000.0,
            10000000.0,
            20000000.0,
            50000000.0
        };
        for (const double rate : soapyRates) {
            sampleBox->addItem(formatSampleRate(rate), rate);
        }
        if (sampleBox->findData(pendingSettings.sampleRate) < 0) {
            pendingSettings.sampleRate = 2048000.0;
        }
        const int index = sampleBox->findData(pendingSettings.sampleRate);
        if (index >= 0) {
            sampleBox->setCurrentIndex(index);
        }
        qDebug() << "[SoapySDR] using generic Soapy sample-rate list";
        return;
    }
    if (isRtlBackendSelected()) {
        sampleBox->clear();
        const QVector<double> rtlRates = {
            1024000.0,
            1536000.0,
            2048000.0,
            2400000.0,
            2560000.0,
            3200000.0
        };
        for (const double rate : rtlRates) {
            sampleBox->addItem(formatSampleRate(rate), rate);
        }
        if (sampleBox->findData(pendingSettings.sampleRate) < 0) {
            pendingSettings.sampleRate = RTL_TCP_SAFE_SAMPLE_RATE;
        }
        const int index = sampleBox->findData(pendingSettings.sampleRate);
        if (index >= 0) {
            sampleBox->setCurrentIndex(index);
        }
        qDebug() << "[RTL-TCP] using RTL-SDR sample-rate list";
        return;
    }

    auto addDefaultSampleRates = [this]() {
        if (sampleBox->count() > 0) {
            return;
        }
        const QVector<double> defaultRates = {
            8000000.0, 10000000.0, 12500000.0, 16000000.0, 20000000.0,
            25000000.0, 32000000.0, 40000000.0, 50000000.0, 80000000.0
        };
        for (const double rate : defaultRates) {
            sampleBox->addItem(formatSampleRate(rate), rate);
        }
        const int defaultIndex = sampleBox->findData(pendingSettings.sampleRate);
        if (defaultIndex >= 0) {
            sampleBox->setCurrentIndex(defaultIndex);
        }
        qDebug() << "[FobosDevices] using fallback sample-rate list; no local receiver is required for client control";
    };

    if (isNetworkClientMode()) {
        addDefaultSampleRates();
        return;
    }

    void *sampleRateDevice = activeFobosDevice();
    FobosApiKind sampleRateApiKind = activeFobosApiKind;
    bool openedForSampleRates = false;

    int ret = FOBOS_ERR_OK;
    if (!sampleRateDevice) {
        const FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
        sampleRateApiKind = selectedInfo.apiKind;
        if (selectedInfo.apiKind == FobosApiKind::Agile) {
            fobos_sdr_dev_t *openedDevice = nullptr;
            ret = openFobosAgileDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        } else {
            fobos_dev_t *openedDevice = nullptr;
            ret = openFobosDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        }
        openedForSampleRates = (ret == FOBOS_ERR_OK && sampleRateDevice);
    }
    if (ret != FOBOS_ERR_OK) {
        qDebug() << "[FobosDevices] sample-rate list unavailable; using fallback, open result:" << ret;
        addDefaultSampleRates();
        return;
    }
    if (!sampleRateDevice) {
        qDebug() << "Device is not initialized.";
        addDefaultSampleRates();
        return;
    }
    double sampleRates[100];
    unsigned int count = 100;
    int result = sampleRateApiKind == FobosApiKind::Agile
                     ? getFobosAgileSampleRatesSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice), sampleRates, &count)
                     : getFobosSampleRatesSafely(static_cast<fobos_dev_t*>(sampleRateDevice), sampleRates, &count);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to get sample rates, error code:" << result;
        if (openedForSampleRates) {
            if (sampleRateApiKind == FobosApiKind::Agile) {
                closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
            } else {
                closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
            }
        }
        addDefaultSampleRates();
        return;
    }
    sampleBox->clear();
    for (unsigned int i = 0; i < count; ++i) {
        QString formattedRate = formatSampleRate(sampleRates[i]);
        sampleBox->addItem(formattedRate, sampleRates[i]);
    }
    int selectedIndex = sampleBox->findData(pendingSettings.sampleRate);
    if (selectedIndex < 0 || isKnownRtlSampleRate(pendingSettings.sampleRate)) {
        int defaultIndex = sampleBox->findData(FOBOS_DEFAULT_SAMPLE_RATE);
        if (defaultIndex < 0 && sampleBox->count() > 0) {
            double bestDelta = std::numeric_limits<double>::max();
            for (int i = 0; i < sampleBox->count(); ++i) {
                const double rate = sampleBox->itemData(i).toDouble();
                const double delta = std::abs(rate - FOBOS_DEFAULT_SAMPLE_RATE);
                if (delta < bestDelta) {
                    bestDelta = delta;
                    defaultIndex = i;
                }
            }
        }
        if (defaultIndex >= 0) {
            pendingSettings.sampleRate = sampleBox->itemData(defaultIndex).toDouble();
            selectedIndex = defaultIndex;
        }
    }
    if (selectedIndex >= 0) {
        sampleBox->setCurrentIndex(selectedIndex);
    }
    if (openedForSampleRates) {
        if (sampleRateApiKind == FobosApiKind::Agile) {
            closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
        } else {
            closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
        }
    }
}

void YourClassName::learnHfInterferenceBaseline() {
    const std::vector<float> &sourceFrequencies =
        !hfInterferenceLastFrequencies.empty() ? hfInterferenceLastFrequencies : spectrumFrequencyScratch;
    const std::vector<float> &sourceMagnitudes =
        !hfInterferenceLastMagnitudes.empty() ? hfInterferenceLastMagnitudes : spectrumMagnitudeScratch;
    if (sourceFrequencies.size() != sourceMagnitudes.size() ||
        sourceFrequencies.size() < 2) {
        qDebug() << "[HF interference] baseline learn skipped: no visible spectrum frame";
        updateHfNoiseCancelControls();
        return;
    }

    struct BaselinePoint {
        float frequency = 0.0f;
        float magnitude = 0.0f;
    };
    std::vector<BaselinePoint> points;
    points.reserve(sourceFrequencies.size());
    for (std::size_t i = 0; i < sourceFrequencies.size(); ++i) {
        const float frequency = sourceFrequencies[i];
        const float magnitude = sourceMagnitudes[i];
        if (std::isfinite(frequency) && std::isfinite(magnitude)) {
            points.push_back({frequency, magnitude});
        }
    }
    if (points.size() < 2) {
        qDebug() << "[HF interference] baseline learn skipped: spectrum frame has no finite bins";
        updateHfNoiseCancelControls();
        return;
    }

    std::sort(points.begin(), points.end(), [](const BaselinePoint &a, const BaselinePoint &b) {
        return a.frequency < b.frequency;
    });

    const int smoothBins = (std::max)(1, hfInterferenceBaselineSmoothBins | 1);
    const int radius = smoothBins / 2;
    std::vector<double> prefix(points.size() + 1, 0.0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        prefix[i + 1] = prefix[i] + points[i].magnitude;
    }

    hfInterferenceBaselineFrequencies.resize(points.size());
    hfInterferenceBaselineMagnitudes.resize(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
        const std::size_t begin = static_cast<std::size_t>((std::max)(0, static_cast<int>(i) - radius));
        const std::size_t end = (std::min)(points.size(), i + static_cast<std::size_t>(radius) + 1);
        const double count = static_cast<double>((std::max)(std::size_t(1), end - begin));
        hfInterferenceBaselineFrequencies[i] = points[i].frequency;
        hfInterferenceBaselineMagnitudes[i] = static_cast<float>((prefix[end] - prefix[begin]) / count);
    }
    hfInterferenceBaselineEnabled = true;
    if (hfInterferenceBaselineCheckbox) {
        QSignalBlocker blocker(hfInterferenceBaselineCheckbox);
        hfInterferenceBaselineCheckbox->setChecked(true);
    }
    updateHfNoiseCancelControls();
    savePersistentSettings();
    qDebug() << "[HF interference] baseline learned"
             << "bins" << hfInterferenceBaselineFrequencies.size()
             << "smoothBins" << smoothBins
             << "depth" << hfInterferenceBaselineDepth;
}

void YourClassName::clearHfInterferenceBaseline() {
    hfInterferenceBaselineFrequencies.clear();
    hfInterferenceBaselineMagnitudes.clear();
    hfInterferenceVisualMagnitudes.clear();
    hfInterferenceBaselineEnabled = false;
    if (hfInterferenceBaselineCheckbox) {
        QSignalBlocker blocker(hfInterferenceBaselineCheckbox);
        hfInterferenceBaselineCheckbox->setChecked(false);
    }
    updateHfNoiseCancelControls();
    savePersistentSettings();
    qDebug() << "[HF interference] baseline cleared";
}

bool YourClassName::buildHfInterferenceBaselineVisual(const std::vector<float> &frequencies,
                                                      const std::vector<float> &magnitudes,
                                                      std::vector<float> &outputMagnitudes) {
    if (!hfInterferenceBaselineEnabled ||
        hfInterferenceBaselineFrequencies.size() < 2 ||
        hfInterferenceBaselineFrequencies.size() != hfInterferenceBaselineMagnitudes.size() ||
        frequencies.size() != magnitudes.size() ||
        frequencies.empty()) {
        return false;
    }

    outputMagnitudes.resize(magnitudes.size());
    const float floorDb = displayLevelMin;
    const float ceilingDb = displayLevelMax;
    const float depth = static_cast<float>((std::clamp)(hfInterferenceBaselineDepth, 0.0, 2.0));
    const auto baselineBegin = hfInterferenceBaselineFrequencies.begin();
    const auto baselineEnd = hfInterferenceBaselineFrequencies.end();
    const bool ascendingFrequencies = frequencies.size() < 2 || frequencies.back() >= frequencies.front();
    std::size_t rollingBaselineIndex = 1;
    for (std::size_t i = 0; i < magnitudes.size(); ++i) {
        const float frequency = frequencies[i];
        const float magnitude = magnitudes[i];
        if (!std::isfinite(frequency) || !std::isfinite(magnitude)) {
            outputMagnitudes[i] = floorDb;
            continue;
        }

        float baseline = hfInterferenceBaselineMagnitudes.front();
        if (ascendingFrequencies) {
            while (rollingBaselineIndex < hfInterferenceBaselineFrequencies.size() &&
                   hfInterferenceBaselineFrequencies[rollingBaselineIndex] < frequency) {
                ++rollingBaselineIndex;
            }
            if (rollingBaselineIndex == 0 || frequency <= hfInterferenceBaselineFrequencies.front()) {
                baseline = hfInterferenceBaselineMagnitudes.front();
            } else if (rollingBaselineIndex >= hfInterferenceBaselineFrequencies.size()) {
                baseline = hfInterferenceBaselineMagnitudes.back();
            } else {
                const std::size_t upperIndex = rollingBaselineIndex;
                const std::size_t lowerIndex = upperIndex - 1;
                const float f0 = hfInterferenceBaselineFrequencies[lowerIndex];
                const float f1 = hfInterferenceBaselineFrequencies[upperIndex];
                const float m0 = hfInterferenceBaselineMagnitudes[lowerIndex];
                const float m1 = hfInterferenceBaselineMagnitudes[upperIndex];
                const float span = f1 - f0;
                const float ratio = span > 0.0f ? (frequency - f0) / span : 0.0f;
                baseline = m0 + (m1 - m0) * (std::clamp)(ratio, 0.0f, 1.0f);
            }
        } else {
            const auto it = std::lower_bound(baselineBegin, baselineEnd, frequency);
            if (it == baselineBegin) {
                baseline = hfInterferenceBaselineMagnitudes.front();
            } else if (it == baselineEnd) {
                baseline = hfInterferenceBaselineMagnitudes.back();
            } else {
                const std::size_t upperIndex = static_cast<std::size_t>(std::distance(baselineBegin, it));
                const std::size_t lowerIndex = upperIndex - 1;
                const float f0 = hfInterferenceBaselineFrequencies[lowerIndex];
                const float f1 = hfInterferenceBaselineFrequencies[upperIndex];
                const float m0 = hfInterferenceBaselineMagnitudes[lowerIndex];
                const float m1 = hfInterferenceBaselineMagnitudes[upperIndex];
                const float span = f1 - f0;
                const float ratio = span > 0.0f ? (frequency - f0) / span : 0.0f;
                baseline = m0 + (m1 - m0) * (std::clamp)(ratio, 0.0f, 1.0f);
            }
        }

        const float excess = magnitude - baseline;
        outputMagnitudes[i] = excess <= 0.0f
                                  ? floorDb
                                  : (std::clamp)(floorDb + excess * depth, floorDb, ceilingDb);
    }
    return true;
}

void YourClassName::updateSpectrum() {
    if (!fftResult) {
        return;
    }

    const bool channelIqRecordingOnly =
        isChannelIqRecordingActive() &&
        !(networkMode == NetworkMode::Server && isClientIqProcessingMode());
    if (channelIqRecordingOnly) {
        return;
    }

    const bool verboseLogging = fobosVerboseLoggingEnabled();
    if (!verboseLogging) {
        spectrumDebugFramesRemaining = 0;
        spectrumTuningDebugFramesRemaining = 0;
    }
    const bool traceFrame = verboseLogging && spectrumDebugFramesRemaining > 0;
    QElapsedTimer traceTimer;
    if (traceFrame) {
        traceTimer.start();
        qDebug() << "[Spectrum] update begin"
                 << "framesLeft" << spectrumDebugFramesRemaining
                 << "sampleRate" << pendingSettings.sampleRate
                 << "fftLength" << pendingSettings.fftLength
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
    }

    auto finishTrace = [&](const char *stage,
                           const std::vector<float> &frequencies,
                           const std::vector<float> &magnitudes) {
        if (!traceFrame) {
            return;
        }
        qDebug() << "[Spectrum] update" << stage
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << frequencies.size()
                 << "magCount" << magnitudes.size()
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
        --spectrumDebugFramesRemaining;
    };

    if (liveRetuneSettleTimer.isValid()) {
        const qint64 elapsedMs = liveRetuneSettleTimer.elapsed();
        const qint64 settleMs = liveRetuneSettleDurationMs > 0 ? liveRetuneSettleDurationMs : LIVE_RETUNE_SETTLE_MS;
        IqBuffer::clear();
        if (elapsedMs < settleMs) {
            if (traceFrame) {
                qDebug() << "[Spectrum] update retune_settle"
                         << "elapsedMs" << elapsedMs
                         << "settleMs" << settleMs
                         << "queuedBlocks" << IqBuffer::queuedBlocks();
                --spectrumDebugFramesRemaining;
            }
            return;
        }
        liveRetuneSettleTimer.invalidate();
        if (traceFrame) {
            qDebug() << "[Spectrum] update retune_settle_done"
                     << "elapsedMs" << elapsedMs
                     << "settleMs" << settleMs
                     << "queuedBlocks" << IqBuffer::queuedBlocks();
            --spectrumDebugFramesRemaining;
        }
        return;
    }

    std::vector<float> &spectrumFrequencies = spectrumFrequencyScratch;
    std::vector<float> &spectrumMagnitudes = spectrumMagnitudeScratch;
    std::vector<float> &referenceMagnitudes = spectrumReferenceScratch;
    bool haveSpectrum = false;
    RadioSettings spectrumSettings = spectrumProcessingSettings();

    int scanIndexBeforeFft = -1;
    int scanIndexAfterFft = -1;
    double scanCenterBeforeFft = std::numeric_limits<double>::quiet_NaN();
    double scanCenterAfterFft = std::numeric_limits<double>::quiet_NaN();
    IqBuffer::BlockMetadata fftBlockMetadata;
    QString scanVisualSource = QStringLiteral("none");
    if (agileScanRunning &&
        activeFobosApiKind == FobosApiKind::Agile &&
        agileDevice &&
        !activeAgileScanFrequencies.isEmpty()) {
        scanVisualSource = QStringLiteral("agile");
    } else if (standardScanRunning && !activeStandardScanFrequencies.isEmpty()) {
        scanVisualSource = QStringLiteral("standard");
        scanIndexBeforeFft = (std::clamp)(standardScanIndex,
                                          0,
                                          activeStandardScanFrequencies.size() - 1);
        scanCenterBeforeFft = activeStandardScanFrequencies.at(scanIndexBeforeFft);
    }

    double scanCenterFrequency = scanCenterBeforeFft;
    if (!std::isfinite(scanCenterFrequency) && scanVisualSource != QStringLiteral("agile")) {
        scanCenterFrequency = currentAgileScanCenterFrequencyHz();
        if (!std::isfinite(scanCenterFrequency)) {
            scanCenterFrequency = currentStandardScanCenterFrequencyHz();
        }
    }
    if (std::isfinite(scanCenterFrequency) && spectrumSettings.inputMode == INPUT_RF) {
        spectrumSettings.centerFrequency = scanCenterFrequency;
        spectrumSettings.actualFrequency = scanCenterFrequency;
    }
    processGnssIqSnapshot(spectrumSettings);
    double fullMinFrequency = minFrequency;
    double fullMaxFrequency = maxFrequency;
    if (spectrumSettings.inputMode == INPUT_RF && spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = spectrumSettings.centerFrequency - spectrumSettings.sampleRate * 0.5;
        fullMaxFrequency = spectrumSettings.centerFrequency + spectrumSettings.sampleRate * 0.5;
    } else if (spectrumSettings.inputMode == INPUT_HF_COMBINED && spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = -spectrumSettings.sampleRate * 0.5;
        fullMaxFrequency = spectrumSettings.sampleRate * 0.5;
    } else if (spectrumSettings.sampleRate > 0.0) {
        fullMinFrequency = 0.0;
        fullMaxFrequency = spectrumSettings.sampleRate * 0.5;
    }
    if (!std::isfinite(fullMinFrequency) ||
        !std::isfinite(fullMaxFrequency) ||
        fullMaxFrequency <= fullMinFrequency) {
        fullMinFrequency = minFrequency;
        fullMaxFrequency = maxFrequency;
    }

    const double fullSpan = (std::max)(1.0, fullMaxFrequency - fullMinFrequency);
    double visibleSpan = spectrumSettings.sampleRate * (currentScale / 100.0);
    if (!std::isfinite(visibleSpan) || visibleSpan <= 0.0) {
        visibleSpan = fullSpan;
    }
    visibleSpan = (std::clamp)(visibleSpan, 1.0, fullSpan);

    double visibleCenter = pendingSettings.listeningFrequency;
    if (!std::isfinite(visibleCenter) ||
        visibleCenter < fullMinFrequency ||
        visibleCenter > fullMaxFrequency) {
        visibleCenter = std::isfinite(scanCenterFrequency)
                            ? scanCenterFrequency
                            : (fullMinFrequency + fullMaxFrequency) * 0.5;
    }
    visibleCenter = (std::clamp)(visibleCenter, fullMinFrequency, fullMaxFrequency);

    double frameMinFrequency = visibleCenter - visibleSpan * 0.5;
    frameMinFrequency = (std::clamp)(frameMinFrequency,
                                     fullMinFrequency,
                                     fullMaxFrequency - visibleSpan);
    double frameMaxFrequency = frameMinFrequency + visibleSpan;

    const bool spectrumWorkerAllowed =
        scanVisualSource == QStringLiteral("none") &&
        !networkSpectrumFrameMetadataValid;

    if (spectrumWorkerAllowed) {
        if (!spectrumFftWorker) {
            spectrumFftWorker = std::make_unique<SpectrumFftWorker>();
            spectrumFftWorker->request(spectrumSettings);
            finishTrace("fft_worker_start", spectrumFrequencies, spectrumMagnitudes);
            return;
        }

        SpectrumFftFrame frame;
        if (!spectrumFftWorker->takeLatest(frame)) {
            spectrumFftWorker->request(spectrumSettings);
            finishTrace("fft_worker_pending", spectrumFrequencies, spectrumMagnitudes);
            return;
        }

        if (frame.badAlloc) {
            qCritical() << "[Spectrum] worker bad_alloc" << frame.error
                        << "sampleRate" << pendingSettings.sampleRate
                        << "fftLength" << pendingSettings.fftLength;
            updateTimer->stop();
            finishTrace("fft_worker_bad_alloc", spectrumFrequencies, spectrumMagnitudes);
            return;
        }

        if (!frame.error.isEmpty()) {
            qCritical() << "[Spectrum] worker exception" << frame.error;
            spectrumFftWorker->request(spectrumSettings);
            finishTrace("fft_worker_exception", spectrumFrequencies, spectrumMagnitudes);
            return;
        }

        if (!frame.valid || !spectrumFftSettingsMatch(frame.settings, spectrumSettings)) {
            spectrumFftWorker->request(spectrumSettings);
            finishTrace(frame.valid ? "fft_worker_stale" : "fft_worker_no_data",
                        spectrumFrequencies,
                        spectrumMagnitudes);
            return;
        }

        spectrumFrequencies = std::move(frame.frequencies);
        spectrumMagnitudes = std::move(frame.magnitudes);
        referenceMagnitudes = std::move(frame.referenceMagnitudes);
        fftBlockMetadata = frame.metadata;
        haveSpectrum = true;
        spectrumFftWorker->request(spectrumSettings);
    } else {
        try {
            haveSpectrum = fftResult->storeFFTResults(spectrumSettings,
                                                      spectrumFrequencies,
                                                      spectrumMagnitudes,
                                                      &referenceMagnitudes,
                                                      &fftBlockMetadata);
        } catch (const std::bad_alloc &error) {
            qCritical() << "[Spectrum] bad_alloc" << error.what()
                        << "sampleRate" << pendingSettings.sampleRate
                        << "fftLength" << pendingSettings.fftLength;
            updateTimer->stop();
            finishTrace("bad_alloc", spectrumFrequencies, spectrumMagnitudes);
            return;
        } catch (const std::exception &error) {
            qCritical() << "[Spectrum] exception" << error.what();
            finishTrace("exception", spectrumFrequencies, spectrumMagnitudes);
            return;
        } catch (...) {
            qCritical() << "[Spectrum] unknown exception";
            finishTrace("unknown_exception", spectrumFrequencies, spectrumMagnitudes);
            return;
        }
    }

    if (!haveSpectrum || spectrumFrequencies.empty() || spectrumMagnitudes.empty()) {
        finishTrace("no_data", spectrumFrequencies, spectrumMagnitudes);
        return;
    }

    if (scanVisualSource == QStringLiteral("agile") &&
        fftBlockMetadata.valid &&
        fftBlockMetadata.scanIndex >= 0 &&
        fftBlockMetadata.scanIndex < activeAgileScanFrequencies.size()) {
        scanIndexBeforeFft = fftBlockMetadata.scanIndex;
        scanIndexAfterFft = fftBlockMetadata.scanIndex;
        scanCenterBeforeFft = fftBlockMetadata.centerFrequencyHz;
        scanCenterAfterFft = fftBlockMetadata.centerFrequencyHz;
        scanCenterFrequency = fftBlockMetadata.centerFrequencyHz;
        if (std::isfinite(scanCenterFrequency) && spectrumSettings.inputMode == INPUT_RF) {
            spectrumSettings.centerFrequency = scanCenterFrequency;
            spectrumSettings.actualFrequency = scanCenterFrequency;
        }
    } else if (scanVisualSource == QStringLiteral("standard") &&
               !activeStandardScanFrequencies.isEmpty()) {
        scanIndexAfterFft = (std::clamp)(standardScanIndex,
                                         0,
                                         activeStandardScanFrequencies.size() - 1);
        scanCenterAfterFft = activeStandardScanFrequencies.at(scanIndexAfterFft);
    }

    updateSpurCalibration(spectrumFrequencies, spectrumMagnitudes, spectrumSettings.centerFrequency);
    applySpurSuppression(spectrumFrequencies, spectrumMagnitudes, spectrumSettings.centerFrequency);
    updateGnssSpurWatch(spectrumFrequencies, spectrumMagnitudes, spectrumSettings.centerFrequency);
    updateScanMeasurement(spectrumFrequencies, spectrumMagnitudes);

    const std::vector<float> *displayFrequenciesPtr = &spectrumFrequencies;
    const std::vector<float> *displayMagnitudesPtr = &spectrumMagnitudes;
    const std::vector<float> *displayReferenceMagnitudesPtr = &referenceMagnitudes;
    const std::vector<float> *displayMeasurementFrequenciesPtr = &spectrumFrequencies;
    const std::vector<float> *dmrHunterFrequenciesPtr = &spectrumFrequencies;
    const std::vector<float> *dmrHunterMagnitudesPtr = &spectrumMagnitudes;
    const std::vector<float> *fpvHunterFrequenciesPtr = &spectrumFrequencies;
    const std::vector<float> *fpvHunterMagnitudesPtr = &spectrumMagnitudes;
    const std::vector<float> *digitalVideoHunterFrequenciesPtr = &spectrumFrequencies;
    const std::vector<float> *digitalVideoHunterMagnitudesPtr = &spectrumMagnitudes;
    double displayCenterFrequency = spectrumSettings.centerFrequency;
    double displayMinFrequency = frameMinFrequency;
    double displayMaxFrequency = frameMaxFrequency;
    int displayFftLength = static_cast<int>(spectrumFrequencies.size());
    QVector<ScanVisualSegment> displayScanSegments;
    bool displayScanSegmentMarkers = false;
    bool updateWaterfallFrame = true;
    bool scanVisualFrameValid = false;
    bool scanVisualWindowed = false;
    int scanVisualSelectedIndex = -1;
    double scanVisualSelectedCenter = std::numeric_limits<double>::quiet_NaN();
    double scanVisualFrameMin = std::numeric_limits<double>::quiet_NaN();
    double scanVisualFrameMax = std::numeric_limits<double>::quiet_NaN();
    ScanVisualFrame scanFrame;
    ScanVisualFrame visibleScanFrame;
    const bool agileScanVisualActive =
        agileScanRunning &&
        activeFobosApiKind == FobosApiKind::Agile &&
        spectrumSettings.inputMode == INPUT_RF &&
        activeAgileScanFrequencies.size() > 1;
    const bool standardScanVisualActive =
        standardScanRunning &&
        spectrumSettings.inputMode == INPUT_RF &&
        activeStandardScanFrequencies.size() > 1;
    const bool scanVisualActive = agileScanVisualActive || standardScanVisualActive;
    const QVector<double> &scanVisualFrequencies =
        standardScanVisualActive ? activeStandardScanFrequencies : activeAgileScanFrequencies;
    if (scanVisualActive) {
        scanVisualSelectedIndex =
            nearestScanFrequencyIndex(scanVisualFrequencies, spectrumSettings.centerFrequency);
        if (scanVisualSelectedIndex >= 0 && scanVisualSelectedIndex < scanVisualFrequencies.size()) {
            scanVisualSelectedCenter = scanVisualFrequencies.at(scanVisualSelectedIndex);
        }
        int scanVisualBins = 4096;
        if (graphWidget && graphWidget->width() > 0) {
            scanVisualBins = (std::max)(scanVisualBins, graphWidget->width() * 2);
        }
        if (waterfallWidget && waterfallWidget->width() > 0) {
            scanVisualBins = (std::max)(scanVisualBins, waterfallWidget->width() * 2);
        }
        if (scanVisualAssembler.configure(scanVisualFrequencies,
                                          spectrumSettings.sampleRate,
                                          scanVisualBins,
                                          scanVisualModeFromInt(scanVisualMode))) {
            scanFrame = scanVisualAssembler.update(spectrumSettings.centerFrequency,
                                                   spectrumFrequencies,
                                                   spectrumMagnitudes,
                                                   referenceMagnitudes);
            if (scanFrame.valid) {
                scanVisualFrameValid = true;
                scanVisualFrameMin = scanFrame.minFrequency;
                scanVisualFrameMax = scanFrame.maxFrequency;
                const double scanFullSpanHz = scanFrame.maxFrequency - scanFrame.minFrequency;
                const double scanVisibleSpanHz =
                    std::isfinite(scanFullSpanHz) && scanFullSpanHz > 0.0
                        ? scanFullSpanHz * (currentScale / 100.0)
                        : scanFullSpanHz;
                const double scanVisibleCenterHz =
                    displayFrequencyForScanActual(pendingSettings.listeningFrequency,
                                                  scanFrame.segments,
                                                  scanFrame.centerFrequency);
                const ScanVisualFrame *displayScanFrame = &scanFrame;
                if (std::isfinite(scanFullSpanHz) &&
                    scanFullSpanHz > 0.0 &&
                    scanVisibleSpanHz < scanFullSpanHz * 0.999) {
                    visibleScanFrame =
                        windowedScanVisualFrame(scanFrame, scanVisibleCenterHz, scanVisibleSpanHz);
                    displayScanFrame = &visibleScanFrame;
                    scanVisualWindowed = true;
                }

                displayFrequenciesPtr = &displayScanFrame->frequencies;
                displayMagnitudesPtr = &displayScanFrame->magnitudes;
                displayReferenceMagnitudesPtr = &displayScanFrame->referenceMagnitudes;
                displayMeasurementFrequenciesPtr = &displayScanFrame->actualFrequencies;
                displayCenterFrequency = displayScanFrame->centerFrequency;
                displayMinFrequency = displayScanFrame->minFrequency;
                displayMaxFrequency = displayScanFrame->maxFrequency;
                displayFftLength = displayScanFrame->fftLength;
                displayScanSegments = displayScanFrame->segments;
                displayScanSegmentMarkers = displayScanFrame->showSegmentMarkers;
                updateWaterfallFrame = displayScanFrame->fresh;
                if (displayScanFrame->actualFrequencies.size() == displayScanFrame->magnitudes.size()) {
                    dmrHunterFrequenciesPtr = &displayScanFrame->actualFrequencies;
                    dmrHunterMagnitudesPtr = &displayScanFrame->magnitudes;
                    fpvHunterFrequenciesPtr = &displayScanFrame->actualFrequencies;
                    fpvHunterMagnitudesPtr = &displayScanFrame->magnitudes;
                    digitalVideoHunterFrequenciesPtr = &displayScanFrame->actualFrequencies;
                    digitalVideoHunterMagnitudesPtr = &displayScanFrame->magnitudes;
                }
            }
        }
    } else {
        scanVisualAssembler.reset();
    }

    if (!scanVisualActive || !verboseLogging) {
        scanVisualDebugFramesRemaining = 0;
        scanVisualDebugSequence = 0;
    } else {
        if (scanVisualDebugFramesRemaining <= 0 && scanVisualDebugSequence == 0) {
            scanVisualDebugFramesRemaining = 48;
            qDebug() << "[ScanVisualDebug] capture begin"
                     << "source" << scanVisualSource
                     << "mode" << normalizedScanVisualMode(scanVisualMode)
                     << "points" << scanVisualFrequencies.size()
                     << "sampleRate" << spectrumSettings.sampleRate
                     << "fftLength" << pendingSettings.fftLength
                     << "rowsPerFrame" << waterfallRowsPerFrame
                     << "updateMs" << updateTimer->interval();
        }
        if (scanVisualDebugFramesRemaining > 0) {
            const IqBuffer::Stats iqStats = IqBuffer::stats();
            const double inputFirstHz =
                spectrumFrequencies.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : static_cast<double>(spectrumFrequencies.front());
            const double inputLastHz =
                spectrumFrequencies.empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : static_cast<double>(spectrumFrequencies.back());
            const double displayFirstHz =
                displayFrequenciesPtr->empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : static_cast<double>(displayFrequenciesPtr->front());
            const double displayLastHz =
                displayFrequenciesPtr->empty()
                    ? std::numeric_limits<double>::quiet_NaN()
                    : static_cast<double>(displayFrequenciesPtr->back());
            qDebug() << "[ScanVisualDebug] row"
                     << "seq" << static_cast<qulonglong>(++scanVisualDebugSequence)
                     << "source" << scanVisualSource
                     << "idxBefore0" << scanIndexBeforeFft
                     << "idxAfter0" << scanIndexAfterFft
                     << "idxBefore1" << (scanIndexBeforeFft >= 0 ? scanIndexBeforeFft + 1 : -1)
                     << "idxAfter1" << (scanIndexAfterFft >= 0 ? scanIndexAfterFft + 1 : -1)
                     << "centerBeforeMHz" << (scanCenterBeforeFft / 1000000.0)
                     << "centerAfterMHz" << (scanCenterAfterFft / 1000000.0)
                     << "centerUsedMHz" << (spectrumSettings.centerFrequency / 1000000.0)
                     << "metaValid" << fftBlockMetadata.valid
                     << "metaTuning" << fftBlockMetadata.tuning
                     << "metaIndex0" << fftBlockMetadata.scanIndex
                     << "metaIndex1" << (fftBlockMetadata.scanIndex >= 0 ? fftBlockMetadata.scanIndex + 1 : -1)
                     << "metaCenterMHz" << (fftBlockMetadata.centerFrequencyHz / 1000000.0)
                     << "metaSeq" << static_cast<qulonglong>(fftBlockMetadata.sequence)
                     << "selected0" << scanVisualSelectedIndex
                     << "selected1" << (scanVisualSelectedIndex >= 0 ? scanVisualSelectedIndex + 1 : -1)
                     << "selectedMHz" << (scanVisualSelectedCenter / 1000000.0)
                     << "frameValid" << scanVisualFrameValid
                     << "fresh" << updateWaterfallFrame
                     << "windowed" << scanVisualWindowed
                     << "inputFirstMHz" << (inputFirstHz / 1000000.0)
                     << "inputLastMHz" << (inputLastHz / 1000000.0)
                     << "displayFirstMHz" << (displayFirstHz / 1000000.0)
                     << "displayLastMHz" << (displayLastHz / 1000000.0)
                     << "frameMinMHz" << (scanVisualFrameMin / 1000000.0)
                     << "frameMaxMHz" << (scanVisualFrameMax / 1000000.0)
                     << "iqSeq" << static_cast<qulonglong>(iqStats.sequence)
                     << "iqSnapshotFloats" << iqStats.snapshotSize
                     << "queuedBlocks" << iqStats.queuedBlocks;
            --scanVisualDebugFramesRemaining;
            if (scanVisualDebugFramesRemaining == 0) {
                qDebug() << "[ScanVisualDebug] capture end"
                         << "rows" << static_cast<qulonglong>(scanVisualDebugSequence);
            }
        }
    }
    const std::vector<float> &displayFrequencies = *displayFrequenciesPtr;
    const std::vector<float> &displayMagnitudes = *displayMagnitudesPtr;
    const std::vector<float> &displayReferenceMagnitudes = *displayReferenceMagnitudesPtr;
    const std::vector<float> &displayMeasurementFrequencies = *displayMeasurementFrequenciesPtr;
    const std::vector<float> &dmrHunterFrequencies = *dmrHunterFrequenciesPtr;
    const std::vector<float> &dmrHunterMagnitudes = *dmrHunterMagnitudesPtr;
    const std::vector<float> &fpvHunterFrequencies = *fpvHunterFrequenciesPtr;
    const std::vector<float> &fpvHunterMagnitudes = *fpvHunterMagnitudesPtr;
    const std::vector<float> &digitalVideoHunterFrequencies = *digitalVideoHunterFrequenciesPtr;
    const std::vector<float> &digitalVideoHunterMagnitudes = *digitalVideoHunterMagnitudesPtr;
    updateDmrHunter(dmrHunterFrequencies, dmrHunterMagnitudes);
    updateFpvHunter(fpvHunterFrequencies, fpvHunterMagnitudes);
    updateDigitalVideoHunter(digitalVideoHunterFrequencies, digitalVideoHunterMagnitudes);

    const std::vector<float> *visualMagnitudesPtr = &displayMagnitudes;
    if (isDirectInputMode(spectrumSettings.inputMode) &&
        buildHfInterferenceBaselineVisual(displayFrequencies,
                                          displayMagnitudes,
                                          hfInterferenceVisualMagnitudes)) {
        visualMagnitudesPtr = &hfInterferenceVisualMagnitudes;
    }
    const std::vector<float> &visualMagnitudes = *visualMagnitudesPtr;
    if ((spectrumFrameBufferEnabled || spectrumFrameRecorder.isRecording()) &&
        updateWaterfallFrame &&
        !displayFrequencies.empty() &&
        displayFrequencies.size() == visualMagnitudes.size()) {
        const int bins = spectrumFrameBinsCombo ? spectrumFrameBinsCombo->currentData().toInt() : 4096;
        SpectrumFrameRecord frame =
            SpectrumFrameRecorder::makeFrame(displayFrequencies,
                                             visualMagnitudes,
                                             displayCenterFrequency,
                                             displayMinFrequency,
                                             displayMaxFrequency,
                                             displayFftLength,
                                             bins,
                                             QDateTime::currentMSecsSinceEpoch());
        if (!frame.magnitudes.empty()) {
            if (spectrumFrameBufferEnabled) {
                spectrumFramePrebuffer.push_back(frame);
                const qint64 keepMs = static_cast<qint64>((std::max)(1, spectrumFramePrebufferSeconds)) * 1000;
                while (!spectrumFramePrebuffer.empty() &&
                       frame.utcMs - spectrumFramePrebuffer.front().utcMs > keepMs) {
                    spectrumFramePrebuffer.pop_front();
                }
            }
            if (spectrumFrameRecorder.isRecording()) {
                if (!spectrumFrameRecorder.appendFrameRecord(frame)) {
                    stopSpectrumFrameRecording();
                    updateSpectrumFrameRecordingStatus(QStringLiteral("Spectrum recording failed: write error"));
                } else if ((spectrumFrameRecorder.frameCount() % 100) == 0) {
                    updateSpectrumFrameRecordingStatus(
                        QStringLiteral("Spectrum recording: %1 frames").arg(spectrumFrameRecorder.frameCount()));
                }
            } else if (spectrumFrameBufferEnabled && (spectrumFramePrebuffer.size() % 100) == 0) {
                updateSpectrumFrameRecordingStatus(
                    QStringLiteral("Spectrum frames: buffering %1 frames").arg(spectrumFramePrebuffer.size()));
            }
        }
    }

    if (spectrumTuningDebugFramesRemaining > 0 &&
        displayFrequencies.size() == displayMagnitudes.size() &&
        !displayFrequencies.empty()) {
        --spectrumTuningDebugFramesRemaining;

        std::array<int, 3> peakIndices = {-1, -1, -1};
        std::array<float, 3> peakLevels = {
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity(),
            -std::numeric_limits<float>::infinity()};
        int listeningIndex = -1;
        double listeningDelta = std::numeric_limits<double>::max();
        for (int i = 0; i < static_cast<int>(displayFrequencies.size()); ++i) {
            const double frequency = displayFrequencies[static_cast<std::size_t>(i)];
            const float level = displayMagnitudes[static_cast<std::size_t>(i)];
            if (!std::isfinite(frequency) || !std::isfinite(level)) {
                continue;
            }

            const double delta = std::abs(frequency - pendingSettings.listeningFrequency);
            if (delta < listeningDelta) {
                listeningDelta = delta;
                listeningIndex = i;
            }

            for (int slot = 0; slot < 3; ++slot) {
                if (level <= peakLevels[static_cast<std::size_t>(slot)]) {
                    continue;
                }
                for (int move = 2; move > slot; --move) {
                    peakLevels[static_cast<std::size_t>(move)] = peakLevels[static_cast<std::size_t>(move - 1)];
                    peakIndices[static_cast<std::size_t>(move)] = peakIndices[static_cast<std::size_t>(move - 1)];
                }
                peakLevels[static_cast<std::size_t>(slot)] = level;
                peakIndices[static_cast<std::size_t>(slot)] = i;
                break;
            }
        }

        QStringList peakSummary;
        for (int slot = 0; slot < 3; ++slot) {
            const int index = peakIndices[static_cast<std::size_t>(slot)];
            if (index < 0) {
                continue;
            }
            const double frequency = displayFrequencies[static_cast<std::size_t>(index)];
            const double offset = frequency - displayCenterFrequency;
            peakSummary << QStringLiteral("%1MHz/%2kHz/%3dB")
                               .arg(frequency / 1000000.0, 0, 'f', 6)
                               .arg(offset / 1000.0, 0, 'f', 1)
                               .arg(peakLevels[static_cast<std::size_t>(slot)], 0, 'f', 1);
        }

        const float listeningLevel =
            listeningIndex >= 0
                ? displayMagnitudes[static_cast<std::size_t>(listeningIndex)]
                : std::numeric_limits<float>::quiet_NaN();
        qDebug() << "[SpectrumTune]"
                 << "center" << displayCenterFrequency
                 << "listening" << pendingSettings.listeningFrequency
                 << "actual" << pendingSettings.actualFrequency
                 << "range" << displayMinFrequency << displayMaxFrequency
                 << "sampleRate" << spectrumSettings.sampleRate
                 << "listenOffset" << (pendingSettings.listeningFrequency - displayCenterFrequency)
                 << "listeningBinDelta" << listeningDelta
                 << "listeningLevel" << listeningLevel
                 << "peaks" << peakSummary.join(QStringLiteral(", "));
    }

    if (traceFrame) {
        qDebug() << "[Spectrum] before graph"
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << displayFrequencies.size()
                 << "magCount" << displayMagnitudes.size()
                 << "scanVisual" << scanVisualActive;
    }
    const bool suppressLocalVisual =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!suppressLocalVisual) {
        if (scaleWidget) {
            double scaleListening = pendingSettings.listeningFrequency;
            if (!displayScanSegments.isEmpty()) {
                if (!scanListeningLockEnabled &&
                    !actualFrequencyInsideScanSegments(scaleListening, displayScanSegments)) {
                    scaleListening =
                        fallbackActualFrequencyForScanSegments(displayScanSegments, displayCenterFrequency);
                }
            } else if (scaleListening < displayMinFrequency ||
                       scaleListening > displayMaxFrequency) {
                scaleListening = displayCenterFrequency;
            }
            scaleWidget->setScanSegments(displayScanSegments);
            scaleWidget->setScanSegmentMarkersVisible(displayScanSegmentMarkers);
            scaleWidget->setTuning(scaleListening,
                                   displayCenterFrequency,
                                   pendingSettings.bandwidth,
                                   pendingSettings.modulationType);
            scaleWidget->setRange(displayMinFrequency, displayMaxFrequency);
        }
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setScanSegments(displayScanSegments);
        graphWidget->setScanSegmentMarkersVisible(displayScanSegmentMarkers);
        graphWidget->setData(displayFrequencies,
                             visualMagnitudes,
                             displayMinFrequency,
                             displayMaxFrequency,
                             displayFftLength,
                             colorf);
        const std::vector<float> &measurementFrequencies =
            displayMeasurementFrequencies.size() == displayMagnitudes.size()
                ? displayMeasurementFrequencies
                : displayFrequencies;
        const std::vector<float> measurementOverlay =
            scanMeasurementOverlay(measurementFrequencies, static_cast<int>(displayMagnitudes.size()));
        graphWidget->setOverlayData(!measurementOverlay.empty() ? measurementOverlay : displayReferenceMagnitudes,
                                    !measurementOverlay.empty() ||
                                        (pendingSettings.inputMode == INPUT_HF_NOISE_CANCEL &&
                                         !displayReferenceMagnitudes.empty()));
        if (traceFrame) {
            qDebug() << "[Spectrum] before waterfall" << "elapsedMs" << traceTimer.elapsed();
        }
        if (updateWaterfallFrame) {
            waterfallWidget->setData(displayFrequencies,
                                     visualMagnitudes,
                                     displayMinFrequency,
                                     displayMaxFrequency,
                                     displayFftLength,
                                     secondGraph,
                                     contrast,
                                     sensitivity,
                                     displayLevelMin,
                                     displayLevelMax);
        }
        waterfallWidget->setScanSegments(displayScanSegments);
        waterfallWidget->setScanSegmentMarkersVisible(displayScanSegmentMarkers);
    } else if (traceFrame) {
        qDebug() << "[Spectrum] local server visual update skipped" << "elapsedMs" << traceTimer.elapsed();
    }
    sendNetworkSpectrumFrame(displayFrequencies,
                             displayMagnitudes,
                             displayReferenceMagnitudes,
                             displayCenterFrequency,
                             displayMinFrequency,
                             displayMaxFrequency,
                             displayScanSegments,
                             updateWaterfallFrame,
                             displayScanSegmentMarkers);
    finishTrace("end", displayFrequencies, displayMagnitudes);
    advanceStandardScanIfNeeded();
}
