#include "main.h"

#include "appconstants.h"
#include "dmrbackendpaths.h"
#include "dmrprivacyutils.h"
#include "dsdneobridge.h"
#include "gophertrunkbridge.h"
#include "tuningutils.h"

#include <QAbstractItemView>
#include <QCoreApplication>
#include <QDebug>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFileInfo>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QScopeGuard>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <utility>
#include <vector>

int YourClassName::selectedDmrBackend() const {
    if (!dmrBackendCombo) {
        return DMR_BACKEND_FOBOS_MBELIB;
    }
    const int backend = dmrBackendCombo->currentData().toInt();
    switch (backend) {
    case DMR_BACKEND_FOBOS_MBELIB:
    case DMR_BACKEND_FOBOS_OPENDMR:
    case DMR_BACKEND_DSD_NEO:
    case DMR_BACKEND_GOPHERTRUNK:
        return backend;
    default:
        return DMR_BACKEND_FOBOS_MBELIB;
    }
}

QStringList YourClassName::dsdNeoProcessArguments() const {
    const int sampleRate =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    const quint16 inputPort =
        dsdNeoInputPortSpin ? static_cast<quint16>(dsdNeoInputPortSpin->value()) : quint16(7355);
    const quint16 outputPort =
        dsdNeoUdpOutputPortSpin ? static_cast<quint16>(dsdNeoUdpOutputPortSpin->value()) : quint16(23456);

    return {
        QStringLiteral("-fs"),
        QStringLiteral("-i"),
        QStringLiteral("udp:127.0.0.1:%1").arg(inputPort),
        QStringLiteral("-s"),
        QString::number(sampleRate),
        QStringLiteral("-o"),
        QStringLiteral("udp:127.0.0.1:%1").arg(outputPort),
        QStringLiteral("-mc"),
        QStringLiteral("-nm")
    };
}

void YourClassName::updateDsdNeoBridgeSettings() {
    if (!dsdNeoBridge) {
        return;
    }

    const bool enabled = selectedDmrBackend() == DMR_BACKEND_DSD_NEO;
    bool autoStart = dsdNeoAutoStartCheckbox && dsdNeoAutoStartCheckbox->isChecked();
    const quint16 inputPort =
        dsdNeoInputPortSpin ? static_cast<quint16>(dsdNeoInputPortSpin->value()) : quint16(7355);
    const quint16 outputPort =
        dsdNeoUdpOutputPortSpin ? static_cast<quint16>(dsdNeoUdpOutputPortSpin->value()) : quint16(23456);
    const QString rawProgram =
        dsdNeoProgramEdit ? dsdNeoProgramEdit->text().trimmed() : QString();
    QString program = rawProgram.isEmpty() ? defaultDsdNeoProgramPath() : rawProgram;
    const QString bundledProgram =
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(defaultDsdNeoProgramPath());
    if ((rawProgram.isEmpty() || isLegacyDsdNeoProgramName(rawProgram)) &&
        QFileInfo::exists(bundledProgram)) {
        program = bundledProgram;
        if (dsdNeoProgramEdit && dsdNeoProgramEdit->text().trimmed() != defaultDsdNeoProgramPath()) {
            const QSignalBlocker blocker(dsdNeoProgramEdit);
            dsdNeoProgramEdit->setText(defaultDsdNeoProgramPath());
        }
        if (enabled && !autoStart && dsdNeoAutoStartCheckbox) {
            const QSignalBlocker blocker(dsdNeoAutoStartCheckbox);
            dsdNeoAutoStartCheckbox->setChecked(true);
            autoStart = true;
        }
    }
    QFileInfo programInfo(program);
    if (!programInfo.isAbsolute() &&
        (program.contains(QLatin1Char('/')) || program.contains(QLatin1Char('\\')))) {
        program = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(program);
        programInfo.setFile(program);
    }
    const QString workingDirectory =
        programInfo.isAbsolute() ? programInfo.absolutePath() : QString();

    if (dsdNeoAutoStartCheckbox) {
        dsdNeoAutoStartCheckbox->setEnabled(enabled);
    }
    if (dsdNeoProgramEdit) {
        dsdNeoProgramEdit->setEnabled(enabled && autoStart);
    }
    if (dsdNeoInputPortSpin) {
        dsdNeoInputPortSpin->setEnabled(enabled);
    }
    if (dsdNeoUdpOutputPortSpin) {
        dsdNeoUdpOutputPortSpin->setEnabled(enabled);
    }

    dsdNeoBridge->configureInputServer(true, inputPort);
    dsdNeoBridge->configureUdpOutput(true, outputPort, 2);
    dsdNeoBridge->configurePrivacy(dmrPrivacyModeId(pendingSettings.dmrPrivacyMode),
                                   pendingSettings.dmrPrivacyKeyId,
                                   pendingSettings.dmrPrivacyKeyHex,
                                   pendingSettings.dmrPrivacyForwardToBackends);
    dsdNeoBridge->configureProcess(autoStart, program, dsdNeoProcessArguments(), workingDirectory);
    dsdNeoBridge->setEnabled(enabled);

    if (dsdNeoStatusLabel && !enabled) {
        dsdNeoStatusLabel->setText(uiText(QStringLiteral("dsd_neo_idle"),
                                          QStringLiteral("DSD-neo bridge idle")));
    }
}

void YourClassName::updateGopherTrunkBridgeSettings() {
    if (!gopherTrunkBridge) {
        return;
    }

    const bool enabled = selectedDmrBackend() == DMR_BACKEND_GOPHERTRUNK;
    const quint16 outputPort =
        dsdNeoUdpOutputPortSpin ? static_cast<quint16>(dsdNeoUdpOutputPortSpin->value()) : quint16(23456);
    QString program = QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(defaultGopherTrunkProgramPath());
    QFileInfo programInfo(program);
    const QString workingDirectory = programInfo.absolutePath();
    gopherTrunkBridge->configure(true, program, quint16(7460), outputPort, workingDirectory);
    gopherTrunkBridge->configurePrivacy(dmrPrivacyModeId(pendingSettings.dmrPrivacyMode),
                                        pendingSettings.dmrPrivacyKeyId,
                                        pendingSettings.dmrPrivacyKeyHex,
                                        pendingSettings.dmrPrivacyForwardToBackends,
                                        pendingSettings.dmrPrivacyVariant,
                                        pendingSettings.dmrPrivacyLayout,
                                        pendingSettings.dmrPrivacyFrameOffset);
    gopherTrunkBridge->setEnabled(enabled);
    if (dsdNeoStatusLabel && !enabled && selectedDmrBackend() != DMR_BACKEND_DSD_NEO) {
        dsdNeoStatusLabel->setText(uiText(QStringLiteral("dsd_neo_idle"),
                                          QStringLiteral("DSD-neo bridge idle")));
    }
}

void YourClassName::processDigitalAudioFrame(const QByteArray &pcmData, int sampleRate) {
    if (!digitalDecoder ||
        !digitalDecoderThread ||
        !digitalDecodeEnabled ||
        !digitalDecodeCheckbox ||
        !digitalDecodeCheckbox->isChecked()) {
        return;
    }
    if (sampleRate <= 0) {
        sampleRate = 48000;
    }

    RadioSettings settings = pendingSettings;
    settings.dmrLabEnabled = dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked();
    settings.dmrLabColorCode = dmrLabColorCodeCombo
                                   ? dmrLabColorCodeCombo->currentData().toInt()
                                   : -1;
    settings.dmrLabTimeslot = dmrLabSlotCombo
                                  ? dmrLabSlotCombo->currentData().toInt()
                                  : 0;
    settings.dmrBasebandSampleRate =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    if (dmrChannelRateCombo) {
        settings.dmrChannelSampleRate = dmrChannelRateCombo->currentData().toInt();
    }
    settings.dmrAmbeLayout =
        dmrAmbeLayoutCombo
            ? normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt())
            : normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    settings.dmrManualTimingEnabled =
        dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    settings.dmrManualTimingOffset =
        dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : settings.dmrManualTimingOffset;
    settings.dmrSlicerRatio =
        dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : settings.dmrSlicerRatio;
    settings.dmrAdaptiveSlicer =
        !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    settings.dmrPrivacyMode = pendingSettings.dmrPrivacyMode;
    settings.dmrPrivacyKeyId = pendingSettings.dmrPrivacyKeyId;
    settings.dmrPrivacyKeyHex = pendingSettings.dmrPrivacyKeyHex;
    settings.dmrPrivacyForwardToBackends = pendingSettings.dmrPrivacyForwardToBackends;
    settings.dmrPrivacyVariant = pendingSettings.dmrPrivacyVariant;
    settings.dmrPrivacyLayout = pendingSettings.dmrPrivacyLayout;
    settings.dmrPrivacyFrameOffset = pendingSettings.dmrPrivacyFrameOffset;
    const auto parseDmrLabId = [](const QLineEdit *edit) {
        if (!edit) {
            return 0;
        }
        bool ok = false;
        const int value = edit->text().trimmed().toInt(&ok);
        return ok && value > 0 ? value : 0;
    };
    settings.dmrLabSourceId = parseDmrLabId(dmrLabSourceIdEdit);
    settings.dmrLabTargetId = parseDmrLabId(dmrLabTargetIdEdit);
    const bool isDmr = settings.modulationType == MOD_DMR;
    const uint64_t decoderGeneration =
        digitalDecoderGeneration.load(std::memory_order_relaxed);
    QByteArray decoderPcmData = pcmData;
    if (isDmr) {
        constexpr int dmrPcmBytesPerSample = static_cast<int>(sizeof(qint16));
        constexpr int dmrPcmChunkMs = 60;
        const int dmrPcmChunkBytes =
            (std::max)(1, sampleRate * dmrPcmBytesPerSample * dmrPcmChunkMs / 1000);
        const int dmrPcmMaxBufferedBytes = dmrPcmChunkBytes * 3;

        if (pendingDmrDecoderSampleRate != sampleRate) {
            qDebug() << "[Digital] DMR input sample-rate changed"
                     << "oldRate" << pendingDmrDecoderSampleRate
                     << "newRate" << sampleRate
                     << "chunkBytes" << dmrPcmChunkBytes;
            if (!pendingDmrDecoderPcm.isEmpty()) {
                qDebug() << "[Digital] clearing DMR input buffer after sample-rate change"
                         << "oldRate" << pendingDmrDecoderSampleRate
                         << "newRate" << sampleRate
                         << "droppedBytes" << pendingDmrDecoderPcm.size();
            }
            pendingDmrDecoderPcm.clear();
            pendingDmrDecoderSampleRate = sampleRate;
        }

        pendingDmrDecoderPcm.append(pcmData);
        if (pendingDmrDecoderPcm.size() > dmrPcmMaxBufferedBytes) {
            const int bytesToDrop = pendingDmrDecoderPcm.size() - dmrPcmChunkBytes;
            pendingDmrDecoderPcm.remove(0, bytesToDrop);
            qWarning() << "[Digital] trimming DMR PCM input buffer"
                       << "droppedBytes" << bytesToDrop
                       << "keptBytes" << pendingDmrDecoderPcm.size();
        }
        if (pendingDmrDecoderPcm.size() < dmrPcmChunkBytes) {
            return;
        }
        decoderPcmData = pendingDmrDecoderPcm.left(dmrPcmChunkBytes);
        pendingDmrDecoderPcm.remove(0, dmrPcmChunkBytes);
    } else if (!pendingDmrDecoderPcm.isEmpty()) {
        pendingDmrDecoderPcm.clear();
        pendingDmrDecoderSampleRate = 48000;
    }

    const int maxQueuedFrames = isDmr ? 6 : 32;
    const int queuedBefore = pendingDigitalDecoderFrames.fetch_add(1, std::memory_order_relaxed);
    if (queuedBefore >= maxQueuedFrames) {
        pendingDigitalDecoderFrames.fetch_sub(1, std::memory_order_relaxed);
        const int dropped = droppedDigitalDecoderFramesSinceLog.fetch_add(1, std::memory_order_relaxed) + 1;
        if (dropped == 1 || dropped % 50 == 0) {
            qWarning() << "[Digital] dropping stale PCM frame"
                       << "mode" << settings.modulationType
                       << "queued" << queuedBefore
                       << "limit" << maxQueuedFrames
                       << "dropped" << dropped
                       << "bytes" << decoderPcmData.size();
        }
        return;
    }

    QMetaObject::invokeMethod(digitalDecoder,
                              [this,
                               decoder = digitalDecoder,
                               pcmData = decoderPcmData,
                               settings,
                               sampleRate,
                               decoderGeneration]() {
                                  const auto releaseQueuedFrame = qScopeGuard([this]() {
                                      pendingDigitalDecoderFrames.fetch_sub(1, std::memory_order_relaxed);
                                  });
                                  if (decoderGeneration !=
                                      digitalDecoderGeneration.load(std::memory_order_relaxed)) {
                                      return;
                                  }
                                  decoder->processPcmFrame(pcmData, settings, sampleRate);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::updateDigitalDecoderMode() {
    if (!digitalDecoder || !digitalDecoderThread) {
        return;
    }
    const bool enabled = digitalDecodeEnabled;
    QString preferredDmrVoiceBackendId;
    const int dmrBackend = selectedDmrBackend();
    if (dmrBackend == DMR_BACKEND_FOBOS_MBELIB) {
        preferredDmrVoiceBackendId = QStringLiteral("fobos.dmr.voice.mbelib");
    } else if (dmrBackend == DMR_BACKEND_FOBOS_OPENDMR) {
        preferredDmrVoiceBackendId = QStringLiteral("fobos.dmr.voice.opendmr");
    }
    const bool internalDmrVoiceOutput =
        dmrBackend != DMR_BACKEND_DSD_NEO &&
        dmrBackend != DMR_BACKEND_GOPHERTRUNK;
    RadioSettings settings = pendingSettings;
    settings.dmrBasebandSampleRate =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(settings.dmrBasebandSampleRate);
    settings.dmrAmbeLayout =
        dmrAmbeLayoutCombo
            ? normalizedDmrAmbeLayout(dmrAmbeLayoutCombo->currentData().toInt())
            : normalizedDmrAmbeLayout(settings.dmrAmbeLayout);
    settings.dmrManualTimingEnabled =
        dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    settings.dmrManualTimingOffset =
        dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : settings.dmrManualTimingOffset;
    settings.dmrSlicerRatio =
        dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : settings.dmrSlicerRatio;
    settings.dmrAdaptiveSlicer =
        !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    settings.dmrPrivacyMode = pendingSettings.dmrPrivacyMode;
    settings.dmrPrivacyKeyId = pendingSettings.dmrPrivacyKeyId;
    settings.dmrPrivacyKeyHex = pendingSettings.dmrPrivacyKeyHex;
    settings.dmrPrivacyForwardToBackends = pendingSettings.dmrPrivacyForwardToBackends;
    settings.dmrPrivacyVariant = pendingSettings.dmrPrivacyVariant;
    settings.dmrPrivacyLayout = pendingSettings.dmrPrivacyLayout;
    settings.dmrPrivacyFrameOffset = pendingSettings.dmrPrivacyFrameOffset;
    const int decoderSampleRate =
        settings.modulationType == MOD_DMR ? settings.dmrBasebandSampleRate : 48000;
    QMetaObject::invokeMethod(digitalDecoder,
                              [decoder = digitalDecoder,
                               enabled,
                               settings,
                               decoderSampleRate,
                               preferredDmrVoiceBackendId,
                               internalDmrVoiceOutput]() {
                                  decoder->setEnabled(enabled);
                                  decoder->setDmrVoiceBackendId(preferredDmrVoiceBackendId);
                                  decoder->setDmrVoiceOutputEnabled(internalDmrVoiceOutput);
                                  decoder->configure(settings, decoderSampleRate);
                              },
                              Qt::QueuedConnection);
}
