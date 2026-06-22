#include "main.h"
#include "appconstants.h"
#include "iqbuffer.h"
#include "spectrumfftworker.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QFileInfoList>
#include <QMetaObject>
#include <QSignalBlocker>

#include <algorithm>
#include <cstddef>
#include <vector>
RecordingManager::Mode YourClassName::selectedRecordingMode() const {
    if (!recordingModeCombo) {
        return RecordingManager::Mode::AudioWav;
    }
    bool ok = false;
    const int value = recordingModeCombo->currentData().toInt(&ok);
    return ok ? static_cast<RecordingManager::Mode>(value) : RecordingManager::Mode::AudioWav;
}

QJsonObject YourClassName::recordingLabMetadata() const {
    QJsonObject lab;
    if (pendingSettings.modulationType != MOD_DMR) {
        return lab;
    }

    bool hasMetadata = false;
    lab["schema"] = QStringLiteral("dmr-profile");
    lab["schemaVersion"] = 2;
    lab["mode"] = QStringLiteral("DMR");
    lab["locked"] = dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked();
    lab["basebandSampleRate"] =
        dmrBasebandRateCombo
            ? normalizedDmrBasebandSampleRate(dmrBasebandRateCombo->currentData().toInt())
            : normalizedDmrBasebandSampleRate(pendingSettings.dmrBasebandSampleRate);
    lab["manualTiming"] = dmrManualTimingCheckbox && dmrManualTimingCheckbox->isChecked();
    lab["timingOffset"] = dmrTimingOffsetSpin ? dmrTimingOffsetSpin->value() : 0;
    lab["slicerRatio"] = dmrSlicerRatioSpin ? dmrSlicerRatioSpin->value() : pendingSettings.dmrSlicerRatio;
    lab["adaptiveSlicer"] = !dmrAdaptiveSlicerCheckbox || dmrAdaptiveSlicerCheckbox->isChecked();
    lab["ambeLayout"] =
        QString::fromLatin1(dmrAmbeLayoutName(dmrAmbeLayoutCombo
                                                  ? dmrAmbeLayoutCombo->currentData().toInt()
                                                  : pendingSettings.dmrAmbeLayout));
    lab["privacyMode"] = dmrPrivacyModeId(pendingSettings.dmrPrivacyMode);
    lab["privacyKeyId"] = pendingSettings.dmrPrivacyKeyId;
    lab["privacyKeySet"] = !pendingSettings.dmrPrivacyKeyHex.isEmpty();
    lab["privacyKeyHexLen"] = pendingSettings.dmrPrivacyKeyHex.size();
    lab["privacyForward"] = pendingSettings.dmrPrivacyForwardToBackends;
    lab["privacyVariant"] = pendingSettings.dmrPrivacyVariant;
    lab["privacyLayout"] = pendingSettings.dmrPrivacyLayout;
    lab["privacyFrameOffset"] = pendingSettings.dmrPrivacyFrameOffset;
    lab["description"] = QStringLiteral("DMR metadata learned from the signal or locked by the user.");

    if (dmrLabColorCodeCombo) {
        const int colorCode = dmrLabColorCodeCombo->currentData().toInt();
        if (colorCode >= 0) {
            lab["colorCode"] = colorCode;
            hasMetadata = true;
        }
    }
    if (dmrLabSlotCombo) {
        const int slot = dmrLabSlotCombo->currentData().toInt();
        if (slot == 1 || slot == 2) {
            lab["timeslot"] = slot;
            hasMetadata = true;
        }
    }
    if (dmrLabCallTypeCombo) {
        const QString callType = dmrLabCallTypeCombo->currentData().toString();
        if (!callType.isEmpty() && callType != QStringLiteral("unknown")) {
            lab["callType"] = callType;
            hasMetadata = true;
        }
    }

    auto addText = [&lab, &hasMetadata](const QString &key, const QLineEdit *edit) {
        if (!edit) {
            return;
        }
        const QString text = edit->text().trimmed();
        if (!text.isEmpty()) {
            lab[key] = text;
            hasMetadata = true;
        }
    };
    addText(QStringLiteral("sourceId"), dmrLabSourceIdEdit);
    addText(QStringLiteral("targetId"), dmrLabTargetIdEdit);
    addText(QStringLiteral("radio"), dmrLabRadioEdit);
    addText(QStringLiteral("notes"), dmrLabNotesEdit);
    if (!hasMetadata && !(dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked())) {
        return QJsonObject();
    }
    return lab;
}

bool YourClassName::isChannelIqRecordingActive() const {
    return recordingManager &&
           recordingManager->isRecording() &&
           recordingManager->mode() == RecordingManager::Mode::ChannelIqWav;
}

void YourClassName::updateIqFrameProducerSettings() {
    if (!processor) {
        return;
    }

    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    processor->configureNetworkIqStreaming(pendingSettings,
                                           serverIqStreaming || channelIqRecording,
                                           serverChannelIqStreaming || channelIqRecording);
}

void YourClassName::updateRecordingStatus(const QString &status) {
    if (recordingStatusLabel) {
        recordingStatusLabel->setProperty("statusRawText", status);
        recordingStatusLabel->setText(localizedStatusText(status));
    }
}

QString YourClassName::selectedPlaybackFilePath() const {
    if (!playbackFileCombo) {
        return QString();
    }
    return playbackFileCombo->currentData().toString();
}

void YourClassName::startRecording(bool momentary) {
    if (!recordingManager) {
        return;
    }
    if (recordingManager->isRecording()) {
        if (momentary) {
            momentaryRecordingActive = false;
        }
        return;
    }

    const RecordingManager::Mode mode = selectedRecordingMode();
    if (mode == RecordingManager::Mode::ChannelIqWav &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode() &&
        runState == RadioRunState::Running) {
        updateRecordingStatus(QStringLiteral("Recording blocked: Channel IQ cannot run during Full IQ streaming"));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
        }
        momentaryRecordingActive = false;
        return;
    }

    QString errorMessage;
    recordingManager->setDisplayScalePercent(currentScale);
    recordingManager->setLabMetadata(recordingLabMetadata());
    if (!recordingManager->start(mode, pendingSettings, &errorMessage)) {
        updateRecordingStatus(QStringLiteral("Recording failed: %1").arg(errorMessage));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
        }
        momentaryRecordingActive = false;
        return;
    }

    momentaryRecordingActive = momentary;
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(true);
        recordButton->setText(momentary
                                  ? uiText(QStringLiteral("hold_f9"), QStringLiteral("Hold F9"))
                                  : uiText(QStringLiteral("stop_rec"), QStringLiteral("Stop Rec")));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(false);
    }
    if (mode == RecordingManager::Mode::ChannelIqWav) {
        if (runState == RadioRunState::Running && !isNetworkClientMode()) {
            qDebug() << "[Recording] restarting receiver to apply Channel IQ recording policy";
            if (!restartStreamForHardwareChange()) {
                qDebug() << "[Recording] Channel IQ recording restart failed; stopping recording";
                stopRecording(false);
                updateRecordingStatus(QStringLiteral("Recording failed: receiver restart failed"));
                return;
            }
        } else {
            updateIqFrameProducerSettings();
        }
    }
}

void YourClassName::stopRecording(bool momentaryRelease) {
    if (!recordingManager) {
        return;
    }
    if (momentaryRelease && !momentaryRecordingActive) {
        return;
    }

    const bool wasChannelIqRecording = isChannelIqRecordingActive();
    momentaryRecordingActive = false;
    recordingManager->stop();
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(false);
        recordButton->setText(uiText(QStringLiteral("record"), QStringLiteral("Record")));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(true);
    }
    if (wasChannelIqRecording) {
        if (runState == RadioRunState::Running && !isNetworkClientMode()) {
            qDebug() << "[Recording] restarting receiver to restore live audio after Channel IQ recording";
            restartStreamForHardwareChange();
        } else {
            updateIqFrameProducerSettings();
        }
    }
}

void YourClassName::refreshPlaybackFiles() {
    if (!playbackFileCombo) {
        return;
    }

    const QString previousPath = selectedPlaybackFilePath();
    playbackFileCombo->blockSignals(true);
    playbackFileCombo->clear();

    QDir recordingsDir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("recordings")));
    const QFileInfoList files = recordingsDir.entryInfoList(QStringList() << QStringLiteral("*.wav"),
                                                            QDir::Files,
                                                            QDir::Time);
    for (const QFileInfo &fileInfo : files) {
        PlaybackManager::WavInfo info;
        if (!PlaybackManager::readWavInfo(fileInfo.absoluteFilePath(), info)) {
            continue;
        }

        const double seconds = info.dataSize / static_cast<double>(
            info.sampleRate * info.channels * (info.bitsPerSample / 8));
        const QString type = info.mode == PlaybackManager::Mode::AudioWav
                                 ? QStringLiteral("Audio")
                                 : QStringLiteral("Ch IQ");
        const QString label = QStringLiteral("%1  %2  %3 Hz  %4 s")
                                  .arg(fileInfo.fileName(),
                                       type,
                                       QString::number(info.sampleRate),
                                       QString::number(seconds, 'f', 1));
        playbackFileCombo->addItem(label, fileInfo.absoluteFilePath());
    }

    if (playbackFileCombo->count() == 0) {
        playbackFileCombo->addItem(uiText(QStringLiteral("no_wav_recordings_found"),
                                          QStringLiteral("No WAV recordings found")),
                                   QString());
        playbackFileCombo->setEnabled(false);
        if (playbackButton) {
            playbackButton->setEnabled(false);
        }
    } else {
        playbackFileCombo->setEnabled(true);
        if (playbackButton) {
            playbackButton->setEnabled(true);
        }
        const int previousIndex = playbackFileCombo->findData(previousPath);
        if (previousIndex >= 0) {
            playbackFileCombo->setCurrentIndex(previousIndex);
        }
    }

    playbackFileCombo->blockSignals(false);
}

void YourClassName::startPlayback() {
    if (!playbackManager) {
        return;
    }
    if (playbackManager->isPlaying()) {
        return;
    }
    if (runState != RadioRunState::Idle || deviceOpened || (processor && processor->isRunning())) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop receiver first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (recordingManager && recordingManager->isRecording()) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop recording first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    const QString path = selectedPlaybackFilePath();
    if (path.isEmpty()) {
        onPlaybackStatusChanged(QStringLiteral("Playback: no file selected"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    PlaybackManager::WavInfo info;
    QString errorMessage;
    if (!PlaybackManager::readWavInfo(path, info, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (info.mode == PlaybackManager::Mode::AudioWav && (info.channels != 1 || info.sampleRate != 48000)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: audio WAV must be mono 48 kHz"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    if (!playbackManager->start(path, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
    }
}

void YourClassName::stopPlayback() {
    if (playbackManager && playbackManager->isPlaying()) {
        playbackManager->stop();
        return;
    }
    if (playbackButton && playbackButton->isChecked()) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(uiText(QStringLiteral("play"), QStringLiteral("Play")));
    }
}

void YourClassName::onPlaybackStarted(const QString &filePath, PlaybackManager::WavInfo info) {
    Q_UNUSED(filePath);
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(true);
        playbackButton->setText(uiText(QStringLiteral("stop_play"), QStringLiteral("Stop Play")));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(false);
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(false);
    }
    if (startButton) {
        startButton->setEnabled(false);
    }

    if (info.mode == PlaybackManager::Mode::ChannelIqWav) {
        if (!playbackSettingsSaved) {
            settingsBeforePlayback = pendingSettings;
            playbackSettingsSaved = true;
        }
        offlineIqPlaybackActive = true;
        offlineIqPlaybackHasMetadata = info.hasRadioSettings;
        offlineIqPlaybackSampleRate = info.sampleRate;
        IqBuffer::clear();
        IqBuffer::setSampleRateEstimate(info.sampleRate);
        const bool audioEnabledBeforePlayback = pendingSettings.audioEnabled;
        const int audioDeviceBeforePlayback = pendingSettings.audioDeviceId;
        if (info.hasRadioSettings) {
            pendingSettings.deviceIndex = info.radioSettings.deviceIndex;
            pendingSettings.clockSource = info.radioSettings.clockSource;
            pendingSettings.inputMode = info.radioSettings.inputMode;
            pendingSettings.centerFrequency = info.radioSettings.centerFrequency;
            pendingSettings.actualFrequency = info.radioSettings.actualFrequency;
            pendingSettings.listeningFrequency = info.radioSettings.listeningFrequency;
            pendingSettings.sampleRate = info.radioSettings.sampleRate;
            pendingSettings.bandwidth = info.radioSettings.bandwidth;
            pendingSettings.modulationType = info.radioSettings.modulationType;
            pendingSettings.fftLength = info.radioSettings.fftLength;
            if (info.radioSettings.lnaGain >= 0) {
                pendingSettings.lnaGain = info.radioSettings.lnaGain;
            }
            if (info.radioSettings.vgaGain >= 0) {
                pendingSettings.vgaGain = info.radioSettings.vgaGain;
            }
            const double playbackCenter = info.radioSettings.listeningFrequency > 0.0
                                              ? info.radioSettings.listeningFrequency
                                              : info.radioSettings.centerFrequency;
            if (playbackCenter > 0.0) {
                pendingSettings.listeningFrequency = playbackCenter;
            }
        } else {
            pendingSettings.sampleRate = info.sampleRate;
            pendingSettings.centerFrequency = pendingSettings.listeningFrequency;
            pendingSettings.actualFrequency = pendingSettings.listeningFrequency;
            pendingSettings.inputMode = INPUT_RF;
        }
        pendingSettings.audioEnabled = audioEnabledBeforePlayback;
        pendingSettings.audioDeviceId = audioDeviceBeforePlayback;
        if (info.hasScalePercent) {
            currentScale = std::clamp(info.scalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        } else if (info.hasRadioSettings &&
                   info.radioSettings.sampleRate > 0.0 &&
                   info.sampleRate > 0) {
            const double channelScalePercent =
                (static_cast<double>(info.sampleRate) / info.radioSettings.sampleRate) * 100.0;
            currentScale = std::clamp(channelScalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        }
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        fftResult = std::make_unique<FFTResult>();
        if (spectrumFftWorker) {
            spectrumFftWorker->resetHfNoiseCancelState();
        }
        updateSpectrumTimerInterval();
        if (audioProcessor) {
            audioProcessor->configure(audioProcessorSettings());
        }
        pendingPlaybackAudioStartAfterIqPrebuffer = pendingSettings.audioEnabled;
        if (updateTimer) {
            updateTimer->start();
        }
        settingRange();
    } else {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
    }
}

void YourClassName::onPlaybackStopped() {
    if (offlineIqPlaybackActive) {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
        pendingPlaybackAudioStartAfterIqPrebuffer = false;
        if (audioProcessor) {
            audioProcessor->stopDemodulation();
        }
        if (updateTimer && runState == RadioRunState::Idle) {
            updateTimer->stop();
        }
    }
    if (playbackSettingsSaved) {
        pendingSettings = settingsBeforePlayback;
        playbackSettingsSaved = false;
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
    }
    if (remoteAudioPlayer && runState == RadioRunState::Idle) {
        remoteAudioPlayer->stop();
    }
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(uiText(QStringLiteral("play"), QStringLiteral("Play")));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(playbackFileCombo->count() > 0 && !selectedPlaybackFilePath().isEmpty());
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(true);
    }
    updateUiForRunState();
}

void YourClassName::onPlaybackStatusChanged(const QString &status) {
    if (playbackStatusLabel) {
        playbackStatusLabel->setProperty("statusRawText", status);
        playbackStatusLabel->setText(localizedStatusText(status));
    }
}

void YourClassName::handlePlaybackAudioFrame(const QByteArray &pcmData) {
    processDigitalAudioFrame(pcmData);
    processSstvAudioFrame(pcmData);
    processAptAudioFrame(pcmData);
    processWefaxAudioFrame(pcmData);
    sendAudioRelayFrame(pcmData);
    sendAudioHttpFrame(pcmData);
    if (remoteAudioPlayer) {
        remoteAudioPlayer->playPcmFrame(pcmData);
    }
}

void YourClassName::handlePlaybackIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (iqData.size() < static_cast<int>(2 * sizeof(qint16))) {
        return;
    }
    processGnssPackedIqFrame(iqData,
                             sampleRate,
                             sampleCount > 0 ? sampleCount : iqData.size() / (2 * static_cast<int>(sizeof(qint16))));
    processVideoIqFrame(iqData,
                        sampleRate,
                        sampleCount > 0 ? sampleCount : iqData.size() / (2 * static_cast<int>(sizeof(qint16))));
    const int bytesPerIq = 2 * static_cast<int>(sizeof(qint16));
    std::vector<float> floatSamples(static_cast<std::size_t>(iqData.size() / sizeof(qint16)));
    const auto *src = reinterpret_cast<const uchar *>(iqData.constData());
    for (int i = 0, out = 0; i + bytesPerIq - 1 < iqData.size(); i += bytesPerIq) {
        const qint16 iSample = static_cast<qint16>(src[i] | (src[i + 1] << 8));
        const qint16 qSample = static_cast<qint16>(src[i + 2] | (src[i + 3] << 8));
        floatSamples[static_cast<std::size_t>(out++)] = iSample / 32768.0f;
        floatSamples[static_cast<std::size_t>(out++)] = qSample / 32768.0f;
    }
    IqBuffer::setSampleRateEstimate(sampleRate);
    IqBuffer::publish(floatSamples.data(), floatSamples.size(), pendingSettings.audioEnabled);
    if (pendingPlaybackAudioStartAfterIqPrebuffer && pendingSettings.audioEnabled && audioProcessor && sampleRate > 0.0) {
        const double queuedIqSamples = static_cast<double>(IqBuffer::queuedFloatCount()) / 2.0;
        const double queuedSeconds = queuedIqSamples / sampleRate;
        if (queuedSeconds >= NETWORK_AUDIO_PREBUFFER_SECONDS) {
            pendingPlaybackAudioStartAfterIqPrebuffer = false;
            qDebug() << "[PlaybackIQ] starting demodulator after IQ prebuffer"
                     << "queuedSeconds" << queuedSeconds
                     << "queuedBlocks" << IqBuffer::queuedBlocks()
                     << "sampleRate" << sampleRate;
            audioProcessor->startDemodulation();
        }
    }
}
