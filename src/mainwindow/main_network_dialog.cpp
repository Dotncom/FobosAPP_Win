#include "main.h"
#include "diagnosticlogging.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QHostAddress>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
void YourClassName::openNetworkSettingsDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("network_settings"), QStringLiteral("Network Settings")));
    dialog.setMinimumWidth(420);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QFormLayout *formLayout = new QFormLayout();

    QComboBox *modeCombo = new QComboBox(&dialog);
    modeCombo->addItem(uiText(QStringLiteral("disabled"), QStringLiteral("Disabled")), static_cast<int>(NetworkMode::Disabled));
    modeCombo->addItem(uiText(QStringLiteral("server"), QStringLiteral("Server")), static_cast<int>(NetworkMode::Server));
    modeCombo->addItem(uiText(QStringLiteral("client"), QStringLiteral("Client")), static_cast<int>(NetworkMode::Client));
    modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(networkMode)));

    QComboBox *processingCombo = new QComboBox(&dialog);
    processingCombo->addItem(uiText(QStringLiteral("network_processing_server"),
                                    QStringLiteral("Server processing (spectrum/audio stream)")),
                             static_cast<int>(NetworkProcessingMode::ServerSide));
    processingCombo->addItem(uiText(QStringLiteral("network_processing_channel_iq"),
                                    QStringLiteral("Channel IQ + client demod")),
                             static_cast<int>(NetworkProcessingMode::ChannelIqClientSide));
    processingCombo->addItem(uiText(QStringLiteral("network_processing_full_iq"),
                                    QStringLiteral("Full IQ client processing (LAN only)")),
                             static_cast<int>(NetworkProcessingMode::FullIqClientSide));
    processingCombo->setCurrentIndex(processingCombo->findData(static_cast<int>(networkProcessingMode)));

    QLineEdit *serverAddressEdit = new QLineEdit(networkServerAddress, &dialog);
    serverAddressEdit->setPlaceholderText(uiText(QStringLiteral("server_ip_placeholder"),
                                                 QStringLiteral("Server IP address")));

    QLineEdit *bindAddressEdit = new QLineEdit(networkBindAddress, &dialog);
    bindAddressEdit->setPlaceholderText("0.0.0.0");

    QSpinBox *portSpin = new QSpinBox(&dialog);
    portSpin->setRange(1, 65535);
    portSpin->setValue(networkControlPort);

    QCheckBox *serverDisableLocalUiCheck = new QCheckBox(uiText(QStringLiteral("network_disable_local_ui"),
                                                                QStringLiteral("Disable local visual/audio on server when streaming is implemented")),
                                                         &dialog);
    serverDisableLocalUiCheck->setChecked(serverDisableLocalVisualAudio);

    QCheckBox *fullResolutionSpectrumCheck = new QCheckBox(uiText(QStringLiteral("network_full_resolution_frames"),
                                                                  QStringLiteral("Send full-resolution spectrum/waterfall frames (heavy LAN only)")),
                                                           &dialog);
    fullResolutionSpectrumCheck->setChecked(networkFullResolutionSpectrumFrames);

    QCheckBox *audioRelayTransmitCheck = new QCheckBox(uiText(QStringLiteral("audio_relay_tx"),
                                                             QStringLiteral("Send ready audio by UDP")),
                                                       &dialog);
    audioRelayTransmitCheck->setChecked(audioRelayTransmitEnabled);

    QLineEdit *audioRelayHostEdit = new QLineEdit(audioRelayHost, &dialog);
    audioRelayHostEdit->setPlaceholderText(uiText(QStringLiteral("target_ip_placeholder"),
                                                  QStringLiteral("Target IP address")));

    QSpinBox *audioRelayPortSpin = new QSpinBox(&dialog);
    audioRelayPortSpin->setRange(1, 65535);
    audioRelayPortSpin->setValue(audioRelayPort);

    QCheckBox *audioRelayReceiveCheck = new QCheckBox(uiText(QStringLiteral("audio_relay_rx"),
                                                            QStringLiteral("Receive ready audio by UDP")),
                                                      &dialog);
    audioRelayReceiveCheck->setChecked(audioRelayReceiveEnabled);

    QSpinBox *audioRelayListenPortSpin = new QSpinBox(&dialog);
    audioRelayListenPortSpin->setRange(1, 65535);
    audioRelayListenPortSpin->setValue(audioRelayListenPort);

    QCheckBox *audioHttpStreamCheck = new QCheckBox(uiText(QStringLiteral("audio_http_stream"),
                                                          QStringLiteral("Serve VLC-compatible HTTP/WAV audio")),
                                                    &dialog);
    audioHttpStreamCheck->setChecked(audioHttpStreamEnabled);

    QSpinBox *audioHttpStreamPortSpin = new QSpinBox(&dialog);
    audioHttpStreamPortSpin->setRange(1, 65535);
    audioHttpStreamPortSpin->setValue(audioHttpStreamPort);

    QLabel *statusLabel = new QLabel(networkController
                                         ? networkController->statusText()
                                         : uiText(QStringLiteral("network_controller_unavailable"),
                                                  QStringLiteral("Network controller unavailable")),
                                     &dialog);
    statusLabel->setWordWrap(true);

    formLayout->addRow(uiText(QStringLiteral("mode"), QStringLiteral("Mode:")), modeCombo);
    formLayout->addRow(uiText(QStringLiteral("processing"), QStringLiteral("Processing:")), processingCombo);
    formLayout->addRow(uiText(QStringLiteral("server_ip"), QStringLiteral("Server IP:")), serverAddressEdit);
    formLayout->addRow(uiText(QStringLiteral("bind_address"), QStringLiteral("Bind address:")), bindAddressEdit);
    formLayout->addRow(uiText(QStringLiteral("control_port"), QStringLiteral("Control port:")), portSpin);
    formLayout->addRow("", serverDisableLocalUiCheck);
    formLayout->addRow(uiText(QStringLiteral("visual_frames"), QStringLiteral("Visual frames:")), fullResolutionSpectrumCheck);
    formLayout->addRow(uiText(QStringLiteral("audio_relay_tx_label"), QStringLiteral("Audio relay TX:")), audioRelayTransmitCheck);
    formLayout->addRow(uiText(QStringLiteral("relay_target_ip"), QStringLiteral("Relay target IP:")), audioRelayHostEdit);
    formLayout->addRow(uiText(QStringLiteral("relay_target_port"), QStringLiteral("Relay target port:")), audioRelayPortSpin);
    formLayout->addRow(uiText(QStringLiteral("audio_relay_rx_label"), QStringLiteral("Audio relay RX:")), audioRelayReceiveCheck);
    formLayout->addRow(uiText(QStringLiteral("relay_listen_port"), QStringLiteral("Relay listen port:")), audioRelayListenPortSpin);
    formLayout->addRow(uiText(QStringLiteral("vlc_http_audio"), QStringLiteral("VLC HTTP audio:")), audioHttpStreamCheck);
    formLayout->addRow(uiText(QStringLiteral("http_audio_port"), QStringLiteral("HTTP audio port:")), audioHttpStreamPortSpin);

    QPushButton *testButton = new QPushButton(uiText(QStringLiteral("apply_test_channel"), QStringLiteral("Apply / Test Channel")), &dialog);
    QPushButton *requestControlButton = new QPushButton(uiText(QStringLiteral("request_control"), QStringLiteral("Request Control")), &dialog);
    QPushButton *stopButton = new QPushButton(uiText(QStringLiteral("stop_network"), QStringLiteral("Stop Network")), &dialog);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    if (QPushButton *closeButton = buttonBox->button(QDialogButtonBox::Close)) {
        closeButton->setText(uiText(QStringLiteral("close"), QStringLiteral("Close")));
    }

    QHBoxLayout *actionLayout = new QHBoxLayout();
    actionLayout->addWidget(testButton);
    actionLayout->addWidget(requestControlButton);
    actionLayout->addWidget(stopButton);

    rootLayout->addLayout(formLayout);
    rootLayout->addWidget(statusLabel);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(buttonBox);

    auto updateFieldState = [=]() {
        const auto selectedMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        serverAddressEdit->setEnabled(selectedMode == NetworkMode::Client);
        bindAddressEdit->setEnabled(selectedMode == NetworkMode::Server);
        serverDisableLocalUiCheck->setEnabled(selectedMode != NetworkMode::Disabled);
        fullResolutionSpectrumCheck->setEnabled(selectedMode != NetworkMode::Disabled);
        processingCombo->setEnabled(selectedMode != NetworkMode::Disabled);
        portSpin->setEnabled(selectedMode != NetworkMode::Disabled);
        requestControlButton->setEnabled(selectedMode == NetworkMode::Client);
        audioRelayHostEdit->setEnabled(audioRelayTransmitCheck->isChecked());
        audioRelayPortSpin->setEnabled(audioRelayTransmitCheck->isChecked());
        audioRelayListenPortSpin->setEnabled(audioRelayReceiveCheck->isChecked());
        audioHttpStreamPortSpin->setEnabled(audioHttpStreamCheck->isChecked());
        testButton->setText(selectedMode == NetworkMode::Disabled
                                ? uiText(QStringLiteral("apply"), QStringLiteral("Apply"))
                                : uiText(QStringLiteral("apply_test_channel"), QStringLiteral("Apply / Test Channel")));
    };
    connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, updateFieldState);
    connect(audioRelayTransmitCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    connect(audioRelayReceiveCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    connect(audioHttpStreamCheck, &QCheckBox::toggled, &dialog, updateFieldState);
    updateFieldState();

    QMetaObject::Connection statusConnection;
    if (networkController) {
        statusConnection = connect(networkController,
                                   &NetworkController::statusChanged,
                                   &dialog,
                                   [statusLabel](const QString &status) {
                                       statusLabel->setText(status);
                                   });
    }

    connect(testButton, &QPushButton::clicked, &dialog, [=]() {
        const NetworkMode previousNetworkMode = networkMode;
        const NetworkProcessingMode previousProcessingMode = networkProcessingMode;
        const bool previousServerDisableLocalVisualAudio = serverDisableLocalVisualAudio;

        networkMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        networkServerAddress = serverAddressEdit->text().trimmed().isEmpty()
                                   ? QString("127.0.0.1")
                                   : serverAddressEdit->text().trimmed();
        networkBindAddress = bindAddressEdit->text().trimmed().isEmpty()
                                 ? QString("0.0.0.0")
                                 : bindAddressEdit->text().trimmed();
        networkControlPort = static_cast<quint16>(portSpin->value());
        serverDisableLocalVisualAudio = serverDisableLocalUiCheck->isChecked();
        networkFullResolutionSpectrumFrames = fullResolutionSpectrumCheck->isChecked();
        networkProcessingMode = static_cast<NetworkProcessingMode>(processingCombo->currentData().toInt());
        audioRelayTransmitEnabled = audioRelayTransmitCheck->isChecked();
        audioRelayHost = audioRelayHostEdit->text().trimmed().isEmpty()
                             ? QString("127.0.0.1")
                             : audioRelayHostEdit->text().trimmed();
        audioRelayPort = static_cast<quint16>(audioRelayPortSpin->value());
        audioRelayReceiveEnabled = audioRelayReceiveCheck->isChecked();
        audioRelayListenPort = static_cast<quint16>(audioRelayListenPortSpin->value());
        audioHttpStreamEnabled = audioHttpStreamCheck->isChecked();
        audioHttpStreamPort = static_cast<quint16>(audioHttpStreamPortSpin->value());
        updateAudioRelaySocket();
        updateAudioHttpStreamServer();
        if (isChannelIqRecordingActive() &&
            networkMode != NetworkMode::Disabled &&
            isFullIqProcessingMode()) {
            stopRecording(false);
            updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
        }

        const bool networkModeChanged = previousNetworkMode != networkMode;
        const bool processingModeChanged = previousProcessingMode != networkProcessingMode;
        const bool serverLocalOutputChanged =
            previousServerDisableLocalVisualAudio != serverDisableLocalVisualAudio;

        if (!networkController) {
            statusLabel->setText("Network controller unavailable");
            return;
        }

        if (networkMode == NetworkMode::Disabled) {
            if (previousNetworkMode == NetworkMode::Client) {
                stopNetworkClientProcessing();
                clearRemoteReceiverDeviceList();
            } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
                if (audioProcessor) {
                    audioProcessor->setLocalPlaybackEnabled(true);
                }
                if (updateTimer) {
                    updateTimer->start();
                }
            }
            if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            networkController->stop();
            if (runState == RadioRunState::Running) {
                updateIqFrameProducerSettings();
            }
            savePersistentSettings();
            return;
        }

        if (previousNetworkMode == NetworkMode::Client && networkMode != NetworkMode::Client) {
            stopNetworkClientProcessing();
            clearRemoteReceiverDeviceList();
        }

        if (remoteAudioPlayer && networkMode != NetworkMode::Client) {
            remoteAudioPlayer->stop();
        }
        if (networkMode == NetworkMode::Server) {
            clearRemoteReceiverDeviceList();
            networkController->startServer(networkBindAddress, networkControlPort);
            const bool restartRequired =
                runState == RadioRunState::Running &&
                (networkModeChanged || processingModeChanged);
            if (restartRequired) {
                restartStreamForHardwareChange();
            } else if (runState == RadioRunState::Running && serverLocalOutputChanged) {
                applyServerLocalOutputPolicy();
            }
            savePersistentSettings();
            return;
        }

        if (runState == RadioRunState::Running &&
            (networkModeChanged || processingModeChanged)) {
            if (isClientIqProcessingMode()) {
                startNetworkClientProcessing();
            } else {
                stopNetworkClientProcessing();
            }
        }

        if (networkMode == NetworkMode::Client) {
            clearRemoteReceiverDeviceList();
        }
        networkController->testClientConnection(networkServerAddress, networkControlPort);
        savePersistentSettings();
    });

    connect(stopButton, &QPushButton::clicked, &dialog, [=]() {
        const NetworkMode previousNetworkMode = networkMode;
        networkMode = NetworkMode::Disabled;
        if (modeCombo) {
            modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(NetworkMode::Disabled)));
        }
        if (networkController) {
            networkController->stop();
        }
        if (remoteAudioPlayer) {
            remoteAudioPlayer->stop();
        }
        if (previousNetworkMode == NetworkMode::Client) {
            stopNetworkClientProcessing();
            clearRemoteReceiverDeviceList();
        } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            if (audioProcessor) {
                audioProcessor->setLocalPlaybackEnabled(true);
            }
            if (updateTimer) {
                updateTimer->start();
            }
        }
        if (runState == RadioRunState::Running) {
            updateIqFrameProducerSettings();
        }
        savePersistentSettings();
    });

    connect(requestControlButton, &QPushButton::clicked, &dialog, [=]() {
        if (networkMode != NetworkMode::Client) {
            statusLabel->setText("Switch to Client mode and connect first.");
            return;
        }
        if (!networkController || !networkController->isControlReady()) {
            statusLabel->setText("Control channel is not ready.");
            return;
        }
        if (networkController->clientHasControl()) {
            statusLabel->setText("This client already has control.");
            return;
        }
        if (sendRemoteControlCommand("requestPriority")) {
            statusLabel->setText("Control request sent.");
        } else {
            statusLabel->setText("Control request could not be sent.");
        }
    });

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
    if (networkController && statusConnection) {
        disconnect(statusConnection);
    }
}
