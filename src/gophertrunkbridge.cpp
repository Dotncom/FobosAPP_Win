#include "gophertrunkbridge.h"

#include "diagnosticlogging.h"

#include <QCoreApplication>
#include <QDir>
#include <QHostAddress>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcessEnvironment>
#include <QTimer>

#include <algorithm>
#include <cmath>

namespace {
constexpr qint64 PROCESS_RESTART_BACKOFF_MAX_MS = 10000;
constexpr int DEFAULT_GOPHER_AUDIO_RATE = 8000;
constexpr int APP_AUDIO_RATE = 48000;
constexpr int UPSAMPLE_FACTOR = APP_AUDIO_RATE / DEFAULT_GOPHER_AUDIO_RATE;
constexpr double GOPHER_OUTPUT_TARGET_PEAK = 0.55;
constexpr double GOPHER_OUTPUT_MAX_GAIN = 16.0;

qint16 readLe16(const char *data) {
    const auto lo = static_cast<quint8>(data[0]);
    const auto hi = static_cast<quint8>(data[1]);
    return static_cast<qint16>(static_cast<quint16>(lo) |
                               (static_cast<quint16>(hi) << 8));
}

void appendLe16(QByteArray &target, qint16 value) {
    const quint16 raw = static_cast<quint16>(value);
    target.append(static_cast<char>(raw & 0xff));
    target.append(static_cast<char>((raw >> 8) & 0xff));
}
} // namespace

GopherTrunkBridge::GopherTrunkBridge(QObject *parent)
    : QObject(parent),
      managedProcess(new QProcess(this)),
      tcpSocket(new QTcpSocket(this)),
      udpOutputSocket(new QUdpSocket(this)) {
    connect(tcpSocket, &QTcpSocket::connected, this, [this]() {
        tcpConnectPending = false;
        emitStatus(QStringLiteral("GopherTrunk connected on TCP 127.0.0.1:%1").arg(tcpPort));
        resetStream();
    });
    connect(tcpSocket, &QTcpSocket::disconnected, this, [this]() {
        if (bridgeEnabled) {
            emitStatus(QStringLiteral("GopherTrunk TCP disconnected"));
            QTimer::singleShot(500, this, [this]() { connectTcpIfNeeded(); });
        }
    });
    connect(tcpSocket,
            QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this,
            [this](QAbstractSocket::SocketError) {
                if (bridgeEnabled && tcpSocket->state() != QAbstractSocket::ConnectedState) {
                    QTimer::singleShot(750, this, [this]() { connectTcpIfNeeded(); });
                }
            });
    connect(tcpSocket, &QTcpSocket::readyRead, this, &GopherTrunkBridge::readTcpOutput);
    connect(udpOutputSocket, &QUdpSocket::readyRead, this, &GopherTrunkBridge::readUdpOutput);
    connect(managedProcess, &QProcess::readyReadStandardOutput, this, &GopherTrunkBridge::readProcessOutput);
    connect(managedProcess, &QProcess::readyReadStandardError, this, &GopherTrunkBridge::readProcessOutput);
    connect(managedProcess, &QProcess::started, this, [this]() {
        processRestartBackoffMs = 0;
        processRestartBackoffTimer.invalidate();
        QTimer::singleShot(250, this, [this]() { connectTcpIfNeeded(); });
    });
    connect(managedProcess,
            QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
            this,
            [this](QProcess::ProcessError error) {
                if (processStoppingIntentionally) {
                    return;
                }
                armProcessRestartBackoff(managedProcess->errorString());
                emitStatus(QStringLiteral("GopherTrunk process error %1: %2")
                               .arg(static_cast<int>(error))
                               .arg(managedProcess->errorString()));
            });
    connect(managedProcess,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this,
            [this](int exitCode, QProcess::ExitStatus exitStatus) {
                const bool intentionalStop =
                    processStoppingIntentionally || !bridgeEnabled || !processAutoStart;
                processStoppingIntentionally = false;
                const QString stoppedText =
                    QStringLiteral("GopherTrunk stopped: code %1, status %2")
                        .arg(exitCode)
                        .arg(static_cast<int>(exitStatus));
                if (intentionalStop) {
                    if (fobosVerboseLoggingEnabled()) {
                        qDebug() << "[GopherTrunk]" << stoppedText;
                    }
                } else {
                    emitStatus(stoppedText);
                }
                if (!intentionalStop &&
                    (exitCode != 0 || exitStatus != QProcess::NormalExit)) {
                    armProcessRestartBackoff(QStringLiteral("process exited"));
                }
                if (!intentionalStop && bridgeEnabled && processAutoStart) {
                    const int delayMs = static_cast<int>((std::max<qint64>)(250, processRestartBackoffMs));
                    QTimer::singleShot(delayMs, this, [this]() { startProcessIfNeeded(); });
                }
            });
}

GopherTrunkBridge::~GopherTrunkBridge() {
    disconnect(this, nullptr, nullptr, nullptr);
    setEnabled(false);
    if (managedProcess && managedProcess->state() != QProcess::NotRunning) {
        managedProcess->kill();
        managedProcess->waitForFinished(500);
    }
}

void GopherTrunkBridge::configure(bool autoStart,
                                  const QString &program,
                                  quint16 tcpPortValue,
                                  quint16 audioUdpPortValue,
                                  const QString &workingDirectory) {
    processAutoStart = autoStart;
    processProgram = program.trimmed();
    tcpPort = tcpPortValue == 0 ? quint16(7460) : tcpPortValue;
    audioUdpPort = audioUdpPortValue == 0 ? quint16(23456) : audioUdpPortValue;
    processArguments = QStringList()
                       << QStringLiteral("-listen")
                       << QStringLiteral("127.0.0.1:%1").arg(tcpPort)
                       << QStringLiteral("-audio-udp")
                       << QStringLiteral("127.0.0.1:%1").arg(audioUdpPort);
    processWorkingDirectory = workingDirectory;
    if (!bridgeEnabled) {
        return;
    }
    startUdpOutput();
    if (processAutoStart) {
        startProcessIfNeeded();
    }
    connectTcpIfNeeded();
}

void GopherTrunkBridge::setEnabled(bool enabled) {
    if (bridgeEnabled == enabled) {
        if (bridgeEnabled) {
            startUdpOutput();
            if (processAutoStart) {
                startProcessIfNeeded();
            }
            connectTcpIfNeeded();
        }
        return;
    }

    bridgeEnabled = enabled;
    if (bridgeEnabled) {
        dibitPacketsSent = 0;
        startUdpOutput();
        if (processAutoStart) {
            startProcessIfNeeded();
        }
        connectTcpIfNeeded();
        emitStatus(QStringLiteral("GopherTrunk bridge enabled"));
    } else {
        disconnectTcp();
        stopProcess();
        stopUdpOutput();
        emitStatus(QStringLiteral("GopherTrunk bridge disabled"));
    }
}

void GopherTrunkBridge::resetStream() {
    if (!bridgeEnabled || tcpSocket->state() != QAbstractSocket::ConnectedState) {
        return;
    }
    tcpSocket->write(QByteArrayLiteral("{\"reset\":true}\n"));
}

void GopherTrunkBridge::sendDibitBurst(const QString &dibits,
                                       quint64 sample,
                                       int burstIndex,
                                       int cadenceSymbols,
                                       int colorCode) {
    if (!bridgeEnabled || dibits.size() != 132) {
        return;
    }
    if (processAutoStart) {
        startProcessIfNeeded();
    }
    if (tcpSocket->state() != QAbstractSocket::ConnectedState) {
        connectTcpIfNeeded();
        return;
    }

    QJsonObject packet;
    packet.insert(QStringLiteral("dibits"), dibits);
    packet.insert(QStringLiteral("sample"), QString::number(sample));
    packet.insert(QStringLiteral("burst"), burstIndex);
    packet.insert(QStringLiteral("cadenceSymbols"), cadenceSymbols);
    if (colorCode >= 0) {
        packet.insert(QStringLiteral("colorCode"), colorCode);
    }
    QByteArray line = QJsonDocument(packet).toJson(QJsonDocument::Compact);
    line.append('\n');
    tcpSocket->write(line);
    ++dibitPacketsSent;
    if (fobosVerboseLoggingEnabled() &&
        (dibitPacketsSent <= 8 || (dibitPacketsSent % 100) == 0)) {
        qDebug() << "[GopherTrunk] sent dibit burst"
                 << "packet" << dibitPacketsSent
                 << "sample" << static_cast<qulonglong>(sample)
                 << "burst" << burstIndex
                 << "cc" << colorCode;
    }
}

void GopherTrunkBridge::startProcessIfNeeded() {
    if (!processAutoStart || !bridgeEnabled || processProgram.trimmed().isEmpty()) {
        return;
    }
    if (managedProcess->state() != QProcess::NotRunning || processRestartBackoffActive()) {
        return;
    }
    processLogLines = 0;
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    managedProcess->setProcessEnvironment(environment);
    managedProcess->setProgram(processProgram);
    managedProcess->setArguments(processArguments);
    if (!processWorkingDirectory.trimmed().isEmpty()) {
        managedProcess->setWorkingDirectory(processWorkingDirectory);
    }
    managedProcess->start();
    emitStatus(QStringLiteral("Starting GopherTrunk: %1 %2")
                   .arg(processProgram, processArguments.join(QLatin1Char(' '))));
}

void GopherTrunkBridge::stopProcess() {
    if (!managedProcess || managedProcess->state() == QProcess::NotRunning) {
        return;
    }
    processStoppingIntentionally = true;
    managedProcess->terminate();
    if (!managedProcess->waitForFinished(1500) &&
        managedProcess->state() != QProcess::NotRunning) {
        managedProcess->kill();
        managedProcess->waitForFinished(500);
    }
}

void GopherTrunkBridge::connectTcpIfNeeded() {
    if (!bridgeEnabled || tcpConnectPending ||
        tcpSocket->state() == QAbstractSocket::ConnectedState ||
        tcpSocket->state() == QAbstractSocket::ConnectingState) {
        return;
    }
    tcpConnectPending = true;
    tcpSocket->connectToHost(QHostAddress::LocalHost, tcpPort);
}

void GopherTrunkBridge::disconnectTcp() {
    tcpConnectPending = false;
    if (tcpSocket->state() != QAbstractSocket::UnconnectedState) {
        tcpSocket->disconnectFromHost();
        if (tcpSocket->state() != QAbstractSocket::UnconnectedState) {
            tcpSocket->abort();
        }
    }
}

void GopherTrunkBridge::startUdpOutput() {
    if (!bridgeEnabled) {
        return;
    }
    if (udpOutputSocket->state() == QAbstractSocket::BoundState &&
        udpOutputSocket->localPort() == audioUdpPort) {
        return;
    }
    stopUdpOutput();
    const bool bound = udpOutputSocket->bind(QHostAddress::LocalHost,
                                             audioUdpPort,
                                             QUdpSocket::ShareAddress |
                                                 QUdpSocket::ReuseAddressHint);
    if (!bound) {
        emitStatus(QStringLiteral("GopherTrunk UDP audio bind failed on 127.0.0.1:%1: %2")
                       .arg(audioUdpPort)
                       .arg(udpOutputSocket->errorString()));
        return;
    }
    emitStatus(QStringLiteral("GopherTrunk UDP audio input ready: 127.0.0.1:%1").arg(audioUdpPort));
}

void GopherTrunkBridge::stopUdpOutput() {
    if (udpOutputSocket->state() != QAbstractSocket::UnconnectedState) {
        udpOutputSocket->close();
    }
}

void GopherTrunkBridge::readTcpOutput() {
    while (tcpSocket->canReadLine()) {
        const QString line = QString::fromUtf8(tcpSocket->readLine()).trimmed();
        if (line.isEmpty()) {
            continue;
        }
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[GopherTrunk output]" << line.left(240);
        }
    }
}

void GopherTrunkBridge::readProcessOutput() {
    const QByteArray output = managedProcess->readAllStandardOutput() +
                              managedProcess->readAllStandardError();
    const QList<QByteArray> lines = output.split('\n');
    for (const QByteArray &rawLine : lines) {
        const QString line = QString::fromUtf8(rawLine).trimmed();
        if (line.isEmpty()) {
            continue;
        }
        ++processLogLines;
        if (processLogLines <= 20 || (processLogLines % 50) == 0) {
            emitStatus(QStringLiteral("GopherTrunk: %1").arg(line.left(160)));
        } else if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[GopherTrunk process]" << line;
        }
    }
}

void GopherTrunkBridge::readUdpOutput() {
    while (udpOutputSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(udpOutputSocket->pendingDatagramSize()));
        udpOutputSocket->readDatagram(datagram.data(), datagram.size());
        const QByteArray mono8k = pcm16ToMono(datagram, 1);
        if (mono8k.isEmpty()) {
            continue;
        }
        ++udpOutputPacketsReceived;
        const QByteArray normalized = normalizeOutputPcmForPlayback(mono8k, udpOutputPacketsReceived);
        emit decodedPcmReady(upsample8kMonoTo48k(normalized));
    }
}

void GopherTrunkBridge::emitStatus(const QString &status) {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[GopherTrunk]" << status;
    }
    emit statusChanged(status);
}

bool GopherTrunkBridge::processRestartBackoffActive() const {
    return processRestartBackoffMs > 0 &&
           processRestartBackoffTimer.isValid() &&
           processRestartBackoffTimer.elapsed() < processRestartBackoffMs;
}

void GopherTrunkBridge::armProcessRestartBackoff(const QString &reason) {
    processRestartBackoffMs = processRestartBackoffMs <= 0
                                  ? qint64(1000)
                                  : (std::min)(processRestartBackoffMs * 2, PROCESS_RESTART_BACKOFF_MAX_MS);
    processRestartBackoffTimer.restart();
    emitStatus(QStringLiteral("GopherTrunk restart backoff %1 ms: %2")
                   .arg(processRestartBackoffMs)
                   .arg(reason.left(120)));
}

QByteArray GopherTrunkBridge::pcm16ToMono(const QByteArray &pcmData, int channels) const {
    const int safeChannels = std::clamp(channels, 1, 2);
    const int frameBytes = safeChannels * int(sizeof(qint16));
    const int frames = pcmData.size() / frameBytes;
    if (frames <= 0) {
        return {};
    }
    if (safeChannels == 1) {
        QByteArray mono = pcmData.left(frames * frameBytes);
        if (mono.size() & 1) {
            mono.chop(1);
        }
        return mono;
    }

    QByteArray mono;
    mono.reserve(frames * int(sizeof(qint16)));
    const char *raw = pcmData.constData();
    for (int frame = 0; frame < frames; ++frame) {
        const qint16 left = readLe16(raw + frame * frameBytes);
        const qint16 right = readLe16(raw + frame * frameBytes + int(sizeof(qint16)));
        appendLe16(mono, static_cast<qint16>((static_cast<int>(left) + static_cast<int>(right)) / 2));
    }
    return mono;
}

QByteArray GopherTrunkBridge::normalizeOutputPcmForPlayback(const QByteArray &pcmData,
                                                            quint64 packetIndex) const {
    const int samples = pcmData.size() / int(sizeof(qint16));
    if (samples <= 0) {
        return {};
    }

    const char *raw = pcmData.constData();
    int peak = 0;
    double squareSum = 0.0;
    for (int i = 0; i < samples; ++i) {
        const int sample = readLe16(raw + i * int(sizeof(qint16)));
        peak = (std::max)(peak, std::abs(sample));
        const double normalized = static_cast<double>(sample) / 32768.0;
        squareSum += normalized * normalized;
    }

    const double peakNorm = static_cast<double>(peak) / 32767.0;
    const double rms = std::sqrt(squareSum / static_cast<double>((std::max)(1, samples)));
    double gain = 1.0;
    if (peakNorm > 0.0 && peakNorm < GOPHER_OUTPUT_TARGET_PEAK) {
        gain = (std::min)(GOPHER_OUTPUT_TARGET_PEAK / peakNorm, GOPHER_OUTPUT_MAX_GAIN);
    }

    if (fobosVerboseLoggingEnabled() &&
        (packetIndex <= 8 || (packetIndex % 100) == 0)) {
        qDebug() << "[GopherTrunk] decoded UDP audio"
                 << "packet" << packetIndex
                 << "samples" << samples
                 << "peak" << QString::number(peakNorm, 'f', 4)
                 << "rms" << QString::number(rms, 'f', 4)
                 << "gain" << QString::number(gain, 'f', 2);
    }

    if (gain <= 1.001) {
        return pcmData.left(samples * int(sizeof(qint16)));
    }

    QByteArray output;
    output.reserve(samples * int(sizeof(qint16)));
    for (int i = 0; i < samples; ++i) {
        const int sample = readLe16(raw + i * int(sizeof(qint16)));
        const int scaled = static_cast<int>(std::lrint(static_cast<double>(sample) * gain));
        const int clamped = (std::clamp)(scaled, -32768, 32767);
        appendLe16(output, static_cast<qint16>(clamped));
    }
    return output;
}

QByteArray GopherTrunkBridge::upsample8kMonoTo48k(const QByteArray &pcmData) const {
    const int inputSamples = pcmData.size() / int(sizeof(qint16));
    if (inputSamples <= 0) {
        return {};
    }

    QByteArray output;
    output.reserve(inputSamples * UPSAMPLE_FACTOR * int(sizeof(qint16)));
    const char *raw = pcmData.constData();
    for (int i = 0; i < inputSamples; ++i) {
        const qint16 current = readLe16(raw + i * int(sizeof(qint16)));
        const qint16 next =
            (i + 1 < inputSamples)
                ? readLe16(raw + (i + 1) * int(sizeof(qint16)))
                : current;
        for (int step = 0; step < UPSAMPLE_FACTOR; ++step) {
            const float fraction = static_cast<float>(step) / static_cast<float>(UPSAMPLE_FACTOR);
            const float sample =
                static_cast<float>(current) +
                fraction * static_cast<float>(static_cast<int>(next) - static_cast<int>(current));
            appendLe16(output, static_cast<qint16>((std::clamp)(static_cast<int>(std::lrint(sample)),
                                                                -32768,
                                                                32767)));
        }
    }
    return output;
}
