#include "dsdneobridge.h"

#include "diagnosticlogging.h"

#include <QDataStream>
#include <QHostAddress>
#include <QProcessEnvironment>
#include <QTimer>
#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace {
constexpr qint64 MAX_UDP_INPUT_DATAGRAM_BYTES = 1200;
constexpr qint64 PROCESS_RESTART_BACKOFF_MAX_MS = 10000;
constexpr int DEFAULT_DSD_AUDIO_RATE = 8000;
constexpr int APP_AUDIO_RATE = 48000;
constexpr int UPSAMPLE_FACTOR = APP_AUDIO_RATE / DEFAULT_DSD_AUDIO_RATE;
constexpr double DSD_INPUT_TARGET_PEAK = 0.45;
constexpr int DSD_INPUT_WARMUP_FRAMES = 3;

QString defaultDsdNeoProgram() {
#if defined(Q_OS_WIN)
    return QStringLiteral("dsd-neo/dsd-neo.exe");
#else
    return QStringLiteral("dsd-neo");
#endif
}

bool stringListsEqual(const QStringList &lhs, const QStringList &rhs) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (int i = 0; i < lhs.size(); ++i) {
        if (lhs.at(i) != rhs.at(i)) {
            return false;
        }
    }
    return true;
}

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

DsdNeoBridge::DsdNeoBridge(QObject *parent)
    : QObject(parent),
      inputServer(new QTcpServer(this)),
      udpInputSocket(new QUdpSocket(this)),
      udpOutputSocket(new QUdpSocket(this)),
      managedProcess(new QProcess(this)) {
    connect(inputServer, &QTcpServer::newConnection, this, &DsdNeoBridge::acceptInputClient);
    connect(udpOutputSocket, &QUdpSocket::readyRead, this, &DsdNeoBridge::readUdpOutput);
    connect(managedProcess, &QProcess::readyReadStandardOutput, this, &DsdNeoBridge::readProcessOutput);
    connect(managedProcess, &QProcess::readyReadStandardError, this, &DsdNeoBridge::readProcessOutput);
    connect(managedProcess, &QProcess::started, this, [this]() {
        processRestartBackoffMs = 0;
        processRestartBackoffTimer.invalidate();
    });
    connect(managedProcess,
            QOverload<QProcess::ProcessError>::of(&QProcess::errorOccurred),
            this,
            [this](QProcess::ProcessError error) {
                if (processStoppingIntentionally) {
                    emitStatus(QStringLiteral("DSD-neo process stopped"));
                    return;
                }
                armProcessRestartBackoff(managedProcess->errorString());
                emitStatus(QStringLiteral("DSD-neo process error %1: %2")
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
                    QStringLiteral("DSD-neo stopped: code %1, status %2")
                        .arg(exitCode)
                        .arg(static_cast<int>(exitStatus));
                if (intentionalStop) {
                    if (fobosVerboseLoggingEnabled()) {
                        qDebug() << "[DSD-neo]" << stoppedText;
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
                    QTimer::singleShot(delayMs, this, [this]() {
                        startProcessIfNeeded();
                    });
                }
            });
    noClientLogTimer.invalidate();
}

DsdNeoBridge::~DsdNeoBridge() {
    disconnect(this, nullptr, nullptr, nullptr);
    setEnabled(false);
    if (managedProcess && managedProcess->state() != QProcess::NotRunning) {
        managedProcess->kill();
        managedProcess->waitForFinished(500);
    }
}

void DsdNeoBridge::configureInputServer(bool enabled, quint16 port) {
    inputServerEnabled = enabled;
    inputPort = port == 0 ? quint16(7355) : port;
    if (!bridgeEnabled) {
        return;
    }
    if (inputServerEnabled) {
        startInputServer();
    } else {
        stopInputServer();
    }
}

void DsdNeoBridge::configureUdpOutput(bool enabled, quint16 port, int channels) {
    udpOutputEnabled = enabled;
    udpOutputPort = port == 0 ? quint16(23456) : port;
    udpOutputChannels = std::clamp(channels, 1, 2);
    if (!bridgeEnabled) {
        return;
    }
    if (udpOutputEnabled) {
        startUdpOutput();
    } else {
        stopUdpOutput();
    }
}

void DsdNeoBridge::configureProcess(bool autoStart,
                                    const QString &program,
                                    const QStringList &arguments,
                                    const QString &workingDirectory) {
    const bool running = managedProcess->state() != QProcess::NotRunning;
    const bool processChanged =
        processProgram != (program.trimmed().isEmpty() ? defaultDsdNeoProgram() : program.trimmed()) ||
        !stringListsEqual(processArguments, arguments) ||
        processWorkingDirectory != workingDirectory;
    processAutoStart = autoStart;
    processProgram = program.trimmed().isEmpty() ? defaultDsdNeoProgram() : program.trimmed();
    processArguments = arguments;
    processWorkingDirectory = workingDirectory;
    if (!bridgeEnabled) {
        return;
    }
    if (running && processChanged) {
        stopProcess();
    }
    if (processAutoStart) {
        startProcessIfNeeded();
    } else {
        stopProcess();
    }
}

void DsdNeoBridge::setEnabled(bool enabled) {
    if (bridgeEnabled == enabled) {
        if (bridgeEnabled) {
            if (inputServerEnabled) {
                startInputServer();
            }
            if (udpOutputEnabled) {
                startUdpOutput();
            }
            if (processAutoStart) {
                startProcessIfNeeded();
            }
        }
        return;
    }

    bridgeEnabled = enabled;
    if (bridgeEnabled) {
        pcmFramesForwarded = 0;
        inputSampleRate = 0;
        if (inputServerEnabled) {
            startInputServer();
        }
        if (udpOutputEnabled) {
            startUdpOutput();
        }
        if (processAutoStart) {
            startProcessIfNeeded();
        }
        emitStatus(QStringLiteral("DSD-neo bridge enabled"));
    } else {
        stopProcess();
        stopUdpOutput();
        stopInputServer();
        emitStatus(QStringLiteral("DSD-neo bridge disabled"));
    }
}

void DsdNeoBridge::sendInputPcm(const QByteArray &pcmData, int sampleRate) {
    if (!bridgeEnabled || !inputServerEnabled || pcmData.size() < int(sizeof(qint16))) {
        return;
    }
    if (processAutoStart) {
        startProcessIfNeeded();
    }

    if (inputSampleRate != sampleRate) {
        inputSampleRate = sampleRate;
        inputWarmupFramesRemaining = DSD_INPUT_WARMUP_FRAMES;
        emitStatus(QStringLiteral("DSD-neo input stream: %1 Hz PCM16LE").arg(inputSampleRate));
    }
    if (inputWarmupFramesRemaining > 0) {
        --inputWarmupFramesRemaining;
        return;
    }

    const QByteArray normalizedPcm = normalizeInputPcmForDsd(pcmData);
    const int evenSize = normalizedPcm.size() & ~1;
    const char *data = normalizedPcm.constData();
    for (int offset = 0; offset < evenSize; offset += int(MAX_UDP_INPUT_DATAGRAM_BYTES)) {
        int chunk = (std::min)(int(MAX_UDP_INPUT_DATAGRAM_BYTES), evenSize - offset);
        chunk &= ~1;
        if (chunk <= 0) {
            continue;
        }
        udpInputSocket->writeDatagram(data + offset,
                                      chunk,
                                      QHostAddress::LocalHost,
                                      inputPort);
        ++udpInputPacketsForwarded;
    }
    ++pcmFramesForwarded;
}

void DsdNeoBridge::startInputServer() {
    if (!inputServerEnabled || !bridgeEnabled) {
        return;
    }
    if (inputServer->isListening()) {
        inputServer->close();
    }
    emitStatus(QStringLiteral("DSD-neo UDP input sender ready: 127.0.0.1:%1").arg(inputPort));
}

void DsdNeoBridge::stopInputServer() {
    for (QTcpSocket *client : std::as_const(inputClients)) {
        if (!client) {
            continue;
        }
        client->disconnect(this);
        client->disconnectFromHost();
        client->deleteLater();
    }
    inputClients.clear();
    if (inputServer->isListening()) {
        inputServer->close();
    }
}

void DsdNeoBridge::startUdpOutput() {
    if (!udpOutputEnabled || !bridgeEnabled) {
        return;
    }
    if (udpOutputSocket->state() == QAbstractSocket::BoundState &&
        udpOutputSocket->localPort() == udpOutputPort) {
        return;
    }
    stopUdpOutput();
    const bool bound = udpOutputSocket->bind(QHostAddress::LocalHost,
                                             udpOutputPort,
                                             QUdpSocket::ShareAddress |
                                                 QUdpSocket::ReuseAddressHint);
    if (!bound) {
        emitStatus(QStringLiteral("DSD-neo UDP audio bind failed on 127.0.0.1:%1: %2")
                       .arg(udpOutputPort)
                       .arg(udpOutputSocket->errorString()));
        return;
    }
    emitStatus(QStringLiteral("DSD-neo UDP audio input ready: 127.0.0.1:%1").arg(udpOutputPort));
}

void DsdNeoBridge::stopUdpOutput() {
    if (udpOutputSocket->state() != QAbstractSocket::UnconnectedState) {
        udpOutputSocket->close();
    }
}

void DsdNeoBridge::startProcessIfNeeded() {
    if (!processAutoStart || !bridgeEnabled || processProgram.trimmed().isEmpty()) {
        return;
    }
    if (managedProcess->state() != QProcess::NotRunning) {
        return;
    }
    if (processRestartBackoffActive()) {
        return;
    }
    processLogLines = 0;
    QProcessEnvironment environment = QProcessEnvironment::systemEnvironment();
    environment.insert(QStringLiteral("DSD_NEO_NO_BOOTSTRAP"), QStringLiteral("1"));
    managedProcess->setProcessEnvironment(environment);
    managedProcess->setProgram(processProgram);
    managedProcess->setArguments(processArguments);
    if (!processWorkingDirectory.trimmed().isEmpty()) {
        managedProcess->setWorkingDirectory(processWorkingDirectory);
    }
    managedProcess->start();
    inputWarmupFramesRemaining = DSD_INPUT_WARMUP_FRAMES;
    emitStatus(QStringLiteral("Starting DSD-neo: %1 %2")
                   .arg(processProgram, processArguments.join(QLatin1Char(' '))));
}

void DsdNeoBridge::stopProcess() {
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

void DsdNeoBridge::emitStatus(const QString &status) {
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[DSD-neo]" << status;
    }
    emit statusChanged(status);
}

bool DsdNeoBridge::processRestartBackoffActive() const {
    return processRestartBackoffMs > 0 &&
           processRestartBackoffTimer.isValid() &&
           processRestartBackoffTimer.elapsed() < processRestartBackoffMs;
}

void DsdNeoBridge::armProcessRestartBackoff(const QString &reason) {
    processRestartBackoffMs = processRestartBackoffMs <= 0
                                  ? qint64(1000)
                                  : (std::min)(processRestartBackoffMs * 2, PROCESS_RESTART_BACKOFF_MAX_MS);
    processRestartBackoffTimer.restart();
    emitStatus(QStringLiteral("DSD-neo restart backoff %1 ms: %2")
                   .arg(processRestartBackoffMs)
                   .arg(reason.left(120)));
}

void DsdNeoBridge::acceptInputClient() {
    while (inputServer->hasPendingConnections()) {
        QTcpSocket *client = inputServer->nextPendingConnection();
        if (!client) {
            continue;
        }
        client->setParent(this);
        inputClients.append(client);
        connect(client, &QTcpSocket::disconnected, this, [this, client]() {
            removeInputClient(client);
        });
        connect(client,
                QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
                this,
                [this, client](QAbstractSocket::SocketError) {
                    emitStatus(QStringLiteral("DSD-neo TCP client error: %1")
                                   .arg(client ? client->errorString() : QString()));
                });
        emitStatus(QStringLiteral("DSD-neo TCP client connected: %1")
                       .arg(client->peerAddress().toString()));
    }
}

void DsdNeoBridge::removeInputClient(QTcpSocket *client) {
    inputClients.removeAll(client);
    if (client) {
        client->deleteLater();
    }
    emitStatus(QStringLiteral("DSD-neo TCP client disconnected"));
}

void DsdNeoBridge::readUdpOutput() {
    while (udpOutputSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(udpOutputSocket->pendingDatagramSize()));
        udpOutputSocket->readDatagram(datagram.data(), datagram.size());
        const QByteArray mono8k = pcm16ToMono(datagram, udpOutputChannels);
        if (mono8k.isEmpty()) {
            continue;
        }
        ++udpOutputPacketsReceived;
        if (fobosVerboseLoggingEnabled() &&
            (udpOutputPacketsReceived <= 8 || (udpOutputPacketsReceived % 100) == 0)) {
            qDebug() << "[DSD-neo] decoded UDP audio"
                     << "packet" << udpOutputPacketsReceived
                     << "bytes" << datagram.size()
                     << "monoBytes" << mono8k.size();
        }
        emit decodedPcmReady(upsample8kMonoTo48k(mono8k));
    }
}

void DsdNeoBridge::readProcessOutput() {
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
            emitStatus(QStringLiteral("DSD-neo: %1").arg(line.left(160)));
        } else if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[DSD-neo process]" << line;
        }
    }
}

QByteArray DsdNeoBridge::normalizeInputPcmForDsd(const QByteArray &pcmData) {
    const int samples = pcmData.size() / int(sizeof(qint16));
    if (samples <= 0) {
        return {};
    }

    const char *raw = pcmData.constData();
    int peak = 0;
    int clipped = 0;
    double squareSum = 0.0;
    for (int i = 0; i < samples; ++i) {
        const int sample = readLe16(raw + i * int(sizeof(qint16)));
        const int absSample = std::abs(sample);
        peak = (std::max)(peak, absSample);
        if (absSample >= 32760) {
            ++clipped;
        }
        const double normalized = static_cast<double>(sample) / 32768.0;
        squareSum += normalized * normalized;
    }

    const double peakNorm = static_cast<double>(peak) / 32767.0;
    const double rms = std::sqrt(squareSum / static_cast<double>((std::max)(1, samples)));
    const double scale =
        peakNorm > DSD_INPUT_TARGET_PEAK && peakNorm > 0.0
            ? DSD_INPUT_TARGET_PEAK / peakNorm
            : 1.0;

    if (fobosVerboseLoggingEnabled() &&
        (pcmFramesForwarded < 8 || (pcmFramesForwarded % 200) == 0)) {
        qDebug() << "[DSD-neo] input PCM"
                 << "frame" << pcmFramesForwarded
                 << "samples" << samples
                 << "peak" << QString::number(peakNorm, 'f', 3)
                 << "rms" << QString::number(rms, 'f', 3)
                 << "scale" << QString::number(scale, 'f', 3)
                 << "clipped" << clipped;
    }

    if (scale >= 0.999 && clipped == 0) {
        QByteArray even = pcmData.left(samples * int(sizeof(qint16)));
        return even;
    }

    QByteArray output;
    output.reserve(samples * int(sizeof(qint16)));
    for (int i = 0; i < samples; ++i) {
        const int sample = readLe16(raw + i * int(sizeof(qint16)));
        const int scaled = static_cast<int>(std::lrint(static_cast<double>(sample) * scale));
        const int clamped = (std::clamp)(scaled, -32768, 32767);
        appendLe16(output, static_cast<qint16>(clamped));
    }
    return output;
}

QByteArray DsdNeoBridge::pcm16ToMono(const QByteArray &pcmData, int channels) const {
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

QByteArray DsdNeoBridge::upsample8kMonoTo48k(const QByteArray &pcmData) const {
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
            const int mixed =
                (static_cast<int>(current) * (UPSAMPLE_FACTOR - step) +
                 static_cast<int>(next) * step) /
                UPSAMPLE_FACTOR;
            appendLe16(output, static_cast<qint16>(std::clamp(mixed, -32768, 32767)));
        }
    }
    return output;
}
