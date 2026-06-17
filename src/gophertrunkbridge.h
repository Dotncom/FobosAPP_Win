#ifndef GOPHERTRUNKBRIDGE_H
#define GOPHERTRUNKBRIDGE_H

#include <QByteArray>
#include <QElapsedTimer>
#include <QObject>
#include <QProcess>
#include <QTcpSocket>
#include <QUdpSocket>

class GopherTrunkBridge : public QObject {
    Q_OBJECT

public:
    explicit GopherTrunkBridge(QObject *parent = nullptr);
    ~GopherTrunkBridge() override;

    void configure(bool autoStart,
                   const QString &program,
                   quint16 tcpPort,
                   quint16 audioUdpPort,
                   const QString &workingDirectory = QString());
    void setEnabled(bool enabled);
    bool isEnabled() const { return bridgeEnabled; }

    void resetStream();
    void sendDibitBurst(const QString &dibits,
                        quint64 sample,
                        quint64 baseDibit,
                        int burstIndex,
                        int cadenceSymbols,
                        int colorCode);

signals:
    void statusChanged(const QString &status);
    void decodedPcmReady(const QByteArray &pcmData);

private:
    void startProcessIfNeeded();
    void stopProcess();
    void connectTcpIfNeeded();
    void disconnectTcp();
    void startUdpOutput();
    void stopUdpOutput();
    void readTcpOutput();
    void readProcessOutput();
    void readUdpOutput();
    void emitStatus(const QString &status);
    bool processRestartBackoffActive() const;
    void armProcessRestartBackoff(const QString &reason);
    QByteArray pcm16ToMono(const QByteArray &pcmData, int channels) const;
    QByteArray normalizeOutputPcmForPlayback(const QByteArray &pcmData, quint64 packetIndex) const;
    QByteArray upsample8kMonoTo48k(const QByteArray &pcmData) const;

    bool bridgeEnabled = false;
    bool processAutoStart = false;
    bool processStoppingIntentionally = false;
    bool tcpConnectPending = false;
    quint16 tcpPort = 7460;
    quint16 audioUdpPort = 23456;
    QString processProgram;
    QStringList processArguments;
    QString processWorkingDirectory;

    QProcess *managedProcess = nullptr;
    QTcpSocket *tcpSocket = nullptr;
    QUdpSocket *udpOutputSocket = nullptr;
    QElapsedTimer processRestartBackoffTimer;
    qint64 processRestartBackoffMs = 0;
    quint64 dibitPacketsSent = 0;
    quint64 virtualBaseDibit = 0;
    quint64 udpOutputPacketsReceived = 0;
    quint64 processLogLines = 0;
    quint64 tcpOutputLines = 0;
    bool streamPolarityKnown = false;
    bool streamPolarityFlip = false;
    int lastColorCode = -1;
};

#endif // GOPHERTRUNKBRIDGE_H
