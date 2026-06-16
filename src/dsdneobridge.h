#ifndef DSDNEOBRIDGE_H
#define DSDNEOBRIDGE_H

#include <QByteArray>
#include <QElapsedTimer>
#include <QList>
#include <QObject>
#include <QProcess>
#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>

class DsdNeoBridge : public QObject {
    Q_OBJECT

public:
    explicit DsdNeoBridge(QObject *parent = nullptr);
    ~DsdNeoBridge() override;

    void configureInputServer(bool enabled, quint16 port);
    void configureUdpOutput(bool enabled, quint16 port, int channels);
    void configureProcess(bool autoStart,
                          const QString &program,
                          const QStringList &arguments,
                          const QString &workingDirectory = QString());
    void setEnabled(bool enabled);
    bool isEnabled() const { return bridgeEnabled; }

    void sendInputPcm(const QByteArray &pcmData, int sampleRate);

signals:
    void statusChanged(const QString &status);
    void decodedPcmReady(const QByteArray &pcmData);

private:
    void startInputServer();
    void stopInputServer();
    void startUdpOutput();
    void stopUdpOutput();
    void startProcessIfNeeded();
    void stopProcess();
    void emitStatus(const QString &status);
    bool processRestartBackoffActive() const;
    void armProcessRestartBackoff(const QString &reason);
    void acceptInputClient();
    void removeInputClient(QTcpSocket *client);
    void readUdpOutput();
    void readProcessOutput();

    QByteArray normalizeInputPcmForDsd(const QByteArray &pcmData);
    QByteArray pcm16ToMono(const QByteArray &pcmData, int channels) const;
    QByteArray upsample8kMonoTo48k(const QByteArray &pcmData) const;

    bool bridgeEnabled = false;
    bool inputServerEnabled = true;
    bool udpOutputEnabled = true;
    bool processAutoStart = false;
    quint16 inputPort = 7355;
    quint16 udpOutputPort = 23456;
    int udpOutputChannels = 1;
    int inputSampleRate = 0;

    QString processProgram;
    QStringList processArguments;
    QString processWorkingDirectory;

    QTcpServer *inputServer = nullptr;
    QUdpSocket *udpInputSocket = nullptr;
    QUdpSocket *udpOutputSocket = nullptr;
    QProcess *managedProcess = nullptr;
    QList<QTcpSocket *> inputClients;
    QElapsedTimer noClientLogTimer;
    QElapsedTimer processRestartBackoffTimer;
    qint64 processRestartBackoffMs = 0;
    bool processStoppingIntentionally = false;
    int inputWarmupFramesRemaining = 0;
    quint64 pcmFramesForwarded = 0;
    quint64 udpInputPacketsForwarded = 0;
    quint64 udpOutputPacketsReceived = 0;
    quint64 processLogLines = 0;
};

#endif // DSDNEOBRIDGE_H
