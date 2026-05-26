#ifndef NETWORKCONTROLLER_H
#define NETWORKCONTROLLER_H

#include <QObject>
#include <QJsonObject>
#include <QString>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>

enum class NetworkMode {
    Disabled,
    Server,
    Client
};

enum class NetworkProcessingMode {
    ServerSide = 0,
    ChannelIqClientSide = 1,
    FullIqClientSide = 2
};

class NetworkController : public QObject {
    Q_OBJECT

public:
    explicit NetworkController(QObject *parent = nullptr);
    ~NetworkController();

    NetworkMode mode() const;
    QString statusText() const;
    bool isControlReady() const;
    qint64 pendingBytes() const;

    void stop();
    void startServer(const QString &bindAddress, quint16 port);
    void testClientConnection(const QString &serverAddress, quint16 port);
    bool sendControlCommand(const QJsonObject &command);

signals:
    void statusChanged(const QString &status);
    void channelReady(const QString &message);
    void channelError(const QString &message);
    void controlCommandReceived(const QJsonObject &command);

private:
    void setStatus(const QString &status);
    void closeClientSocket();
    void closePeerSocket();
    void sendHello();
    void processSocketData(QTcpSocket *socket, QByteArray &buffer);
    void processLine(QTcpSocket *socket, const QByteArray &line);

    QTcpServer *server = nullptr;
    QTcpSocket *clientSocket = nullptr;
    QTcpSocket *peerSocket = nullptr;
    QTimer *handshakeTimer = nullptr;
    QByteArray clientReadBuffer;
    QByteArray peerReadBuffer;
    NetworkMode currentMode = NetworkMode::Disabled;
    QString currentStatus = "Network disabled";
    bool controlReady = false;
};

#endif // NETWORKCONTROLLER_H
