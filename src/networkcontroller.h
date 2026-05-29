#ifndef NETWORKCONTROLLER_H
#define NETWORKCONTROLLER_H

#include <QObject>
#include <QJsonObject>
#include <QString>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTimer>
#include <QElapsedTimer>
#include <QHash>
#include <QSet>
#include <QVector>

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
    bool clientHasControl() const;
    QString controllerPeerId() const;

    void stop();
    void startServer(const QString &bindAddress, quint16 port);
    void testClientConnection(const QString &serverAddress, quint16 port);
    bool sendControlCommand(const QJsonObject &command);
    bool sendBinaryCommand(const QJsonObject &command, const QByteArray &payload);
    bool sendControlCommandToPeer(const QString &peerId, const QJsonObject &command);
    bool sendControlCommandToController(const QJsonObject &command);
    bool setControllerPeer(const QString &peerId);
    bool blockPriorityRequestsFromPeer(const QString &peerId);
    bool isPriorityRequestBlocked(const QString &peerId) const;

signals:
    void statusChanged(const QString &status);
    void channelReady(const QString &message);
    void channelError(const QString &message);
    void controlCommandReceived(const QJsonObject &command);
    void binaryCommandReceived(const QJsonObject &command, const QByteArray &payload);

private:
    void setStatus(const QString &status);
    void closeClientSocket();
    void closePeerSocket(QTcpSocket *socket = nullptr);
    void sendHello();
    void startHeartbeat();
    void stopHeartbeat();
    void sendProtocolLine(QTcpSocket *socket, const QByteArray &line);
    QTcpSocket *activeSocket() const;
    bool sendJsonToSocket(QTcpSocket *socket, const QJsonObject &command);
    bool sendBinaryToSocket(QTcpSocket *socket, const QJsonObject &command, const QByteArray &payload);
    QTcpSocket *peerForId(const QString &peerId) const;
    QString peerId(QTcpSocket *socket) const;
    QString peerLabel(QTcpSocket *socket) const;
    QString peerPriorityKey(QTcpSocket *socket) const;
    bool isReadyPeer(QTcpSocket *socket) const;
    void promoteControllerIfNeeded();
    void sendRoleUpdate(QTcpSocket *socket);
    void broadcastRoleUpdates();
    void handleConnectionLost(const QString &message);
    void processSocketData(QTcpSocket *socket, QByteArray &buffer);
    void processLine(QTcpSocket *socket, const QByteArray &line);
    bool deliverPendingBinaryPayload(QTcpSocket *socket, QByteArray &buffer);

    QTcpServer *server = nullptr;
    QTcpSocket *clientSocket = nullptr;
    QTcpSocket *peerSocket = nullptr;
    QVector<QTcpSocket*> peerSockets;
    QSet<QTcpSocket*> readyPeerSockets;
    QHash<QTcpSocket*, QByteArray> peerReadBuffers;
    QHash<QTcpSocket*, QJsonObject> peerPendingBinaryCommands;
    QHash<QTcpSocket*, qint64> peerPendingBinaryBytes;
    QHash<QTcpSocket*, QString> peerIds;
    QHash<QTcpSocket*, QString> peerLabels;
    QHash<QTcpSocket*, QString> peerPriorityKeys;
    QSet<QString> blockedPriorityRequesterKeys;
    QTimer *handshakeTimer = nullptr;
    QTimer *heartbeatTimer = nullptr;
    QElapsedTimer lastMessageTimer;
    QHash<QTcpSocket*, QElapsedTimer> peerLastMessageTimers;
    QByteArray clientReadBuffer;
    QJsonObject clientPendingBinaryCommand;
    qint64 clientPendingBinaryBytes = 0;
    NetworkMode currentMode = NetworkMode::Disabled;
    QString currentStatus = "Network disabled";
    bool controlReady = false;
    bool localClientHasControl = true;
    quint64 nextPeerId = 1;
};

#endif // NETWORKCONTROLLER_H
