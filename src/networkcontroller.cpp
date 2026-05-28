#include "networkcontroller.h"

#include <algorithm>
#include <QHostAddress>
#include <QJsonDocument>
#include <utility>

namespace {
constexpr int CONTROL_HANDSHAKE_TIMEOUT_MS = 3000;
constexpr int CONTROL_HEARTBEAT_INTERVAL_MS = 1000;
constexpr int CONTROL_HEARTBEAT_TIMEOUT_MS = 5000;
const QByteArray HELLO_MESSAGE = "FOBOSAPP_HELLO 1\n";
const QByteArray OK_MESSAGE = "FOBOSAPP_OK 1\n";
const QByteArray PING_MESSAGE = "FOBOSAPP_PING 1\n";
const QByteArray PONG_MESSAGE = "FOBOSAPP_PONG 1\n";
}

NetworkController::NetworkController(QObject *parent)
    : QObject(parent),
      server(new QTcpServer(this)),
      handshakeTimer(new QTimer(this)),
      heartbeatTimer(new QTimer(this)) {
    handshakeTimer->setSingleShot(true);
    heartbeatTimer->setInterval(CONTROL_HEARTBEAT_INTERVAL_MS);

    connect(server, &QTcpServer::newConnection, this, [this]() {
        while (server->hasPendingConnections()) {
            QTcpSocket *socket = server->nextPendingConnection();
            if (!socket) {
                continue;
            }

            const QString id = QString("client-%1").arg(nextPeerId++);
            const QString label = QString("%1:%2")
                                      .arg(socket->peerAddress().toString())
                                      .arg(socket->peerPort());
            const QString priorityKey = socket->peerAddress().toString();

            peerSockets.append(socket);
            peerIds.insert(socket, id);
            peerLabels.insert(socket, label);
            peerPriorityKeys.insert(socket, priorityKey);
            peerReadBuffers.insert(socket, QByteArray());

            setStatus(QString("Client connected from %1").arg(label));

            connect(socket, &QTcpSocket::readyRead, this, [this, socket]() {
                if (!peerSockets.contains(socket)) {
                    return;
                }
                processSocketData(socket, peerReadBuffers[socket]);
            });

            connect(socket, &QTcpSocket::disconnected, this, [this, socket]() {
                const QString label = peerLabel(socket);
                const bool wasController = socket == peerSocket;
                closePeerSocket(socket);
                promoteControllerIfNeeded();
                if (currentMode == NetworkMode::Server && server->isListening()) {
                    const QString message = QString("%1 disconnected%2; server listening on %3:%4")
                                                .arg(label)
                                                .arg(wasController ? QStringLiteral(" (controller)") : QString())
                                                .arg(server->serverAddress().toString())
                                                .arg(server->serverPort());
                    setStatus(message);
                    emit channelError(message);
                }
            });

            connect(socket,
                    &QTcpSocket::errorOccurred,
                    this,
                    [this, socket](QAbstractSocket::SocketError) {
                        const QString message = QString("Client %1 socket error: %2")
                                                    .arg(peerLabel(socket))
                                                    .arg(socket ? socket->errorString() : QString("socket closed"));
                        closePeerSocket(socket);
                        promoteControllerIfNeeded();
                        setStatus(message);
                        emit channelError(message);
                    });
        }
    });

    connect(handshakeTimer, &QTimer::timeout, this, [this]() {
        if (currentMode == NetworkMode::Client &&
            clientSocket &&
            !controlReady) {
            const QString message = "Control channel timeout";
            closeClientSocket();
            setStatus(message);
            emit channelError(message);
        }
    });

    connect(heartbeatTimer, &QTimer::timeout, this, [this]() {
        if (currentMode == NetworkMode::Client) {
            if (!controlReady) {
                stopHeartbeat();
                return;
            }

            QTcpSocket *socket = activeSocket();
            if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
                handleConnectionLost("Control channel lost");
                return;
            }

            if (lastMessageTimer.isValid() &&
                lastMessageTimer.elapsed() > CONTROL_HEARTBEAT_TIMEOUT_MS) {
                handleConnectionLost("Control channel heartbeat timeout");
                return;
            }

            sendProtocolLine(socket, PING_MESSAGE);
            return;
        }

        if (currentMode != NetworkMode::Server) {
            stopHeartbeat();
            return;
        }

        for (QTcpSocket *socket : std::as_const(peerSockets)) {
            if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
                continue;
            }
            QElapsedTimer &timer = peerLastMessageTimers[socket];
            if (timer.isValid() && timer.elapsed() > CONTROL_HEARTBEAT_TIMEOUT_MS) {
                const QString message = QString("Client %1 heartbeat timeout").arg(peerLabel(socket));
                closePeerSocket(socket);
                promoteControllerIfNeeded();
                setStatus(message);
                emit channelError(message);
                return;
            }
            sendProtocolLine(socket, PING_MESSAGE);
        }
    });
}

NetworkController::~NetworkController() {
    stop();
}

NetworkMode NetworkController::mode() const {
    return currentMode;
}

QString NetworkController::statusText() const {
    return currentStatus;
}

bool NetworkController::isControlReady() const {
    if (currentMode == NetworkMode::Server) {
        return !readyPeerSockets.isEmpty();
    }
    return controlReady;
}

qint64 NetworkController::pendingBytes() const {
    if (currentMode == NetworkMode::Client) {
        return clientSocket ? clientSocket->bytesToWrite() : 0;
    }

    qint64 maxPending = 0;
    if (currentMode == NetworkMode::Server) {
        for (QTcpSocket *socket : readyPeerSockets) {
            if (socket) {
                maxPending = (std::max)(maxPending, socket->bytesToWrite());
            }
        }
    }
    return maxPending;
}

bool NetworkController::clientHasControl() const {
    return currentMode != NetworkMode::Client || localClientHasControl;
}

QString NetworkController::controllerPeerId() const {
    return peerId(peerSocket);
}

void NetworkController::stop() {
    handshakeTimer->stop();
    stopHeartbeat();
    controlReady = false;
    localClientHasControl = true;
    closeClientSocket();
    closePeerSocket();
    if (server->isListening()) {
        server->close();
    }
    currentMode = NetworkMode::Disabled;
    setStatus("Network disabled");
}

void NetworkController::startServer(const QString &bindAddress, quint16 port) {
    handshakeTimer->stop();
    stopHeartbeat();
    controlReady = false;
    localClientHasControl = true;
    closeClientSocket();
    closePeerSocket();
    if (server->isListening()) {
        server->close();
    }

    const QHostAddress address(bindAddress.trimmed().isEmpty() ? QStringLiteral("0.0.0.0") : bindAddress.trimmed());
    if (!server->listen(address, port)) {
        currentMode = NetworkMode::Disabled;
        const QString message = QString("Server listen failed: %1").arg(server->errorString());
        setStatus(message);
        emit channelError(message);
        return;
    }

    currentMode = NetworkMode::Server;
    setStatus(QString("Server listening on %1:%2")
                  .arg(server->serverAddress().toString())
                  .arg(server->serverPort()));
}

void NetworkController::testClientConnection(const QString &serverAddress, quint16 port) {
    handshakeTimer->stop();
    stopHeartbeat();
    if (server->isListening()) {
        server->close();
    }
    closeClientSocket();
    closePeerSocket();
    controlReady = false;
    localClientHasControl = true;

    clientSocket = new QTcpSocket(this);
    currentMode = NetworkMode::Client;
    setStatus(QString("Connecting to %1:%2").arg(serverAddress).arg(port));

    connect(clientSocket, &QTcpSocket::connected, this, [this]() {
        sendHello();
        setStatus("Connected, waiting for server handshake");
    });

    connect(clientSocket, &QTcpSocket::readyRead, this, [this]() {
        processSocketData(clientSocket, clientReadBuffer);
    });

    connect(clientSocket, &QTcpSocket::disconnected, this, [this]() {
        handshakeTimer->stop();
        stopHeartbeat();
        controlReady = false;
        closeClientSocket();
        if (currentMode == NetworkMode::Client) {
            const QString message = "Disconnected from server";
            setStatus(message);
            emit channelError(message);
        }
    });

    connect(clientSocket,
            &QTcpSocket::errorOccurred,
            this,
            [this](QAbstractSocket::SocketError) {
                handshakeTimer->stop();
                stopHeartbeat();
                const QString message = QString("Client connection failed: %1")
                                            .arg(clientSocket ? clientSocket->errorString() : QString("socket closed"));
                controlReady = false;
                closeClientSocket();
                currentMode = NetworkMode::Disabled;
                setStatus(message);
                emit channelError(message);
            });

    clientSocket->connectToHost(serverAddress, port);
    handshakeTimer->start(CONTROL_HANDSHAKE_TIMEOUT_MS);
}

bool NetworkController::sendControlCommand(const QJsonObject &command) {
    if (currentMode == NetworkMode::Client) {
        QTcpSocket *socket = activeSocket();
        if (!socket || socket->state() != QAbstractSocket::ConnectedState || !controlReady) {
            setStatus("Control channel is not ready");
            return false;
        }
        return sendJsonToSocket(socket, command);
    }

    if (currentMode != NetworkMode::Server || readyPeerSockets.isEmpty()) {
        setStatus("Control channel is not ready");
        return false;
    }

    bool sentAny = false;
    for (QTcpSocket *socket : std::as_const(peerSockets)) {
        if (!isReadyPeer(socket)) {
            continue;
        }
        sentAny = sendJsonToSocket(socket, command) || sentAny;
    }
    return sentAny;
}

bool NetworkController::sendControlCommandToPeer(const QString &peerIdValue, const QJsonObject &command) {
    QTcpSocket *socket = peerForId(peerIdValue);
    if (!isReadyPeer(socket)) {
        return false;
    }
    return sendJsonToSocket(socket, command);
}

bool NetworkController::sendControlCommandToController(const QJsonObject &command) {
    if (!isReadyPeer(peerSocket)) {
        return false;
    }
    return sendJsonToSocket(peerSocket, command);
}

bool NetworkController::setControllerPeer(const QString &peerIdValue) {
    QTcpSocket *socket = peerForId(peerIdValue);
    if (!isReadyPeer(socket)) {
        return false;
    }
    peerSocket = socket;
    broadcastRoleUpdates();
    setStatus(QString("Controller client: %1").arg(peerLabel(peerSocket)));
    return true;
}

bool NetworkController::blockPriorityRequestsFromPeer(const QString &peerIdValue) {
    QTcpSocket *socket = peerForId(peerIdValue);
    if (!socket) {
        return false;
    }
    blockedPriorityRequesterKeys.insert(peerPriorityKey(socket));
    return true;
}

bool NetworkController::isPriorityRequestBlocked(const QString &peerIdValue) const {
    QTcpSocket *socket = peerForId(peerIdValue);
    return socket && blockedPriorityRequesterKeys.contains(peerPriorityKey(socket));
}

void NetworkController::setStatus(const QString &status) {
    if (currentStatus == status) {
        return;
    }
    currentStatus = status;
    emit statusChanged(status);
}

void NetworkController::closeClientSocket() {
    if (!clientSocket) {
        return;
    }
    clientSocket->disconnect(this);
    clientSocket->disconnectFromHost();
    clientSocket->deleteLater();
    clientSocket = nullptr;
    clientReadBuffer.clear();
}

void NetworkController::closePeerSocket(QTcpSocket *socket) {
    auto closeOne = [this](QTcpSocket *peer) {
        if (!peer) {
            return;
        }
        readyPeerSockets.remove(peer);
        peerReadBuffers.remove(peer);
        peerIds.remove(peer);
        peerLabels.remove(peer);
        peerPriorityKeys.remove(peer);
        peerLastMessageTimers.remove(peer);
        peerSockets.removeAll(peer);
        if (peerSocket == peer) {
            peerSocket = nullptr;
        }
        peer->disconnect(this);
        peer->disconnectFromHost();
        peer->deleteLater();
    };

    if (socket) {
        closeOne(socket);
        return;
    }

    const QVector<QTcpSocket*> sockets = peerSockets;
    for (QTcpSocket *peer : sockets) {
        closeOne(peer);
    }
    peerSockets.clear();
    readyPeerSockets.clear();
    peerReadBuffers.clear();
    peerIds.clear();
    peerLabels.clear();
    peerPriorityKeys.clear();
    peerLastMessageTimers.clear();
    peerSocket = nullptr;
}

void NetworkController::sendHello() {
    if (!clientSocket) {
        return;
    }
    sendProtocolLine(clientSocket, HELLO_MESSAGE);
}

void NetworkController::startHeartbeat() {
    if (currentMode == NetworkMode::Client) {
        lastMessageTimer.restart();
    }
    if (!heartbeatTimer->isActive()) {
        heartbeatTimer->start();
    }
}

void NetworkController::stopHeartbeat() {
    if (heartbeatTimer->isActive()) {
        heartbeatTimer->stop();
    }
    lastMessageTimer.invalidate();
    peerLastMessageTimers.clear();
}

void NetworkController::sendProtocolLine(QTcpSocket *socket, const QByteArray &line) {
    if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
        return;
    }
    socket->write(line);
    socket->flush();
}

QTcpSocket *NetworkController::activeSocket() const {
    if (currentMode == NetworkMode::Client) {
        return clientSocket;
    }
    if (currentMode == NetworkMode::Server) {
        return peerSocket;
    }
    return nullptr;
}

bool NetworkController::sendJsonToSocket(QTcpSocket *socket, const QJsonObject &command) {
    if (!socket || socket->state() != QAbstractSocket::ConnectedState) {
        return false;
    }
    const QByteArray payload = QJsonDocument(command).toJson(QJsonDocument::Compact) + '\n';
    const qint64 written = socket->write(payload);
    if (written != payload.size()) {
        const QString message = QString("Control channel write failed: %1").arg(socket->errorString());
        if (currentMode == NetworkMode::Server && peerSockets.contains(socket)) {
            closePeerSocket(socket);
            promoteControllerIfNeeded();
            setStatus(message);
            emit channelError(message);
        } else {
            handleConnectionLost(message);
        }
        return false;
    }
    return true;
}

QTcpSocket *NetworkController::peerForId(const QString &peerIdValue) const {
    for (QTcpSocket *socket : peerSockets) {
        if (peerIds.value(socket) == peerIdValue) {
            return socket;
        }
    }
    return nullptr;
}

QString NetworkController::peerId(QTcpSocket *socket) const {
    return peerIds.value(socket);
}

QString NetworkController::peerLabel(QTcpSocket *socket) const {
    return peerLabels.value(socket, QString("unknown client"));
}

QString NetworkController::peerPriorityKey(QTcpSocket *socket) const {
    return peerPriorityKeys.value(socket, peerLabel(socket));
}

bool NetworkController::isReadyPeer(QTcpSocket *socket) const {
    return socket &&
           socket->state() == QAbstractSocket::ConnectedState &&
           readyPeerSockets.contains(socket);
}

void NetworkController::promoteControllerIfNeeded() {
    if (isReadyPeer(peerSocket)) {
        return;
    }

    peerSocket = nullptr;
    for (QTcpSocket *socket : std::as_const(peerSockets)) {
        if (isReadyPeer(socket)) {
            peerSocket = socket;
            break;
        }
    }

    if (peerSocket) {
        broadcastRoleUpdates();
    } else if (currentMode == NetworkMode::Server) {
        controlReady = false;
        if (heartbeatTimer->isActive()) {
            heartbeatTimer->stop();
        }
    }
}

void NetworkController::sendRoleUpdate(QTcpSocket *socket) {
    if (!isReadyPeer(socket)) {
        return;
    }

    QJsonObject role;
    role["type"] = "control";
    role["action"] = "role";
    role["peerId"] = peerId(socket);
    role["peerLabel"] = peerLabel(socket);
    role["controllerPeerId"] = controllerPeerId();
    role["canControl"] = socket == peerSocket;
    sendJsonToSocket(socket, role);
}

void NetworkController::broadcastRoleUpdates() {
    for (QTcpSocket *socket : std::as_const(peerSockets)) {
        sendRoleUpdate(socket);
    }
}

void NetworkController::handleConnectionLost(const QString &message) {
    handshakeTimer->stop();

    if (currentMode == NetworkMode::Client) {
        stopHeartbeat();
        controlReady = false;
        closeClientSocket();
        setStatus(message);
        emit channelError(message);
        return;
    }

    if (currentMode == NetworkMode::Server) {
        QTcpSocket *senderSocket = qobject_cast<QTcpSocket*>(sender());
        if (senderSocket && peerSockets.contains(senderSocket)) {
            closePeerSocket(senderSocket);
        } else if (peerSocket) {
            closePeerSocket(peerSocket);
        }
        promoteControllerIfNeeded();
        if (server->isListening()) {
            const QString status = QString("%1; server listening on %2:%3")
                                       .arg(message)
                                       .arg(server->serverAddress().toString())
                                       .arg(server->serverPort());
            setStatus(status);
            emit channelError(status);
        } else {
            setStatus(message);
            emit channelError(message);
        }
    }
}

void NetworkController::processSocketData(QTcpSocket *socket, QByteArray &buffer) {
    if (!socket) {
        return;
    }

    buffer += socket->readAll();
    int newlineIndex = -1;
    while ((newlineIndex = buffer.indexOf('\n')) >= 0) {
        QByteArray line = buffer.left(newlineIndex).trimmed();
        buffer.remove(0, newlineIndex + 1);
        if (!line.isEmpty()) {
            if (currentMode == NetworkMode::Client) {
                lastMessageTimer.restart();
            } else if (currentMode == NetworkMode::Server) {
                peerLastMessageTimers[socket].restart();
            }
            processLine(socket, line);
        }
    }
}

void NetworkController::processLine(QTcpSocket *socket, const QByteArray &line) {
    if (line == HELLO_MESSAGE.trimmed()) {
        if (currentMode == NetworkMode::Server && peerSockets.contains(socket)) {
            if (readyPeerSockets.contains(socket)) {
                return;
            }
            readyPeerSockets.insert(socket);
            peerLastMessageTimers[socket].restart();
            if (!peerSocket) {
                peerSocket = socket;
            }
            sendProtocolLine(socket, OK_MESSAGE);
            const QString message = QString("Control channel ready: %1")
                                        .arg(peerLabel(socket));
            setStatus(message);
            if (socket == peerSocket) {
                broadcastRoleUpdates();
            } else {
                sendRoleUpdate(socket);
            }
            startHeartbeat();
            controlReady = true;
            emit channelReady(message);
        }
        return;
    }

    if (line == OK_MESSAGE.trimmed()) {
        if (currentMode == NetworkMode::Client && socket == clientSocket) {
            if (controlReady) {
                return;
            }
            handshakeTimer->stop();
            controlReady = true;
            const QString message = QString("Control channel ready: %1:%2")
                                        .arg(clientSocket->peerAddress().toString())
                                        .arg(clientSocket->peerPort());
            setStatus(message);
            startHeartbeat();
            emit channelReady(message);
        }
        return;
    }

    if (line == PING_MESSAGE.trimmed()) {
        sendProtocolLine(socket, PONG_MESSAGE);
        return;
    }

    if (line == PONG_MESSAGE.trimmed()) {
        return;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(line, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        setStatus(QString("Ignoring invalid control message: %1").arg(parseError.errorString()));
        return;
    }

    QJsonObject command = document.object();
    if (currentMode == NetworkMode::Client &&
        command.value("type").toString() == QStringLiteral("control") &&
        command.value("action").toString() == QStringLiteral("role")) {
        localClientHasControl = command.value("canControl").toBool(localClientHasControl);
    }
    if (currentMode == NetworkMode::Server && peerSockets.contains(socket)) {
        command["_networkPeerId"] = peerId(socket);
        command["_networkPeerLabel"] = peerLabel(socket);
        command["_networkPeerIsController"] = socket == peerSocket;
    }
    emit controlCommandReceived(command);
}
