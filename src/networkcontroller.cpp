#include "networkcontroller.h"

#include <QHostAddress>
#include <QJsonDocument>

namespace {
constexpr int CONTROL_HANDSHAKE_TIMEOUT_MS = 3000;
const QByteArray HELLO_MESSAGE = "FOBOSAPP_HELLO 1\n";
const QByteArray OK_MESSAGE = "FOBOSAPP_OK 1\n";
}

NetworkController::NetworkController(QObject *parent)
    : QObject(parent),
      server(new QTcpServer(this)),
      handshakeTimer(new QTimer(this)) {
    handshakeTimer->setSingleShot(true);

    connect(server, &QTcpServer::newConnection, this, [this]() {
        closePeerSocket();
        peerSocket = server->nextPendingConnection();
        if (!peerSocket) {
            return;
        }

        setStatus(QString("Client connected from %1:%2")
                      .arg(peerSocket->peerAddress().toString())
                      .arg(peerSocket->peerPort()));

        connect(peerSocket, &QTcpSocket::readyRead, this, [this]() {
            processSocketData(peerSocket, peerReadBuffer);
        });

        connect(peerSocket, &QTcpSocket::disconnected, this, [this]() {
            closePeerSocket();
            controlReady = false;
            if (currentMode == NetworkMode::Server && server->isListening()) {
                setStatus(QString("Server listening on %1:%2")
                              .arg(server->serverAddress().toString())
                              .arg(server->serverPort()));
            }
        });
    });

    connect(handshakeTimer, &QTimer::timeout, this, [this]() {
        if (currentMode == NetworkMode::Client &&
            clientSocket &&
            clientSocket->state() != QAbstractSocket::ConnectedState) {
            const QString message = "Control channel timeout";
            closeClientSocket();
            setStatus(message);
            emit channelError(message);
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
    return controlReady;
}

qint64 NetworkController::pendingBytes() const {
    const QTcpSocket *socket = nullptr;
    if (currentMode == NetworkMode::Client) {
        socket = clientSocket;
    } else if (currentMode == NetworkMode::Server) {
        socket = peerSocket;
    }
    return socket ? socket->bytesToWrite() : 0;
}

void NetworkController::stop() {
    handshakeTimer->stop();
    controlReady = false;
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
    controlReady = false;
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
    if (server->isListening()) {
        server->close();
    }
    closeClientSocket();
    closePeerSocket();
    controlReady = false;

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

    connect(clientSocket,
            &QTcpSocket::errorOccurred,
            this,
            [this](QAbstractSocket::SocketError) {
                handshakeTimer->stop();
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
    QTcpSocket *socket = nullptr;
    if (currentMode == NetworkMode::Client) {
        socket = clientSocket;
    } else if (currentMode == NetworkMode::Server) {
        socket = peerSocket;
    }

    if (!socket || socket->state() != QAbstractSocket::ConnectedState || !controlReady) {
        setStatus("Control channel is not ready");
        return false;
    }

    const QByteArray payload = QJsonDocument(command).toJson(QJsonDocument::Compact) + '\n';
    return socket->write(payload) == payload.size();
}

void NetworkController::setStatus(const QString &status) {
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

void NetworkController::closePeerSocket() {
    if (!peerSocket) {
        return;
    }
    peerSocket->disconnect(this);
    peerSocket->disconnectFromHost();
    peerSocket->deleteLater();
    peerSocket = nullptr;
    peerReadBuffer.clear();
}

void NetworkController::sendHello() {
    if (!clientSocket) {
        return;
    }
    clientSocket->write(HELLO_MESSAGE);
    clientSocket->flush();
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
            processLine(socket, line);
        }
    }
}

void NetworkController::processLine(QTcpSocket *socket, const QByteArray &line) {
    if (line == HELLO_MESSAGE.trimmed()) {
        if (currentMode == NetworkMode::Server && socket == peerSocket) {
            controlReady = true;
            peerSocket->write(OK_MESSAGE);
            peerSocket->flush();
            const QString message = QString("Control channel ready: %1:%2")
                                        .arg(peerSocket->peerAddress().toString())
                                        .arg(peerSocket->peerPort());
            setStatus(message);
            emit channelReady(message);
        }
        return;
    }

    if (line == OK_MESSAGE.trimmed()) {
        if (currentMode == NetworkMode::Client && socket == clientSocket) {
            handshakeTimer->stop();
            controlReady = true;
            const QString message = QString("Control channel ready: %1:%2")
                                        .arg(clientSocket->peerAddress().toString())
                                        .arg(clientSocket->peerPort());
            setStatus(message);
            emit channelReady(message);
        }
        return;
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(line, &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject()) {
        setStatus(QString("Ignoring invalid control message: %1").arg(parseError.errorString()));
        return;
    }

    emit controlCommandReceived(document.object());
}
