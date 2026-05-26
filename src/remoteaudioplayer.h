#ifndef REMOTEAUDIOPLAYER_H
#define REMOTEAUDIOPLAYER_H

#include <QObject>
#include <QByteArray>
#include <QList>
#include <QTimer>
#include <windows.h>
#include <mmsystem.h>

class RemoteAudioPlayer : public QObject {
    Q_OBJECT

public:
    explicit RemoteAudioPlayer(QObject *parent = nullptr);
    ~RemoteAudioPlayer();

    void playPcmFrame(const QByteArray &pcmData);
    void stop();

private slots:
    void cleanupCompletedBuffers();

private:
    struct AudioBlock {
        WAVEHDR header;
        QByteArray data;
    };

    bool openDevice();
    void releaseBlock(AudioBlock *block);

    HWAVEOUT waveOut = nullptr;
    QList<AudioBlock *> activeBlocks;
    QTimer cleanupTimer;

    static constexpr int SAMPLE_RATE = 48000;
    static constexpr int MAX_ACTIVE_BLOCKS = 24;
};

#endif // REMOTEAUDIOPLAYER_H
