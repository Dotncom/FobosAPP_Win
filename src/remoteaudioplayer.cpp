#include "remoteaudioplayer.h"

#include <QDebug>
#include <cstring>

#pragma comment(lib, "winmm.lib")

RemoteAudioPlayer::RemoteAudioPlayer(QObject *parent)
    : QObject(parent) {
    cleanupTimer.setInterval(100);
    connect(&cleanupTimer, &QTimer::timeout, this, &RemoteAudioPlayer::cleanupCompletedBuffers);
}

RemoteAudioPlayer::~RemoteAudioPlayer() {
    stop();
}

bool RemoteAudioPlayer::openDevice() {
    if (waveOut) {
        return true;
    }

    WAVEFORMATEX format;
    std::memset(&format, 0, sizeof(format));
    format.wFormatTag = WAVE_FORMAT_PCM;
    format.nChannels = 1;
    format.nSamplesPerSec = SAMPLE_RATE;
    format.wBitsPerSample = 16;
    format.nBlockAlign = static_cast<WORD>((format.nChannels * format.wBitsPerSample) / 8);
    format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;

    const MMRESULT result = waveOutOpen(&waveOut,
                                        WAVE_MAPPER,
                                        &format,
                                        0,
                                        0,
                                        CALLBACK_NULL);
    if (result != MMSYSERR_NOERROR) {
        waveOut = nullptr;
        qDebug() << "[NetworkAudio] waveOutOpen failed" << result;
        return false;
    }

    cleanupTimer.start();
    qDebug() << "[NetworkAudio] remote audio output opened";
    return true;
}

void RemoteAudioPlayer::playPcmFrame(const QByteArray &pcmData) {
    if (pcmData.isEmpty()) {
        return;
    }
    if (!openDevice()) {
        return;
    }

    cleanupCompletedBuffers();
    if (activeBlocks.size() >= MAX_ACTIVE_BLOCKS) {
        qDebug() << "[NetworkAudio] dropping remote audio frame because playback queue is full"
                 << activeBlocks.size();
        return;
    }

    auto *block = new AudioBlock();
    std::memset(&block->header, 0, sizeof(WAVEHDR));
    block->data = pcmData;
    block->header.lpData = block->data.data();
    block->header.dwBufferLength = static_cast<DWORD>(block->data.size());

    MMRESULT result = waveOutPrepareHeader(waveOut, &block->header, sizeof(WAVEHDR));
    if (result != MMSYSERR_NOERROR) {
        qDebug() << "[NetworkAudio] waveOutPrepareHeader failed" << result;
        delete block;
        return;
    }

    result = waveOutWrite(waveOut, &block->header, sizeof(WAVEHDR));
    if (result != MMSYSERR_NOERROR) {
        qDebug() << "[NetworkAudio] waveOutWrite failed" << result;
        releaseBlock(block);
        return;
    }

    activeBlocks.append(block);
}

void RemoteAudioPlayer::cleanupCompletedBuffers() {
    if (!waveOut) {
        return;
    }

    for (int i = activeBlocks.size() - 1; i >= 0; --i) {
        AudioBlock *block = activeBlocks.at(i);
        if (block->header.dwFlags & WHDR_DONE) {
            activeBlocks.removeAt(i);
            releaseBlock(block);
        }
    }
}

void RemoteAudioPlayer::releaseBlock(AudioBlock *block) {
    if (!block) {
        return;
    }
    if (waveOut && (block->header.dwFlags & WHDR_PREPARED)) {
        waveOutUnprepareHeader(waveOut, &block->header, sizeof(WAVEHDR));
    }
    delete block;
}

void RemoteAudioPlayer::stop() {
    cleanupTimer.stop();

    if (waveOut) {
        waveOutReset(waveOut);
    }

    while (!activeBlocks.isEmpty()) {
        releaseBlock(activeBlocks.takeLast());
    }

    if (waveOut) {
        waveOutClose(waveOut);
        waveOut = nullptr;
        qDebug() << "[NetworkAudio] remote audio output closed";
    }
}
