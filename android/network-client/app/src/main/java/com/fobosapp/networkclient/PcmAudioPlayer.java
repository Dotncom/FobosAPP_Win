package com.fobosapp.networkclient;

import android.media.AudioAttributes;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;
import android.os.Process;

import java.util.ArrayDeque;

final class PcmAudioPlayer {
    private static final int SAMPLE_RATE = 48_000;
    private static final int BYTES_PER_SAMPLE = 2;
    private static final int MAX_QUEUED_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE * 3 / 4;
    private static final int START_BUFFER_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE / 5;
    private static final int MIN_BUFFER_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE / 5;

    private final Object lock = new Object();
    private final ArrayDeque<byte[]> queue = new ArrayDeque<>();
    private AudioTrack track;
    private Thread audioThread;
    private boolean running;
    private int queuedBytes;

    PcmAudioPlayer() {
        synchronized (lock) {
            startThreadLocked();
        }
    }

    void playPcm(byte[] pcmData) {
        if (pcmData == null || pcmData.length == 0) {
            return;
        }
        byte[] copy = new byte[pcmData.length];
        System.arraycopy(pcmData, 0, copy, 0, pcmData.length);
        synchronized (lock) {
            startThreadLocked();
            queue.addLast(copy);
            queuedBytes += copy.length;
            while (queuedBytes > MAX_QUEUED_BYTES && !queue.isEmpty()) {
                queuedBytes -= queue.removeFirst().length;
            }
            lock.notifyAll();
        }
    }

    void stop() {
        synchronized (lock) {
            queue.clear();
            queuedBytes = 0;
            releaseTrackLocked();
            lock.notifyAll();
        }
    }

    void shutdown() {
        Thread threadToJoin;
        synchronized (lock) {
            running = false;
            queue.clear();
            queuedBytes = 0;
            releaseTrackLocked();
            threadToJoin = audioThread;
            audioThread = null;
            lock.notifyAll();
        }
        if (threadToJoin != null) {
            try {
                threadToJoin.join(500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void startThreadLocked() {
        if (audioThread != null && audioThread.isAlive()) {
            return;
        }
        running = true;
        audioThread = new Thread(this::audioLoop, "FobosAndroidAudio");
        audioThread.setDaemon(true);
        audioThread.start();
    }

    private void audioLoop() {
        Process.setThreadPriority(Process.THREAD_PRIORITY_AUDIO);
        while (true) {
            byte[] frame;
            synchronized (lock) {
                while (running && (queue.isEmpty() ||
                        (track == null && queuedBytes < START_BUFFER_BYTES))) {
                    try {
                        lock.wait();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        running = false;
                    }
                }
                if (!running) {
                    break;
                }
                frame = queue.removeFirst();
                queuedBytes -= frame.length;
                ensureTrackLocked();
            }

            AudioTrack localTrack;
            synchronized (lock) {
                localTrack = track;
            }
            if (localTrack == null) {
                continue;
            }
            try {
                localTrack.write(frame, 0, frame.length, AudioTrack.WRITE_BLOCKING);
            } catch (RuntimeException ignored) {
                synchronized (lock) {
                    releaseTrackLocked();
                }
            }
        }
        synchronized (lock) {
            releaseTrackLocked();
        }
    }

    private void ensureTrackLocked() {
        if (track != null) {
            return;
        }
        int minBuffer = AudioTrack.getMinBufferSize(
                SAMPLE_RATE,
                AudioFormat.CHANNEL_OUT_MONO,
                AudioFormat.ENCODING_PCM_16BIT);
        if (minBuffer <= 0) {
            minBuffer = MIN_BUFFER_BYTES;
        }
        int bufferSize = Math.max(minBuffer * 3, MIN_BUFFER_BYTES);
        try {
            AudioFormat format = new AudioFormat.Builder()
                    .setSampleRate(SAMPLE_RATE)
                    .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                    .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                    .build();
            AudioAttributes attributes = new AudioAttributes.Builder()
                    .setUsage(AudioAttributes.USAGE_MEDIA)
                    .setContentType(AudioAttributes.CONTENT_TYPE_MUSIC)
                    .build();
            track = new AudioTrack(
                    attributes,
                    format,
                    bufferSize,
                    AudioTrack.MODE_STREAM,
                    AudioManager.AUDIO_SESSION_ID_GENERATE);
            if (track.getState() != AudioTrack.STATE_INITIALIZED) {
                releaseTrackLocked();
                return;
            }
            track.play();
        } catch (RuntimeException e) {
            releaseTrackLocked();
        }
    }

    private void releaseTrackLocked() {
        if (track == null) {
            return;
        }
        try {
            track.pause();
            track.flush();
        } catch (RuntimeException ignored) {
            // The track can already be stopped by Android during shutdown.
        }
        try {
            track.release();
        } catch (RuntimeException ignored) {
            // Release should be best effort.
        }
        track = null;
    }
}
