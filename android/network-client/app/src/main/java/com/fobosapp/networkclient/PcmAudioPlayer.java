package com.fobosapp.networkclient;

import android.media.AudioAttributes;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioTrack;
import android.os.Process;

import java.util.ArrayDeque;
import java.util.Locale;

final class PcmAudioPlayer {
    private static final int SAMPLE_RATE = 48_000;
    private static final int BYTES_PER_SAMPLE = 2;
    private static final int MAX_QUEUED_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE * 3 / 4;
    private static final int START_BUFFER_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE / 25;
    private static final int MIN_BUFFER_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE / 10;
    private static final int TARGET_CHUNK_BYTES = SAMPLE_RATE * BYTES_PER_SAMPLE / 200;

    private final Object lock = new Object();
    private final ArrayDeque<byte[]> queue = new ArrayDeque<>();
    private final byte[] pendingChunk = new byte[TARGET_CHUNK_BYTES];
    private AudioTrack track;
    private Thread audioThread;
    private boolean running;
    private int queuedBytes;
    private int pendingBytes;
    private long droppedBytesTotal;
    private long underrunsTotal;
    private long writtenBytesTotal;

    PcmAudioPlayer() {
        synchronized (lock) {
            startThreadLocked();
        }
    }

    void playPcm(byte[] pcmData) {
        if (pcmData == null || pcmData.length == 0) {
            return;
        }
        synchronized (lock) {
            startThreadLocked();
            int offset = 0;
            while (offset < pcmData.length) {
                int copy = Math.min(pcmData.length - offset, TARGET_CHUNK_BYTES - pendingBytes);
                System.arraycopy(pcmData, offset, pendingChunk, pendingBytes, copy);
                pendingBytes += copy;
                offset += copy;
                flushPendingChunkIfReadyLocked();
            }
            if (queue.isEmpty()) {
                flushPartialPendingChunkLocked();
            }
            lock.notifyAll();
        }
    }

    void stop() {
        synchronized (lock) {
            queue.clear();
            queuedBytes = 0;
            pendingBytes = 0;
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
            pendingBytes = 0;
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

    private void enqueueFrameLocked(byte[] source, int length) {
        if (length <= 0) {
            return;
        }
        byte[] copy = new byte[length];
        System.arraycopy(source, 0, copy, 0, length);
        queue.addLast(copy);
        queuedBytes += copy.length;
        while (queuedBytes > MAX_QUEUED_BYTES && !queue.isEmpty()) {
            byte[] dropped = queue.removeFirst();
            queuedBytes -= dropped.length;
            droppedBytesTotal += dropped.length;
        }
    }

    String statsSnapshot() {
        synchronized (lock) {
            int bufferedBytes = queuedBytes + pendingBytes;
            long queuedMs = bufferedBytes * 1000L / (SAMPLE_RATE * BYTES_PER_SAMPLE);
            long droppedMs = droppedBytesTotal * 1000L / (SAMPLE_RATE * BYTES_PER_SAMPLE);
            long writtenMs = writtenBytesTotal * 1000L / (SAMPLE_RATE * BYTES_PER_SAMPLE);
            return String.format(Locale.US,
                    "Audio q %d ms, drop %d ms, underrun %d, played %d ms",
                    queuedMs,
                    droppedMs,
                    underrunsTotal,
                    writtenMs);
        }
    }

    private void flushPendingChunkIfReadyLocked() {
        if (pendingBytes < TARGET_CHUNK_BYTES) {
            return;
        }
        enqueueFrameLocked(pendingChunk, TARGET_CHUNK_BYTES);
        pendingBytes = 0;
    }

    private void flushPartialPendingChunkLocked() {
        if (pendingBytes < TARGET_CHUNK_BYTES / 2) {
            return;
        }
        int alignedBytes = pendingBytes & ~1;
        if (alignedBytes <= 0) {
            return;
        }
        enqueueFrameLocked(pendingChunk, alignedBytes);
        if (pendingBytes > alignedBytes) {
            pendingChunk[0] = pendingChunk[alignedBytes];
            pendingBytes = 1;
        } else {
            pendingBytes = 0;
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
                if (running && queue.isEmpty() && track != null) {
                    ++underrunsTotal;
                }
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
                int written = localTrack.write(frame, 0, frame.length, AudioTrack.WRITE_BLOCKING);
                if (written > 0) {
                    synchronized (lock) {
                        writtenBytesTotal += written;
                    }
                }
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
