package com.fobosapp.networkclient;

import android.util.Base64;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.BufferedInputStream;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;

final class FobosNetworkClient {
    interface Listener {
        void onStatus(String status);
        void onDisconnected(String reason);
        void onRole(boolean canControl, String peerId, String peerLabel, String controllerPeerId);
        void onSpectrum(SpectrumFrame frame);
        void onAudio(byte[] pcmData);
        void onControlMessage(JSONObject command);
    }

    static final class SpectrumFrame {
        final double[] frequencies;
        final float[] magnitudes;
        final double minFrequency;
        final double maxFrequency;
        final double centerFrequency;
        final double listeningFrequency;
        final double sampleRate;
        final double bandwidth;
        final int modulationType;
        final int inputMode;

        SpectrumFrame(double[] frequencies,
                      float[] magnitudes,
                      double minFrequency,
                      double maxFrequency,
                      double centerFrequency,
                      double listeningFrequency,
                      double sampleRate,
                      double bandwidth,
                      int modulationType,
                      int inputMode) {
            this.frequencies = frequencies;
            this.magnitudes = magnitudes;
            this.minFrequency = minFrequency;
            this.maxFrequency = maxFrequency;
            this.centerFrequency = centerFrequency;
            this.listeningFrequency = listeningFrequency;
            this.sampleRate = sampleRate;
            this.bandwidth = bandwidth;
            this.modulationType = modulationType;
            this.inputMode = inputMode;
        }
    }

    private static final String HELLO = "FOBOSAPP_HELLO 1";
    private static final String OK = "FOBOSAPP_OK 1";
    private static final String PING = "FOBOSAPP_PING 1";
    private static final String PONG = "FOBOSAPP_PONG 1";
    private static final int CONNECT_TIMEOUT_MS = 4000;
    private static final int MAX_LINE_BYTES = 8 * 1024 * 1024;
    private static final long MAX_BINARY_BYTES = 512L * 1024L * 1024L;

    private final ExecutorService executor = Executors.newSingleThreadExecutor();
    private final ExecutorService writerExecutor = Executors.newSingleThreadExecutor();
    private final Object writeLock = new Object();
    private final Listener listener;
    private Socket socket;
    private OutputStream outputStream;
    private volatile boolean running;
    private volatile boolean controlReady;
    private volatile boolean localClientHasControl = true;

    FobosNetworkClient(Listener listener) {
        this.listener = listener;
    }

    boolean isConnected() {
        Socket current = socket;
        return current != null && current.isConnected() && !current.isClosed() && controlReady;
    }

    boolean isActive() {
        Socket current = socket;
        return running || (current != null && current.isConnected() && !current.isClosed());
    }

    boolean canControl() {
        return localClientHasControl;
    }

    void connect(String host, int port) {
        disconnect();
        running = true;
        controlReady = false;
        localClientHasControl = true;
        executor.execute(() -> runConnection(host, port));
    }

    void disconnect() {
        running = false;
        controlReady = false;
        synchronized (writeLock) {
            if (socket != null) {
                try {
                    socket.close();
                } catch (IOException ignored) {
                    // Socket is being intentionally torn down.
                }
            }
            socket = null;
            outputStream = null;
        }
    }

    void shutdown() {
        disconnect();
        executor.shutdownNow();
        writerExecutor.shutdownNow();
    }

    boolean sendControl(String action,
                        RadioSettings settings,
                        boolean serverDisableLocalVisualAudio,
                        JSONObject extra) {
        if (!isConnected()) {
            notifyStatus("Control channel is not ready");
            return false;
        }
        if (!localClientHasControl &&
                !"requestPriority".equals(action) &&
                !"priorityResponse".equals(action)) {
            notifyStatus("This Android client is observer. Request control first.");
            return false;
        }
        try {
            JSONObject command = new JSONObject();
            command.put("type", "control");
            command.put("action", action);
            command.put("processingMode", RadioSettings.PROCESSING_SERVER_SIDE);
            command.put("serverDisableLocalVisualAudio", serverDisableLocalVisualAudio);
            command.put("settings", settings.toJson());
            if (extra != null) {
                JSONArray keys = extra.names();
                if (keys != null) {
                    for (int i = 0; i < keys.length(); ++i) {
                        String key = keys.getString(i);
                        command.put(key, extra.get(key));
                    }
                }
            }
            return queueJson(command);
        } catch (JSONException e) {
            notifyStatus("Could not build command: " + e.getMessage());
            return false;
        }
    }

    private void runConnection(String host, int port) {
        Socket nextSocket = null;
        try {
            notifyStatus("Connecting to " + host + ":" + port);
            nextSocket = new Socket();
            nextSocket.setTcpNoDelay(true);
            nextSocket.connect(new InetSocketAddress(host, port), CONNECT_TIMEOUT_MS);
            synchronized (writeLock) {
                socket = nextSocket;
                outputStream = nextSocket.getOutputStream();
            }
            sendProtocolLine(HELLO);
            notifyStatus("Connected, waiting for server handshake");
            readLoop(new BufferedInputStream(nextSocket.getInputStream(), 64 * 1024));
        } catch (IOException e) {
            if (running) {
                notifyDisconnected("Connection failed: " + e.getMessage());
            }
        } finally {
            closeSocketIfCurrent(nextSocket);
        }
    }

    private void readLoop(InputStream inputStream) throws IOException {
        while (running) {
            String line = readLine(inputStream);
            if (line == null) {
                throw new EOFException("Server closed connection");
            }
            if (line.isEmpty()) {
                continue;
            }
            if (OK.equals(line)) {
                controlReady = true;
                notifyStatus("Control channel ready");
                continue;
            }
            if (PING.equals(line)) {
                sendProtocolLine(PONG);
                continue;
            }
            if (PONG.equals(line) || HELLO.equals(line)) {
                continue;
            }

            JSONObject command;
            try {
                command = new JSONObject(line);
            } catch (JSONException e) {
                notifyStatus("Ignoring invalid JSON: " + e.getMessage());
                continue;
            }

            if ("iqbin".equals(command.optString("type"))) {
                long bytes = parsePayloadBytes(command);
                if (bytes <= 0L || bytes > MAX_BINARY_BYTES) {
                    notifyStatus("Ignoring invalid IQ payload header");
                    continue;
                }
                skipFully(inputStream, bytes);
                continue;
            }

            handleJson(command);
        }
    }

    private void handleJson(JSONObject command) {
        String type = command.optString("type");
        if ("spectrum".equals(type)) {
            SpectrumFrame frame = parseSpectrum(command);
            if (frame != null) {
                listener.onSpectrum(frame);
            }
            return;
        }
        if ("audio".equals(type)) {
            if (command.optInt("sampleRate", 48_000) != 48_000 ||
                    command.optInt("channels", 1) != 1 ||
                    !"pcm_s16le".equals(command.optString("sampleFormat", "pcm_s16le"))) {
                notifyStatus("Unsupported audio frame format");
                return;
            }
            String encoded = command.optString("pcm", "");
            if (!encoded.isEmpty()) {
                try {
                    listener.onAudio(Base64.decode(encoded, Base64.DEFAULT));
                } catch (IllegalArgumentException e) {
                    notifyStatus("Ignoring invalid audio payload");
                }
            }
            return;
        }
        if ("control".equals(type)) {
            String action = command.optString("action");
            if ("role".equals(action)) {
                localClientHasControl = command.optBoolean("canControl", localClientHasControl);
                listener.onRole(
                        localClientHasControl,
                        command.optString("peerId", ""),
                        command.optString("peerLabel", ""),
                        command.optString("controllerPeerId", ""));
            }
            listener.onControlMessage(command);
        }
    }

    private SpectrumFrame parseSpectrum(JSONObject command) {
        JSONArray frequencyArray = command.optJSONArray("frequencies");
        JSONArray magnitudeArray = command.optJSONArray("magnitudes");
        if (frequencyArray == null || magnitudeArray == null) {
            return null;
        }
        int count = Math.min(frequencyArray.length(), magnitudeArray.length());
        if (count <= 0) {
            return null;
        }
        double[] frequencies = new double[count];
        float[] magnitudes = new float[count];
        for (int i = 0; i < count; ++i) {
            frequencies[i] = frequencyArray.optDouble(i, 0.0);
            magnitudes[i] = (float) magnitudeArray.optDouble(i, -160.0);
        }
        return new SpectrumFrame(
                frequencies,
                magnitudes,
                command.optDouble("minFrequency", frequencies[0]),
                command.optDouble("maxFrequency", frequencies[count - 1]),
                optionalDouble(command, "centerFrequency"),
                optionalDouble(command, "listeningFrequency"),
                command.optDouble("sampleRate", 0.0),
                command.optDouble("bandwidth", 0.0),
                command.optInt("modulationType", RadioSettings.MOD_AM),
                command.has("inputMode") ? command.optInt("inputMode", -1) : -1);
    }

    private static double optionalDouble(JSONObject command, String key) {
        return command.has(key) ? command.optDouble(key, Double.NaN) : Double.NaN;
    }

    private String readLine(InputStream inputStream) throws IOException {
        ByteArrayOutputStream line = new ByteArrayOutputStream(512);
        while (running) {
            int value = inputStream.read();
            if (value < 0) {
                return null;
            }
            if (value == '\n') {
                break;
            }
            if (value != '\r') {
                line.write(value);
            }
            if (line.size() > MAX_LINE_BYTES) {
                throw new IOException("Protocol line is too large");
            }
        }
        return line.toString(StandardCharsets.UTF_8.name()).trim();
    }

    private void skipFully(InputStream inputStream, long bytes) throws IOException {
        long remaining = bytes;
        byte[] scratch = new byte[32 * 1024];
        while (running && remaining > 0L) {
            int read = inputStream.read(scratch, 0, (int) Math.min(scratch.length, remaining));
            if (read < 0) {
                throw new EOFException("Server closed connection during binary payload");
            }
            remaining -= read;
        }
    }

    private long parsePayloadBytes(JSONObject command) {
        Object value = command.opt("payloadBytes");
        if (value instanceof Number) {
            return ((Number) value).longValue();
        }
        if (value instanceof String) {
            try {
                return Long.parseLong((String) value);
            } catch (NumberFormatException ignored) {
                return -1L;
            }
        }
        return -1L;
    }

    private boolean queueJson(JSONObject command) {
        final String line = command.toString();
        try {
            writerExecutor.execute(() -> sendProtocolLine(line));
            return true;
        } catch (RejectedExecutionException e) {
            notifyStatus("Network writer is not available");
            return false;
        }
    }

    private boolean sendProtocolLine(String line) {
        synchronized (writeLock) {
            if (outputStream == null) {
                return false;
            }
            try {
                outputStream.write((line + "\n").getBytes(StandardCharsets.UTF_8));
                outputStream.flush();
                return true;
            } catch (IOException e) {
                notifyDisconnected("Control write failed: " + e.getMessage());
                disconnect();
                return false;
            }
        }
    }

    private void closeSocketIfCurrent(Socket closingSocket) {
        synchronized (writeLock) {
            if (closingSocket == null || socket != closingSocket) {
                return;
            }
            running = false;
            controlReady = false;
            try {
                closingSocket.close();
            } catch (IOException ignored) {
                // Socket is already closed.
            }
            socket = null;
            outputStream = null;
        }
    }

    private void notifyStatus(String status) {
        listener.onStatus(status);
    }

    private void notifyDisconnected(String reason) {
        running = false;
        controlReady = false;
        listener.onDisconnected(reason);
    }
}
