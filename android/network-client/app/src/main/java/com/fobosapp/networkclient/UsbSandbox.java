package com.fobosapp.networkclient;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Build;
import android.util.Log;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;

final class UsbSandbox {
    interface Listener {
        void onUsbLog(String message);
        void onUsbSpectrum(FobosNetworkClient.SpectrumFrame frame);
        void onUsbAudio(byte[] pcmData);
        void onUsbTelemetry(String message);
    }

    static final int FOBOS_VENDOR_ID = 0x16d0;
    static final int FOBOS_PRODUCT_ID = 0x132e;
    static final int FOBOS_STANDARD_BCD_DEVICE = 0x0000;
    static final int FOBOS_AGILE_BCD_DEVICE = 0x0101;

    private static final String ACTION_USB_PERMISSION =
            "com.fobosapp.networkclient.USB_PERMISSION";
    private static final String LOG_TAG = "FobosUsbSandbox";
    private static final int USB_VENDOR_IN = UsbConstants.USB_TYPE_VENDOR | UsbConstants.USB_DIR_IN;
    private static final int USB_VENDOR_OUT = UsbConstants.USB_TYPE_VENDOR | UsbConstants.USB_DIR_OUT;
    private static final int FOBOS_INFO_REQUEST = 0xE8;
    private static final int FOBOS_SDR_CMD = 0xE1;
    private static final int CMD_OPEN = 0x00;
    private static final int CMD_CLOSE = 0x01;
    private static final int CMD_START = 0x02;
    private static final int CMD_STOP = 0x03;
    private static final int CMD_SET_FREQ = 0x10;
    private static final int CMD_SET_SR = 0x11;
    private static final int CMD_SET_AUTOBW = 0x13;
    private static final int CMD_SET_DIRECT = 0x20;
    private static final int CMD_SET_LNA = 0x22;
    private static final int CMD_SET_VGA = 0x23;
    private static final int CTRL_TIMEOUT_MS = 300;
    private static final double PREVIEW_MAX_SAMPLE_RATE = 80_000_000.0;
    private static final int PREVIEW_MAX_FFT = 4_096;
    private static final int AUDIO_SAMPLE_RATE = 48_000;
    private static final long TELEMETRY_INTERVAL_NANOS = 1_000_000_000L;

    private final Activity activity;
    private final UsbManager usbManager;
    private final Listener listener;
    private final Object previewControlLock = new Object();
    private final PendingIntent permissionIntent;
    private final BroadcastReceiver usbReceiver;
    private boolean receiverRegistered;
    private UsbDevice activeDevice;
    private UsbDeviceConnection activeConnection;
    private UsbInterface activeInterface;
    private UsbEndpoint activeBulkInEndpoint;
    private UsbEndpoint activeBulkOutEndpoint;
    private boolean activeAgileApi;
    private boolean openSessionAfterPermission;
    private volatile boolean previewRunning;
    private Thread previewThread;
    private PreviewRequest pendingPreviewRestart;
    private boolean previewRestartWorkerRunning;
    private volatile boolean previewRestartCancelled;
    private volatile double previewCenterFrequency;
    private volatile double previewListeningFrequency;
    private volatile double previewSampleRate;
    private volatile int previewInputMode;
    private volatile int previewFftLength;
    private volatile int previewRequestedFftLength;
    private volatile double previewBandwidth;
    private volatile int previewModulationType = RadioSettings.MOD_AM;
    private volatile boolean previewAudioEnabled;
    private double audioSourceCursor;
    private double audioMixerPhase;
    private double audioFmLastPhase;
    private double audioFmLastI;
    private double audioFmLastQ;
    private boolean audioFmPhaseValid;
    private double audioEnvelopeAverage = 0.1;
    private double audioDcAverage;
    private double audioLowPassAverage;
    private double audioChannelSumI;
    private double audioChannelSumQ;
    private int audioChannelDecimationCount;
    private double audioChannelLowPassI;
    private double audioChannelLowPassQ;
    private double audioResamplePhase;
    private double audioAgcLevel = 0.05;
    private double[] audioFirTaps = new double[0];
    private double[] audioFirRingI = new double[0];
    private double[] audioFirRingQ = new double[0];
    private int audioFirRingIndex;
    private int audioFirConfiguredDecimation;
    private int audioFirConfiguredModulation;
    private double audioFirConfiguredSampleRate;
    private double audioFirConfiguredBandwidth;
    private double audioFirOutputI;
    private double audioFirOutputQ;
    private double audioDeemphasisAverage;
    private long audioDebugLastLogNanos;
    private double audioRawDcI = 8192.0;
    private double audioRawDcQ = 8192.0;
    private double previewAudioSampleRate;
    private double previewMeasuredSampleRate;
    private long previewAudioRateLastNanos;
    private double convertDcI;
    private double convertDcQ;

    private static final class PreviewRequest {
        final double centerFrequency;
        final double listeningFrequency;
        final double sampleRate;
        final int inputMode;
        final double bandwidth;
        final int modulationType;
        final boolean audioEnabled;
        final int lnaGain;
        final int vgaGain;
        final int fftLength;
        final int requestedFftLength;

        PreviewRequest(double centerFrequency,
                       double listeningFrequency,
                       double sampleRate,
                       int inputMode,
                       double bandwidth,
                       int modulationType,
                       boolean audioEnabled,
                       int lnaGain,
                       int vgaGain,
                       int fftLength,
                       int requestedFftLength) {
            this.centerFrequency = centerFrequency;
            this.listeningFrequency = listeningFrequency;
            this.sampleRate = sampleRate;
            this.inputMode = inputMode;
            this.bandwidth = bandwidth;
            this.modulationType = modulationType;
            this.audioEnabled = audioEnabled;
            this.lnaGain = lnaGain;
            this.vgaGain = vgaGain;
            this.fftLength = fftLength;
            this.requestedFftLength = requestedFftLength;
        }
    }

    UsbSandbox(Activity activity, Listener listener) {
        this.activity = activity;
        this.listener = listener;
        this.usbManager = (UsbManager) activity.getSystemService(Context.USB_SERVICE);
        int flags = PendingIntent.FLAG_UPDATE_CURRENT;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            flags |= PendingIntent.FLAG_MUTABLE;
        }
        this.permissionIntent = PendingIntent.getBroadcast(
                activity,
                0,
                new Intent(ACTION_USB_PERMISSION).setPackage(activity.getPackageName()),
                flags);
        this.usbReceiver = new BroadcastReceiver() {
            @Override
            public void onReceive(Context context, Intent intent) {
                handleUsbBroadcast(intent);
            }
        };
    }

    void start() {
        if (receiverRegistered) {
            return;
        }
        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            activity.registerReceiver(usbReceiver, filter, Context.RECEIVER_NOT_EXPORTED);
        } else {
            activity.registerReceiver(usbReceiver, filter);
        }
        receiverRegistered = true;
    }

    void stop() {
        closeActiveSession();
        if (!receiverRegistered) {
            return;
        }
        try {
            activity.unregisterReceiver(usbReceiver);
        } catch (IllegalArgumentException ignored) {
            // Receiver was already gone during Activity teardown.
        }
        receiverRegistered = false;
    }

    String scanReport() {
        List<UsbDevice> devices = sortedDevices();
        if (devices.isEmpty()) {
            return noUsbDevicesMessage("[USB]");
        }
        StringBuilder report = new StringBuilder();
        report.append("[USB] visible devices: ").append(devices.size());
        for (UsbDevice device : devices) {
            appendDeviceReport(report, device);
        }
        return report.toString();
    }

    String requestPermissionForBestDevice() {
        UsbDevice device = bestDevice();
        if (device == null) {
            return noUsbDevicesMessage("[USB]");
        }
        if (usbManager.hasPermission(device)) {
            return probeOpenDevice(device);
        }
        openSessionAfterPermission = false;
        usbManager.requestPermission(device, permissionIntent);
        return "[USB] permission requested for " + shortDeviceName(device);
    }

    String handleLaunchIntent(Intent intent) {
        if (intent == null || !UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(intent.getAction())) {
            return "";
        }
        UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        if (device == null) {
            return "[USB] app opened from USB attach intent, but Android did not provide a device";
        }
        StringBuilder report = new StringBuilder();
        report.append("[USB] app opened from attached device: ").append(shortDeviceName(device));
        if (!usbManager.hasPermission(device)) {
            report.append("\n[USB] press USB permission/open or OTG open to grant access");
            return report.toString();
        }
        report.append('\n').append(probeOpenDevice(device));
        return report.toString();
    }

    String openReceiverSession() {
        UsbDevice device = bestDevice();
        if (device == null) {
            return noUsbDevicesMessage("[USB OTG]");
        }
        if (!usbManager.hasPermission(device)) {
            openSessionAfterPermission = true;
            usbManager.requestPermission(device, permissionIntent);
            return "[USB OTG] permission requested for " + shortDeviceName(device);
        }
        return openReceiverSession(device);
    }

    String readReceiverProbe() {
        StringBuilder report = new StringBuilder();
        if (activeConnection == null || activeBulkInEndpoint == null) {
            String openReport = openReceiverSession();
            report.append(openReport);
            if (activeConnection == null || activeBulkInEndpoint == null) {
                return report.toString();
            }
            report.append('\n');
        }
        int maxPacket = Math.max(64, activeBulkInEndpoint.getMaxPacketSize());
        int length = Math.max(512, Math.min(16_384, maxPacket * 16));
        byte[] buffer = new byte[length];
        long startNanos = System.nanoTime();
        int result;
        try {
            result = activeConnection.bulkTransfer(activeBulkInEndpoint, buffer, buffer.length, 50);
        } catch (RuntimeException e) {
            return report.append("[USB OTG] bulk read failed: ").append(e.getMessage()).toString();
        }
        long elapsedMicros = (System.nanoTime() - startNanos) / 1000L;
        report.append("[USB OTG] bulk read probe ep ")
                .append(formatByte(activeBulkInEndpoint.getAddress()))
                .append(" result ").append(result)
                .append(" elapsed ").append(elapsedMicros).append(" us");
        if (result > 0) {
            report.append("\n[USB OTG] first bytes ").append(hexPreview(buffer, result, 32));
        } else {
            report.append("\n[USB OTG] no data is expected until the libfobos init/start sequence is ported");
        }
        return report.toString();
    }

    String readReceiverInfo() {
        StringBuilder report = new StringBuilder();
        if (activeConnection == null) {
            String openReport = openReceiverSession();
            report.append(openReport);
            if (activeConnection == null) {
                return report.toString();
            }
            report.append('\n');
        }
        appendFirmwareInfoProbe(report);
        return report.toString();
    }

    String closeReceiverSession() {
        synchronized (previewControlLock) {
            pendingPreviewRestart = null;
            previewRestartCancelled = true;
        }
        stopPreviewThread(700L);
        boolean hadConnection = activeConnection != null;
        closeActiveSession();
        return hadConnection ? "[USB OTG] session closed" : "[USB OTG] no open session";
    }

    private PreviewRequest makePreviewRequest(double centerFrequency,
                                              double listeningFrequency,
                                              double sampleRate,
                                              int inputMode,
                                              double bandwidth,
                                              int modulationType,
                                              boolean audioEnabled,
                                              int lnaGain,
                                              int vgaGain,
                                              int requestedFftLength) {
        double cappedSampleRate = sampleRate > 0.0
                ? Math.min(sampleRate, PREVIEW_MAX_SAMPLE_RATE)
                : PREVIEW_MAX_SAMPLE_RATE;
        int fftLength = previewFftLength(requestedFftLength);
        int normalizedModulation = normalizeModulationType(modulationType);
        double resolvedBandwidth = bandwidth > 0.0
                ? bandwidth
                : RadioSettings.defaultBandwidthForModulation(normalizedModulation);
        int resolvedRequestedFft = requestedFftLength > 0 ? requestedFftLength : fftLength;
        return new PreviewRequest(centerFrequency,
                listeningFrequency,
                cappedSampleRate,
                inputMode,
                resolvedBandwidth,
                normalizedModulation,
                audioEnabled,
                Math.max(0, Math.min(3, lnaGain)),
                Math.max(0, Math.min(31, vgaGain)),
                fftLength,
                resolvedRequestedFft);
    }

    private void applyPreviewRequestState(PreviewRequest request) {
        previewCenterFrequency = request.inputMode == RadioSettings.INPUT_RF
                ? request.centerFrequency
                : 0.0;
        previewListeningFrequency = request.listeningFrequency;
        previewSampleRate = request.sampleRate;
        previewInputMode = request.inputMode;
        previewFftLength = request.fftLength;
        previewRequestedFftLength = request.requestedFftLength;
        previewModulationType = request.modulationType;
        previewBandwidth = request.bandwidth;
        previewAudioEnabled = request.audioEnabled;
    }

    String startReceiverPreview(double centerFrequency,
                                double listeningFrequency,
                                double sampleRate,
                                int inputMode,
                                double bandwidth,
                                int modulationType,
                                boolean audioEnabled,
                                int lnaGain,
                                int vgaGain,
                                int requestedFftLength) {
        PreviewRequest request = makePreviewRequest(centerFrequency,
                listeningFrequency,
                sampleRate,
                inputMode,
                bandwidth,
                modulationType,
                audioEnabled,
                lnaGain,
                vgaGain,
                requestedFftLength);
        synchronized (previewControlLock) {
            previewRestartCancelled = false;
            if (previewRunning || previewThread != null || previewRestartWorkerRunning) {
                pendingPreviewRestart = request;
                if (!previewRestartWorkerRunning) {
                    startPreviewRestartWorkerLocked();
                }
                return "[USB OTG] preview start queued with latest settings";
            }
        }
        return startReceiverPreviewNow(request);
    }

    private String startReceiverPreviewNow(PreviewRequest request) {
        synchronized (previewControlLock) {
            if (previewRunning || previewThread != null) {
                return "[USB OTG] previous preview is still stopping; start deferred";
            }
        }
        StringBuilder report = new StringBuilder();
        if (activeConnection == null || activeBulkInEndpoint == null) {
            String openReport = openReceiverSession();
            report.append(openReport);
            if (activeConnection == null || activeBulkInEndpoint == null) {
                return report.toString();
            }
            report.append('\n');
        }
        synchronized (previewControlLock) {
            applyPreviewRequestState(request);
        }
        resetAudioDemodState();
        resetFobosConversionState();
        previewRunning = true;
        previewThread = new Thread(
                () -> runPreviewLoop(request),
                "FobosOtgPreview");
        previewThread.start();
        report.append("[USB OTG] preview starting at ")
                .append(formatMhz(request.centerFrequency))
                .append(" MHz, ")
                .append(formatMhz(request.sampleRate))
                .append(" Msps, FFT ")
                .append(request.fftLength);
        if (request.sampleRate >= PREVIEW_MAX_SAMPLE_RATE) {
            report.append("\n[USB OTG] preview sample rate capped for first Android streaming tests");
        }
        return report.toString();
    }

    String stopReceiverPreview() {
        synchronized (previewControlLock) {
            pendingPreviewRestart = null;
            previewRestartCancelled = true;
        }
        boolean wasRunning = previewRunning || previewThread != null;
        stopPreviewThread(1000L);
        return wasRunning ? "[USB OTG] preview stop requested" : "[USB OTG] preview is not running";
    }

    boolean isReceiverPreviewRunning() {
        return previewRunning;
    }

    String applyReceiverPreviewSettings(double centerFrequency,
                                        double listeningFrequency,
                                        double sampleRate,
                                        int inputMode,
                                        double bandwidth,
                                        int modulationType,
                                        boolean audioEnabled,
                                        int lnaGain,
                                        int vgaGain,
                                        int requestedFftLength) {
        if (!previewRunning || activeConnection == null) {
            return "[USB OTG] preview is not running";
        }
        PreviewRequest request = makePreviewRequest(centerFrequency,
                listeningFrequency,
                sampleRate,
                inputMode,
                bandwidth,
                modulationType,
                audioEnabled,
                lnaGain,
                vgaGain,
                requestedFftLength);
        StringBuilder report = new StringBuilder("[USB OTG] preview settings");
        try {
            boolean audioResetNeeded;
            boolean needsRestart;
            synchronized (previewControlLock) {
                audioResetNeeded = Math.abs(request.sampleRate - previewSampleRate) > 1.0 ||
                        request.inputMode != previewInputMode ||
                        request.modulationType != previewModulationType ||
                        Math.abs(request.listeningFrequency - previewListeningFrequency) > 10.0 ||
                        Math.abs(request.centerFrequency - previewCenterFrequency) > 10.0;
                needsRestart = Math.abs(request.sampleRate - previewSampleRate) > 1.0 ||
                        request.inputMode != previewInputMode ||
                        request.fftLength != previewFftLength;

                if (needsRestart) {
                    previewRestartCancelled = false;
                    pendingPreviewRestart = request;
                    if (!previewRestartWorkerRunning) {
                        startPreviewRestartWorkerLocked();
                    }
                    return report.append(" queued restart for sample/input/FFT change").toString();
                }

                applyPreviewRequestState(request);
            }

            int lnaResult = fx3Command(CMD_SET_LNA, request.lnaGain);
            int vgaResult = fx3Command(CMD_SET_VGA, request.vgaGain);
            report.append(" LNA ").append(lnaResult).append(", VGA ").append(vgaResult);

            if (request.inputMode == RadioSettings.INPUT_RF) {
                long frequency = Math.max(50_000_000L, Math.round(request.centerFrequency));
                int freqResult = controlOutU64(CMD_SET_FREQ, frequency);
                previewCenterFrequency = frequency;
                previewListeningFrequency = request.listeningFrequency;
                report.append(", RF ").append(formatMhz(frequency)).append(" MHz ").append(freqResult);
            } else {
                previewCenterFrequency = 0.0;
                previewListeningFrequency = request.listeningFrequency;
                report.append(", direct input frequency scale only");
            }
            if (audioResetNeeded) {
                resetAudioDemodState();
            }
        } catch (RuntimeException e) {
            report.append(" failed: ").append(e.getMessage());
        }
        return report.toString();
    }

    String runReceiverSampleTest() {
        StringBuilder report = new StringBuilder();
        if (activeConnection == null || activeBulkInEndpoint == null) {
            String openReport = openReceiverSession();
            report.append(openReport);
            if (activeConnection == null || activeBulkInEndpoint == null) {
                return report.toString();
            }
            report.append('\n');
        }

        boolean openedByCommand = false;
        boolean started = false;
        try {
            int result = fx3Command(CMD_OPEN, 0);
            openedByCommand = result >= 0;
            report.append("[USB OTG] CMD_OPEN result ").append(result);

            result = controlOutU64(CMD_SET_FREQ, 100_000_000L);
            report.append("\n[USB OTG] SET_FREQ 100 MHz result ").append(result);

            result = controlOutU64(CMD_SET_SR, 8_000_000L);
            report.append("\n[USB OTG] SET_SR 8 Msps result ").append(result);

            result = controlOutU64(CMD_SET_AUTOBW, Math.round(0.9 * 1024.0));
            report.append("\n[USB OTG] SET_AUTOBW 0.9 result ").append(result);

            int packsPerTransfer = 16;
            result = fx3Command(CMD_START, packsPerTransfer);
            started = result >= 0;
            report.append("\n[USB OTG] CMD_START packs ")
                    .append(packsPerTransfer)
                    .append(" result ")
                    .append(result);

            byte[] buffer = new byte[65_536];
            long startNanos = System.nanoTime();
            result = activeConnection.bulkTransfer(activeBulkInEndpoint, buffer, buffer.length, 1000);
            long elapsedMicros = (System.nanoTime() - startNanos) / 1000L;
            report.append("\n[USB OTG] sample bulk read result ")
                    .append(result)
                    .append(" elapsed ")
                    .append(elapsedMicros)
                    .append(" us");
            if (result > 0) {
                report.append("\n[USB OTG] sample first bytes ")
                        .append(hexPreview(buffer, result, 32));
            }
        } catch (RuntimeException e) {
            report.append("\n[USB OTG] sample test failed: ").append(e.getMessage());
        } finally {
            if (started) {
                int stopResult = fx3Command(CMD_STOP, 0);
                report.append("\n[USB OTG] CMD_STOP result ").append(stopResult);
            }
            if (openedByCommand) {
                int closeResult = fx3Command(CMD_CLOSE, 0);
                report.append("\n[USB OTG] CMD_CLOSE result ").append(closeResult);
            }
        }
        return report.toString();
    }

    private void runPreviewLoop(PreviewRequest request) {
        boolean openedByCommand = false;
        boolean started = false;
        boolean loggedFirstFrame = false;
        long lastFrameNanos = 0L;
        long telemetryStartNanos = System.nanoTime();
        long telemetryBytes = 0L;
        long telemetryReads = 0L;
        long telemetryIqSamples = 0L;
        long telemetryPcmSamples = 0L;
        long telemetryAudioFrames = 0L;
        long telemetryFftFrames = 0L;
        int readFailures = 0;
        byte[] buffer = new byte[65_536];
        try {
            int result = fx3Command(CMD_OPEN, 0);
            openedByCommand = result >= 0;
            log("[USB OTG] preview CMD_OPEN result " + result);

            int packsPerTransfer = 16;
            result = configureAndStartPreviewHardware(request.centerFrequency,
                    request.sampleRate,
                    request.inputMode,
                    request.lnaGain,
                    request.vgaGain,
                    packsPerTransfer,
                    "");
            if (result < 0) {
                log("[USB OTG] preview CMD_START failed; retrying clean open/start once");
                fx3Command(CMD_STOP, 0);
                fx3Command(CMD_CLOSE, 0);
                openedByCommand = false;
                sleepQuietly(250L);
                result = fx3Command(CMD_OPEN, 0);
                openedByCommand = result >= 0;
                log("[USB OTG] preview retry CMD_OPEN result " + result);
                if (openedByCommand) {
                    result = configureAndStartPreviewHardware(request.centerFrequency,
                            request.sampleRate,
                            request.inputMode,
                            request.lnaGain,
                            request.vgaGain,
                            packsPerTransfer,
                            " retry");
                }
            }
            started = result >= 0;
            if (!started) {
                log("[USB OTG] preview start failed; stream is not running");
                return;
            }

            while (previewRunning) {
                result = activeConnection.bulkTransfer(activeBulkInEndpoint, buffer, buffer.length, 250);
                if (result <= 0) {
                    ++readFailures;
                    if (readFailures >= 8) {
                        log("[USB OTG] preview bulk read stalled result " + result);
                        break;
                    }
                    continue;
                }
                readFailures = 0;
                telemetryBytes += result;
                ++telemetryReads;
                int complexSamples = rawFobosComplexSampleCount(buffer, result);
                if (complexSamples < 2) {
                    continue;
                }
                telemetryIqSamples += complexSamples;
                long now = System.nanoTime();
                double audioSampleRate = updatePreviewAudioSampleRate(now,
                        complexSamples,
                        previewSampleRate);
                float[] iqSamples = null;
                if (previewAudioEnabled && listener != null) {
                    iqSamples = convertRawFobosIq(buffer, result, previewInputMode);
                    byte[] pcm = audioFrameFromIq(iqSamples, previewCenterFrequency,
                            previewListeningFrequency, previewSampleRate, audioSampleRate,
                            previewInputMode, previewModulationType);
                    if (pcm.length > 0) {
                        telemetryPcmSamples += pcm.length / 2;
                        ++telemetryAudioFrames;
                        listener.onUsbAudio(pcm);
                    }
                }
                if (now - telemetryStartNanos >= TELEMETRY_INTERVAL_NANOS) {
                    emitPreviewTelemetry(now,
                            telemetryStartNanos,
                            telemetryBytes,
                            telemetryReads,
                            telemetryIqSamples,
                            telemetryPcmSamples,
                            telemetryAudioFrames,
                            telemetryFftFrames);
                    telemetryStartNanos = now;
                    telemetryBytes = 0L;
                    telemetryReads = 0L;
                    telemetryIqSamples = 0L;
                    telemetryPcmSamples = 0L;
                    telemetryAudioFrames = 0L;
                    telemetryFftFrames = 0L;
                }
                if (now - lastFrameNanos < 90_000_000L) {
                    continue;
                }
                lastFrameNanos = now;
                if (iqSamples == null) {
                    iqSamples = convertRawFobosIq(buffer, result, previewInputMode);
                }
                FobosNetworkClient.SpectrumFrame frame =
                        spectrumFrameFromIq(iqSamples, previewCenterFrequency,
                                previewListeningFrequency, previewSampleRate,
                                previewInputMode, previewFftLength);
                if (!loggedFirstFrame && frame != null) {
                    loggedFirstFrame = true;
                    log("[USB OTG] preview first FFT frame " + frame.magnitudes.length + " bins");
                }
                if (listener != null && frame != null) {
                    ++telemetryFftFrames;
                    listener.onUsbSpectrum(frame);
                }
            }
        } catch (RuntimeException e) {
            log("[USB OTG] preview failed: " + e.getMessage());
        } finally {
            if (started) {
                int stopResult = fx3Command(CMD_STOP, 0);
                log("[USB OTG] preview CMD_STOP result " + stopResult);
            }
            if (openedByCommand) {
                int closeResult = fx3Command(CMD_CLOSE, 0);
                log("[USB OTG] preview CMD_CLOSE result " + closeResult);
            }
            previewRunning = false;
            previewThread = null;
            log("[USB OTG] preview stopped");
        }
    }

    private void startPreviewRestartWorkerLocked() {
        previewRestartWorkerRunning = true;
        Thread worker = new Thread(this::previewRestartWorkerLoop, "FobosOtgSettings");
        worker.setDaemon(true);
        worker.start();
    }

    private void previewRestartWorkerLoop() {
        while (true) {
            PreviewRequest request;
            synchronized (previewControlLock) {
                if (previewRestartCancelled) {
                    pendingPreviewRestart = null;
                    previewRestartWorkerRunning = false;
                    return;
                }
                request = pendingPreviewRestart;
                pendingPreviewRestart = null;
                if (request == null) {
                    previewRestartWorkerRunning = false;
                    return;
                }
            }

            stopPreviewThread(1000L);

            synchronized (previewControlLock) {
                if (previewRestartCancelled) {
                    previewRestartWorkerRunning = false;
                    return;
                }
                if (previewThread != null) {
                    pendingPreviewRestart = request;
                    log("[USB OTG] queued preview restart is waiting for the old preview thread to stop");
                }
            }
            if (previewThread != null) {
                sleepQuietly(250L);
                continue;
            }

            synchronized (previewControlLock) {
                if (previewRestartCancelled) {
                    previewRestartWorkerRunning = false;
                    return;
                }
            }

            String message = startReceiverPreviewNow(request);
            log("[USB OTG] queued preview restart applied\n" + message);
        }
    }

    private void emitPreviewTelemetry(long nowNanos,
                                      long startNanos,
                                      long bytesRead,
                                      long reads,
                                      long iqSamples,
                                      long pcmSamples,
                                      long audioFrames,
                                      long fftFrames) {
        if (listener == null) {
            return;
        }
        double elapsedSeconds = Math.max(0.001, (nowNanos - startNanos) / 1_000_000_000.0);
        double usbMbps = (bytesRead * 8.0) / (elapsedSeconds * 1_000_000.0);
        double iqMsps = iqSamples / (elapsedSeconds * 1_000_000.0);
        double pcmRate = pcmSamples / elapsedSeconds;
        double readRate = reads / elapsedSeconds;
        double fftRate = fftFrames / elapsedSeconds;
        double audioMsps = previewAudioSampleRate / 1_000_000.0;
        String message = String.format(Locale.US,
                "USB %.1f Mb/s, %.2f MS/s, %.0f reads/s, PCM %.0f/s, audio %d/s, FFT %.1f/s, audioSR %.2f MS/s",
                usbMbps,
                iqMsps,
                readRate,
                pcmRate,
                Math.round(audioFrames / elapsedSeconds),
                fftRate,
                audioMsps);
        listener.onUsbTelemetry(message);
    }

    private int configureAndStartPreviewHardware(double centerFrequency,
                                                 double sampleRate,
                                                 int inputMode,
                                                 int lnaGain,
                                                 int vgaGain,
                                                 int packsPerTransfer,
                                                 String logSuffix) {
        int directMode = inputMode == RadioSettings.INPUT_RF ? 0 : 1;
        int result = fx3Command(CMD_SET_DIRECT, directMode);
        log("[USB OTG] preview" + logSuffix + " SET_DIRECT " + directMode + " result " + result);

        if (inputMode == RadioSettings.INPUT_RF) {
            result = controlOutU64(CMD_SET_FREQ, Math.max(50_000_000L, Math.round(centerFrequency)));
            log("[USB OTG] preview" + logSuffix + " SET_FREQ result " + result);
        }

        result = controlOutU64(CMD_SET_SR, Math.round(sampleRate));
        log("[USB OTG] preview" + logSuffix + " SET_SR result " + result);

        result = controlOutU64(CMD_SET_AUTOBW, Math.round(0.9 * 1024.0));
        log("[USB OTG] preview" + logSuffix + " SET_AUTOBW result " + result);

        result = fx3Command(CMD_SET_LNA, Math.max(0, Math.min(3, lnaGain)));
        log("[USB OTG] preview" + logSuffix + " SET_LNA result " + result);

        result = fx3Command(CMD_SET_VGA, Math.max(0, Math.min(31, vgaGain)));
        log("[USB OTG] preview" + logSuffix + " SET_VGA result " + result);

        result = fx3Command(CMD_START, packsPerTransfer);
        log("[USB OTG] preview" + logSuffix + " CMD_START result " + result);
        return result;
    }

    private void sleepQuietly(long millis) {
        try {
            Thread.sleep(Math.max(0L, millis));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void handleUsbBroadcast(Intent intent) {
        if (intent == null) {
            return;
        }
        String action = intent.getAction();
        UsbDevice device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
        if (ACTION_USB_PERMISSION.equals(action)) {
            boolean granted = intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false);
            if (device == null) {
                log("[USB] permission result without device");
                return;
            }
            if (!granted) {
                openSessionAfterPermission = false;
                log("[USB] permission denied for " + shortDeviceName(device));
                return;
            }
            if (openSessionAfterPermission) {
                openSessionAfterPermission = false;
                log(openReceiverSession(device));
                return;
            }
            log(probeOpenDevice(device));
            return;
        }
        if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
            log("[USB] attached: " + (device != null ? shortDeviceName(device) : "unknown device"));
            log(scanReport());
            return;
        }
        if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
            log("[USB] detached: " + (device != null ? shortDeviceName(device) : "unknown device"));
            if (device != null && activeDevice != null &&
                    device.getDeviceName().equals(activeDevice.getDeviceName())) {
                closeActiveSession();
                log("[USB OTG] active session closed after detach");
            }
        }
    }

    private UsbDevice bestDevice() {
        List<UsbDevice> devices = sortedDevices();
        return devices.isEmpty() ? null : devices.get(0);
    }

    private String noUsbDevicesMessage(String prefix) {
        return prefix + " no devices visible to Android USB host API" +
                "\n" + prefix + " disconnect the phone from the PC USB cable and use a powered OTG hub for Fobos";
    }

    private List<UsbDevice> sortedDevices() {
        HashMap<String, UsbDevice> deviceList = usbManager.getDeviceList();
        List<UsbDevice> devices = new ArrayList<>(deviceList.values());
        devices.sort(Comparator
                .comparing((UsbDevice device) -> !isFobosCandidate(device))
                .thenComparing(UsbDevice::getDeviceName));
        return devices;
    }

    private void appendDeviceReport(StringBuilder report, UsbDevice device) {
        report.append('\n')
                .append(isFobosCandidate(device) ? "  * " : "  - ")
                .append(shortDeviceName(device))
                .append('\n')
                .append("    class ").append(formatByte(device.getDeviceClass()))
                .append(" subclass ").append(formatByte(device.getDeviceSubclass()))
                .append(" protocol ").append(formatByte(device.getDeviceProtocol()))
                .append('\n')
                .append("    manufacturer: ").append(safeManufacturer(device))
                .append(" product: ").append(safeProduct(device))
                .append(" serial: ").append(safeSerial(device))
                .append('\n')
                .append("    permission: ").append(usbManager.hasPermission(device) ? "granted" : "not granted")
                .append(" interfaces: ").append(device.getInterfaceCount());
        for (int i = 0; i < device.getInterfaceCount(); ++i) {
            UsbInterface usbInterface = device.getInterface(i);
            report.append('\n')
                    .append("      if").append(i)
                    .append(" id ").append(usbInterface.getId())
                    .append(" class ").append(formatByte(usbInterface.getInterfaceClass()))
                    .append(" subclass ").append(formatByte(usbInterface.getInterfaceSubclass()))
                    .append(" protocol ").append(formatByte(usbInterface.getInterfaceProtocol()))
                    .append(" endpoints ").append(usbInterface.getEndpointCount());
            for (int j = 0; j < usbInterface.getEndpointCount(); ++j) {
                UsbEndpoint endpoint = usbInterface.getEndpoint(j);
                report.append('\n')
                        .append("        ep").append(j)
                        .append(' ').append(endpointDirection(endpoint))
                        .append(' ').append(endpointType(endpoint))
                        .append(" addr ").append(formatByte(endpoint.getAddress()))
                        .append(" maxPacket ").append(endpoint.getMaxPacketSize())
                        .append(" interval ").append(endpoint.getInterval());
            }
        }
    }

    private String probeOpenDevice(UsbDevice device) {
        StringBuilder report = new StringBuilder();
        report.append("[USB] permission granted for ").append(shortDeviceName(device));
        UsbDeviceConnection connection = null;
        try {
            connection = usbManager.openDevice(device);
            if (connection == null) {
                report.append("\n[USB] openDevice returned null");
                return report.toString();
            }
            byte[] rawDescriptors = connection.getRawDescriptors();
            report.append("\n[USB] openDevice OK");
            if (rawDescriptors != null) {
                report.append(", raw descriptors ").append(rawDescriptors.length).append(" bytes");
                int bcdDevice = bcdDeviceFromRawDescriptors(rawDescriptors);
                if (bcdDevice >= 0) {
                    report.append(", bcdDevice ").append(formatWord(bcdDevice));
                }
            }
            if (isFobosCandidate(device)) {
                report.append("\n[USB] Fobos candidate detected: ")
                        .append(fobosApiHint(rawDescriptors));
            }
        } catch (SecurityException e) {
            report.append("\n[USB] open blocked by Android security: ").append(e.getMessage());
        } catch (RuntimeException e) {
            report.append("\n[USB] open failed: ").append(e.getMessage());
        } finally {
            if (connection != null) {
                connection.close();
            }
        }
        return report.toString();
    }

    private String openReceiverSession(UsbDevice device) {
        StringBuilder report = new StringBuilder();
        report.append("[USB OTG] opening receiver session for ").append(shortDeviceName(device));
        closeActiveSession();
        UsbDeviceConnection connection = null;
        try {
            connection = usbManager.openDevice(device);
            if (connection == null) {
                report.append("\n[USB OTG] openDevice returned null");
                return report.toString();
            }
            byte[] rawDescriptors = connection.getRawDescriptors();
            if (rawDescriptors != null) {
                report.append("\n[USB OTG] raw descriptors ").append(rawDescriptors.length)
                        .append(" bytes, ").append(fobosApiHint(rawDescriptors));
            }
            activeAgileApi = bcdDeviceFromRawDescriptors(rawDescriptors) == FOBOS_AGILE_BCD_DEVICE;
            StreamInterfaceChoice choice = chooseStreamInterface(device);
            if (choice == null) {
                report.append("\n[USB OTG] no bulk IN endpoint found; cannot prepare streaming path");
                connection.close();
                return report.toString();
            }
            if (!connection.claimInterface(choice.usbInterface, true)) {
                report.append("\n[USB OTG] claimInterface failed for if")
                        .append(choice.interfaceIndex);
                connection.close();
                return report.toString();
            }
            activeDevice = device;
            activeConnection = connection;
            activeInterface = choice.usbInterface;
            activeBulkInEndpoint = choice.bulkInEndpoint;
            activeBulkOutEndpoint = choice.bulkOutEndpoint;
            convertDcI = 0.0;
            convertDcQ = 0.0;
            connection = null;
            report.append("\n[USB OTG] claimed if").append(choice.interfaceIndex)
                    .append(" id ").append(activeInterface.getId())
                    .append(", bulk IN ").append(endpointSummary(activeBulkInEndpoint));
            if (activeBulkOutEndpoint != null) {
                report.append(", bulk OUT ").append(endpointSummary(activeBulkOutEndpoint));
            } else {
                report.append(", no bulk OUT endpoint");
            }
            appendFirmwareInfoProbe(report);
            report.append("\n[USB OTG] base ready; OTG preview and sample test are available");
        } catch (SecurityException e) {
            report.append("\n[USB OTG] open blocked by Android security: ").append(e.getMessage());
            if (connection != null) {
                connection.close();
            }
        } catch (RuntimeException e) {
            report.append("\n[USB OTG] open failed: ").append(e.getMessage());
            if (connection != null) {
                connection.close();
            }
        }
        return report.toString();
    }

    private StreamInterfaceChoice chooseStreamInterface(UsbDevice device) {
        StreamInterfaceChoice best = null;
        int bestScore = -1;
        for (int i = 0; i < device.getInterfaceCount(); ++i) {
            UsbInterface usbInterface = device.getInterface(i);
            UsbEndpoint bulkIn = null;
            UsbEndpoint bulkOut = null;
            for (int j = 0; j < usbInterface.getEndpointCount(); ++j) {
                UsbEndpoint endpoint = usbInterface.getEndpoint(j);
                if (endpoint.getType() != UsbConstants.USB_ENDPOINT_XFER_BULK) {
                    continue;
                }
                if (endpoint.getDirection() == UsbConstants.USB_DIR_IN && bulkIn == null) {
                    bulkIn = endpoint;
                } else if (endpoint.getDirection() == UsbConstants.USB_DIR_OUT && bulkOut == null) {
                    bulkOut = endpoint;
                }
            }
            if (bulkIn == null) {
                continue;
            }
            int score = 10;
            if (bulkOut != null) {
                score += 5;
            }
            if (usbInterface.getInterfaceClass() == UsbConstants.USB_CLASS_VENDOR_SPEC) {
                score += 2;
            }
            score += usbInterface.getEndpointCount();
            if (score > bestScore) {
                best = new StreamInterfaceChoice(i, usbInterface, bulkIn, bulkOut);
                bestScore = score;
            }
        }
        return best;
    }

    private void appendFirmwareInfoProbe(StringBuilder report) {
        if (activeConnection == null) {
            report.append("\n[USB OTG] firmware info unavailable: no open session");
            return;
        }
        ControlStringResult hw = readFobosInfoString(0);
        ControlStringResult fw = readFobosInfoString(1);
        ControlStringResult build = readFobosInfoString(2);
        report.append("\n[USB OTG] ctrl 0xE8 hw ")
                .append(controlResultSummary(hw))
                .append(", fw ")
                .append(controlResultSummary(fw))
                .append(", build ")
                .append(controlResultSummary(build));
    }

    private ControlStringResult readFobosInfoString(int value) {
        byte[] buffer = new byte[32];
        int result;
        try {
            result = activeConnection.controlTransfer(
                    USB_VENDOR_IN,
                    FOBOS_INFO_REQUEST,
                    value,
                    0,
                    buffer,
                    buffer.length,
                    CTRL_TIMEOUT_MS);
        } catch (RuntimeException e) {
            return new ControlStringResult(-9999, e.getMessage());
        }
        if (result <= 0) {
            return new ControlStringResult(result, "");
        }
        return new ControlStringResult(result, decodeAsciiString(buffer, result));
    }

    private int fx3Command(int command, int value) {
        if (activeConnection == null) {
            return -1;
        }
        return activeConnection.controlTransfer(
                USB_VENDOR_OUT,
                FOBOS_SDR_CMD,
                command,
                value,
                new byte[0],
                0,
                CTRL_TIMEOUT_MS);
    }

    private int controlOutU64(int command, long value) {
        if (activeConnection == null) {
            return -1;
        }
        byte[] buffer = new byte[8];
        for (int i = 0; i < buffer.length; ++i) {
            buffer[i] = (byte) ((value >> (8 * i)) & 0xff);
        }
        return activeConnection.controlTransfer(
                USB_VENDOR_OUT,
                FOBOS_SDR_CMD,
                command,
                0,
                buffer,
                buffer.length,
                CTRL_TIMEOUT_MS);
    }

    private String controlResultSummary(ControlStringResult result) {
        if (result.bytesRead > 0) {
            return "'" + result.value + "' (" + result.bytesRead + " bytes)";
        }
        if (result.value != null && !result.value.isEmpty()) {
            return "error " + result.bytesRead + " " + result.value;
        }
        return "result " + result.bytesRead;
    }

    private String decodeAsciiString(byte[] buffer, int length) {
        int count = Math.max(0, Math.min(length, buffer.length));
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < count; ++i) {
            int value = buffer[i] & 0xff;
            if (value == 0) {
                break;
            }
            if (value >= 32 && value <= 126) {
                builder.append((char) value);
            }
        }
        return builder.toString();
    }

    private void stopPreviewThread(long joinTimeoutMs) {
        previewRunning = false;
        Thread thread = previewThread;
        if (thread == null) {
            return;
        }
        if (thread == Thread.currentThread()) {
            return;
        }
        try {
            thread.join(Math.max(0L, joinTimeoutMs));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if (!thread.isAlive()) {
            previewThread = null;
        }
    }

    private int previewFftLength(int requested) {
        int capped = Math.max(256, Math.min(PREVIEW_MAX_FFT, requested > 0 ? requested : 2048));
        int fftLength = 1;
        while ((fftLength << 1) > 0 && (fftLength << 1) <= capped) {
            fftLength <<= 1;
        }
        return Math.max(256, fftLength);
    }

    private int normalizeModulationType(int modulationType) {
        switch (modulationType) {
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_SAM:
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
            case RadioSettings.MOD_DSB:
            case RadioSettings.MOD_CW:
            case RadioSettings.MOD_WFM:
            case RadioSettings.MOD_DMR:
            case RadioSettings.MOD_AM:
                return modulationType;
            default:
                return RadioSettings.MOD_AM;
        }
    }

    private void resetAudioDemodState() {
        audioSourceCursor = 0.0;
        audioMixerPhase = 0.0;
        audioFmLastPhase = 0.0;
        audioFmLastI = 0.0;
        audioFmLastQ = 0.0;
        audioFmPhaseValid = false;
        audioEnvelopeAverage = 0.1;
        audioDcAverage = 0.0;
        audioLowPassAverage = 0.0;
        audioChannelSumI = 0.0;
        audioChannelSumQ = 0.0;
        audioChannelDecimationCount = 0;
        audioChannelLowPassI = 0.0;
        audioChannelLowPassQ = 0.0;
        audioResamplePhase = 0.0;
        audioAgcLevel = 0.05;
        audioFirTaps = new double[0];
        audioFirRingI = new double[0];
        audioFirRingQ = new double[0];
        audioFirRingIndex = 0;
        audioFirConfiguredDecimation = 0;
        audioFirConfiguredModulation = -1;
        audioFirConfiguredSampleRate = 0.0;
        audioFirConfiguredBandwidth = 0.0;
        audioFirOutputI = 0.0;
        audioFirOutputQ = 0.0;
        audioDeemphasisAverage = 0.0;
        audioDebugLastLogNanos = 0L;
        audioRawDcI = 8192.0;
        audioRawDcQ = 8192.0;
        previewAudioSampleRate = 0.0;
        previewMeasuredSampleRate = 0.0;
        previewAudioRateLastNanos = 0L;
    }

    private void resetFobosConversionState() {
        convertDcI = 0.0;
        convertDcQ = 0.0;
    }

    private float[] convertRawFobosIq(byte[] buffer, int bytesRead, int inputMode) {
        int usableBytes = Math.max(0, Math.min(bytesRead, buffer.length)) & ~0x3;
        if (usableBytes < 4) {
            return new float[0];
        }
        float[] samples = new float[(usableBytes / 4) * 2];
        boolean directSampling = RadioSettings.isDirectInput(inputMode);
        int p0 = readLe16(buffer, 0);
        int p1 = usableBytes >= 4 ? readLe16(buffer, 2) : 0;
        boolean rawSwapIq = (p0 & 0x8000) != 0 && (p1 & 0x8000) != 0;
        if (activeAgileApi) {
            boolean swappedHalves = (p0 & 0x4000) != 0 && (p1 & 0x4000) != 0;
            int firstOffset = p0 & p1 & 0x4000;
            int secondOffset = firstOffset ^ 0x4000;
            if (swappedHalves && usableBytes >= 0x8000) {
                int pairs = usableBytes / 0x8000;
                for (int pair = 0; pair < pairs; ++pair) {
                    int inputBase = pair * 0x8000;
                    int outputBase = pair * 0x4000;
                    convertFobosBlock(buffer, inputBase + firstOffset, 0x4000,
                            samples, outputBase, directSampling, rawSwapIq);
                    convertFobosBlock(buffer, inputBase + secondOffset, 0x4000,
                            samples, outputBase + 0x2000, directSampling, rawSwapIq);
                }
                int consumedBytes = pairs * 0x8000;
                if (consumedBytes < usableBytes) {
                    convertFobosBlock(buffer, consumedBytes, usableBytes - consumedBytes,
                            samples, (consumedBytes / 4) * 2, directSampling, rawSwapIq);
                }
                return samples;
            }
        }
        convertFobosBlock(buffer, 0, usableBytes, samples, 0, directSampling, rawSwapIq);
        return samples;
    }

    private void convertFobosBlock(byte[] buffer,
                                   int offset,
                                   int length,
                                   float[] output,
                                   int outputOffset,
                                   boolean directSampling,
                                   boolean rawSwapIq) {
        if (offset < 0 || offset >= buffer.length || outputOffset >= output.length) {
            return;
        }
        int usableBytes = Math.max(0, Math.min(length, buffer.length - offset)) & ~0x3;
        boolean swapIq = directSampling || rawSwapIq != true;
        double scale = 1.0 / 32768.0;
        double dcRate = 0.0004;
        int outputIndex = Math.max(0, outputOffset);
        int end = offset + usableBytes;
        for (int pos = offset; pos + 3 < end && outputIndex + 1 < output.length; pos += 4) {
            int first = readLe16(buffer, pos);
            int second = readLe16(buffer, pos + 2);
            double real = (swapIq ? second : first) & 0x3fff;
            double imag = (swapIq ? first : second) & 0x3fff;
            convertDcI += dcRate * (real - convertDcI);
            convertDcQ += dcRate * (imag - convertDcQ);
            output[outputIndex++] = (float) ((real - convertDcI) * scale);
            output[outputIndex++] = (float) ((imag - convertDcQ) * scale);
        }
    }

    private int readLe16(byte[] buffer, int offset) {
        if (offset < 0 || offset + 1 >= buffer.length) {
            return 0;
        }
        return (buffer[offset] & 0xff) | ((buffer[offset + 1] & 0xff) << 8);
    }

    private int rawFobosComplexSampleCount(byte[] buffer, int bytesRead) {
        int usableBytes = Math.max(0, Math.min(bytesRead, buffer.length)) & ~0x3;
        return usableBytes / 4;
    }

    private double updatePreviewAudioSampleRate(long nowNanos,
                                                int complexSamples,
                                                double requestedSampleRate) {
        long lastNanos = previewAudioRateLastNanos;
        previewAudioRateLastNanos = nowNanos;
        if (lastNanos > 0L && complexSamples > 0) {
            double elapsedSeconds = (nowNanos - lastNanos) / 1_000_000_000.0;
            if (elapsedSeconds > 0.00005 && elapsedSeconds < 0.5) {
                double instantRate = complexSamples / elapsedSeconds;
                double upperBound = requestedSampleRate > 0.0
                        ? requestedSampleRate * 1.25
                        : PREVIEW_MAX_SAMPLE_RATE;
                if (instantRate > 100_000.0 && instantRate < upperBound) {
                    double clampedRate = requestedSampleRate > 0.0
                            ? Math.min(requestedSampleRate, instantRate)
                            : instantRate;
                    if (previewMeasuredSampleRate <= 0.0) {
                        previewMeasuredSampleRate = clampedRate;
                    } else {
                        double alpha = clampedRate < previewMeasuredSampleRate ? 0.25 : 0.08;
                        previewMeasuredSampleRate += alpha * (clampedRate - previewMeasuredSampleRate);
                    }
                    previewAudioSampleRate = Math.max(AUDIO_SAMPLE_RATE * 2.0,
                            Math.min(requestedSampleRate > 0.0 ? requestedSampleRate : PREVIEW_MAX_SAMPLE_RATE,
                                    previewMeasuredSampleRate * 1.06));
                    return previewAudioSampleRate;
                }
            }
        }
        double effectiveRate = previewMeasuredSampleRate > 100_000.0
                ? previewMeasuredSampleRate
                : requestedSampleRate;
        if (requestedSampleRate > 0.0) {
            effectiveRate = Math.min(requestedSampleRate, effectiveRate);
        }
        effectiveRate = Math.max(AUDIO_SAMPLE_RATE * 2.0, effectiveRate);
        previewAudioSampleRate = effectiveRate;
        return effectiveRate;
    }

    private byte[] audioFrameFromRawFobosIq(byte[] buffer,
                                           int bytesRead,
                                           double centerFrequency,
                                           double listeningFrequency,
                                           double nominalSampleRate,
                                           double timingSampleRate,
                                           int inputMode,
                                           int modulationType) {
        int usableBytes = Math.max(0, Math.min(bytesRead, buffer.length)) & ~0x3;
        int complexSamples = usableBytes / 4;
        if (complexSamples <= 2 ||
                nominalSampleRate <= AUDIO_SAMPLE_RATE ||
                timingSampleRate <= AUDIO_SAMPLE_RATE) {
            return new byte[0];
        }

        double tuneOffsetHz = RadioSettings.isDirectInput(inputMode)
                ? listeningFrequency
                : listeningFrequency - centerFrequency;
        double phaseStep = -2.0 * Math.PI * tuneOffsetHz / nominalSampleRate;
        int decimationFactor = audioChannelDecimationFactor(nominalSampleRate, modulationType, previewBandwidth);
        double channelRate = nominalSampleRate / Math.max(1, decimationFactor);
        double channelCutoff = Math.min(audioChannelCutoffForMode(modulationType, previewBandwidth),
                channelRate * 0.45);
        double channelAlpha = clamp01(1.0 - Math.exp(-2.0 * Math.PI * channelCutoff / channelRate));
        double demodCutoff = Math.min(audioDemodCutoffForMode(modulationType, previewBandwidth),
                channelRate * 0.45);
        double demodAlpha = clamp01(1.0 - Math.exp(-2.0 * Math.PI * demodCutoff / channelRate));
        double outputStep = (AUDIO_SAMPLE_RATE * Math.max(1, decimationFactor)) / nominalSampleRate;
        int estimatedOutputSamples = Math.max(8,
                (int) Math.ceil((complexSamples / nominalSampleRate) * AUDIO_SAMPLE_RATE) + 16);
        byte[] pcm = new byte[estimatedOutputSamples * 2];
        int output = 0;

        boolean directSampling = RadioSettings.isDirectInput(inputMode);
        int p0 = readLe16(buffer, 0);
        int p1 = usableBytes >= 4 ? readLe16(buffer, 2) : 0;
        boolean rawSwapIq = (p0 & 0x8000) != 0 && (p1 & 0x8000) != 0;
        boolean swapIq = directSampling || rawSwapIq != true;
        boolean agileSwappedHalves = false;
        int agileFirstOffset = 0;
        int agileSecondOffset = 0x4000;
        if (activeAgileApi) {
            agileSwappedHalves = (p0 & 0x4000) != 0 && (p1 & 0x4000) != 0 && usableBytes >= 0x8000;
            agileFirstOffset = p0 & p1 & 0x4000;
            agileSecondOffset = agileFirstOffset ^ 0x4000;
        }

        int decimatedSamples = 0;
        double oscillatorI = Math.cos(audioMixerPhase);
        double oscillatorQ = Math.sin(audioMixerPhase);
        double stepI = Math.cos(phaseStep);
        double stepQ = Math.sin(phaseStep);
        ensureAudioFirConfigured(decimationFactor, nominalSampleRate, modulationType, previewBandwidth);
        double rawScale = 1.0 / 8192.0;
        double dcRate = 0.0015;
        double rawPeak = 0.0;
        double channelPeak = 0.0;
        int pcmPeak = 0;

        for (int sample = 0; sample < complexSamples; ++sample) {
            int pos = rawFobosSampleOffset(sample, usableBytes, agileSwappedHalves,
                    agileFirstOffset, agileSecondOffset);
            if (pos < 0 || pos + 3 >= usableBytes) {
                continue;
            }

            int first = readLe16(buffer, pos);
            int second = readLe16(buffer, pos + 2);
            double real = (swapIq ? second : first) & 0x3fff;
            double imag = (swapIq ? first : second) & 0x3fff;
            audioRawDcI += dcRate * (real - audioRawDcI);
            audioRawDcQ += dcRate * (imag - audioRawDcQ);
            double iSample = (real - audioRawDcI) * rawScale;
            double qSample = (imag - audioRawDcQ) * rawScale;
            rawPeak = Math.max(rawPeak, Math.max(Math.abs(iSample), Math.abs(qSample)));

            if (inputMode == RadioSettings.INPUT_HF_COMBINED) {
                if (tuneOffsetHz < 0.0) {
                    qSample = 0.0;
                } else {
                    iSample = qSample;
                    qSample = 0.0;
                }
            } else if (inputMode == RadioSettings.INPUT_HF1 ||
                    inputMode == RadioSettings.INPUT_HF_NOISE_CANCEL) {
                qSample = 0.0;
            } else if (inputMode == RadioSettings.INPUT_HF2) {
                iSample = qSample;
                qSample = 0.0;
            }

            double mixedI = iSample * oscillatorI - qSample * oscillatorQ;
            double mixedQ = iSample * oscillatorQ + qSample * oscillatorI;
            pushAudioFirSample(mixedI, mixedQ);
            ++audioChannelDecimationCount;

            if (audioChannelDecimationCount < decimationFactor) {
                double nextOscillatorI = oscillatorI * stepI - oscillatorQ * stepQ;
                oscillatorQ = oscillatorI * stepQ + oscillatorQ * stepI;
                oscillatorI = nextOscillatorI;
                if ((sample & 0x1ff) == 0) {
                    double magnitude = oscillatorI * oscillatorI + oscillatorQ * oscillatorQ;
                    if (magnitude > 0.0) {
                        double scale = 1.0 / Math.sqrt(magnitude);
                        oscillatorI *= scale;
                        oscillatorQ *= scale;
                    }
                }
                continue;
            }

            double channelI = audioChannelSumI / audioChannelDecimationCount;
            double channelQ = audioChannelSumQ / audioChannelDecimationCount;
            audioChannelSumI = 0.0;
            audioChannelSumQ = 0.0;
            audioChannelDecimationCount = 0;
            ++decimatedSamples;
            channelPeak = Math.max(channelPeak, Math.hypot(channelI, channelQ));

            double nextOscillatorI = oscillatorI * stepI - oscillatorQ * stepQ;
            oscillatorQ = oscillatorI * stepQ + oscillatorQ * stepI;
            oscillatorI = nextOscillatorI;
            if ((sample & 0x1ff) == 0) {
                double magnitude = oscillatorI * oscillatorI + oscillatorQ * oscillatorQ;
                if (magnitude > 0.0) {
                    double scale = 1.0 / Math.sqrt(magnitude);
                    oscillatorI *= scale;
                    oscillatorQ *= scale;
                }
            }

            audioChannelLowPassI += channelAlpha * (channelI - audioChannelLowPassI);
            audioChannelLowPassQ += channelAlpha * (channelQ - audioChannelLowPassQ);
            double demodulated = demodulateChannelSample(audioChannelLowPassI,
                    audioChannelLowPassQ,
                    modulationType,
                    channelRate);
            demodulated = applyAudioDeemphasis(demodulated, modulationType, channelRate);
            audioLowPassAverage += demodAlpha * (demodulated - audioLowPassAverage);
            demodulated = audioLowPassAverage;

            audioResamplePhase += outputStep;
            while (audioResamplePhase >= 1.0) {
                audioResamplePhase -= 1.0;
                double audioValue = normalizeAudioSample(demodulated, modulationType);
                int pcmValue = (int) Math.round(Math.max(-1.0, Math.min(1.0, audioValue)) * 32767.0);
                pcmPeak = Math.max(pcmPeak, Math.abs(pcmValue));
                if (output + 2 > pcm.length) {
                    byte[] grown = new byte[pcm.length * 2 + 64];
                    System.arraycopy(pcm, 0, grown, 0, output);
                    pcm = grown;
                }
                pcm[output++] = (byte) (pcmValue & 0xff);
                pcm[output++] = (byte) ((pcmValue >> 8) & 0xff);
            }
        }
        audioMixerPhase = Math.atan2(oscillatorQ, oscillatorI);

        long now = System.nanoTime();
        if (now - audioDebugLastLogNanos > 1_000_000_000L) {
            audioDebugLastLogNanos = now;
            Log.d(LOG_TAG, String.format(Locale.US,
                    "audio raw inIq=%d decimated=%d out=%d sr=%.3fMHz timing=%.3fMHz off=%.0fHz dec=%d chRate=%.0f rawPeak=%.3f chPeak=%.5f pcmPeak=%d mod=%d",
                    complexSamples,
                    decimatedSamples,
                    output / 2,
                    nominalSampleRate / 1_000_000.0,
                    timingSampleRate / 1_000_000.0,
                    tuneOffsetHz,
                    decimationFactor,
                    channelRate,
                    rawPeak,
                    channelPeak,
                    pcmPeak,
                    modulationType));
        }
        if (output == pcm.length) {
            return pcm;
        }
        byte[] trimmed = new byte[output];
        System.arraycopy(pcm, 0, trimmed, 0, output);
        return trimmed;
    }

    private int rawFobosSampleOffset(int sample,
                                     int usableBytes,
                                     boolean agileSwappedHalves,
                                     int agileFirstOffset,
                                     int agileSecondOffset) {
        int directOffset = sample * 4;
        if (!agileSwappedHalves) {
            return directOffset;
        }
        int pairBytes = 0x8000;
        int halfBytes = 0x4000;
        int pairSamples = pairBytes / 4;
        int halfSamples = halfBytes / 4;
        int pair = sample / pairSamples;
        int withinPair = sample - pair * pairSamples;
        int pairBase = pair * pairBytes;
        if (pairBase + pairBytes > usableBytes) {
            return directOffset;
        }
        if (withinPair < halfSamples) {
            return pairBase + agileFirstOffset + withinPair * 4;
        }
        return pairBase + agileSecondOffset + (withinPair - halfSamples) * 4;
    }

    private double normalizeRadians(double phase) {
        if (!Double.isFinite(phase)) {
            return 0.0;
        }
        return Math.IEEEremainder(phase, 2.0 * Math.PI);
    }

    private byte[] audioFrameFromIq(float[] iqSamples,
                                    double centerFrequency,
                                    double listeningFrequency,
                                    double nominalSampleRate,
                                    double timingSampleRate,
                                    int inputMode,
                                    int modulationType) {
        int complexSamples = iqSamples.length / 2;
        if (complexSamples <= 2 ||
                nominalSampleRate <= AUDIO_SAMPLE_RATE ||
                timingSampleRate <= AUDIO_SAMPLE_RATE) {
            return new byte[0];
        }

        double tuneOffsetHz = RadioSettings.isDirectInput(inputMode)
                ? listeningFrequency
                : listeningFrequency - centerFrequency;
        double phaseStep = -2.0 * Math.PI * tuneOffsetHz / nominalSampleRate;
        int decimationFactor = audioChannelDecimationFactor(nominalSampleRate, modulationType, previewBandwidth);
        double channelRate = timingSampleRate / Math.max(1, decimationFactor);
        double channelCutoff = Math.min(audioChannelCutoffForMode(modulationType, previewBandwidth),
                channelRate * 0.45);
        double channelAlpha = clamp01(1.0 - Math.exp(-2.0 * Math.PI * channelCutoff / channelRate));
        double demodCutoff = Math.min(audioDemodCutoffForMode(modulationType, previewBandwidth),
                channelRate * 0.45);
        double demodAlpha = clamp01(1.0 - Math.exp(-2.0 * Math.PI * demodCutoff / channelRate));
        double outputStep = (AUDIO_SAMPLE_RATE * Math.max(1, decimationFactor)) / timingSampleRate;
        int estimatedOutputSamples = Math.max(8,
                (int) Math.ceil((complexSamples / timingSampleRate) * AUDIO_SAMPLE_RATE) + 16);
        byte[] pcm = new byte[estimatedOutputSamples * 2];
        int output = 0;
        double rawPeak = 0.0;
        double channelPeak = 0.0;
        int pcmPeak = 0;
        double oscillatorI = Math.cos(audioMixerPhase);
        double oscillatorQ = Math.sin(audioMixerPhase);
        double stepI = Math.cos(phaseStep);
        double stepQ = Math.sin(phaseStep);

        for (int sample = 0; sample < complexSamples; ++sample) {
            int base = sample * 2;
            double iSample = iqSamples[base];
            double qSample = iqSamples[base + 1];
            rawPeak = Math.max(rawPeak, Math.max(Math.abs(iSample), Math.abs(qSample)));

            if (inputMode == RadioSettings.INPUT_HF_COMBINED) {
                if (tuneOffsetHz < 0.0) {
                    qSample = 0.0;
                } else {
                    iSample = qSample;
                    qSample = 0.0;
                }
            } else if (inputMode == RadioSettings.INPUT_HF1 ||
                    inputMode == RadioSettings.INPUT_HF_NOISE_CANCEL) {
                qSample = 0.0;
            } else if (inputMode == RadioSettings.INPUT_HF2) {
                iSample = qSample;
                qSample = 0.0;
            }

            audioChannelSumI += iSample * oscillatorI - qSample * oscillatorQ;
            audioChannelSumQ += iSample * oscillatorQ + qSample * oscillatorI;
            ++audioChannelDecimationCount;

            double nextOscillatorI = oscillatorI * stepI - oscillatorQ * stepQ;
            oscillatorQ = oscillatorI * stepQ + oscillatorQ * stepI;
            oscillatorI = nextOscillatorI;
            if ((sample & 0x1ff) == 0) {
                double magnitude = oscillatorI * oscillatorI + oscillatorQ * oscillatorQ;
                if (magnitude > 0.0) {
                    double scale = 1.0 / Math.sqrt(magnitude);
                    oscillatorI *= scale;
                    oscillatorQ *= scale;
                }
            }

            if (audioChannelDecimationCount < decimationFactor) {
                continue;
            }

            updateAudioFirOutput();
            double channelI = audioFirOutputI;
            double channelQ = audioFirOutputQ;
            channelPeak = Math.max(channelPeak, Math.hypot(channelI, channelQ));
            audioChannelSumI = 0.0;
            audioChannelSumQ = 0.0;
            audioChannelDecimationCount = 0;

            audioChannelLowPassI += channelAlpha * (channelI - audioChannelLowPassI);
            audioChannelLowPassQ += channelAlpha * (channelQ - audioChannelLowPassQ);
            double demodulated = demodulateChannelSample(audioChannelLowPassI,
                    audioChannelLowPassQ,
                    modulationType,
                    channelRate);
            demodulated = applyAudioDeemphasis(demodulated, modulationType, channelRate);
            audioLowPassAverage += demodAlpha * (demodulated - audioLowPassAverage);
            demodulated = audioLowPassAverage;

            audioResamplePhase += outputStep;
            while (audioResamplePhase >= 1.0) {
                audioResamplePhase -= 1.0;
                double audioValue = normalizeAudioSample(demodulated, modulationType);
                int pcmValue = (int) Math.round(Math.max(-1.0, Math.min(1.0, audioValue)) * 32767.0);
                pcmPeak = Math.max(pcmPeak, Math.abs(pcmValue));
                if (output + 2 > pcm.length) {
                    byte[] grown = new byte[pcm.length * 2 + 64];
                    System.arraycopy(pcm, 0, grown, 0, output);
                    pcm = grown;
                }
                pcm[output++] = (byte) (pcmValue & 0xff);
                pcm[output++] = (byte) ((pcmValue >> 8) & 0xff);
            }
        }
        audioMixerPhase = Math.atan2(oscillatorQ, oscillatorI);
        long now = System.nanoTime();
        if (now - audioDebugLastLogNanos > 1_000_000_000L) {
            audioDebugLastLogNanos = now;
            Log.d(LOG_TAG, String.format(Locale.US,
                    "audio frame inIq=%d out=%d sr=%.3fMHz timing=%.3fMHz off=%.0fHz dec=%d chRate=%.0f rawPeak=%.3f chPeak=%.5f pcmPeak=%d mod=%d",
                    complexSamples,
                    output / 2,
                    nominalSampleRate / 1_000_000.0,
                    timingSampleRate / 1_000_000.0,
                    tuneOffsetHz,
                    decimationFactor,
                    channelRate,
                    rawPeak,
                    channelPeak,
                    pcmPeak,
                    modulationType));
        }
        if (output == pcm.length) {
            return pcm;
        }
        byte[] trimmed = new byte[output];
        System.arraycopy(pcm, 0, trimmed, 0, output);
        return trimmed;
    }

    private void ensureAudioFirConfigured(int decimationFactor,
                                          double sampleRate,
                                          int modulationType,
                                          double bandwidth) {
        int normalizedDecimation = Math.max(1, decimationFactor);
        if (audioFirTaps.length > 0 &&
                audioFirConfiguredDecimation == normalizedDecimation &&
                audioFirConfiguredModulation == modulationType &&
                Math.abs(audioFirConfiguredSampleRate - sampleRate) < 1.0 &&
                Math.abs(audioFirConfiguredBandwidth - bandwidth) < 1.0) {
            return;
        }

        int tapCount = audioFirTapCount(normalizedDecimation, modulationType);
        double outputNyquist = sampleRate / (2.0 * normalizedDecimation);
        double cutoff = Math.min(audioChannelCutoffForMode(modulationType, bandwidth) * 1.15,
                outputNyquist * 0.82);
        cutoff = Math.max(500.0, Math.min(cutoff, sampleRate * 0.45));

        double[] taps = new double[tapCount];
        double normalizedCutoff = cutoff / sampleRate;
        int center = tapCount / 2;
        double sum = 0.0;
        for (int i = 0; i < tapCount; ++i) {
            int m = i - center;
            double sinc = m == 0
                    ? 2.0 * normalizedCutoff
                    : Math.sin(2.0 * Math.PI * normalizedCutoff * m) / (Math.PI * m);
            double window = 0.42
                    - 0.5 * Math.cos((2.0 * Math.PI * i) / (tapCount - 1))
                    + 0.08 * Math.cos((4.0 * Math.PI * i) / (tapCount - 1));
            taps[i] = sinc * window;
            sum += taps[i];
        }
        if (Math.abs(sum) > 1.0e-12) {
            for (int i = 0; i < taps.length; ++i) {
                taps[i] /= sum;
            }
        }

        audioFirTaps = taps;
        audioFirRingI = new double[tapCount];
        audioFirRingQ = new double[tapCount];
        audioFirRingIndex = 0;
        audioChannelDecimationCount = 0;
        audioFirConfiguredDecimation = normalizedDecimation;
        audioFirConfiguredModulation = modulationType;
        audioFirConfiguredSampleRate = sampleRate;
        audioFirConfiguredBandwidth = bandwidth;
    }

    private int audioFirTapCount(int decimationFactor, int modulationType) {
        int taps;
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                taps = decimationFactor * 4 + 1;
                taps = Math.max(41, Math.min(81, taps));
                break;
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_DMR:
                taps = decimationFactor * 5 + 1;
                taps = Math.max(49, Math.min(97, taps));
                break;
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
            case RadioSettings.MOD_CW:
                taps = decimationFactor * 5 + 1;
                taps = Math.max(49, Math.min(97, taps));
                break;
            default:
                taps = decimationFactor * 4 + 1;
                taps = Math.max(33, Math.min(81, taps));
                break;
        }
        return (taps & 1) == 0 ? taps + 1 : taps;
    }

    private void pushAudioFirSample(double sampleI, double sampleQ) {
        if (audioFirRingI.length == 0) {
            return;
        }
        audioFirRingIndex = (audioFirRingIndex + 1) % audioFirRingI.length;
        audioFirRingI[audioFirRingIndex] = sampleI;
        audioFirRingQ[audioFirRingIndex] = sampleQ;
    }

    private void updateAudioFirOutput() {
        if (audioFirTaps.length == 0 || audioFirRingI.length != audioFirTaps.length) {
            audioFirOutputI = 0.0;
            audioFirOutputQ = 0.0;
            return;
        }
        double sumI = 0.0;
        double sumQ = 0.0;
        int ringLength = audioFirRingI.length;
        int index = audioFirRingIndex;
        for (int tap = 0; tap < audioFirTaps.length; ++tap) {
            double coefficient = audioFirTaps[tap];
            sumI += audioFirRingI[index] * coefficient;
            sumQ += audioFirRingQ[index] * coefficient;
            --index;
            if (index < 0) {
                index = ringLength - 1;
            }
        }
        audioFirOutputI = sumI;
        audioFirOutputQ = sumQ;
    }

    private double applyAudioDeemphasis(double demodulated, int modulationType, double channelRate) {
        if (modulationType != RadioSettings.MOD_WFM || channelRate <= 0.0) {
            return demodulated;
        }
        double tauSeconds = 50.0e-6;
        double alpha = clamp01(1.0 - Math.exp(-1.0 / (tauSeconds * channelRate)));
        audioDeemphasisAverage += alpha * (demodulated - audioDeemphasisAverage);
        return audioDeemphasisAverage;
    }

    private double demodulateChannelSample(double channelI,
                                           double channelQ,
                                           int modulationType,
                                           double channelRate) {
        switch (modulationType) {
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_WFM:
            case RadioSettings.MOD_DMR:
                double magnitude = Math.hypot(channelI, channelQ);
                if (magnitude < 1.0e-9) {
                    return 0.0;
                }
                channelI /= magnitude;
                channelQ /= magnitude;
                if (!audioFmPhaseValid) {
                    audioFmLastI = channelI;
                    audioFmLastQ = channelQ;
                    audioFmPhaseValid = true;
                    return 0.0;
                }
                double cross = audioFmLastI * channelQ - audioFmLastQ * channelI;
                double power = audioFmLastI * audioFmLastI + audioFmLastQ * audioFmLastQ + 1.0e-12;
                double diff = cross / power;
                audioFmLastI = channelI;
                audioFmLastQ = channelQ;
                if (modulationType == RadioSettings.MOD_DMR) {
                    return diff * 10.0;
                }
                double deviationHz = modulationType == RadioSettings.MOD_WFM ? 75_000.0 : 5_000.0;
                double rate = Math.max(AUDIO_SAMPLE_RATE, channelRate);
                return diff * rate / (2.0 * Math.PI * deviationHz);
            case RadioSettings.MOD_AM:
            case RadioSettings.MOD_SAM:
                return Math.hypot(channelI, channelQ);
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
            case RadioSettings.MOD_DSB:
            case RadioSettings.MOD_CW:
                return channelI;
            default:
                return 0.0;
        }
    }

    private double normalizeAudioSample(double demodulated, int modulationType) {
        boolean digital = modulationType == RadioSettings.MOD_DMR;
        boolean fm = modulationType == RadioSettings.MOD_WFM || modulationType == RadioSettings.MOD_NFM;
        double dcRate = digital ? 0.0001 : (fm ? 0.0008 : 0.0005);
        audioDcAverage += (demodulated - audioDcAverage) * dcRate;
        double acSample = demodulated - audioDcAverage;
        double absSample = Math.abs(acSample);
        double agcRate = absSample > audioAgcLevel ? (fm ? 0.015 : 0.04) : 0.0002;
        audioAgcLevel += (absSample - audioAgcLevel) * agcRate;
        audioAgcLevel = Math.max(audioAgcLevel, 0.0001);
        double target = digital ? 0.25 : (modulationType == RadioSettings.MOD_WFM ? 0.42 :
                (modulationType == RadioSettings.MOD_NFM ? 0.34 : 0.28));
        double normalized = acSample * (target / audioAgcLevel);
        return digital ? normalized : Math.tanh(normalized * (fm ? 0.75 : 1.0));
    }

    private int audioChannelDecimationFactor(double sampleRate, int modulationType, double bandwidth) {
        double targetRate = audioTargetChannelRate(modulationType, bandwidth);
        int factor = Math.max(1, (int) Math.floor(sampleRate / Math.max(1.0, targetRate)));
        if (sampleRate > 1_000_000.0) {
            int minimum;
            switch (modulationType) {
                case RadioSettings.MOD_WFM:
                    minimum = 2;
                    break;
                case RadioSettings.MOD_NFM:
                case RadioSettings.MOD_DMR:
                    minimum = 3;
                    break;
                default:
                    minimum = 2;
                    break;
            }
            factor = Math.max(minimum, factor);
        }
        return factor;
    }

    private double audioTargetChannelRate(int modulationType, double bandwidth) {
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                return Math.max(384_000.0, Math.min(768_000.0, bandwidth * 3.0));
            case RadioSettings.MOD_DMR:
                return 96_000.0;
            case RadioSettings.MOD_NFM:
                return Math.max(240_000.0, Math.min(384_000.0, bandwidth * 4.0));
            default:
                return 192_000.0;
        }
    }

    private double audioChannelCutoffForMode(int modulationType, double bandwidth) {
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                return Math.min(140_000.0, Math.max(80_000.0, bandwidth * 0.55));
            case RadioSettings.MOD_NFM:
                return Math.min(25_000.0, Math.max(6_000.0, bandwidth * 0.55));
            case RadioSettings.MOD_DMR:
                return Math.min(9_500.0, Math.max(6_000.0, bandwidth * 0.75));
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
                return Math.min(3_600.0, Math.max(700.0, bandwidth * 0.95));
            case RadioSettings.MOD_CW:
                return Math.min(1_500.0, Math.max(250.0, bandwidth * 0.5));
            case RadioSettings.MOD_DSB:
            case RadioSettings.MOD_SAM:
                return Math.min(12_000.0, Math.max(1_000.0, bandwidth * 0.45));
            case RadioSettings.MOD_AM:
            default:
                return Math.min(10_000.0, Math.max(1_000.0, bandwidth * 0.45));
        }
    }

    private double audioDemodCutoffForMode(int modulationType, double bandwidth) {
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                return 15_000.0;
            case RadioSettings.MOD_NFM:
                return 3_000.0;
            case RadioSettings.MOD_DMR:
                return 6_000.0;
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
                return Math.min(3_600.0, Math.max(700.0, bandwidth * 0.95));
            case RadioSettings.MOD_CW:
                return 1_200.0;
            case RadioSettings.MOD_DSB:
            case RadioSettings.MOD_SAM:
                return Math.min(6_500.0, Math.max(1_000.0, bandwidth * 0.45));
            case RadioSettings.MOD_AM:
            default:
                return Math.min(6_000.0, Math.max(1_000.0, bandwidth * 0.45));
        }
    }

    private double clamp01(double value) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, value));
    }

    private int audioTapCount(double sourceStep, double sampleRate, int modulationType) {
        if (!Double.isFinite(sourceStep) || sourceStep <= 1.0 || sampleRate <= 0.0) {
            return 8;
        }
        double channelHz;
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                channelHz = 220_000.0;
                break;
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_DMR:
                channelHz = Math.max(9_000.0, Math.min(20_000.0, previewBandwidth));
                break;
            case RadioSettings.MOD_AM:
            case RadioSettings.MOD_SAM:
            case RadioSettings.MOD_DSB:
                channelHz = Math.max(6_000.0, Math.min(18_000.0, previewBandwidth * 1.6));
                break;
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
            case RadioSettings.MOD_CW:
                channelHz = Math.max(2_400.0, Math.min(6_000.0, previewBandwidth * 1.8));
                break;
            default:
                channelHz = Math.max(8_000.0, Math.min(20_000.0, previewBandwidth));
                break;
        }
        int taps = (int) Math.round((sampleRate / channelHz) * 2.2);
        return Math.max(12, Math.min(256, taps));
    }

    private double demodulateAudioSample(double mixedI, double mixedQ, int modulationType) {
        switch (modulationType) {
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_WFM:
            case RadioSettings.MOD_DMR:
                double phase = Math.atan2(mixedQ, mixedI);
                if (!audioFmPhaseValid) {
                    audioFmLastPhase = phase;
                    audioFmPhaseValid = true;
                    return 0.0;
                }
                double diff = wrapPhase(phase - audioFmLastPhase);
                audioFmLastPhase = phase;
                double fmGain = modulationType == RadioSettings.MOD_WFM ? 0.12 : 0.55;
                return audioLowPass(removeAudioDc(diff * fmGain, 0.0008), modulationType);
            case RadioSettings.MOD_AM:
            case RadioSettings.MOD_SAM:
                double envelope = Math.hypot(mixedI, mixedQ);
                audioEnvelopeAverage += (envelope - audioEnvelopeAverage) * 0.002;
                return audioLowPass(removeAudioDc((envelope - audioEnvelopeAverage) * 2.4, 0.0015),
                        modulationType);
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
            case RadioSettings.MOD_DSB:
            case RadioSettings.MOD_CW:
                return audioLowPass(removeAudioDc(mixedI * 1.4, 0.001), modulationType);
            default:
                return 0.0;
        }
    }

    private double audioLowPass(double value, int modulationType) {
        double cutoffHz;
        switch (modulationType) {
            case RadioSettings.MOD_WFM:
                cutoffHz = 15_000.0;
                break;
            case RadioSettings.MOD_NFM:
            case RadioSettings.MOD_DMR:
                cutoffHz = 3_600.0;
                break;
            case RadioSettings.MOD_CW:
                cutoffHz = 900.0;
                break;
            case RadioSettings.MOD_USB:
            case RadioSettings.MOD_LSB:
                cutoffHz = 3_000.0;
                break;
            default:
                cutoffHz = 5_000.0;
                break;
        }
        double alpha = 1.0 - Math.exp(-2.0 * Math.PI * cutoffHz / AUDIO_SAMPLE_RATE);
        audioLowPassAverage += (value - audioLowPassAverage) * Math.max(0.0, Math.min(1.0, alpha));
        return audioLowPassAverage;
    }

    private double removeAudioDc(double value, double rate) {
        audioDcAverage += (value - audioDcAverage) * Math.max(0.0, Math.min(1.0, rate));
        return value - audioDcAverage;
    }

    private double wrapPhase(double phase) {
        while (phase > Math.PI) {
            phase -= 2.0 * Math.PI;
        }
        while (phase < -Math.PI) {
            phase += 2.0 * Math.PI;
        }
        return phase;
    }

    private FobosNetworkClient.SpectrumFrame spectrumFrameFromIq(float[] iqSamples,
                                                                 double centerFrequency,
                                                                 double listeningFrequency,
                                                                 double sampleRate,
                                                                 int inputMode,
                                                                 int fftLength) {
        int availableComplexSamples = iqSamples.length / 2;
        if (availableComplexSamples < fftLength) {
            return null;
        }

        float[] real = new float[fftLength];
        float[] imag = new float[fftLength];
        int sourceStart = Math.max(0, availableComplexSamples - fftLength);
        for (int i = 0; i < fftLength; ++i) {
            int base = (sourceStart + i) * 2;
            double window = 0.5 - 0.5 * Math.cos((2.0 * Math.PI * i) / Math.max(1, fftLength - 1));
            real[i] = (float) (iqSamples[base] * window);
            imag[i] = (float) (iqSamples[base + 1] * window);
        }

        fftForward(real, imag);

        double displayCenter = inputMode == RadioSettings.INPUT_RF ? centerFrequency : 0.0;
        double minFrequency = displayCenter - sampleRate * 0.5;
        double maxFrequency = displayCenter + sampleRate * 0.5;
        if (inputMode == RadioSettings.INPUT_HF1 || inputMode == RadioSettings.INPUT_HF2) {
            minFrequency = 0.0;
            maxFrequency = sampleRate * 0.5;
        }

        double[] frequencies = new double[fftLength];
        float[] magnitudes = new float[fftLength];
        double binWidth = (maxFrequency - minFrequency) / Math.max(1, fftLength - 1);
        double normalizer = 1.0 / Math.max(1, fftLength);
        for (int i = 0; i < fftLength; ++i) {
            int shifted = (i + fftLength / 2) % fftLength;
            double magnitude = Math.hypot(real[shifted], imag[shifted]) * normalizer;
            double db = 20.0 * Math.log10(Math.max(1.0e-12, magnitude)) + 84.0;
            frequencies[i] = minFrequency + binWidth * i;
            magnitudes[i] = (float) Math.max(-180.0, Math.min(90.0, db));
        }

        return new FobosNetworkClient.SpectrumFrame(
                frequencies,
                magnitudes,
                minFrequency,
                maxFrequency,
                displayCenter,
                listeningFrequency,
                sampleRate,
                previewBandwidth,
                previewModulationType,
                inputMode,
                previewRequestedFftLength,
                true);
    }

    private void fftForward(float[] real, float[] imag) {
        int n = real.length;
        for (int i = 1, j = 0; i < n; ++i) {
            int bit = n >> 1;
            for (; (j & bit) != 0; bit >>= 1) {
                j ^= bit;
            }
            j ^= bit;
            if (i < j) {
                float tempReal = real[i];
                real[i] = real[j];
                real[j] = tempReal;
                float tempImag = imag[i];
                imag[i] = imag[j];
                imag[j] = tempImag;
            }
        }

        for (int len = 2; len <= n; len <<= 1) {
            double angle = -2.0 * Math.PI / len;
            float wLenReal = (float) Math.cos(angle);
            float wLenImag = (float) Math.sin(angle);
            int half = len >> 1;
            for (int start = 0; start < n; start += len) {
                float wReal = 1.0f;
                float wImag = 0.0f;
                for (int j = 0; j < half; ++j) {
                    int even = start + j;
                    int odd = even + half;
                    float oddReal = real[odd] * wReal - imag[odd] * wImag;
                    float oddImag = real[odd] * wImag + imag[odd] * wReal;
                    real[odd] = real[even] - oddReal;
                    imag[odd] = imag[even] - oddImag;
                    real[even] += oddReal;
                    imag[even] += oddImag;

                    float nextReal = wReal * wLenReal - wImag * wLenImag;
                    wImag = wReal * wLenImag + wImag * wLenReal;
                    wReal = nextReal;
                }
            }
        }
    }

    private String formatMhz(double valueHz) {
        return String.format(Locale.US, "%.3f", valueHz / 1_000_000.0);
    }

    private void closeActiveSession() {
        stopPreviewThread(700L);
        UsbDeviceConnection connection = activeConnection;
        UsbInterface usbInterface = activeInterface;
        activeConnection = null;
        activeDevice = null;
        activeInterface = null;
        activeBulkInEndpoint = null;
        activeBulkOutEndpoint = null;
        if (connection == null) {
            return;
        }
        if (usbInterface != null) {
            try {
                connection.releaseInterface(usbInterface);
            } catch (RuntimeException ignored) {
                // Android USB stacks differ; close is the real cleanup boundary.
            }
        }
        connection.close();
    }

    private boolean isFobosCandidate(UsbDevice device) {
        return device.getVendorId() == FOBOS_VENDOR_ID &&
                device.getProductId() == FOBOS_PRODUCT_ID;
    }

    private String fobosApiHint(byte[] rawDescriptors) {
        int bcdDevice = bcdDeviceFromRawDescriptors(rawDescriptors);
        if (bcdDevice == FOBOS_AGILE_BCD_DEVICE) {
            return "agile API expected";
        }
        if (bcdDevice == FOBOS_STANDARD_BCD_DEVICE) {
            return "standard API expected";
        }
        if (bcdDevice >= 0) {
            return "unknown bcdDevice " + formatWord(bcdDevice);
        }
        return "bcdDevice unavailable";
    }

    private int bcdDeviceFromRawDescriptors(byte[] rawDescriptors) {
        if (rawDescriptors == null || rawDescriptors.length < 14) {
            return -1;
        }
        return (rawDescriptors[12] & 0xff) | ((rawDescriptors[13] & 0xff) << 8);
    }

    private String shortDeviceName(UsbDevice device) {
        return String.format(Locale.US,
                "%s vid:pid %s:%s version %s%s",
                device.getDeviceName(),
                formatWord(device.getVendorId()),
                formatWord(device.getProductId()),
                safeVersion(device),
                isFobosCandidate(device) ? " (Fobos)" : "");
    }

    private String safeVersion(UsbDevice device) {
        try {
            String value = device.getVersion();
            return value != null ? value : "?";
        } catch (RuntimeException e) {
            return "?";
        }
    }

    private String safeManufacturer(UsbDevice device) {
        try {
            String value = device.getManufacturerName();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String safeProduct(UsbDevice device) {
        try {
            String value = device.getProductName();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String safeSerial(UsbDevice device) {
        try {
            String value = device.getSerialNumber();
            return value != null ? value : "?";
        } catch (SecurityException e) {
            return "<permission needed>";
        }
    }

    private String endpointDirection(UsbEndpoint endpoint) {
        return endpoint.getDirection() == UsbConstants.USB_DIR_IN ? "IN" : "OUT";
    }

    private String endpointType(UsbEndpoint endpoint) {
        switch (endpoint.getType()) {
            case UsbConstants.USB_ENDPOINT_XFER_CONTROL:
                return "control";
            case UsbConstants.USB_ENDPOINT_XFER_ISOC:
                return "iso";
            case UsbConstants.USB_ENDPOINT_XFER_BULK:
                return "bulk";
            case UsbConstants.USB_ENDPOINT_XFER_INT:
                return "interrupt";
            default:
                return "type " + endpoint.getType();
        }
    }

    private String endpointSummary(UsbEndpoint endpoint) {
        if (endpoint == null) {
            return "none";
        }
        return endpointDirection(endpoint) + " " +
                endpointType(endpoint) +
                " addr " + formatByte(endpoint.getAddress()) +
                " maxPacket " + endpoint.getMaxPacketSize();
    }

    private String hexPreview(byte[] buffer, int length, int limit) {
        int count = Math.max(0, Math.min(Math.min(length, buffer.length), limit));
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < count; ++i) {
            if (i > 0) {
                builder.append(' ');
            }
            builder.append(formatByte(buffer[i]));
        }
        if (length > count) {
            builder.append(" ...");
        }
        return builder.toString();
    }

    private String formatByte(int value) {
        return String.format(Locale.US, "0x%02x", value & 0xff);
    }

    private String formatWord(int value) {
        return String.format(Locale.US, "0x%04x", value & 0xffff);
    }

    private void log(String message) {
        if (listener != null) {
            listener.onUsbLog(message);
        } else {
            Log.d(LOG_TAG, message != null ? message : "<null USB sandbox message>");
        }
    }

    private static final class StreamInterfaceChoice {
        final int interfaceIndex;
        final UsbInterface usbInterface;
        final UsbEndpoint bulkInEndpoint;
        final UsbEndpoint bulkOutEndpoint;

        StreamInterfaceChoice(
                int interfaceIndex,
                UsbInterface usbInterface,
                UsbEndpoint bulkInEndpoint,
                UsbEndpoint bulkOutEndpoint) {
            this.interfaceIndex = interfaceIndex;
            this.usbInterface = usbInterface;
            this.bulkInEndpoint = bulkInEndpoint;
            this.bulkOutEndpoint = bulkOutEndpoint;
        }
    }

    private static final class ControlStringResult {
        final int bytesRead;
        final String value;

        ControlStringResult(int bytesRead, String value) {
            this.bytesRead = bytesRead;
            this.value = value;
        }
    }
}
