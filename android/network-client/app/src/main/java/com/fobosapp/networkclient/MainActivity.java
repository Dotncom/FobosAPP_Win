package com.fobosapp.networkclient;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.text.Editable;
import android.text.InputType;
import android.text.TextWatcher;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.SeekBar;
import android.widget.Spinner;
import android.widget.TextView;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.Locale;

public final class MainActivity extends Activity implements FobosNetworkClient.Listener {
    private static final String PREFS = "fobos_network_client";
    private static final int DEFAULT_PORT = 21090;
    private static final int AUTO_APPLY_DELAY_MS = 650;
    private static final int FAST_APPLY_DELAY_MS = 120;
    private static final int RELIABLE_SETTINGS_REPEAT_MS = 280;
    private static final int RELIABLE_SETTINGS_REPEAT_COUNT = 2;
    private static final int REMOTE_ECHO_GUARD_MS = 2500;
    private static final int LEVEL_DB_MIN = -160;
    private static final int LEVEL_DB_MAX = 0;
    private static final int LEVEL_MIN_GAP_DB = 5;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final RadioSettings settings = new RadioSettings();
    private final Runnable deferredSettingsApply = () -> sendControlNow("settings", false, true);
    private final Runnable reliableSettingsRepeat = new Runnable() {
        @Override
        public void run() {
            if (pendingReliableSettingsRepeats <= 0) {
                return;
            }
            if (client == null || !client.isConnected() || !client.canControl()) {
                pendingReliableSettingsRepeats = 0;
                return;
            }
            sendControlNow("settings", false, true);
            --pendingReliableSettingsRepeats;
            if (pendingReliableSettingsRepeats > 0) {
                mainHandler.postDelayed(this, RELIABLE_SETTINGS_REPEAT_MS);
            }
        }
    };
    private FobosNetworkClient client;
    private PcmAudioPlayer audioPlayer;
    private UsbSandbox usbSandbox;

    private EditText hostEdit;
    private EditText portEdit;
    private EditText centerEdit;
    private EditText listenEdit;
    private EditText bandwidthEdit;
    private Spinner sampleRateSpinner;
    private Spinner fftSpinner;
    private Spinner inputSpinner;
    private Spinner modulationSpinner;
    private CheckBox audioCheck;
    private CheckBox suppressServerCheck;
    private CheckBox fullResolutionSpectrumCheck;
    private CheckBox generalBandMarkersCheck;
    private CheckBox amateurBandMarkersCheck;
    private Button connectButton;
    private Button startButton;
    private Button stopButton;
    private Button settingsButton;
    private Button controlsButton;
    private Button requestControlButton;
    private Button usbScanButton;
    private Button usbPermissionButton;
    private TextView statusText;
    private TextView levelMinLabel;
    private TextView levelMaxLabel;
    private TextView fineTuneLabel;
    private View fineTuneModeButton;
    private TextView logText;
    private SeekBar levelMinSeek;
    private SeekBar levelMaxSeek;
    private FineTuneDialView fineTuneDial;
    private SpectrumView spectrumView;
    private ScrollView controlsScroll;
    private LinearLayout rootLayout;
    private LinearLayout commandPanel;
    private LinearLayout contentPanel;
    private LinearLayout levelPanel;

    private volatile boolean audioPlaybackEnabled = true;
    private boolean controlsVisible = true;
    private boolean fullResolutionSpectrumFrames = false;
    private int pendingReliableSettingsRepeats = 0;
    private boolean suppressUiCallbacks = false;
    private boolean suppressLevelCallbacks = false;
    private long localSettingsGuardUntilMs = 0L;
    private float displayLevelMin = -130.0f;
    private float displayLevelMax = -50.0f;
    private boolean fineTuneHoldMode = false;
    private boolean showGeneralBandMarkers = false;
    private boolean showAmateurBandMarkers = false;

    private final double[] sampleRates = new double[] {
            8_000_000.0, 10_000_000.0, 12_500_000.0, 16_000_000.0,
            20_000_000.0, 25_000_000.0, 32_000_000.0, 40_000_000.0,
            50_000_000.0, 64_000_000.0, 80_000_000.0
    };
    private final int[] fftLengths = new int[] {
            2_048, 4_096, 8_192, 16_384, 32_768, 65_536, 131_072,
            262_144, 524_288
    };
    private final int[] modulationIds = new int[] {
            RadioSettings.MOD_AM, RadioSettings.MOD_NFM, RadioSettings.MOD_WFM,
            RadioSettings.MOD_USB, RadioSettings.MOD_LSB, RadioSettings.MOD_DSB,
            RadioSettings.MOD_CW, RadioSettings.MOD_DMR
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        client = new FobosNetworkClient(this);
        audioPlayer = new PcmAudioPlayer();
        buildUi();
        usbSandbox = new UsbSandbox(this, this::appendUsbLog);
        usbSandbox.start();
        loadPrefs();
        updateButtons();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        savePrefs();
        mainHandler.removeCallbacks(deferredSettingsApply);
        mainHandler.removeCallbacks(reliableSettingsRepeat);
        if (client != null) {
            client.shutdown();
        }
        if (audioPlayer != null) {
            audioPlayer.shutdown();
        }
        if (usbSandbox != null) {
            usbSandbox.stop();
        }
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        applyResponsiveLayout();
    }

    @Override
    public void onStatus(String status) {
        runOnMain(() -> {
            statusText.setText(status);
            appendLog(status);
            updateButtons();
        });
    }

    @Override
    public void onDisconnected(String reason) {
        runOnMain(() -> {
            statusText.setText(reason);
            appendLog(reason);
            if (audioPlayer != null) {
                audioPlayer.stop();
            }
            updateButtons();
        });
    }

    @Override
    public void onRole(boolean canControl, String peerId, String peerLabel, String controllerPeerId) {
        runOnMain(() -> {
            String role = canControl ? "controller" : "observer";
            statusText.setText("Connected as " + role);
            appendLog("Role: " + role + (peerLabel.isEmpty() ? "" : " (" + peerLabel + ")"));
            updateButtons();
        });
    }

    @Override
    public void onSpectrum(FobosNetworkClient.SpectrumFrame frame) {
        runOnMain(() -> {
            boolean waitingForLocalSettingsEcho =
                    client != null &&
                    client.canControl() &&
                    SystemClock.uptimeMillis() < localSettingsGuardUntilMs;
            if (!waitingForLocalSettingsEcho) {
                if (frame.inputMode >= RadioSettings.INPUT_RF &&
                        frame.inputMode <= RadioSettings.INPUT_HF_NOISE_CANCEL) {
                    settings.inputMode = frame.inputMode;
                }
                if (isFinite(frame.centerFrequency)) {
                    settings.centerFrequency = frame.centerFrequency;
                    settings.actualFrequency = frame.centerFrequency;
                }
                if (RadioSettings.isDirectInput(settings.inputMode)) {
                    settings.centerFrequency = 0.0;
                    settings.actualFrequency = 0.0;
                }
                if (isFinite(frame.listeningFrequency)) {
                    settings.listeningFrequency = frame.listeningFrequency;
                }
                settings.sampleRate = frame.sampleRate > 0.0 ? frame.sampleRate : settings.sampleRate;
                settings.bandwidth = frame.bandwidth > 0.0 ? frame.bandwidth : settings.bandwidth;
                settings.modulationType = frame.modulationType;
                updateControlsFromSettings(false);
            }
            spectrumView.setSpectrum(
                    frame.frequencies,
                    frame.magnitudes,
                    frame.minFrequency,
                    frame.maxFrequency,
                    settings.listeningFrequency,
                    settings.bandwidth,
                    settings.modulationType);
            updateFineTuneControl();
        });
    }

    @Override
    public void onAudio(byte[] pcmData) {
        if (audioPlayer != null && audioPlaybackEnabled) {
            audioPlayer.playPcm(pcmData);
        }
    }

    @Override
    public void onControlMessage(JSONObject command) {
        String action = command.optString("action");
        if ("priorityRequest".equals(action)) {
            runOnMain(() -> showPriorityRequest(command));
        } else if ("priorityDenied".equals(action) || "controlRejected".equals(action)) {
            runOnMain(() -> appendLog(command.optString("reason", "Control request denied")));
        }
    }

    private void buildUi() {
        rootLayout = new LinearLayout(this);
        rootLayout.setBackgroundColor(0xff101418);
        commandPanel = new LinearLayout(this);
        commandPanel.setGravity(Gravity.CENTER);
        commandPanel.setBackgroundColor(0xff101418);
        contentPanel = new LinearLayout(this);
        contentPanel.setOrientation(LinearLayout.VERTICAL);
        contentPanel.setBackgroundColor(0xff101418);

        connectButton = button("Connect");
        startButton = button("Start");
        stopButton = button("Stop");
        settingsButton = button("Apply");
        controlsButton = button("Hide");
        connectButton.setOnClickListener(v -> toggleConnection());
        startButton.setOnClickListener(v -> {
            if (sendCommand("start", true)) {
                setControlsVisible(false);
            }
        });
        stopButton.setOnClickListener(v -> {
            if (audioPlayer != null) {
                audioPlayer.stop();
            }
            sendCommand("stop", true);
        });
        settingsButton.setOnClickListener(v -> sendCommand("settings", true));
        controlsButton.setOnClickListener(v -> setControlsVisible(!controlsVisible));

        statusText = label("Network disabled");
        statusText.setTextColor(0xffe7edf2);
        statusText.setPadding(dp(10), dp(5), dp(10), dp(3));
        levelPanel = buildLevelPanel();

        controlsScroll = new ScrollView(this);
        controlsScroll.setFillViewport(false);
        controlsScroll.setBackgroundColor(0xff182129);
        LinearLayout settingsPanel = new LinearLayout(this);
        settingsPanel.setOrientation(LinearLayout.VERTICAL);
        settingsPanel.setPadding(dp(8), dp(5), dp(8), dp(6));
        controlsScroll.addView(settingsPanel);

        LinearLayout connectionRow = row();
        hostEdit = edit("Server IP");
        hostEdit.setInputType(InputType.TYPE_CLASS_TEXT |
                InputType.TYPE_TEXT_VARIATION_URI |
                InputType.TYPE_TEXT_FLAG_NO_SUGGESTIONS);
        portEdit = edit("Port");
        portEdit.setInputType(InputType.TYPE_CLASS_NUMBER);
        requestControlButton = button("Request control");
        requestControlButton.setOnClickListener(v -> sendCommand("requestPriority", true));
        connectionRow.addView(fieldWithLabel("Server IP", hostEdit), new LinearLayout.LayoutParams(0, dp(62), 1.5f));
        connectionRow.addView(fieldWithLabel("Port", portEdit), new LinearLayout.LayoutParams(0, dp(62), 0.7f));
        connectionRow.addView(requestControlButton, new LinearLayout.LayoutParams(0, dp(48), 1.0f));

        LinearLayout usbRow = row();
        usbScanButton = button("USB scan");
        usbPermissionButton = button("USB permission/open");
        usbScanButton.setOnClickListener(v -> scanUsbDevices());
        usbPermissionButton.setOnClickListener(v -> requestUsbPermission());
        usbRow.addView(usbScanButton, new LinearLayout.LayoutParams(0, dp(44), 1.0f));
        usbRow.addView(usbPermissionButton, new LinearLayout.LayoutParams(0, dp(44), 1.0f));

        LinearLayout freqRow = row();
        centerEdit = edit("Center MHz");
        listenEdit = edit("Listen MHz");
        bandwidthEdit = edit("BW kHz");
        addAutoApplyTextWatcher(centerEdit);
        addAutoApplyTextWatcher(listenEdit);
        addAutoApplyTextWatcher(bandwidthEdit);
        freqRow.addView(fieldWithLabel("Center MHz", centerEdit), new LinearLayout.LayoutParams(0, dp(62), 1.0f));
        freqRow.addView(fieldWithLabel("Listen MHz", listenEdit), new LinearLayout.LayoutParams(0, dp(62), 1.0f));
        freqRow.addView(fieldWithLabel("BW kHz", bandwidthEdit), new LinearLayout.LayoutParams(0, dp(62), 0.8f));

        LinearLayout modeRow = row();
        sampleRateSpinner = spinner(sampleRateLabels());
        inputSpinner = spinner(new String[] {"RF", "HF1 + HF2", "HF1", "HF2", "HF1 - HF2 cancel lab"});
        modulationSpinner = spinner(modulationLabels());
        sampleRateSpinner.setOnItemSelectedListener(autoApplySpinnerListener(false));
        inputSpinner.setOnItemSelectedListener(inputModeSpinnerListener());
        modulationSpinner.setOnItemSelectedListener(autoApplySpinnerListener(true));
        modeRow.addView(fieldWithLabel("Sample", sampleRateSpinner), new LinearLayout.LayoutParams(0, dp(62), 1.0f));
        modeRow.addView(fieldWithLabel("Input", inputSpinner), new LinearLayout.LayoutParams(0, dp(62), 0.8f));
        modeRow.addView(fieldWithLabel("Mode", modulationSpinner), new LinearLayout.LayoutParams(0, dp(62), 0.8f));

        LinearLayout fftRow = row();
        fftSpinner = spinner(fftLengthLabels());
        fftSpinner.setOnItemSelectedListener(autoApplySpinnerListener(false));
        fullResolutionSpectrumCheck = new CheckBox(this);
        fullResolutionSpectrumCheck.setText("Full FFT frames");
        fullResolutionSpectrumCheck.setTextColor(0xffe7edf2);
        fullResolutionSpectrumCheck.setChecked(fullResolutionSpectrumFrames);
        fullResolutionSpectrumCheck.setOnCheckedChangeListener((buttonView, isChecked) -> {
            fullResolutionSpectrumFrames = isChecked;
            sendReliableSettingsNow();
        });
        fftRow.addView(fieldWithLabel("FFT", fftSpinner), new LinearLayout.LayoutParams(0, dp(62), 0.9f));
        fftRow.addView(fullResolutionSpectrumCheck, new LinearLayout.LayoutParams(0, dp(44), 1.1f));

        LinearLayout optionsRow = row();
        audioCheck = new CheckBox(this);
        audioCheck.setText("Audio");
        audioCheck.setTextColor(0xffe7edf2);
        audioCheck.setChecked(true);
        audioCheck.setOnCheckedChangeListener((buttonView, isChecked) -> {
            audioPlaybackEnabled = isChecked;
            if (!isChecked && audioPlayer != null) {
                audioPlayer.stop();
            }
            scheduleSettingsApply(FAST_APPLY_DELAY_MS);
        });
        suppressServerCheck = new CheckBox(this);
        suppressServerCheck.setText("Disable server local output");
        suppressServerCheck.setTextColor(0xffe7edf2);
        suppressServerCheck.setChecked(true);
        suppressServerCheck.setOnCheckedChangeListener((buttonView, isChecked) -> scheduleSettingsApply(FAST_APPLY_DELAY_MS));
        optionsRow.addView(audioCheck, new LinearLayout.LayoutParams(0, dp(44), 0.7f));
        optionsRow.addView(suppressServerCheck, new LinearLayout.LayoutParams(0, dp(44), 1.3f));

        LinearLayout markerRow = row();
        generalBandMarkersCheck = new CheckBox(this);
        generalBandMarkersCheck.setText("Bands");
        generalBandMarkersCheck.setTextColor(0xffe7edf2);
        generalBandMarkersCheck.setChecked(showGeneralBandMarkers);
        generalBandMarkersCheck.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (suppressUiCallbacks) {
                return;
            }
            showGeneralBandMarkers = isChecked;
            updateBandMarkerVisibility();
            savePrefs();
        });
        amateurBandMarkersCheck = new CheckBox(this);
        amateurBandMarkersCheck.setText("HAM bands");
        amateurBandMarkersCheck.setTextColor(0xffe7edf2);
        amateurBandMarkersCheck.setChecked(showAmateurBandMarkers);
        amateurBandMarkersCheck.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (suppressUiCallbacks) {
                return;
            }
            showAmateurBandMarkers = isChecked;
            updateBandMarkerVisibility();
            savePrefs();
        });
        markerRow.addView(generalBandMarkersCheck, new LinearLayout.LayoutParams(0, dp(44), 1.0f));
        markerRow.addView(amateurBandMarkersCheck, new LinearLayout.LayoutParams(0, dp(44), 1.0f));

        logText = label("");
        logText.setTextSize(12.0f);
        ScrollView logScroll = new ScrollView(this);
        logScroll.setBackgroundColor(0xff0b1014);
        logScroll.addView(logText);

        settingsPanel.addView(connectionRow);
        settingsPanel.addView(usbRow);
        settingsPanel.addView(freqRow);
        settingsPanel.addView(modeRow);
        settingsPanel.addView(fftRow);
        settingsPanel.addView(optionsRow);
        settingsPanel.addView(markerRow);
        settingsPanel.addView(logScroll, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(76)));

        spectrumView = new SpectrumView(this);
        spectrumView.setLevelRange(displayLevelMin, displayLevelMax);
        updateBandMarkerVisibility();
        spectrumView.setTuneRequestListener(frequencyHz -> {
            settings.listeningFrequency = RadioSettings.clampDirectListeningFrequency(
                    settings.inputMode,
                    settings.sampleRate,
                    frequencyHz);
            if (RadioSettings.isDirectInput(settings.inputMode)) {
                settings.centerFrequency = 0.0;
                settings.actualFrequency = 0.0;
            }
            setTextIfNotFocused(centerEdit, formatMhz(settings.centerFrequency), true);
            setTextIfNotFocused(listenEdit, formatMhz(settings.listeningFrequency), true);
            sendCommand("settings", true);
        });

        contentPanel.addView(levelPanel, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(52)));
        contentPanel.addView(controlsScroll, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(230)));
        contentPanel.addView(spectrumView, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f));

        setContentView(rootLayout);
        applyResponsiveLayout();
    }

    private void toggleConnection() {
        if (client.isActive()) {
            client.disconnect();
            if (audioPlayer != null) {
                audioPlayer.stop();
            }
            statusText.setText("Network disabled");
            appendLog("Disconnected");
            updateButtons();
            return;
        }
        savePrefs();
        String host = hostEdit.getText().toString().trim();
        int port = safeInt(portEdit.getText().toString(), DEFAULT_PORT);
        client.connect(host.isEmpty() ? "127.0.0.1" : host, port);
        updateButtons();
    }

    private void scanUsbDevices() {
        if (usbSandbox == null) {
            appendLog("[USB] sandbox is not ready");
            return;
        }
        appendUsbLog(usbSandbox.scanReport());
    }

    private void requestUsbPermission() {
        if (usbSandbox == null) {
            appendLog("[USB] sandbox is not ready");
            return;
        }
        appendUsbLog(usbSandbox.requestPermissionForBestDevice());
    }

    private boolean sendCommand(String action, boolean logExplicitly) {
        mainHandler.removeCallbacks(deferredSettingsApply);
        mainHandler.removeCallbacks(reliableSettingsRepeat);
        pendingReliableSettingsRepeats = 0;
        return sendControlNow(action, logExplicitly, true);
    }

    private boolean sendControlNow(String action, boolean logExplicitly, boolean guardLocalSettings) {
        collectSettingsFromUi();
        if (guardLocalSettings && ("settings".equals(action) || "start".equals(action))) {
            localSettingsGuardUntilMs = SystemClock.uptimeMillis() + REMOTE_ECHO_GUARD_MS;
        }
        boolean sent = client.sendControl(
                action,
                settings,
                suppressServerCheck.isChecked(),
                fullResolutionSpectrumFrames,
                null);
        if (sent && logExplicitly) {
            appendLog("Sent: " + action);
        }
        return sent;
    }

    private void sendReliableSettingsNow() {
        if (suppressUiCallbacks ||
                client == null ||
                !client.isConnected() ||
                !client.canControl()) {
            return;
        }
        mainHandler.removeCallbacks(deferredSettingsApply);
        mainHandler.removeCallbacks(reliableSettingsRepeat);
        pendingReliableSettingsRepeats = RELIABLE_SETTINGS_REPEAT_COUNT;
        sendControlNow("settings", false, true);
        mainHandler.postDelayed(reliableSettingsRepeat, RELIABLE_SETTINGS_REPEAT_MS);
    }

    private void showPriorityRequest(JSONObject command) {
        if (!client.canControl()) {
            return;
        }
        String requesterId = command.optString("requesterId");
        String requesterLabel = command.optString("requesterLabel", "unknown client");
        CheckBox blockCheck = new CheckBox(this);
        blockCheck.setText("Block further requests from this client");
        blockCheck.setPadding(dp(8), dp(4), dp(8), dp(4));
        new AlertDialog.Builder(this)
                .setTitle("Control request")
                .setMessage("Client " + requesterLabel + " requests receiver control.")
                .setView(blockCheck)
                .setPositiveButton("Allow", (dialog, which) ->
                        sendPriorityResponse(requesterId, true, blockCheck.isChecked()))
                .setNegativeButton("Deny", (dialog, which) ->
                        sendPriorityResponse(requesterId, false, blockCheck.isChecked()))
                .show();
    }

    private void sendPriorityResponse(String requesterId, boolean accepted, boolean blocked) {
        collectSettingsFromUi();
        try {
            JSONObject response = new JSONObject();
            response.put("requesterId", requesterId);
            response.put("accepted", accepted);
            response.put("blocked", blocked);
            client.sendControl("priorityResponse",
                    settings,
                    suppressServerCheck.isChecked(),
                    fullResolutionSpectrumFrames,
                    response);
        } catch (JSONException e) {
            appendLog("Priority response failed: " + e.getMessage());
        }
    }

    private void collectSettingsFromUi() {
        int sampleIndex = sampleRateSpinner.getSelectedItemPosition();
        if (sampleIndex >= 0 && sampleIndex < sampleRates.length) {
            settings.sampleRate = sampleRates[sampleIndex];
        }
        int fftIndex = fftSpinner.getSelectedItemPosition();
        if (fftIndex >= 0 && fftIndex < fftLengths.length) {
            settings.fftLength = fftLengths[fftIndex];
        }
        settings.inputMode = Math.max(0, inputSpinner.getSelectedItemPosition());
        double requestedListening =
                safeDouble(listenEdit.getText().toString(), settings.listeningFrequency / 1_000_000.0) *
                        1_000_000.0;
        if (RadioSettings.isDirectInput(settings.inputMode)) {
            settings.centerFrequency = 0.0;
            settings.actualFrequency = 0.0;
            settings.listeningFrequency = RadioSettings.clampDirectListeningFrequency(
                    settings.inputMode,
                    settings.sampleRate,
                    requestedListening);
        } else {
            settings.centerFrequency =
                    safeDouble(centerEdit.getText().toString(), settings.centerFrequency / 1_000_000.0) *
                            1_000_000.0;
            settings.actualFrequency = settings.centerFrequency;
            settings.listeningFrequency = requestedListening;
        }
        settings.bandwidth = safeDouble(bandwidthEdit.getText().toString(), settings.bandwidth / 1_000.0) * 1_000.0;
        int modulationIndex = modulationSpinner.getSelectedItemPosition();
        if (modulationIndex >= 0 && modulationIndex < modulationIds.length) {
            settings.modulationType = modulationIds[modulationIndex];
        }
        settings.audioEnabled = audioCheck.isChecked();
        audioPlaybackEnabled = settings.audioEnabled;
    }

    private void updateButtons() {
        boolean connected = client != null && client.isConnected();
        boolean active = client != null && client.isActive();
        boolean canControl = connected && client.canControl();
        connectButton.setText(active ? "Disconnect" : "Connect");
        startButton.setEnabled(canControl);
        stopButton.setEnabled(canControl);
        settingsButton.setEnabled(canControl);
        requestControlButton.setEnabled(connected && !client.canControl());
    }

    private void loadPrefs() {
        SharedPreferences prefs = getSharedPreferences(PREFS, MODE_PRIVATE);
        hostEdit.setText(prefs.getString("host", "127.0.0.1"));
        portEdit.setText(String.valueOf(prefs.getInt("port", DEFAULT_PORT)));
        settings.centerFrequency = Double.longBitsToDouble(prefs.getLong("center", Double.doubleToLongBits(settings.centerFrequency)));
        settings.listeningFrequency = Double.longBitsToDouble(prefs.getLong("listen", Double.doubleToLongBits(settings.listeningFrequency)));
        settings.sampleRate = Double.longBitsToDouble(prefs.getLong("sampleRate", Double.doubleToLongBits(settings.sampleRate)));
        settings.bandwidth = Double.longBitsToDouble(prefs.getLong("bandwidth", Double.doubleToLongBits(settings.bandwidth)));
        settings.inputMode = prefs.getInt("inputMode", settings.inputMode);
        settings.modulationType = prefs.getInt("modulation", settings.modulationType);
        settings.hfNoiseCancelDepth = Double.longBitsToDouble(
                prefs.getLong("hfNoiseCancelDepth", Double.doubleToLongBits(settings.hfNoiseCancelDepth)));
        settings.hfNoiseCancelRefGainDb = Double.longBitsToDouble(
                prefs.getLong("hfNoiseCancelRefGainDb", Double.doubleToLongBits(settings.hfNoiseCancelRefGainDb)));
        settings.hfNoiseCancelRefDelayNs = Double.longBitsToDouble(
                prefs.getLong("hfNoiseCancelRefDelayNs", Double.doubleToLongBits(settings.hfNoiseCancelRefDelayNs)));
        settings.hfNoiseCancelRefTiltDb = Double.longBitsToDouble(
                prefs.getLong("hfNoiseCancelRefTiltDb", Double.doubleToLongBits(settings.hfNoiseCancelRefTiltDb)));
        settings.hfNoiseCancelFreeze = prefs.getBoolean("hfNoiseCancelFreeze", settings.hfNoiseCancelFreeze);
        settings.audioEnabled = prefs.getBoolean("audio", settings.audioEnabled);
        settings.fftLength = prefs.getInt("fftLength", settings.fftLength);
        fullResolutionSpectrumFrames = prefs.getBoolean("fullResolutionSpectrumFrames", fullResolutionSpectrumFrames);
        fineTuneHoldMode = prefs.getBoolean("fineTuneHoldMode", fineTuneHoldMode);
        showGeneralBandMarkers = prefs.getBoolean("showGeneralBandMarkers", showGeneralBandMarkers);
        showAmateurBandMarkers = prefs.getBoolean("showAmateurBandMarkers", showAmateurBandMarkers);
        boolean suppressServer = prefs.getBoolean("suppressServer", true);
        displayLevelMin = prefs.getFloat("levelMin", displayLevelMin);
        displayLevelMax = prefs.getFloat("levelMax", displayLevelMax);
        if (displayLevelMax <= displayLevelMin + LEVEL_MIN_GAP_DB) {
            displayLevelMin = -130.0f;
            displayLevelMax = -50.0f;
        }

        updateControlsFromSettings(true);
        updateLevelControls();
        suppressUiCallbacks = true;
        audioCheck.setChecked(settings.audioEnabled);
        audioPlaybackEnabled = settings.audioEnabled;
        fullResolutionSpectrumCheck.setChecked(fullResolutionSpectrumFrames);
        generalBandMarkersCheck.setChecked(showGeneralBandMarkers);
        amateurBandMarkersCheck.setChecked(showAmateurBandMarkers);
        suppressServerCheck.setChecked(suppressServer);
        if (fineTuneDial != null) {
            fineTuneDial.setHoldOffsetMode(fineTuneHoldMode);
        }
        updateFineTuneModeButton();
        updateBandMarkerVisibility();
        suppressUiCallbacks = false;
    }

    private void savePrefs() {
        if (hostEdit == null) {
            return;
        }
        collectSettingsFromUi();
        getSharedPreferences(PREFS, MODE_PRIVATE)
                .edit()
                .putString("host", hostEdit.getText().toString().trim())
                .putInt("port", safeInt(portEdit.getText().toString(), DEFAULT_PORT))
                .putLong("center", Double.doubleToLongBits(settings.centerFrequency))
                .putLong("listen", Double.doubleToLongBits(settings.listeningFrequency))
                .putLong("sampleRate", Double.doubleToLongBits(settings.sampleRate))
                .putLong("bandwidth", Double.doubleToLongBits(settings.bandwidth))
                .putInt("fftLength", settings.fftLength)
                .putInt("inputMode", settings.inputMode)
                .putInt("modulation", settings.modulationType)
                .putLong("hfNoiseCancelDepth", Double.doubleToLongBits(settings.hfNoiseCancelDepth))
                .putLong("hfNoiseCancelRefGainDb", Double.doubleToLongBits(settings.hfNoiseCancelRefGainDb))
                .putLong("hfNoiseCancelRefDelayNs", Double.doubleToLongBits(settings.hfNoiseCancelRefDelayNs))
                .putLong("hfNoiseCancelRefTiltDb", Double.doubleToLongBits(settings.hfNoiseCancelRefTiltDb))
                .putBoolean("hfNoiseCancelFreeze", settings.hfNoiseCancelFreeze)
                .putBoolean("audio", settings.audioEnabled)
                .putBoolean("fullResolutionSpectrumFrames", fullResolutionSpectrumFrames)
                .putBoolean("fineTuneHoldMode", fineTuneHoldMode)
                .putBoolean("showGeneralBandMarkers", showGeneralBandMarkers)
                .putBoolean("showAmateurBandMarkers", showAmateurBandMarkers)
                .putBoolean("suppressServer", suppressServerCheck.isChecked())
                .putFloat("levelMin", displayLevelMin)
                .putFloat("levelMax", displayLevelMax)
                .apply();
    }

    private void updateControlsFromSettings(boolean force) {
        suppressUiCallbacks = true;
        setTextIfNotFocused(centerEdit, formatMhz(settings.centerFrequency), force);
        setTextIfNotFocused(listenEdit, formatMhz(settings.listeningFrequency), force);
        setTextIfNotFocused(bandwidthEdit, formatKhz(settings.bandwidth), force);
        sampleRateSpinner.setSelection(indexOfSampleRate(settings.sampleRate));
        fftSpinner.setSelection(indexOfFftLength(settings.fftLength));
        inputSpinner.setSelection(Math.max(0, Math.min(inputSpinner.getCount() - 1, settings.inputMode)));
        modulationSpinner.setSelection(indexOfModulation(settings.modulationType));
        boolean directInput = RadioSettings.isDirectInput(settings.inputMode);
        centerEdit.setEnabled(!directInput);
        centerEdit.setAlpha(directInput ? 0.55f : 1.0f);
        suppressUiCallbacks = false;
        updateFineTuneControl();
    }

    private void setTextIfNotFocused(EditText editText, String text, boolean force) {
        if (editText == null || (!force && editText.hasFocus())) {
            return;
        }
        if (!text.contentEquals(editText.getText())) {
            editText.setText(text);
        }
    }

    private void scheduleSettingsApply() {
        scheduleSettingsApply(AUTO_APPLY_DELAY_MS);
    }

    private void scheduleSettingsApply(int delayMs) {
        if (suppressUiCallbacks ||
                client == null ||
                !client.isConnected() ||
                !client.canControl()) {
            return;
        }
        collectSettingsFromUi();
        localSettingsGuardUntilMs = SystemClock.uptimeMillis() + REMOTE_ECHO_GUARD_MS;
        mainHandler.removeCallbacks(reliableSettingsRepeat);
        pendingReliableSettingsRepeats = 0;
        mainHandler.removeCallbacks(deferredSettingsApply);
        mainHandler.postDelayed(deferredSettingsApply, Math.max(0, delayMs));
    }

    private void setControlsVisible(boolean visible) {
        controlsVisible = visible;
        controlsScroll.setVisibility(visible ? View.VISIBLE : View.GONE);
        controlsButton.setText(visible ? "Hide" : "Controls");
    }

    private void applyResponsiveLayout() {
        if (rootLayout == null || commandPanel == null || contentPanel == null) {
            return;
        }
        boolean landscape = getResources().getConfiguration().orientation ==
                Configuration.ORIENTATION_LANDSCAPE;
        rootLayout.removeAllViews();
        rootLayout.setOrientation(landscape ? LinearLayout.HORIZONTAL : LinearLayout.VERTICAL);
        commandPanel.setOrientation(landscape ? LinearLayout.VERTICAL : LinearLayout.HORIZONTAL);
        commandPanel.setPadding(
                dp(landscape ? 3 : 4),
                dp(landscape ? 4 : 3),
                dp(landscape ? 3 : 4),
                dp(landscape ? 4 : 3));
        rebuildCommandPanel(landscape);

        ViewGroup.LayoutParams controlsParams = controlsScroll.getLayoutParams();
        if (controlsParams instanceof LinearLayout.LayoutParams) {
            ((LinearLayout.LayoutParams) controlsParams).height = dp(landscape ? 188 : 230);
            controlsScroll.setLayoutParams(controlsParams);
        }
        ViewGroup.LayoutParams levelParams = levelPanel.getLayoutParams();
        if (levelParams instanceof LinearLayout.LayoutParams) {
            ((LinearLayout.LayoutParams) levelParams).height = dp(landscape ? 44 : 52);
            levelPanel.setLayoutParams(levelParams);
        }

        if (landscape) {
            rootLayout.addView(commandPanel, new LinearLayout.LayoutParams(
                    dp(92), LinearLayout.LayoutParams.MATCH_PARENT));
            rootLayout.addView(contentPanel, new LinearLayout.LayoutParams(
                    0, LinearLayout.LayoutParams.MATCH_PARENT, 1.0f));
        } else {
            rootLayout.addView(commandPanel, new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, dp(48)));
            rootLayout.addView(contentPanel, new LinearLayout.LayoutParams(
                    LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f));
        }
    }

    private void rebuildCommandPanel(boolean landscape) {
        commandPanel.removeAllViews();
        Button[] buttons = new Button[] {
                connectButton, startButton, stopButton, settingsButton, controlsButton
        };
        for (Button button : buttons) {
            button.setTextSize(landscape ? 11.0f : 12.0f);
            LinearLayout.LayoutParams params = landscape
                    ? new LinearLayout.LayoutParams(
                            LinearLayout.LayoutParams.MATCH_PARENT, 0, 1.0f)
                    : weightButtonParams();
            params.setMargins(dp(landscape ? 0 : 1), dp(landscape ? 1 : 0),
                    dp(landscape ? 0 : 1), dp(landscape ? 1 : 0));
            commandPanel.addView(button, params);
        }
    }

    private LinearLayout buildLevelPanel() {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.HORIZONTAL);
        panel.setGravity(Gravity.CENTER_VERTICAL);
        panel.setPadding(dp(6), dp(2), dp(6), dp(2));
        panel.setBackgroundColor(0xff101418);

        levelMinLabel = label("");
        levelMinLabel.setTextSize(11.0f);
        levelMaxLabel = label("");
        levelMaxLabel.setTextSize(11.0f);
        fineTuneLabel = label("");
        fineTuneLabel.setTextSize(10.0f);
        fineTuneLabel.setGravity(Gravity.CENTER);
        fineTuneModeButton = new View(this);
        fineTuneModeButton.setOnClickListener(v -> {
            fineTuneHoldMode = !fineTuneHoldMode;
            if (fineTuneDial != null) {
                fineTuneDial.setHoldOffsetMode(fineTuneHoldMode);
            }
            updateFineTuneModeButton();
            savePrefs();
        });

        levelMinSeek = levelSeekBar();
        levelMaxSeek = levelSeekBar();
        levelMinSeek.setOnSeekBarChangeListener(levelChangeListener(true));
        levelMaxSeek.setOnSeekBarChangeListener(levelChangeListener(false));
        fineTuneDial = new FineTuneDialView(this);
        fineTuneDial.setListener(deltaHz -> applyFineTuneDelta(deltaHz));
        fineTuneDial.setModeListener(enabled -> {
            fineTuneHoldMode = enabled;
            updateFineTuneModeButton();
            savePrefs();
        });

        panel.addView(levelMinLabel, new LinearLayout.LayoutParams(dp(58), dp(38)));
        panel.addView(levelMinSeek, new LinearLayout.LayoutParams(0, dp(38), 1.0f));
        panel.addView(levelMaxLabel, new LinearLayout.LayoutParams(dp(58), dp(38)));
        panel.addView(levelMaxSeek, new LinearLayout.LayoutParams(0, dp(38), 1.0f));
        panel.addView(fineTuneLabel, new LinearLayout.LayoutParams(dp(72), dp(42)));
        LinearLayout.LayoutParams modeParams = new LinearLayout.LayoutParams(dp(18), dp(18));
        modeParams.setMargins(0, 0, dp(4), 0);
        panel.addView(fineTuneModeButton, modeParams);
        panel.addView(fineTuneDial, new LinearLayout.LayoutParams(0, dp(46), 1.1f));
        updateFineTuneControl();
        updateFineTuneModeButton();
        return panel;
    }

    private SeekBar levelSeekBar() {
        SeekBar seekBar = new SeekBar(this);
        seekBar.setMax(LEVEL_DB_MAX - LEVEL_DB_MIN);
        return seekBar;
    }

    private SeekBar.OnSeekBarChangeListener levelChangeListener(boolean minSlider) {
        return new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (suppressLevelCallbacks || !fromUser) {
                    return;
                }
                handleLevelChange(minSlider);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
                savePrefs();
            }
        };
    }

    private void handleLevelChange(boolean minSlider) {
        int minDb = progressToLevelDb(levelMinSeek.getProgress());
        int maxDb = progressToLevelDb(levelMaxSeek.getProgress());
        if (minDb > maxDb - LEVEL_MIN_GAP_DB) {
            if (minSlider) {
                minDb = maxDb - LEVEL_MIN_GAP_DB;
            } else {
                maxDb = minDb + LEVEL_MIN_GAP_DB;
            }
        }
        minDb = Math.max(LEVEL_DB_MIN, Math.min(LEVEL_DB_MAX - LEVEL_MIN_GAP_DB, minDb));
        maxDb = Math.max(LEVEL_DB_MIN + LEVEL_MIN_GAP_DB, Math.min(LEVEL_DB_MAX, maxDb));
        displayLevelMin = minDb;
        displayLevelMax = maxDb;
        updateLevelControls();
    }

    private void updateLevelControls() {
        if (levelMinSeek == null || levelMaxSeek == null) {
            return;
        }
        suppressLevelCallbacks = true;
        int minDb = Math.round(displayLevelMin);
        int maxDb = Math.round(displayLevelMax);
        minDb = Math.max(LEVEL_DB_MIN, Math.min(LEVEL_DB_MAX - LEVEL_MIN_GAP_DB, minDb));
        maxDb = Math.max(minDb + LEVEL_MIN_GAP_DB, Math.min(LEVEL_DB_MAX, maxDb));
        displayLevelMin = minDb;
        displayLevelMax = maxDb;
        levelMinSeek.setProgress(levelDbToProgress(minDb));
        levelMaxSeek.setProgress(levelDbToProgress(maxDb));
        levelMinLabel.setText(String.format(Locale.US, "Min\n%d", minDb));
        levelMaxLabel.setText(String.format(Locale.US, "Max\n%d", maxDb));
        if (spectrumView != null) {
            spectrumView.setLevelRange(displayLevelMin, displayLevelMax);
        }
        suppressLevelCallbacks = false;
    }

    private void updateFineTuneControl() {
        if (fineTuneDial == null || fineTuneLabel == null) {
            return;
        }
        double spanHz = spectrumView != null ? spectrumView.visibleSpanHz() : settings.sampleRate;
        fineTuneDial.setVisibleSpanHz(spanHz);
        fineTuneDial.setHoldOffsetMode(fineTuneHoldMode);
        fineTuneLabel.setText(String.format(Locale.US,
                "Fine +/- %s",
                formatFineTuneRange(fineTuneDial.getRangeHz())));
        updateFineTuneModeButton();
    }

    private void updateBandMarkerVisibility() {
        if (spectrumView != null) {
            spectrumView.setBandMarkersEnabled(showGeneralBandMarkers, showAmateurBandMarkers);
        }
    }

    private void updateFineTuneModeButton() {
        if (fineTuneModeButton == null) {
            return;
        }
        fineTuneModeButton.setBackgroundColor(fineTuneHoldMode ? 0xffd65050 : 0xff3dbb68);
        fineTuneModeButton.setContentDescription(fineTuneHoldMode
                ? "Held fine tuning offset"
                : "Temporary fine tuning");
    }

    private void applyFineTuneDelta(double deltaHz) {
        if (Double.isNaN(deltaHz) || Double.isInfinite(deltaHz) || Math.abs(deltaHz) < 0.01) {
            return;
        }
        collectSettingsFromUi();
        settings.listeningFrequency += deltaHz;
        if (RadioSettings.isDirectInput(settings.inputMode)) {
            settings.centerFrequency = 0.0;
            settings.actualFrequency = 0.0;
            settings.listeningFrequency = RadioSettings.clampDirectListeningFrequency(
                    settings.inputMode,
                    settings.sampleRate,
                    settings.listeningFrequency);
        } else {
            settings.listeningFrequency = Math.max(25_000_000.0,
                    Math.min(6_000_000_000.0, settings.listeningFrequency));
            double halfRate = Math.max(1.0, settings.sampleRate * 0.5);
            if (settings.listeningFrequency < settings.centerFrequency - halfRate) {
                settings.centerFrequency = Math.max(50_000_000.0,
                        settings.listeningFrequency + halfRate);
                settings.actualFrequency = settings.centerFrequency;
            } else if (settings.listeningFrequency > settings.centerFrequency + halfRate) {
                settings.centerFrequency = Math.max(50_000_000.0,
                        settings.listeningFrequency - halfRate);
                settings.actualFrequency = settings.centerFrequency;
            }
        }
        setTextIfNotFocused(centerEdit, formatMhz(settings.centerFrequency), true);
        setTextIfNotFocused(listenEdit, formatMhz(settings.listeningFrequency), true);
        scheduleSettingsApply(FAST_APPLY_DELAY_MS);
    }

    private String formatFineTuneRange(double hz) {
        if (hz >= 1_000_000.0) {
            return String.format(Locale.US, "%.1fM", hz / 1_000_000.0);
        }
        if (hz >= 1_000.0) {
            return String.format(Locale.US, "%.0fk", hz / 1_000.0);
        }
        return String.format(Locale.US, "%.0f", hz);
    }

    private int levelDbToProgress(int db) {
        return Math.max(0, Math.min(LEVEL_DB_MAX - LEVEL_DB_MIN, db - LEVEL_DB_MIN));
    }

    private int progressToLevelDb(int progress) {
        return LEVEL_DB_MIN + Math.max(0, Math.min(LEVEL_DB_MAX - LEVEL_DB_MIN, progress));
    }

    private void addAutoApplyTextWatcher(EditText editText) {
        editText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {
            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                scheduleSettingsApply();
            }

            @Override
            public void afterTextChanged(Editable s) {
            }
        });
    }

    private AdapterView.OnItemSelectedListener autoApplySpinnerListener(boolean modePreset) {
        return new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (suppressUiCallbacks) {
                    return;
                }
                if (modePreset && position >= 0 && position < modulationIds.length) {
                    settings.modulationType = modulationIds[position];
                    bandwidthEdit.setText(formatKhz(RadioSettings.defaultBandwidthForModulation(settings.modulationType)));
                }
                sendReliableSettingsNow();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        };
    }

    private AdapterView.OnItemSelectedListener inputModeSpinnerListener() {
        return new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (suppressUiCallbacks) {
                    return;
                }
                int nextMode = Math.max(RadioSettings.INPUT_RF,
                        Math.min(RadioSettings.INPUT_HF_NOISE_CANCEL, position));
                if (settings.inputMode != nextMode) {
                    settings.inputMode = nextMode;
                    int sampleIndex = sampleRateSpinner.getSelectedItemPosition();
                    if (sampleIndex >= 0 && sampleIndex < sampleRates.length) {
                        settings.sampleRate = sampleRates[sampleIndex];
                    }
                    if (RadioSettings.isDirectInput(nextMode)) {
                        settings.centerFrequency = 0.0;
                        settings.actualFrequency = 0.0;
                        settings.listeningFrequency =
                                nextMode == RadioSettings.INPUT_HF_COMBINED ? 0.0 : 1_250_000.0;
                    } else {
                        settings.centerFrequency = 100_000_000.0;
                        settings.actualFrequency = 100_000_000.0;
                        settings.listeningFrequency = 100_000_000.0;
                    }
                    updateControlsFromSettings(true);
                }
                sendReliableSettingsNow();
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        };
    }

    private LinearLayout row() {
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.HORIZONTAL);
        layout.setGravity(Gravity.CENTER_VERTICAL);
        layout.setPadding(dp(4), dp(3), dp(4), dp(3));
        return layout;
    }

    private EditText edit(String hint) {
        EditText view = new EditText(this);
        view.setSingleLine(true);
        view.setHint(hint);
        view.setTextColor(0xff101418);
        view.setHintTextColor(0xff60717d);
        view.setTextSize(14.0f);
        view.setBackgroundColor(0xffffffff);
        view.setPadding(dp(8), 0, dp(8), 0);
        view.setInputType(InputType.TYPE_CLASS_NUMBER |
                InputType.TYPE_NUMBER_FLAG_DECIMAL |
                InputType.TYPE_NUMBER_FLAG_SIGNED);
        return view;
    }

    private Button button(String text) {
        Button view = new Button(this);
        view.setText(text);
        view.setAllCaps(false);
        view.setTextSize(12.0f);
        return view;
    }

    private TextView label(String text) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextColor(0xffdbe6ec);
        view.setTextSize(14.0f);
        return view;
    }

    private Spinner spinner(String[] values) {
        Spinner view = new Spinner(this);
        ArrayAdapter<String> adapter = new ArrayAdapter<>(
                this,
                android.R.layout.simple_spinner_item,
                values);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        view.setAdapter(adapter);
        view.setBackgroundColor(0xffffffff);
        return view;
    }

    private View fieldWithLabel(String label, View field) {
        LinearLayout layout = new LinearLayout(this);
        layout.setOrientation(LinearLayout.VERTICAL);
        layout.setPadding(dp(3), 0, dp(3), 0);
        TextView text = label(label);
        text.setTextSize(11.0f);
        layout.addView(text, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(18)));
        layout.addView(field, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT, dp(40)));
        return layout;
    }

    private LinearLayout.LayoutParams weightButtonParams() {
        return new LinearLayout.LayoutParams(0, dp(42), 1.0f);
    }

    private String[] sampleRateLabels() {
        String[] labels = new String[sampleRates.length];
        for (int i = 0; i < sampleRates.length; ++i) {
            labels[i] = formatMhz(sampleRates[i]);
        }
        return labels;
    }

    private String[] fftLengthLabels() {
        String[] labels = new String[fftLengths.length];
        for (int i = 0; i < fftLengths.length; ++i) {
            labels[i] = String.format(Locale.US, "%d", fftLengths[i]);
        }
        return labels;
    }

    private String[] modulationLabels() {
        String[] labels = new String[modulationIds.length];
        for (int i = 0; i < modulationIds.length; ++i) {
            labels[i] = RadioSettings.modulationLabel(modulationIds[i]);
        }
        return labels;
    }

    private int indexOfSampleRate(double value) {
        int best = 0;
        double bestDelta = Double.MAX_VALUE;
        for (int i = 0; i < sampleRates.length; ++i) {
            double delta = Math.abs(sampleRates[i] - value);
            if (delta < bestDelta) {
                bestDelta = delta;
                best = i;
            }
        }
        return best;
    }

    private int indexOfFftLength(int value) {
        int best = 0;
        int bestDelta = Integer.MAX_VALUE;
        for (int i = 0; i < fftLengths.length; ++i) {
            int delta = Math.abs(fftLengths[i] - value);
            if (delta < bestDelta) {
                bestDelta = delta;
                best = i;
            }
        }
        return best;
    }

    private int indexOfModulation(int value) {
        for (int i = 0; i < modulationIds.length; ++i) {
            if (modulationIds[i] == value) {
                return i;
            }
        }
        return 0;
    }

    private void appendLog(String text) {
        String previous = logText.getText().toString();
        String next = previous.isEmpty() ? text : previous + "\n" + text;
        int maxLength = 7000;
        if (next.length() > maxLength) {
            next = next.substring(next.length() - maxLength);
        }
        logText.setText(next);
    }

    private void appendUsbLog(String text) {
        runOnMain(() -> {
            if (statusText != null) {
                statusText.setText("USB sandbox");
            }
            appendLog(text);
        });
    }

    private void runOnMain(Runnable runnable) {
        if (Looper.myLooper() == Looper.getMainLooper()) {
            runnable.run();
        } else {
            mainHandler.post(runnable);
        }
    }

    private int dp(int value) {
        return (int) (value * getResources().getDisplayMetrics().density + 0.5f);
    }

    private static int safeInt(String text, int fallback) {
        try {
            return Integer.parseInt(text.trim());
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    private static double safeDouble(String text, double fallback) {
        try {
            return Double.parseDouble(text.trim().replace(',', '.'));
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static String formatMhz(double hz) {
        return String.format(Locale.US, "%.6f", hz / 1_000_000.0);
    }

    private static String formatKhz(double hz) {
        double khz = hz / 1_000.0;
        if (Math.abs(khz - Math.rint(khz)) < 0.01) {
            return String.format(Locale.US, "%.0f", khz);
        }
        return String.format(Locale.US, "%.1f", khz);
    }
}
