package com.fobosapp.networkclient;

import org.json.JSONException;
import org.json.JSONObject;

final class RadioSettings {
    static final int MOD_AM = 0;
    static final int MOD_NFM = 1;
    static final int MOD_SAM = 2;
    static final int MOD_USB = 3;
    static final int MOD_LSB = 4;
    static final int MOD_DSB = 5;
    static final int MOD_CW = 6;
    static final int MOD_WFM = 7;
    static final int MOD_DMR = 17;

    static final int PROCESSING_SERVER_SIDE = 0;
    static final int INPUT_RF = 0;
    static final int INPUT_HF_COMBINED = 1;
    static final int INPUT_HF1 = 2;
    static final int INPUT_HF2 = 3;
    private static final double DIRECT_MIN_FREQUENCY = 1.0;

    int deviceIndex = 0;
    int clockSource = 0;
    int inputMode = 0;
    double centerFrequency = 100_000_000.0;
    double actualFrequency = 100_000_000.0;
    double listeningFrequency = 100_000_000.0;
    double sampleRate = 8_000_000.0;
    double bandwidth = 10_000.0;
    int modulationType = MOD_AM;
    int fftLength = 32_768;
    int lnaGain = 1;
    int vgaGain = 3;
    double audioLowPassHz = 0.0;
    double audioHighPassHz = 0.0;
    boolean audioEnabled = true;
    boolean syncEnabled = false;
    int gpoValue = 0;
    double scalePercent = 100.0;

    JSONObject toJson() throws JSONException {
        JSONObject json = new JSONObject();
        json.put("deviceIndex", deviceIndex);
        json.put("clockSource", clockSource);
        json.put("inputMode", inputMode);
        json.put("centerFrequency", centerFrequency);
        json.put("actualFrequency", actualFrequency);
        json.put("listeningFrequency", listeningFrequency);
        json.put("sampleRate", sampleRate);
        json.put("bandwidth", bandwidth);
        json.put("modulationType", modulationType);
        json.put("fftLength", fftLength);
        json.put("lnaGain", lnaGain);
        json.put("vgaGain", vgaGain);
        json.put("audioLowPassHz", audioLowPassHz);
        json.put("audioHighPassHz", audioHighPassHz);
        json.put("audioEnabled", audioEnabled);
        json.put("syncEnabled", syncEnabled);
        json.put("gpoValue", gpoValue);
        json.put("scalePercent", scalePercent);
        return json;
    }

    void updateFromJson(JSONObject json) {
        if (json == null) {
            return;
        }
        deviceIndex = json.optInt("deviceIndex", deviceIndex);
        clockSource = json.optInt("clockSource", clockSource);
        inputMode = json.optInt("inputMode", inputMode);
        centerFrequency = json.optDouble("centerFrequency", centerFrequency);
        actualFrequency = json.optDouble("actualFrequency", actualFrequency);
        listeningFrequency = json.optDouble("listeningFrequency", listeningFrequency);
        sampleRate = json.optDouble("sampleRate", sampleRate);
        bandwidth = json.optDouble("bandwidth", bandwidth);
        modulationType = json.optInt("modulationType", modulationType);
        fftLength = json.optInt("fftLength", fftLength);
        lnaGain = json.optInt("lnaGain", lnaGain);
        vgaGain = json.optInt("vgaGain", vgaGain);
        audioLowPassHz = json.optDouble("audioLowPassHz", audioLowPassHz);
        audioHighPassHz = json.optDouble("audioHighPassHz", audioHighPassHz);
        audioEnabled = json.optBoolean("audioEnabled", audioEnabled);
        syncEnabled = json.optBoolean("syncEnabled", syncEnabled);
        gpoValue = json.optInt("gpoValue", gpoValue);
        scalePercent = json.optDouble("scalePercent", scalePercent);
    }

    static double defaultBandwidthForModulation(int modulationType) {
        switch (modulationType) {
            case MOD_NFM:
            case MOD_DMR:
                return 12_500.0;
            case MOD_SAM:
            case MOD_DSB:
                return 6_000.0;
            case MOD_USB:
            case MOD_LSB:
                return 2_700.0;
            case MOD_CW:
                return 500.0;
            case MOD_WFM:
                return 200_000.0;
            case MOD_AM:
            default:
                return 10_000.0;
        }
    }

    static String modulationLabel(int modulationType) {
        switch (modulationType) {
            case MOD_NFM:
                return "NFM";
            case MOD_SAM:
                return "SAM";
            case MOD_USB:
                return "USB";
            case MOD_LSB:
                return "LSB";
            case MOD_DSB:
                return "DSB";
            case MOD_CW:
                return "CW";
            case MOD_WFM:
                return "WFM";
            case MOD_DMR:
                return "DMR";
            case MOD_AM:
            default:
                return "AM";
        }
    }

    static boolean isDirectInput(int inputMode) {
        return inputMode != INPUT_RF;
    }

    static double directMaxFrequency(double sampleRate) {
        return Math.max(DIRECT_MIN_FREQUENCY, sampleRate / 2.0 - 1.0);
    }

    static double directMinFrequencyForMode(int inputMode, double sampleRate) {
        return inputMode == INPUT_HF_COMBINED
                ? -directMaxFrequency(sampleRate)
                : DIRECT_MIN_FREQUENCY;
    }

    static double clampDirectListeningFrequency(int inputMode, double sampleRate, double frequencyHz) {
        if (!isDirectInput(inputMode)) {
            return frequencyHz;
        }
        double min = directMinFrequencyForMode(inputMode, sampleRate);
        double max = directMaxFrequency(sampleRate);
        if (Double.isNaN(frequencyHz) || Double.isInfinite(frequencyHz)) {
            return inputMode == INPUT_HF_COMBINED ? 0.0 : 1_250_000.0;
        }
        return Math.max(min, Math.min(max, frequencyHz));
    }
}
