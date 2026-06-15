package com.fobosapp.networkclient;

final class NativeUsbBridge {
    private static final boolean AVAILABLE;

    static {
        boolean loaded = false;
        try {
            System.loadLibrary("fobos_native");
            loaded = true;
        } catch (UnsatisfiedLinkError ignored) {
        }
        AVAILABLE = loaded;
    }

    private NativeUsbBridge() {
    }

    static boolean isAvailable() {
        return AVAILABLE;
    }

    static String bulkBenchmark(int fd,
                                int endpointAddress,
                                int transferBytes,
                                int durationMs,
                                int queueDepth) {
        if (!AVAILABLE) {
            return "native USB benchmark unavailable: fobos_native is not loaded";
        }
        return nativeBulkBenchmark(fd, endpointAddress, transferBytes, durationMs, queueDepth);
    }

    static byte[] audioFromRawFobosIq(byte[] buffer,
                                      int bytesRead,
                                      double centerFrequency,
                                      double listeningFrequency,
                                      double sampleRate,
                                      double timingSampleRate,
                                      int inputMode,
                                      int modulationType,
                                      double bandwidth,
                                      boolean activeAgileApi,
                                      int[] stats) {
        if (!AVAILABLE) {
            return new byte[0];
        }
        return nativeAudioFromRawFobosIq(buffer,
                bytesRead,
                centerFrequency,
                listeningFrequency,
                sampleRate,
                timingSampleRate,
                inputMode,
                modulationType,
                bandwidth,
                activeAgileApi,
                stats);
    }

    static void resetAudioDemodState() {
        if (AVAILABLE) {
            nativeResetAudioDemodState();
        }
    }

    private static native String nativeBulkBenchmark(int fd,
                                                     int endpointAddress,
                                                     int transferBytes,
                                                     int durationMs,
                                                     int queueDepth);

    private static native byte[] nativeAudioFromRawFobosIq(byte[] buffer,
                                                          int bytesRead,
                                                          double centerFrequency,
                                                          double listeningFrequency,
                                                          double sampleRate,
                                                          double timingSampleRate,
                                                          int inputMode,
                                                          int modulationType,
                                                          double bandwidth,
                                                          boolean activeAgileApi,
                                                          int[] stats);

    private static native void nativeResetAudioDemodState();
}
