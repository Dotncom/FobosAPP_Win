# FobosAPP Roadmap

This file is the live roadmap. Historical release details belong in
`CHANGELOG.md`; stability handoff notes belong in `docs/release_stability_handoff.md`.

## Current State

Ready or usable enough to release as the current stable baseline:

1. Fobos SDR Standard and Fobos SDR Agile receive paths.
2. Native RTL-SDR receive as a secondary/test backend.
3. Serial GNSS modules through NMEA/UBX, including NEO-M8N-style modules.
4. QTH map, Maidenhead overlay, marker workflow, online/offline map providers,
   satellite table, sky view, and GNSS status UI.
5. Agile scan, standard retune scan, listening scan, stitched/floating scan
   visuals, measurement overlays, and editable presets.
6. Low-copy waterfall row generation with the legacy palette/contrast behavior
   restored.
7. Optional/beta receiver paths: bladeRF RX and SoapySDR. These are not
   release-critical until tested on real hardware.

Experimental and not release-grade:

1. DMR voice. `FobosAPP + mbelib` is the current practical baseline, but voice
   quality and metadata stability are still lab-level.
2. DSD-neo and GopherTrunk bridges. Useful for comparison, not finished
   backends.
3. SDR-only GNSS acquisition. Synthetic/replay tools exist, but repeatable real
   GPS L1 C/A lock is not proven.
4. FPV/video decoding. Detector and preset scaffolding exist, but real decoding
   is paused until useful captures/signals are available.

Known compatibility risks:

1. Some Fobos Agile units or host/firmware combinations may start with shifted
   spectrum/IQ placement. The current mitigation is applying idle hardware
   settings before reading and a minimal startup frequency kick before async IQ
   starts.
2. Startup, stop, and close paths can still pause on some machines. Keep this
   separate from live-retune work.
3. SDR GNSS and DMR are both synchronization-sensitive; do not treat live RF
   tests as proof until the same path is repeatable offline.

## Release Checklist

Before every public upload:

1. Build Windows x64 and smoke-test from the staged release folder.
2. Package Raspberry/Linux source and verify helper scripts are current.
3. Check Android source sync. Upload Android artifacts only when rebuilt and
   tested.
4. Verify Windows and Raspberry archives contain no:
   `FobosAPP.ini`, diagnostic logs, real IQ/audio recordings, GNSS raw logs,
   GNSS reports, screenshots, private tokens, API keys, local tool folders, or
   temporary downloads.
5. Keep `release/bin/FobosAPP.ini` local only. It may contain QTH coordinates,
   map API keys, DMR keys, ports, and personal presets.
6. Verify `translations.json` parses.
7. Verify README and release notes describe:
   Fobos Agile startup-shift risk, optional DMR backends, RTL-SDR runtime files,
   GNSS serial support, and settings preservation during updates.

Current release recommendation:

1. Publish the current cleanup as `4.5.1`, not by overwriting `4.5.0`.
2. Describe it as a stability/privacy/visual-performance patch.
3. Do not publish Android unless rebuilt and smoke-tested.

## Privacy Rules

1. Release archives must not include user settings, logs, recordings, or local
   generated reports.
2. Automatic GNSS IQ sidecar JSON and acquisition reports should not store exact
   latitude/longitude. Use QTH locator plus a privacy note.
3. Diagnostic logs should avoid exact coordinates unless the user explicitly
   exports or shares them.
4. Map provider API keys are user settings, not project defaults.
5. DMR privacy keys are user settings, not project defaults.
6. Keep `token.txt`, root screenshots, local text notes, and generated captures
   ignored by git.

## Near-Term Work

The order below is the useful path after the current release checkpoint.

1. Stability cleanup:
   - reduce remaining duplicate settings saves;
   - keep normal diagnostic logs small;
   - inspect startup/stop/close pauses around `closeEvent`,
     `stopFobosProcessing`, `DataProcessor::requestStop`, async cancel, and IQ
     queue drain;
   - avoid device teardown waits on the UI thread where possible.
2. Main-window unloading:
   - continue moving cold feature glue out of `main.cpp`;
   - keep start/stop, tuning, Fobos session, and spectrum update changes small
     unless directly needed;
   - next candidates: remaining GNSS UI glue, DMR UI glue, release/settings
     helpers, and preset/band-plan glue.
3. IQ pipeline audit:
   - keep `docs/architecture_iq_path.md` as the IQ contract document;
   - add or preserve diagnostics for callback block size, measured sample rate,
     callback intervals, queue depth, clipping, DC, non-finite samples, and
     discontinuities;
   - do not merge `DataProcessor` and `IqBuffer` unless profiling proves that
     boundary is the bottleneck.
4. Waterfall/spectrum performance:
   - keep the current low-copy waterfall path as baseline;
   - keep rows-per-frame and update interval as user controls;
   - optimize measurement overlay cost only if it visibly slows the waterfall;
   - GPU waterfall remains a later opt-in prototype, not a release blocker.

## GNSS Roadmap

Implemented:

1. Serial NMEA/UBX receiver workflow with port selection, live fix status,
   time-zone display, satellite table, sky view, per-system/per-satellite
   filters, and map/QTH updates.
2. UBX controls for NAV-PVT/NAV-SAT/NAV-DOP, CFG-GNSS polling/apply, CFG-CFG
   save, and raw UBX/NMEA capture.
3. SDR GNSS tools: presets, IQ monitor, raw IQ snapshot, GPS L1 C/A acquisition
   prototype, synthetic tests, replay, diagnostic plots, and report artifacts.

Next:

1. Test with better antenna/RF conditions when hardware is available.
2. Compare saved Fobos IQ against GNSS-SDR or another mature reference receiver.
3. Improve acquisition only after reference comparison:
   bounded Doppler search, false-alarm thresholding, second-stage Doppler
   refinement, and non-coherent accumulation.
4. Add assisted-GNSS context later: visible PRN prediction, almanac/ephemeris
   files, and internet-free operation where practical.

Do not block stable releases on SDR-only GNSS lock.

## DMR Roadmap

Current status:

1. DMR is experimental.
2. Metadata can appear, but CC/TS/SRC/TG and encrypted-state confidence still
   need deterministic CACH/LC/SlotType handling.
3. Voice can be partially recognizable, but quality is not stable enough.
4. ARC4/AES UI/key management exists for authorized lab tests, but decrypted
   voice is not confirmed.

Next:

1. Stop broad live trial-and-error changes until offline replay can reproduce
   each stage.
2. Build a repeatable DMR offline harness around known real recordings.
3. Make CACH, SlotType, LC, EMB, cadence voting, and privacy indicators
   deterministic before more AMBE tuning.
4. Use DSD-neo and GopherTrunk as comparison/verifier paths:
   compare burst timing, metadata, privacy state, and audio output stage by
   stage.
5. Public app non-goals:
   no unknown-key recovery, no bypass features, no hidden decryption pipeline,
   and no claim that encrypted DMR can be monitored without lawful user-provided
   keys/configuration.

## Receiver Backends

Primary:

1. Fobos Standard and Agile stay the optimized first-class path.
2. Optional backends must not add latency or fragility to the Fobos path.

Implemented or started:

1. Native RTL-SDR runtime loading on desktop.
2. RTL-TCP path for lightweight remote RTL tests.
3. Optional SoapySDR runtime path, mostly unverified.
4. Native bladeRF RX beta, unverified without real hardware.

Next:

1. Keep RTL-SDR good enough for testing and community fallback.
2. Keep SoapySDR/bladeRF clearly marked beta until tested.
3. Avoid making generic backend abstraction slower than the Fobos-specific path.
4. Future multi-receiver mode:
   two Fobos receivers, local plus remote Fobos, or Fobos plus RTL/Soapy;
   decide later between split panes, overlay layers, or detachable receiver
   windows.

## Network And Android

Current rule: keep Android lighter than desktop, and keep heavy workflows on
desktop or desktop-to-desktop network mode.

Implemented enough:

1. Desktop network mode mirrors core receiver/spectrum state.
2. Scan visual frames can carry scan segments for stitched views.
3. Android can display spectrum/waterfall, control receiver basics, use network
   mode, and has USB experiments.

Deferred:

1. Full DMR network state.
2. Full GNSS network state.
3. Automatic preset/band-plan sync. Prefer explicit import/export unless a real
   remote workflow needs more.

## FPV And Video

Status:

1. FPV presets, scans, hunter controls, and detector scaffolding exist.
2. Real analog/digital video decoding is paused.

Next:

1. Resume only when useful saved captures or live signals are available.
2. Improve detector output metrics before deep demodulation work.

## Future Transceiver Mode

Long-term idea: use FobosAPP as the receive/control side of a homebuilt
transceiver system. Fobos itself is RX-only; TX must be a separate controlled
subsystem.

Implementation order:

1. Define `TransmitterBackend` separately from `ReceiverBackend`.
2. Add a simulator-only TX backend that cannot transmit.
3. Add UI/state plumbing for simulator PTT, sequencing, and fault display.
4. Prototype Raspberry-side GPIO/service control with no RF connected.
5. Add hardware interlocks, watchdog, timeout, RX mute/protect sequencing,
   relay timing, PA enable, SWR/current/temp fault handling, and emergency
   inhibit.
6. Only after bench tests with dummy load: add real external TX module control.
7. Keep real TX controls disabled unless explicitly configured.
