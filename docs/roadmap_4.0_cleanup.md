# FobosAPP 4.0 cleanup and receiver roadmap

## Before publishing 4.0

1. Stabilize the DMR path enough for a beta-quality 4.0 label:
   - reliable burst/cadence lock on known Motorola test IQ;
   - stable Color Code, timeslot, source ID, and talkgroup output;
   - voice path that is repeatable across start/stop cycles, even if quality is still marked experimental.
   - Status for 4.0: deferred. Ship current DMR as experimental/lab quality.
2. Keep release packages clean:
   - do not ship `FobosAPP.ini`, diagnostic logs, recordings, screenshots, backups, or lab replay tools in the normal Windows runtime zip;
   - package Raspberry/Linux source from an allowlist or explicit excludes, not from a dirty working directory by accident;
   - keep reference repositories and reverse-engineering traces outside release artifacts.
   - Status for 4.0: Windows package script verified against logs, recordings, and lab replay tools.
3. Keep local defaults recoverable:
   - band plans and presets must be created from code defaults or a small checked-in default config;
   - if a user config contains an empty/corrupt marker array, the app should fall back to defaults.
   - Status for 4.0: code defaults are active; `config/fobosapp_defaults_4.0.json` records the release snapshot.

## Third-party code policy

Keep for now:
   - patched `libfobos` and `libfobos_sdr` sources needed for Linux/Raspberry builds;
   - `mbelib-neo` only as a temporary AMBE backend while DMR voice is still experimental.

Do not vendor into releases:
   - external reference repositories used during research;
   - local trace, decompilation, and reverse-engineering work folders;
   - downloaded SDKs, firmware archives, build products, or private tokens.

Longer term:
   - make DMR voice backend pluggable: internal test backend, external process backend, and optional AMBE library backend;
   - keep license notes explicit in `THIRD_PARTY_LICENSES.txt`;
   - only consider writing a custom AMBE decoder after the DMR sync/framing path is proven correct.
   - Status for 4.0: optional GPL DMR voice backend DLLs and release license bundle are included; external process backend remains next work.

## Other receiver support

First split the hardware layer behind a `ReceiverBackend` interface:
   - capabilities: sample rates, frequency range, gains, inputs, direct sampling, scan features;
   - lifecycle: enumerate, open, close, start stream, stop stream;
   - controls: set center frequency, sample rate, gain/input/clock;
   - streaming: IQ callback with format metadata and actual center/sample-rate values.

Recommended order:
   - Fobos backend adapter first, preserving current behavior behind the interface;
   - RTL-SDR via `rtl_tcp` as the easiest first non-Fobos backend, because it avoids direct USB/library deployment complexity;
   - direct RTL-SDR via `librtlsdr` after the network backend works;
   - optional SoapySDR backend later for Airspy, HackRF, SDRplay, LimeSDR, and similar receivers.

## Standard-firmware scan mode

Goal: a slower but stable scan mode that works without Agile firmware.

First version:
   - define one or more scan segments by start/end frequency or by explicit center frequencies;
   - for each segment, tune the receiver center frequency, wait a short settle time, collect a few FFT frames, and write the result into a stitched scan canvas;
   - adjacent centers should normally be spaced by at least the selected sample rate to avoid self-overlap.
   - Status for 4.0: first-pass explicit-center standard scan is implemented with sample-rate-safe auto-spacing, +/- sample-rate add buttons, adjustable dwell time, and stitched ScanVisualAssembler output. Performance depends on host/libusb retune overhead.

Visualization:
   - continuous ranges can be drawn as one long spectrum/waterfall row;
   - sparse ranges should be drawn as stitched sections with visible segment separators and labels;
   - each segment stores its own center/sample-rate so frequency labels do not rely on the current receiver center alone.

Demodulation:
   - first pass: visual scan and candidate detection only;
   - second pass: when a candidate is selected, retune and hold on that segment for normal demodulation;
   - later: optional scan-follow mode where demodulation runs only while the active scan segment contains the listening frequency.

## GNSS/GPS and QTH mapping

Goal: explore whether FobosAPP can receive and use GNSS signals, then expose the operator location in a radio-friendly map view.

Research first:
   - evaluate whether the current SDR path can practically capture GNSS L1 signals with enough bandwidth, stability, and timestamp quality for acquisition/tracking;
   - start with offline IQ recordings and a small GNSS acquisition prototype before adding live UI;
   - keep expectations realistic: full GPS/GNSS positioning from raw SDR is a spread-spectrum receiver project, not a normal narrowband demodulator.

Fallback location sources:
   - allow manual latitude/longitude entry for testing and offline use;
   - later consider OS/location-service input, serial/NMEA GPS receivers, or network-provided location as easier non-SDR sources.

Map window:
   - separate dock/window showing own position, current receiver frequency context, and optional saved markers;
   - online map source first if acceptable, with an offline tile/cache option later;
   - never require network access for the rest of the SDR app to function.

Radio amateur overlay:
   - compute Maidenhead locator/QTH grid from current coordinates;
   - draw Maidenhead grid squares over the map with locator labels at useful zoom levels;
   - show current QTH locator prominently and make it copyable for logs or radio contacts.

## Practical priority

1. Clean packaging and config defaults.
2. External DMR decoder bridge for lab comparison.
3. Standard-firmware visual scan.
4. RTL-SDR network backend.
5. Direct RTL-SDR backend.
6. Optional SoapySDR backend.
7. GNSS/GPS and QTH map research prototype.
8. Replace or isolate remaining DMR voice third-party code.
