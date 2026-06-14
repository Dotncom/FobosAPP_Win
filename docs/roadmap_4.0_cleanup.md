# FobosAPP roadmap after 4.0

This file tracks the practical roadmap after the 4.0 release work. It should stay ordered by the next useful engineering steps, not by the order in which features were originally imagined.

## Current 4.0 State

Completed or good enough for the current release:

1. Windows release packaging:
   - package script excludes logs, recordings, screenshots, backups, private tokens, and local lab artifacts;
   - README documents the release package layout and update warning.
2. User settings protection:
   - settings import/export exists in `Settings...`;
   - README tells users to preserve or export `FobosAPP.ini` before replacing the runtime folder;
   - presets, scan lists, QTH markers, map provider keys, and UI settings are treated as user data.
3. Default configuration:
   - runtime defaults are created from code;
   - `config/fobosapp_defaults_4.0.json` stores the release snapshot.
4. DMR status:
   - DMR remains experimental/lab quality, not a stable voice feature;
   - metadata detection and DMR UI are useful for tests, but Motorola voice quality is not release-grade;
   - optional GPL DMR voice backend DLLs and license notes are release-ready.
5. Standard firmware scan:
   - explicit-center scan is implemented;
   - sample-rate-safe spacing, +/- sample-rate add/remove controls, range fill, dwell/settle parameters, presets, and stitched `ScanVisualAssembler` output are implemented;
   - performance remains limited by host/libusb retune overhead.
6. GNSS/QTH first pass:
   - GPS/QTH menu section exists;
   - manual coordinates, NMEA GGA/RMC paste-from-clipboard, Maidenhead/QTH locator, copy QTH, user markers, marker editing, online/offline map layers, map cache options, mouse zoom/pan, provider selection, MapTiler support, and satellite map providers are implemented;
   - GNSS L1 presets, GNSS IQ save, IQ monitor, GPS L1 C/A acquisition/self-test/deep test/offline replay, synthetic multi-satellite position solver self-test, configurable Doppler search bounds, NTP time check, cancellation, GNSS spur watch logging, acquisition diagnostic plots, and acquisition JSON/CSV reports are implemented.

## 4.1 Draft Scope

The useful 4.1 candidate is a GNSS/QTH laboratory release:

1. QTH map and marker workflow is usable online and offline.
2. GNSS L1 tuning, saved IQ, replay, acquisition diagnostics, and synthetic tests are usable.
3. Acquisition attempts write repeatable artifacts to `recordings/gnss_reports`.
4. Real GNSS positioning is not claimed until a real GPS L1 C/A lock is proven with better RF conditions.
5. Receiver-backend abstraction and RTL-SDR support stay next after this GNSS/QTH checkpoint.
6. Receiver-backend abstraction, RTL-SDR native/runtime loading, and optional SoapySDR runtime loading are now first-pass implemented.
7. Startup/close freezes are tracked as a separate stability task; current logs show close can spend several seconds waiting while the IQ reader still receives blocks.

## Active Focus: GNSS/GPS

Goal: determine whether the current Fobos RF path can acquire real GNSS signals, then decide how far raw-SDR positioning should go inside FobosAPP.

What is implemented:

1. QTH map workflow:
   - manual latitude/longitude entry;
   - NMEA GGA/RMC clipboard import for coordinates from phones, GPS loggers, or terminal output;
   - Maidenhead locator calculation and overlay;
   - map window with online/offline modes;
   - map providers including OpenStreetMap, MapTiler, Mapbox placeholder, NASA GIBS, and custom XYZ;
   - user/search/current markers with editable marker list.
2. GNSS RF workflow:
   - GNSS L1 presets for all L1, GPS L1 C/A, Galileo E1, BeiDou B1I, and GLONASS L1OF;
   - GNSS listening scan presets;
   - raw IQ snapshot saving with tuning context;
   - IQ monitor for level, DC, clipping, crest factor, and I/Q balance.
3. GPS L1 C/A acquisition prototype:
   - synthetic GPS IQ self-test passes;
   - real IQ acquisition searches PRN, Doppler, and code phase;
   - selected Channel IQ WAV files can be replayed directly through GPS acquisition without real-time playback;
   - Doppler half-span and Doppler step are user-configurable for fast/coarse versus wide/deep searches;
   - acquisition is cancellable and safe to stop;
   - diagnostic plot shows PRN/Doppler heatmap, best code-phase correlation, and peak-to-second history;
   - each completed live/deep/replay/synthetic acquisition writes JSON plus heatmap/profile CSV files to `recordings/gnss_reports`;
   - NTP time query exists as assisted-GNSS context, not as code-phase lock.
4. Navigation solver prototype:
   - synthetic multi-satellite pseudoranges solve receiver latitude/longitude/altitude and receiver clock bias;
   - result is applied to QTH coordinates and shown on the QTH map as a temporary marker;
   - this verifies the future bridge from real/synthetic pseudoranges into map/QTH workflows.

Current blocker:

1. Real GPS L1 C/A lock is not proven yet.
2. The visible waterfall/spectrum can show narrow parallel lines near 1575.42 MHz, but true GPS C/A should usually appear as spread-spectrum energy below the noise floor.
3. We need to distinguish real correlation peaks from receiver/USB/power/display spurs.
4. DMR and GNSS both show synchronization-sensitive failures, so the raw IQ stream contract must be verified before deeper decoder work.

Next GNSS steps:

1. IQ stream contract audit:
   - log callback block sizes, measured sample rate, callback intervals, late callbacks, queue depth, clipping, DC, and non-finite samples before any decoder;
   - verify whether live IQ is continuous enough for DMR bursts and GNSS coherent/non-coherent accumulation;
   - add sequence/timing metadata to internal IQ frames if diagnostics show drops or discontinuities;
   - keep current findings and optimization ideas in `docs/iq_pipeline_audit.md`.
2. Offline replay:
   - run the same acquisition on saved GNSS IQ recordings; implemented for selected stereo Channel IQ WAV files;
   - preserve acquisition diagnostic metrics per recording; implemented as JSON plus PRN/Doppler and correlation-profile CSV dumps;
   - compare multiple recordings, gains, antennas, and sample rates.
3. External GNSS reference comparison:
   - temporarily use GNSS-SDR or another mature receiver as a verifier on saved IQ;
   - convert/export Fobos IQ into a format accepted by the reference tool;
   - compare acquisition grids, Doppler/code-phase candidates, and failure modes against FobosAPP.
4. Acquisition improvements:
   - follow GNSS-SDR style acquisition more closely: bounded Doppler search, configurable Doppler step, false-alarm based threshold, and optional second-stage Doppler refinement;
   - expand the current dump-style acquisition artifacts if GNSS-SDR comparison needs additional fields;
   - add non-coherent accumulation modes that tolerate navigation bit transitions better than long naive coherent accumulation.
5. Interference/spur handling:
   - GNSS spur watch logs stable narrow peaks around the current GNSS target;
   - add averaged spectrum overlay for GNSS mode with expected GPS L1 C/A bandwidth marked;
   - optionally feed detected/calibrated narrow spurs into acquisition preprocessing;
   - log spur candidates near the GNSS target frequency.
6. Assisted GNSS options:
   - use NTP/system time as a coarse context source;
   - later add almanac/ephemeris download or user-provided files to predict visible PRNs and Doppler ranges;
   - do not make internet access mandatory.
7. Alternative location sources:
   - NMEA GGA/RMC clipboard import is implemented;
   - serial/USB NMEA GPS receiver input;
   - OS/location-service input where available;
   - network-provided coordinates as a fallback for map/QTH workflows.

## Next Receiver Backends

These come immediately after the current GNSS work, as requested.

1. Receiver backend abstraction:
   - split hardware control behind a `ReceiverBackend` interface;
   - capabilities: sample rates, frequency range, gains, inputs, direct sampling, scan features;
   - lifecycle: enumerate, open, close, start stream, stop stream;
   - controls: set center frequency, sample rate, gain/input/clock;
   - streaming: IQ callback with format metadata and actual center/sample-rate values.
   - Status: first boundary started after 4.1; generic backend contract, backend registry, Fobos safe-control wrappers, and Fobos stream safe wrappers are in place; `DataProcessor` now starts from a backend stream descriptor while keeping the live IQ read path behavior unchanged.
2. Fobos backend adapter:
   - move existing standard and Agile Fobos behavior behind the interface without changing user-visible behavior.
3. RTL-SDR through `rtl_tcp`:
   - first non-Fobos backend;
   - avoids direct USB/library deployment complexity;
   - useful for Raspberry Pi and remote receiver tests.
   - Status: first client path implemented; the app can select `RTL-SDR via rtl_tcp`, configure center/sample-rate over the rtl_tcp command protocol, and convert unsigned 8-bit IQ into the existing float IQ pipeline. A separate `rtl_tcp` executable/server is still required.
4. Direct RTL-SDR through `librtlsdr`:
   - add after `rtl_tcp` backend is working;
   - keep deployment and driver notes explicit.
   - Status: native Windows path implemented with runtime `rtlsdr.dll`/`librtlsdr.dll` loading, device enumeration, center/sample-rate configuration, async IQ reading, and unsigned 8-bit IQ conversion into the existing float IQ pipeline. Local test DLLs can live next to `FobosAPP.exe`; release packaging/licensing notes still need to be finalized.
5. Optional SoapySDR backend:
   - later path for Airspy, HackRF, SDRplay, LimeSDR, and similar receivers;
   - keep optional to avoid making the core package fragile.
   - Status: first optional runtime path implemented with dynamic `SoapySDR.dll` loading, generic `SoapySDR auto` device selection, CF32 RX stream setup, center/sample-rate/bandwidth configuration, live retune, and IQ forwarding into the existing float IQ pipeline. This is theoretical/unverified because no non-Fobos Soapy hardware is currently available.

## Architecture And Reference Audit

Goal: compare FobosAPP architecture against mature SDR applications and decide what to simplify, optimize, or isolate before the codebase becomes harder to move.

Near-term cleanup order:

1. Document the hot IQ-path contract before refactoring it.
   - Status: first contract written in `docs/architecture_iq_path.md`.
   - Keep Fobos native IQ reading as the optimized primary path.
   - Treat snapshots and ordered IQ blocks as different consumer models.
   - Do not merge `DataProcessor` and `IqBuffer` unless profiling proves the boundary itself is the bottleneck.
2. Run a sanitary pass over critical modules.
   - Remove stale debug hooks, obsolete traces, old comments, and unbounded logging.
   - Keep behavior unchanged unless a clear bug is found.
3. Extract cold modules from `main.cpp`.
   - Prefer help/settings/translations, preset manager glue, QTH/GNSS map UI, GNSS test UI, and scan preset helpers first.
   - Avoid broad changes to start/stop, tuning, Fobos session, and spectrum update until they are isolated by tests or diagnostics.
4. Optimize IQ only after the contract is stable.
   - Prefer reusable queued buffers, cheaper snapshot reads, and block-level metadata over a large architectural rewrite.
   - Keep UI and Qt signal work outside the receiver callback.

Stability notes to keep:

1. Startup and close can still freeze the UI:
   - inspect `closeEvent`, `stopFobosProcessing`, `DataProcessor::requestStop`, async cancel, and IQ queue drain;
   - avoid closing USB/device handles on the UI thread when the reader is still delivering callbacks;
   - keep this separate from live-retune work, which is currently stable with a guarded retune interval.

References to review:

1. Gqrx:
   - release/source entry point: https://github.com/gqrx-sdr/gqrx/releases
   - compare device abstraction, DSP chain separation, demodulator structure, bookmarks/presets, audio path, settings model, and GNU Radio integration.
2. SDR++:
   - compare modular plugin architecture, source modules, DSP routing, UI responsiveness, waterfall/spectrum performance, and device backend handling.
3. GNU Radio:
   - compare flowgraph-style DSP composition, block boundaries, scheduler assumptions, buffering, metadata, and how much of that model is useful inside a Qt desktop SDR app.
4. GNSS-SDR:
   - compare acquisition/tracking architecture, acquisition grid dumps, thresholds, Doppler refinement, channelization, and offline replay workflow.

Expected decisions:

1. Whether to keep extending the current monolithic `main.cpp` path or start extracting feature modules.
2. Whether scan, GNSS, DMR, video, and network paths should become explicit processing modules with shared IQ-frame contracts.
3. Whether spectrum/waterfall rendering and scan assembly need a shared visual data model.
4. Whether receiver backends should be loaded statically first and optionally as plugins later.
5. Which optimizations are worth doing now:
   - fewer UI-thread operations;
   - stricter worker cancellation;
   - bounded logging;
   - lower-copy IQ buffers;
   - reusable channelizers for DMR/GNSS/video.

## DMR Path

Current status:

1. DMR is experimental/lab quality.
2. The app can expose metadata and attempt voice, but real Motorola DMR voice quality has been inconsistent.
3. Optional AMBE/voice backend architecture exists, but the sync/framing/voice path is not stable enough to call complete.

Next DMR steps:

1. Stop broad trial-and-error changes until a repeatable offline harness proves each stage.
2. Preserve the current optional GPL backend approach:
   - internal test backend;
   - optional mbelib/OpenDMR style backend;
   - external process bridge later.
3. External decoder bridge for lab comparison:
   - mirror selected narrow DMR baseband/audio frames to an external decoder;
   - compare CC/TS/SRC/TG and audio output against our parser;
   - keep this as a verifier, not as a hidden dependency.
4. Only consider a custom AMBE decoder after sync/framing is proven with clean synthetic and real IQ.

## Third-Party Code Policy

Keep for now:

1. Patched `libfobos` and `libfobos_sdr` sources needed for Linux/Raspberry builds.
2. `mbelib-neo` as a temporary optional AMBE backend while DMR voice is experimental.

Do not vendor into releases:

1. External reference repositories used during research.
2. Local trace, decompilation, and reverse-engineering folders.
3. Downloaded SDKs, firmware archives, build products, diagnostic logs, recordings, screenshots, backups, or private tokens.

Release rules:

1. Keep license notes explicit in `THIRD_PARTY_LICENSES.txt`.
2. Keep optional DMR voice modules clearly separated from the core app.
3. Treat non-core external tools as submodules or user-installed dependencies when practical.

## Release And Maintenance

Routine before each public build:

1. Verify Windows package contents.
2. Verify README update instructions and optional DMR backend notes.
3. Verify `translations.json` parses.
4. Verify app starts without local-only files.
5. Verify settings import/export.
6. Verify standard scan presets and GNSS/QTH defaults.
7. Verify GNSS preflight workflow and generated `recordings/gnss_reports` artifacts.
8. Verify Raspberry/Linux source package allowlist.
9. Verify Android network-client build/upload if that release includes Android.

## Practical Priority

1. Fix startup/stop/close freezes by moving unsafe waits/device teardown off the UI thread.
2. Finalize release packaging/licensing notes for RTL-SDR and optional SoapySDR runtime/modules.
3. Continue GNSS acquisition investigation with the new diagnostic plot and saved IQ replays.
4. Test saved Fobos IQ against GNSS-SDR or another mature GNSS reference receiver.
5. Add GNSS non-coherent acquisition improvements.
6. Audit Gqrx, SDR++, GNU Radio, and GNSS-SDR architecture for useful refactors, including remaining IQ stream contract diagnostics such as sequence/timestamp, queue depth, and drop reporting.
7. Add external/non-SDR location fallbacks: NMEA GPS, OS location, network/manual presets.
8. Keep receiver backend abstraction Fobos-first; only polish RTL/Soapy enough that optional fallback backends do not disturb the Fobos path.
9. Build the DMR external decoder bridge for comparison.
10. Return to DMR voice only with a repeatable offline harness and clearer stage-by-stage metrics.
