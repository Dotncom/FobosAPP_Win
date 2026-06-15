# FobosAPP roadmap after 4.0

This file tracks the practical roadmap after the 4.0 release work. It should stay ordered by the next useful engineering steps, not by the order in which features were originally imagined.

## Roadmap Inventory After 4.3.0

This section is the current working inventory after the 4.3.0 release cleanup. Older sections below are kept as historical context and detailed task lists.

Completed or usable enough to keep building on:

1. Release and repository hygiene:
   - public 4.3.0 release packages are rebuilt without local proxy/logger/lab tools;
   - 4.2.0, 4.2.1, and 4.2.2 releases/tags were removed because their source packages exposed local diagnostic proxy tools;
   - Windows package excludes local logs, captures, screenshots, user settings, and private files;
   - Raspberry source package now excludes Android, Windows-only release scripts, sandbox files, lab replay tools, proxy loggers, local reference repositories, and generated artifacts.
2. Windows desktop baseline:
   - Fobos standard and Agile paths remain the primary supported receiver paths;
   - Windows package includes Qt, FFTW, Fobos runtime DLLs, and separate RTL-SDR runtime files;
   - startup from the staged release folder was verified during 4.3 packaging.
3. Linux/Raspberry baseline:
   - Linux build helpers exist for dependencies, Fobos libraries, build, and run;
   - Raspberry source package is small and build-focused;
   - `librtlsdr-dev` and `rtl-sdr` are now part of the Debian dependency helper;
   - Linux runtime lookup now supports local/system `librtlsdr.so` candidates for native RTL-SDR.
4. Android network/USB client:
   - Android network client exists and can display spectrum/waterfall, control frequency, and play audio in network mode;
   - Android USB work includes direct Fobos and RTL-SDR experiments through Android USB Host APIs;
   - Android native bridge exists for heavier DSP/audio paths;
   - Android UI now has receiver mode, controls/settings separation, pan/zoom, band overlays, fine tuning, and RTL sample-rate choices.
   - RTL-SDR already served its main purpose as a lightweight reference receiver and performance baseline for average Android devices; do not treat perfect RTL audio as a core roadmap item.
5. Network mode:
   - desktop server/client control synchronization is much more consistent after the 4.3 pass;
   - server is treated as the source of truth for receiver state;
   - desktop clients can mirror scan visual frames and scan state;
   - Android is intentionally kept lighter and does not mirror all heavy scan/GNSS/DMR state.
6. Receiver backends:
   - first backend boundary exists for Fobos, RTL-SDR native, RTL-TCP, and optional SoapySDR;
   - RTL-SDR native works by runtime loading on desktop and direct USB experiments on Android;
   - RTL-TCP path remains available for networked lightweight RTL tests;
   - SoapySDR support is present as an optional runtime path, but mostly unverified.
7. Scan and spectrum tools:
   - Agile scan and standard retune-based scan exist;
   - stitched scan visualization exists through `ScanVisualAssembler`;
   - scan visualization now supports `Compressed/Mosaic`, `Floating/True axis`, and `Pass composite` modes;
   - measurement overlays, bandwidth drag measurement, spur calibration/suppression, band-plan overlays, compact overlays, and editable presets/band plans exist.
8. GNSS/QTH:
   - QTH map, marker workflow, map providers/cache, NMEA paste import, Maidenhead locator, GNSS presets, IQ save, GPS L1 acquisition experiments, diagnostic reports, and synthetic solver tests exist;
   - real GNSS lock is still not proven.
9. DMR:
   - DMR sync/metadata/voice path is active but still experimental;
   - color code/timeslot/source/target metadata have appeared in lab tests, but not with enough repeatability;
   - voice has produced recognizable fragments, but sync/framing/audio quality are not stable.
10. FPV/video:
   - FPV hunter controls, presets, sparse scans, analog video attempts, and digital-video detector scaffolding exist;
   - real-world video decoding remains paused until better live signals are available.

Still open or risky:

1. DMR fundamentals:
   - CACH/SlotType/LC/EMB/cadence voting must become deterministic before more AMBE tuning;
   - offline replay must be the main comparison tool, not live trial-and-error only;
   - external decoder comparison should verify metadata and burst timing stage by stage.
2. GNSS:
   - real GPS L1 C/A lock is not proven yet;
   - a dedicated active/passive GNSS antenna and cleaner RF test conditions are still needed;
   - acquisition logic exists, but needs real-signal validation against known-good recordings or a reference receiver.
3. FPV/video:
   - FPV/video hunter scaffolding exists, but real-world signal validation is incomplete;
   - analog video demodulation and digital-video detection still need live captures and clearer output metrics.
4. Transceiver architecture:
   - RX path is useful, but TX/control path is only a plan;
   - hardware interlocks, PTT sequencing, Raspberry service boundaries, and external TX module control are not implemented.
5. Network completeness:
   - DMR and GNSS network state are intentionally deferred;
   - preset/band-plan sync is not automatic and should remain explicit import/export unless remote workflows demand more.
6. UI architecture:
   - `main.cpp` is still too large;
   - settings, network, scan, GNSS, DMR, video, and receiver UI glue should continue moving to focused modules when risk is low.
7. Regression watch:
   - Fobos/Agile start, retune, and sample-rate behavior were heavily stabilized during Steam Deck work;
   - keep them under release-test coverage, but do not make them the top active task unless the bug returns.
8. Multi-receiver scan/display:
   - future work: run two sources at once, such as two Fobos receivers, local Fobos plus remote Fobos, or Fobos plus RTL/Soapy;
   - decide whether the UI should show sources as split spectrum/waterfall panes, transparent overlay layers, or detachable receiver windows;
   - keep per-source control isolated so the primary Fobos path does not inherit latency from optional secondary receivers.

Near-term order after 4.3:

1. Build a repeatable DMR offline harness around real known Motorola recordings before more live voice changes.
2. Finish CACH/LC/EMB/cadence voting diagnostics and expose confidence metrics for CC/TS/SRC/TG/encryption state.
3. Keep GNSS ready for the next antenna/test opportunity: acquisition replay, reporting, and reference-tool comparison.
4. Resume FPV/video only when live signals or saved captures are available, then improve detector output before deep demodulation work.
5. Start transceiver-mode architecture with a simulator-only `TransmitterBackend` and no real TX control yet.
6. Treat Android RTL as a completed reference experiment unless a specific transceiver/mobile use case makes it important again.

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

## Network Synchronization Backlog

Current near-term rule: keep Android lightweight, and use the desktop-to-desktop network path for heavier remote-control workflows.

Implemented or in progress:

1. Scan state is part of the desktop network settings contract:
   - Agile scan enabled/ranges/step/auto-step;
   - standard scan enabled/centers/dwell/settle/range start/range end;
   - listening scan enabled/targets/dwell/settle;
   - scan listening lock;
   - spectrum measurement enabled/bin size.
2. Scan visual frames carry scan segments so a desktop client can render stitched scan spectrum/waterfall views.

Deferred until the feature modules are complete:

1. DMR network sync:
   - DMR lock/lab fields such as color code, timeslot, source ID, target ID, call type, radio label, and notes;
   - DMR metadata/status events;
   - DMR hunter settings if remote operation needs the detector state mirrored.
2. GNSS network sync:
   - GNSS acquisition/runtime status, plots, and reports;
   - GNSS monitor history and spur-watch state;
   - explicit remote commands for tune, scan, acquire, deep acquire, replay, and network time.

Keep local by default:

1. UI language, layout, collapsed panels, graph/waterfall levels, and local audio devices.
2. Preset databases and band plans, unless a future explicit import/export or send-to-peer workflow is added.
3. Measurement baseline/peak history, because it belongs to the operator's current analysis session.

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
8. Keep receiver backend abstraction Fobos-first; only polish RTL/Soapy/bladeRF enough that optional fallback backends do not disturb the Fobos path.
9. Build the DMR external decoder bridge for comparison.
10. Return to DMR voice only with a repeatable offline harness and clearer stage-by-stage metrics.

## Future Transceiver Mode

Long-term goal: use Fobos SDR, Raspberry Pi, FobosAPP, and custom modules as the receive/control side of a homebuilt transceiver.

Important boundary: Fobos is a receiver. Transmit support must be designed as a separate controlled subsystem, not as a hidden extension of the receive path.

Target architecture:

1. Receiver side:
   - keep Fobos/Fobos Agile as the wideband RX frontend;
   - keep bladeRF as an experimental native RX backend for community testing, not as a reason to slow down the primary Fobos path;
   - keep spectrum, waterfall, demodulation, scan, DMR/GNSS/video detectors, presets, and band plans inside FobosAPP;
   - keep RX-only operation safe and unchanged when no transmit module is connected.
2. Transmit side:
   - define a `TransmitterBackend` interface separate from `ReceiverBackend`;
   - support PTT, TX frequency, TX mode, TX audio source, TX gain/power request, PA enable, bias/relay controls, ALC/power/SWR telemetry, and fault state;
   - never assume the TX hardware is Fobos; it may be a custom DAC/modulator, external exciter, GPIO-controlled radio module, bladeRF, or later a different SDR.
3. Raspberry/controller role:
   - run the trusted hardware-control service close to the radio hardware;
   - isolate GPIO, relays, bias tee, PA enable, cooling/fan, sensor reads, and watchdog logic from the desktop UI;
   - provide a network API for FobosAPP clients to request TX state changes.
4. Safety interlocks:
   - hardware PTT interlock independent of the UI;
   - RX mute/protect relay before TX;
   - PA enable only after frequency/mode/path validation;
   - emergency TX inhibit;
   - timeout timer;
   - SWR/overcurrent/overtemperature shutdown;
   - band-plan based TX permission hints, with user-region profiles.
5. Duplex and sequencing:
   - support RX-only, simplex PTT, split RX/TX, and repeater-style offset profiles;
   - sequencer order should be configurable: mute RX, switch relays, enable exciter, enable PA, transmit, disable PA, disable exciter, restore relays, unmute RX;
   - expose timing in milliseconds for relays and PA settling.
6. Audio/baseband path:
   - start with microphone/line-in audio routed to an external TX chain;
   - later add generated tones, CW keying, digital mode audio, and test carriers;
   - keep modulation generation modular so FM/SSB/AM/digital transmit experiments do not pollute receive DSP.
7. UI concept:
   - add a dedicated Transceiver panel, initially disabled unless a TX backend is configured;
   - show RX frequency, TX frequency, split/offset, PTT state, selected antenna/path, PA state, power/SWR/current/temp, and fault reason;
   - require an explicit "Enable TX controls" step before any transmit-capable command is accepted.
8. Configuration:
   - add transceiver profiles for bands, antennas, offsets, power limits, GPIO mapping, relay timing, and TX permissions;
   - keep profiles exportable/importable and do not ship dangerous defaults that can key unknown hardware.

Suggested implementation order:

1. Write a TX backend contract and a dummy simulator backend that cannot transmit.
2. Add UI/state plumbing with simulator-only PTT, sequencing, and fault display.
3. Add Raspberry-side GPIO/service prototype with no RF connected.
4. Add hardware interlock and watchdog tests.
5. Add RX mute/protect sequencing around existing Fobos RX.
6. Add external TX module control only after bench tests with dummy load and power/SWR sensors.
7. Add band-plan TX hints and region profiles.
8. Only then add real modulation/audio TX integration.
9. Add bladeRF TX only as a laboratory backend after simulator, sequencing, dummy-load tests, and explicit TX enable/fault UI exist.

## Encrypted DMR Questions

Scope note: DMR encryption must be handled carefully. The project can identify that a call is encrypted and expose legal/user-provided metadata, but it should not include bypass/decryption features for traffic the user is not authorized to access.

Useful supported goals:

1. Detect and display encrypted/privacy status:
   - Basic Privacy / Enhanced Privacy indicators if recoverable from DMR signaling;
   - privacy algorithm or key ID fields only if they are explicitly present in metadata;
   - color code, timeslot, call type, source ID, target ID, and group ID should still be decoded when unencrypted signaling permits it.
2. Improve operator feedback:
   - show "encrypted voice" rather than silent failure or broken AMBE audio;
   - suppress or mark voice decoding attempts when frames are privacy-protected;
   - log encrypted-call cadence and metadata confidence for diagnostics.
3. Authorized lab mode:
   - allow known test fixtures with user-owned radios and user-provided keys to validate detection and metadata handling;
   - if any future decryption support is considered, it must be opt-in, local-only, documented as "authorized systems only", and separated from normal release builds unless legal/safety review says otherwise.
4. Questions to answer:
   - Which DMR privacy indicators are visible in the bursts we already parse?
   - Can CACH/LC/SlotType identify privacy before AMBE frames are handed to the vocoder?
   - How do Motorola Basic Privacy and Enhanced Privacy differ in visible metadata?
   - What should the UI show when metadata is clear but voice is encrypted?
   - Should encrypted-call detection be network-synchronized to clients as a status-only field?

Non-goals for the public app:

1. No recovery of unknown keys.
2. No bypass of access controls.
3. No hidden decryption pipeline.
4. No claim that encrypted DMR audio can be monitored unless the operator owns the system and provides lawful keys/configuration.
