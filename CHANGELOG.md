# Changelog

## 4.0 - 2026-06-11

### Added

- Added optional GPL DMR voice backend modules under
  `dmr_voice_backends/`, with mbelib-neo and OpenDMR/OP25-backed paths.
- Added Ukrainian UI coverage for the main control sections, DMR/digital
  controls, preset manager, network settings, and scan/hunter panels.
- Added release-facing default configuration snapshot in
  `config/fobosapp_defaults_4.0.json`.
- Added DMR backend licensing and attribution notes for GitHub/binary release
  preparation.
- Added a standard-firmware visual scan prototype with an explicit center
  frequency list, sample-rate-safe auto-spacing, and adjustable dwell time for
  slower hosts or libusb-heavy retune paths.
- Added persistent collapsible control-section state so rarely used panels can
  stay closed between launches.

### Changed

- DMR remains experimental, but the 4.0 package can expose the current DMR
  metadata/voice lab state instead of hiding the feature.
- Detailed DMR logging is user-controlled through the settings dialog, with
  log rotation to reduce runaway diagnostic files.
- Standard scan retune timing now uses a small dedicated timer instead of being
  quantized by the spectrum/waterfall refresh interval.

### Known Issues

- DMR voice quality is not beta-stable yet. Real Motorola DMR voice may still
  alternate between intelligible speech, artifacts, and silence.
- External DMR decoder mirroring is documented as the next bridge target but is
  not implemented in the desktop UI yet.

## 3.6 - 2026-06-04

### Added

- Added agile scan measurement tools for broad coverage checks, including
  peak/baseline tracking, CSV export, and graph overlay support.
- Added calibrated spur suppression for stable receiver spurs, with a 50 ohm
  load calibration flow and persisted spur masks.
- Added spectrum hover readout and drag bandwidth measurement on the desktop
  graph.
- Added experimental extended RF tuning range up to 7.75 GHz for receivers and
  firmware that expose useful operation above the official range.
- Added Android network-client controls for FFT length, full/truncated spectrum
  frames, presets editing, fine tuning, and band markers.

### Changed

- Improved DMR laboratory monitoring using controlled Motorola test captures,
  stronger burst/CACH/EMB reporting, and more conservative sync diagnostics.
- Improved auto-center tuning so middle-click/double-click lands on sensible
  rounded frequency steps instead of raw Hz-level estimates.
- Improved desktop persistence for center/listening frequency widgets, unit
  selections, frequency steps, presets, and waterfall/graph level sliders.
- Improved Raspberry Pi/Linux operation with safer Fobos USB refresh/recovery
  behavior and source packaging that excludes the Android project.
- Reworked desktop controls into collapsible sections and refined compact band
  marker display.

### Fixed

- Fixed listening-frequency resets caused by startup/UI refresh paths and
  direct-sampling mode changes.
- Fixed zoomed spectrum/waterfall frame ranges so network/server-side FFT views
  follow the selected visible span.
- Fixed graph/waterfall alignment issues caused by the right-side graph margin.

### Known Issues

- DMR remains a beta sync/activity monitor, not a complete DMR voice decoder.
- Video/image modes remain beta-quality and still need real-signal validation.
- Android is still network-client focused; direct Android USB/OTG receiver
  operation is not production-ready.

## 3.5 - 2026-06-01

### Added

- Added Raspberry Pi/Linux network-server and client hardening around high
  sample-rate Fobos operation, including binary spectrum frames, ready-audio
  relay, and VLC-compatible HTTP/WAV audio streaming.
- Added an Android network client with spectrum/waterfall display, audio
  playback, zoom/pan, tap-to-tune, remote setting controls, preset editor, and
  optional band-marker overlays.
- Added desktop and Android fine-tuning controls, including a horizontal
  mouse-wheel scale mode and an alternate dial mode on desktop.
- Added editable frequency presets and editable spectrum band markers for
  general ranges and amateur/HAM ranges.
- Added collapsible desktop control panels and a compact band-marker display
  mode that draws markers in the lower graph strip instead of tinting the full
  spectrum.
- Added initial Fobos SDR agile compatibility and scan-range UI preparation.
- Added HF1/HF2 noise-cancel laboratory controls and reference spectrum overlay
  tools for local-interference experiments.

### Changed

- Reworked the desktop control layout to reduce vertical space, move less-used
  options into the settings dialog, and add English/Ukrainian UI text loading.
- Improved network command reliability for Android and desktop clients,
  including setting retransmission/debouncing and safer controller/observer
  state updates.
- Improved waterfall persistence and resize behavior so display-history is not
  cleared during routine UI layout changes.
- Improved Linux/Raspberry Pi packaging so vendored patched libfobos sources are
  used instead of silently cloning upstream libraries.

### Fixed

- Fixed Android HF input ordering and direct-sampling tuning synchronization.
- Fixed server/client priority transfer and several network disconnect/restart
  edge cases.
- Fixed client-side frequency and listening-frequency sync in Full IQ and
  server-side spectrum modes.
- Fixed desktop USB/LSB remote display behavior and direct-sampling spectrum
  layout issues found during network testing.

### Known Issues

- DMR remains a beta sync/activity monitor, not a complete DMR voice decoder.
- Video/image modes remain beta-quality and still need real-signal validation.
- Android is still network-client focused; direct Android USB/OTG receiver
  operation is prepared for future work but not production-ready.

## 3.2.1 - 2026-05-29

### Added

- Added an Android USB sandbox for the network client: visible USB device
  listing, Fobos VID/PID highlighting, Android permission request flow, and a
  safe open/close probe that reports raw descriptor `bcdDevice` hints.

### Fixed

- Fixed Android network-client input selection order to match the desktop/server
  Fobos input IDs: RF, HF1+HF2, HF1, HF2.
- Fixed Android HF tuning sync so zero/negative direct-sampling frequencies are
  accepted from spectrum frames and direct inputs always send center frequency
  as 0 Hz.

## 3.2.0-beta - 2026-05-29

### Added

- Added `android/network-client`, an initial Android Studio project for a
  lightweight FobosAPP network client. The skeleton connects to the existing TCP
  control channel, sends basic server-side processing commands, receives
  spectrum/audio frames, draws a simple spectrum/waterfall, plays 48 kHz mono
  PCM audio, and handles basic observer/controller roles.
- Improved the Android client skeleton with a collapsible controls panel,
  explicit light edit-field styling for dark Android themes, server metadata
  syncing from spectrum frames, debounced settings apply, and tap-to-tune on the
  spectrum/waterfall frequency scale.
- Added Android network-client spectrum zoom/pan, tap-to-tune scale handling,
  waterfall level min/max sliders, and a desktop-style waterfall palette.
- Embedded the FobosAPP icon into the Windows executable, Qt runtime resources,
  Linux desktop install rules, and Android launcher resources.
- Added Windows signing helper scripts and optional Android release signing
  configuration driven by environment variables.

### Known Issues

- The Android client is still network-only; direct USB/OTG Fobos receiver access
  is not implemented yet.
- Android audio/display latency and remote-control responsiveness still need
  wider testing on different phones and tablets.

## 3.1.0-beta - 2026-05-29

### Changed

- Started Linux/Raspberry Pi preparation by replacing Windows-only CMake paths
  with platform-aware dependency discovery for Qt5, FFTW, standard libfobos,
  and agile libfobos_sdr.
- Guarded WinMM/Win32-only audio, diagnostics, thread-priority, and device
  enumeration code so non-Windows builds can progress to dependency and runtime
  integration work instead of failing immediately on Windows headers.
- Added an initial non-Windows Qt5 Multimedia PCM playback path for local,
  network, and playback audio frames.
- Added Raspberry Pi OS/Debian helper scripts for dependency installation,
  Linux configure/build, Linux launch, and a Windows source zip packager.
- Added `tools/prepare_fobos_linux.sh` to build the vendored patched libfobos
  and libfobos-sdr-agile source trees into a private project-local Linux prefix.
- Added `third_party/patched/` source trees so Linux/Raspberry Pi builds use
  the FobosAPP-compatible libraries instead of silently cloning official
  upstream code. Upstream cloning is now an explicit opt-in test path only.
- Disabled root-level udev rule installation in the vendored Fobos CMake builds;
  Linux rules are installed by the documented packaging step instead.
- Switched network IQ frames from base64 JSON blobs to binary payload frames with
  compact JSON headers to reduce bandwidth and Raspberry Pi client CPU load.
- In Full IQ client-processing mode, audio now falls back to server-demodulated
  PCM while the client still receives Full IQ for the wide spectrum/waterfall.
  This avoids forcing low-power clients to demodulate audio from 16+ MHz IQ.
- Reverted aggressive Full IQ rate throttling after Raspberry Pi tests showed
  lower traffic did not guarantee lower audio jitter when IQ continuity was lost.
- Made the live IQ audio queue duration-based instead of a fixed float count, so
  low-rate Channel IQ cannot build seconds of stale audio backlog on Raspberry Pi.
- Increased the Qt audio output buffer on Linux to tolerate more network and
  scheduler jitter during remote playback.
- Added a lightweight UDP ready-audio relay: any instance can transmit
  demodulated 48 kHz mono PCM to another FobosAPP instance for low-latency
  playback without an external VLC loop.
- Added a VLC-compatible HTTP/WAV ready-audio stream server so a Raspberry Pi
  client can expose demodulated audio at `http://<host>:<port>/audio.wav`
  without running a second FobosAPP instance on the listener.
- Added Linux packaging drafts for udev USB access and a desktop launcher.

### Known Issues

- DMR remains a beta sync/activity monitor, not a complete DMR voice decoder.
- Video/image modes remain beta-quality and still need real-signal validation.
- The Qt5 Multimedia audio path still needs real Linux/Raspberry Pi testing and
  may need latency/buffer tuning.

## 3.0.0-beta - 2026-05-28

FobosAPP 3.0 beta starts the DMR work as a laboratory/test build. DMR is not a
finished voice decoder yet; this release exposes a DMR sync/activity monitor and
recording metadata tools so controlled test captures can drive the next stage.

### Added

- DMR beta mode in the Digital Audio dock.
- DMR 4FSK sync/activity monitor with frequency-tagged lock and burst reports.
- DMR Lab Capture fields for known test values such as Color Code, timeslot,
  source ID, target/talkgroup ID, radio model, and notes.
- JSON sidecar metadata written next to audio and Channel IQ recordings.

### Fixed

- Corrected the `HF1 + HF2` direct-sampling display model: the spectrum,
  waterfall, and scale now span `-sampleRate/2 .. 0 .. +sampleRate/2`.
- `HF1 + HF2` audio/channel-IQ selection now follows the negative-frequency
  side for HF1 and the positive-frequency side for HF2.

### Known Issues

- DMR is currently a beta sync/activity monitor, not a complete DMR decoder.
- DMR symbol/frame synchronization still needs controlled lab captures before
  payload and voice decoding can be considered reliable.
- Video/image beta limitations from 2.9 remain.

## 2.9.0-beta - 2026-05-28

FobosAPP 2.9 beta is an intentionally experimental video/image decoding build.
The new decoders are present so users with suitable signals can help test them.

### Added

- Video dock modes for ATV, SSTV, NOAA APT, HF WEFAX, and Meteor LRPT beta.
- SSTV decoder support for Robot, Martin, Scottie, Wraase, and PD-family modes,
  plus an internal SSTV test pattern.
- NOAA APT image decoder with a real decoder-path test pattern.
- HF WEFAX beta decoder with a real decoder-path test pattern.
- Meteor LRPT beta QPSK IQ monitor for signal inspection and future image work.
- Analog video VSync option and PAL/NTSC line-rate presets.

### Changed

- Moved video/image modes into the Video dock and digital text/audio modes into
  the Digital Audio dock.
- Added video/image bandwidth presets for SSTV, APT, WEFAX, and LRPT.
- Extended audio filtering and FT8 decode work from the 2.x development builds.

### Known Issues

- WEFAX, LRPT, and analog video are beta-quality and need real-signal testing.
- LRPT currently shows a QPSK/IQ monitor rather than a finished satellite image.
- Video decoding can still be CPU-heavy on weak clients or wide IQ streams.

## 2.0.0 - 2026-05-27

FobosAPP 2.0 is the first broadly usable Windows release after the audio,
stability, and network-control rebuild.

### Added

- Network server/client operation with control-channel handshake.
- Multi-client networking: the first client gets control, later clients become
  observers, and observers can request control transfer.
- Server-side spectrum/audio streaming, channel IQ streaming with client
  demodulation, and full IQ client processing mode.
- Persistent `FobosAPP.ini` settings for receiver, display, audio, and network
  options.
- Support code for both standard `fobos.dll` and agile `fobos_sdr.dll` APIs.
- Device listing with multiple receiver support.
- New frequency controls with units, presets, and step buttons for center
  frequency, listening frequency, and audio bandwidth.
- Right-click spectrum tuning actions for signal center, USB lower edge, and
  LSB upper edge.
- Resizable main window and dockable control panel.
- Spectrum Y-axis level labels and min/max level controls.
- Zoom range extended down to 0.1 percent.

### Changed

- Reworked Fobos start/stop lifecycle around async streaming and safer device
  close/reset handling.
- Improved live retuning while running, including sample-rate and FFT changes.
- Reworked AM/FM/SSB/CW/FT8/FSK/RTTY/PSK audio path and mode bandwidth presets.
- Split NFM and WFM modes.
- Optimized waterfall drawing and sparse network spectrum rendering.
- Reduced server load when streaming to clients by allowing local visual/audio
  suppression.
- Reduced dependency footprint by dropping unused Qt OpenGL and Multimedia
  modules from the link list.

### Fixed

- Frequent crashes on stop, restart, FFT length changes, and sample-rate changes.
- Client stop-button state after remote start.
- Network disconnect detection.
- Client visual freezes after processing-mode changes.
- Missing scaling propagation in network visual modes.
- Waterfall/spectrum gaps when zooming networked spectrum frames.
- Scientific-notation frequency display in normal UI fields.

### Known Issues

- Digital protocol decoding is not complete yet.
- Full IQ network mode can fragment audio on weak links or slow clients.
- Multi-client mode needs more outside testing.
