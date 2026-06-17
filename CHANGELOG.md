# Changelog

## 4.5.0 - 2026-06-17

### Added

- Added a full external serial GNSS workflow for NMEA/u-blox receivers, tested
  with a NEO-M8N module: serial port selection, live fix status, satellite
  table, sky view, QTH map overlay, time-zone display, and per-system/per-
  satellite filtering for FobosAPP's own display and analysis.
- Added UBX support for enabling NAV-PVT/NAV-SAT/NAV-DOP output, polling and
  applying `CFG-GNSS` constellation settings, saving module configuration with
  `CFG-CFG`, and capturing exact raw UBX/NMEA serial logs for later replay or
  analysis.
- Added GNSS raw serial capture alongside the existing NMEA text logging and
  SDR IQ acquisition reports.
- Added SDR GPS L1 C/A acquisition diagnostics improvements: per-millisecond DC
  cleanup, GNSS tone-notch diagnostics, stronger report metadata, and clearer
  weak/no-lock status for difficult RF conditions.

### Changed

- Reduced normal diagnostic-log noise from live GNSS serial and IQ-monitor
  telemetry. Frequent NMEA/UBX/IQ detail is still available when verbose
  logging is enabled in Settings.
- Native RTL-SDR starts with tuner auto-gain/AGC by default, which is more
  useful for weak GNSS experiments and avoids the previous zero-gain trap.
- Reorganized the GNSS/QTH controls into a more compact layout so the main
  control panel no longer grows excessively wide.

### Fixed

- Fixed UBX `CFG-GNSS` handling so the first button press polls and synchronizes
  the module state instead of blindly applying unsupported constellation
  combinations.
- Fixed UBX configuration saves by sending `CFG-CFG` and reporting ACK/NAK in
  the GNSS status area.
- Fixed raw serial logging cleanup so active UBX/NMEA captures are closed when
  the serial receiver is stopped.

### Notes

- Generic NMEA GNSS modules should provide most live map, QTH, sky-view and
  satellite-table functionality. UBX configuration controls are intended for
  u-blox-compatible modules and were field-tested on NEO-M8N.
- SDR-only GPS L1 C/A reception remains experimental. The current tools can
  record IQ, monitor RF quality, run acquisition and produce reports, but a real
  position lock from SDR IQ has not been proven with the tested antenna setup.
- DMR voice remains a laboratory feature. The mbelib/OpenDMR, DSD-neo and
  GopherTrunk paths are included for testing and comparison, not as a finished
  commercial-grade DMR receiver.

## 4.4.0 - 2026-06-16

### Added

- Added selectable DMR backend routing for the internal mbelib path, the
  internal OpenDMR/OP25-compatible path, DSD-neo, and the experimental
  GopherTrunk bridge.
- Added DSD-neo bridge controls, UDP PCM handoff, decoded PCM return path, and
  release packaging support for bundled optional DSD-neo runtime files.
- Added the experimental GopherTrunk bridge scaffold for future virtual-source
  tests.
- Added DMR channel-rate control and improved DMR metadata stability filtering.
- Added live USB/serial NMEA GNSS input through Qt SerialPort, including
  NEO-M8N-friendly COM-port controls, fix/satellite status reporting, and
  automatic QTH/map updates when valid GGA/RMC coordinates are received.
- Added QTH privacy controls for clearing the current real position, stopping
  NMEA reception on request, hiding the green current-location marker, and
  removing test/search markers from the map with right-click.

### Fixed

- Reworked stale `DataProcessor` recovery so the Start button no longer silently
  switches the UI to Running when the reader is stuck in an idle-looking state.
- Kept the DMR center offset workaround active so DMR decoding is not run with
  listening and RF center exactly equal on Fobos Agile.
- Bundled `Qt5SerialPort.dll` in the Windows runtime package so serial GNSS
  receiver support works from the release folder.

### Notes

- The attempted Agile startup frequency jog was removed after field testing
  showed it could make the shifted spectrum state persistent. Startup retune
  recovery remains under investigation and should not be released as fixed yet.
- Some Fobos/Agile receivers or host/firmware combinations may still show
  intermittent center-frequency placement, spectrum shift, or I/Q mirroring
  after tuning. This is documented as a known issue until more hardware can be
  tested with the manufacturer.

## 4.3.2-beta - 2026-06-15

### Added

- Added an experimental native bladeRF RX backend that loads `bladeRF.dll` /
  `libbladeRF` at runtime and feeds SC16_Q11 IQ into the existing FobosAPP
  float-IQ pipeline.
- Added bladeRF device enumeration, open/configure/start, live center retune,
  standard scan compatibility, and optional detailed RX statistics logging.
- Bundled the Nuand 2025.10 Windows `bladeRF.dll` runtime and matching
  `libusb-1.0.dll` in the beta Windows package under `bladerf/`.

### Notes

- bladeRF support is RX-only and untested without community hardware.
- TX/PTT support is intentionally not enabled yet; it remains planned for the
  separate future `TransmitterBackend` safety architecture.

## 4.3.1 - 2026-06-15

### Changed

- Added block-level Agile scan IQ metadata so spectrum and waterfall stitching
  use the scan point that actually produced each IQ block.
- Improved scan visual behavior in `Floating/True axis` and `Pass composite`
  modes by carrying scan-point metadata through `IqBuffer` and FFT processing.
- Moved the scan visual mode selector above the scan controls so it is clearly
  shared by all scan display modes.

### Fixed

- Fixed Agile scan waterfall rows being placed with a stale or unrelated scan
  index, which caused irregular sector ordering and visual gaps.
- Skipped tuning-period Agile scan IQ blocks (`scan_index == -1`) instead of
  drawing them as if they belonged to a valid scan center.

## 4.2.2 - 2026-06-14

### Added

- Added the separated diagnostics, help, GNSS/QTH helper, preset helper, and
  preset manager modules to reduce the load carried by `main.cpp`.
- Added SATCOM and expanded FPV/LTE/3G default presets, plus ordered preset
  controls for moving entries up and down.
- Added the built-in bilingual feature guide and refreshed the default desktop
  startup profile for wide FM, 200 kHz bandwidth, larger FFT, color spectrum,
  visible band overlays, and safer scan defaults.

### Changed

- Optimized spectrum/waterfall hot-path handling and exposed faster waterfall
  frame timing/row duplication defaults.
- Moved scan/spectrum measurement and spur controls into a clearer Spectrum
  measurement section.
- Shortened the GNSS/QTH controls and moved acquisition diagnostics into a
  separate plot window so the left control panel no longer grows too wide.
- Kept the RTL-SDR backend wired into Standard scan and kept optional SoapySDR
  support documented as experimental.

### Fixed

- Reduced unnecessary diagnostic noise when verbose logging is disabled and
  tightened start/stop/retune handling after the Fobos IQ offset investigation.
- Kept release packaging focused on runtime files by excluding local settings,
  logs, and real IQ/recording artifacts.

## 4.2.1 - 2026-06-14

### Fixed

- Fixed native RTL-SDR startup when `rtlsdr.dll` enumerates more than one
  device and index `0` cannot be opened. FobosAPP now lists real native RTL-SDR
  indices in the receiver combo and falls back to the next visible RTL-SDR
  device if the selected one fails to open.

## 4.2 - 2026-06-14

### Added

- Added a runtime receiver backend layer with the existing Fobos Standard and
  Fobos Agile paths plus native RTL-SDR, rtl_tcp, and optional SoapySDR
  backends.
- Added Standard scan support for external live-retune backends, so native
  RTL-SDR/rtl_tcp/Soapy paths can use the same explicit-center scan workflow.
- Added ordered preset management controls, LTE/3G default presets, larger FFT
  sizes, faster waterfall refresh options, and a built-in bilingual feature
  guide from General settings.
- Added a 4.2 default configuration snapshot.

### Changed

- Retune/start/stop handling now uses a safer IQ-buffer generation path after
  the Fobos retune-offset investigation.
- Spectrum/waterfall drawing avoids extra row copies in the hot path and can
  duplicate rows for a faster visual waterfall.
- Android is now labeled and versioned as the USB/Network client
  (`4.2-usb-network`) because it includes the direct USB/OTG Fobos preview path
  as well as the network client path.
- Release packaging now documents optional RTL-SDR runtime files and keeps the
  local RTL-SDR source checkout out of Raspberry/source packages.

### Known Issues

- Native RTL-SDR and optional SoapySDR support are new and much less tested
  than the Fobos backends.
- Android USB/OTG is still a diagnostic live preview, not a full Android
  replacement for the desktop receiver pipeline.
- DMR voice and GNSS acquisition remain laboratory features.

## 4.1 - 2026-06-13

### Added

- Added the GNSS/QTH laboratory workflow with map window, Maidenhead/QTH grid
  overlay, online/offline map layers, provider selection, marker editing,
  coordinate search, NMEA paste, and QTH copy support.
- Added GNSS L1 tuning presets, GNSS IQ monitor, raw GNSS IQ save, listening
  scan presets, GPS L1 C/A live/deep acquisition, offline replay, synthetic GPS
  acquisition self-test, synthetic multi-satellite position solver test, NTP
  time query, Doppler search controls, and spur-watch diagnostics.
- Added repeatable GNSS acquisition artifacts under `recordings/gnss_reports`:
  JSON summary reports plus PRN/Doppler heatmap and correlation-profile CSV
  dumps.
- Added `docs/gnss_preflight_4.1.md` and expanded the roadmap/IQ audit notes
  for future GNSS-SDR/reference comparison.

### Changed

- Standard-firmware scan now includes explicit-center controls, range fill,
  safe spacing, presets, adjustable dwell/settle timing, and stitched scan
  display improvements.
- Android network client version is bumped to `4.1-network`.
- Release deployment now includes the 4.1 default configuration snapshot.

### Known Issues

- Real GPS L1 C/A lock is not proven yet. The GNSS tools are meant for saved IQ,
  replay, RF diagnostics, and future active-antenna testing.
- DMR voice remains experimental/lab quality and is not a stable voice receiver
  in this release.

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
