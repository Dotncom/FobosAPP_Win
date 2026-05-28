# Changelog

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
