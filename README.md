# FobosAPP

FobosAPP is a Windows SDR receiver application for RigExpert Fobos SDR hardware.
Version 3.0 beta keeps the stable real-device, network, and video/image work
from the 2.x line, and adds the first DMR laboratory monitoring tools.

## Windows Release Package

Use the packaged `release/bin` folder as a self-contained runtime directory.
Keep `FobosAPP.exe`, the Qt DLLs, Fobos DLLs, FFTW DLL, libusb DLL, VC runtime
DLLs, and the `platforms/qwindows.dll` plugin together in the same folder tree.

The app stores local settings in `FobosAPP.ini` next to the executable and writes
diagnostic logs to `FobosAPP_diagnostic.log`.

## Main Features

- AM, SAM, NFM, WFM, USB, LSB, DSB, CW, FT8, RTTY, FSK, PSK, ATV, SSTV, APT,
  WEFAX, Meteor LRPT beta, and DMR beta mode selection.
- Digital Audio dock with FT8/RTTY/FSK/PSK decoding work and DMR beta
  sync/activity monitoring.
- DMR Lab Capture metadata fields and JSON sidecar files for controlled audio
  and Channel IQ recordings.
- Video dock with analog TV, SSTV, NOAA APT, HF WEFAX, and Meteor LRPT beta
  monitor/test modes.
- Correct `HF1 + HF2` direct-sampling spectrum layout from `-Fs/2` through
  zero to `+Fs/2`.
- Stable async Fobos streaming path with safer start, stop, and retune handling.
- Live retuning of frequency, sample rate, FFT length, gains, bandwidth, audio,
  and display settings.
- Resizable/dockable UI, zoomable spectrum and waterfall, Y-axis level display,
  level range controls, and frequency widgets with units, presets, and step
  buttons.
- Network server/client mode with remote control, audio/spectrum streaming,
  channel IQ streaming, full IQ client processing, and observer clients.
- Standard Fobos API and Fobos SDR agile API device discovery.

## Build Notes

The repository tracks the application source code and release helper scripts.
The local third-party SDK/runtime folders are intentionally ignored:

- `fobos/`
- `fobos_agile/`
- `fftw-3.3.5-dll64/`
- `libusb-1.0.27/`
- `release/`

The current Windows build expects Qt 5.15.2 for MSVC, MSVC 2022, CMake, FFTW,
libusb, the standard Fobos SDK, and the Fobos SDR agile SDK to be available in
the paths used by `CMakeLists.txt` and `tools/deploy_windows.ps1`.

Build and deploy locally:

```powershell
cmake --build build\Desktop_x86_windows_msvc2022_pe_64bit-Release --config Release
powershell -ExecutionPolicy Bypass -File tools\deploy_windows.ps1
```

## Current Limitations

- DMR is currently a beta sync/activity monitor, not a complete DMR voice or
  payload decoder.
- The new WEFAX, LRPT, and analog video paths are beta features and need more
  testing with real signals.
- Meteor LRPT currently provides a QPSK IQ monitor, not a final decoded image.
- Full IQ client processing needs a fast LAN and can be too heavy for slower
  network links.
- Multi-client observer/control transfer mode is new and should receive more
  real-world testing.
