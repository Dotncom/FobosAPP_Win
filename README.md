# FobosAPP

FobosAPP is a Windows SDR receiver application for RigExpert Fobos SDR hardware.
Version 2.0 focuses on stable real-device operation, usable audio demodulation,
network remote operation, and support for both the standard Fobos API and the
special Fobos SDR agile API.

## Windows Release Package

Use the packaged `release/bin` folder as a self-contained runtime directory.
Keep `FobosAPP.exe`, the Qt DLLs, Fobos DLLs, FFTW DLL, libusb DLL, VC runtime
DLLs, and the `platforms/qwindows.dll` plugin together in the same folder tree.

The app stores local settings in `FobosAPP.ini` next to the executable and writes
diagnostic logs to `FobosAPP_diagnostic.log`.

## Main Features

- AM, SAM, NFM, WFM, USB, LSB, DSB, CW, FT8, RTTY, FSK, and PSK mode selection.
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

- Digital modes are prepared as demodulation modes, but full protocol decoders
  are not implemented yet.
- Full IQ client processing needs a fast LAN and can be too heavy for slower
  network links.
- Multi-client observer/control transfer mode is new and should receive more
  real-world testing.
