# FobosAPP

FobosAPP is an SDR receiver application for RigExpert Fobos SDR hardware.
The current packaged release is Windows-first, while the codebase has started
moving toward Linux/Raspberry Pi support. Version 3.0 beta keeps the stable
real-device, network, and video/image work
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
libusb, the standard Fobos SDK, and the Fobos SDR agile SDK. `CMakeLists.txt`
now uses cache paths instead of hard-coded include/link directives:

- `FOBOS_STANDARD_ROOT`
- `FOBOS_AGILE_ROOT`
- `FFTW_ROOT`

On the existing Windows development machine those default to the local SDK
folders above. On Linux, install Qt5 Core/Widgets/Network/Multimedia, FFTW
single-precision libraries, libusb, and Linux builds of the standard/agile Fobos
libraries, or point the three cache paths at local builds.

Build and deploy locally:

```powershell
cmake --build build\Desktop_x86_windows_msvc2022_pe_64bit-Release --config Release
powershell -ExecutionPolicy Bypass -File tools\deploy_windows.ps1
```

Initial Linux configure example:

```bash
cmake -S . -B build/linux-release -DCMAKE_BUILD_TYPE=Release \
  -DFOBOS_STANDARD_ROOT=/usr/local \
  -DFOBOS_AGILE_ROOT=/usr/local \
  -DFFTW_ROOT=/usr
cmake --build build/linux-release -j
```

Raspberry Pi OS quick trial:

```bash
unzip FobosAPP-raspberry-source.zip
cd FobosAPP-raspberry-source
chmod +x tools/*.sh
sudo ./tools/install_deps_debian.sh
sudo cp packaging/linux/99-fobos-sdr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
./tools/prepare_fobos_linux.sh
./tools/build_linux.sh
./tools/run_linux.sh
```

On Windows, create that source zip with:

```powershell
powershell -ExecutionPolicy Bypass -File tools\package_raspberry_source.ps1
```

The archive is written to `release/FobosAPP-raspberry-source.zip`.

If `libfobos` or `libfobos_sdr` are not installed under `/usr/local`, pass their
roots explicitly:

```bash
FOBOS_STANDARD_ROOT=/path/to/libfobos \
FOBOS_AGILE_ROOT=/path/to/libfobos_sdr \
./tools/build_linux.sh
```

`tools/prepare_fobos_linux.sh` builds the vendored Fobos source trees shipped
with the project and installs both Linux libraries into
`third_party/fobos-linux`:

- `third_party/patched/libfobos`
- `third_party/patched/libfobos-sdr-agile`

The standard tree is the patched `libfobos` source used by FobosAPP, so Linux
builds do not silently fall back to the official upstream version. If the
vendored trees are missing, the script stops with an error. For a deliberate
upstream experiment only, run:

```bash
ALLOW_UPSTREAM_FOBOS_CLONE=1 ./tools/prepare_fobos_linux.sh
```

That clone path is intentionally opt-in because it may not contain the stability
fixes expected by this application.

Optional local install after a successful Linux build:

```bash
sudo cmake --install build/linux-release --prefix /usr/local
```

## Ready Audio Relay

The Network Settings dialog includes an optional ready-audio UDP relay. Enable
`Send ready audio by UDP` on the machine that demodulates audio and enter the
target IP/port. On the listening machine, run another FobosAPP instance and
enable `Receive ready audio by UDP` on the same port. The relay carries 48 kHz
mono PCM frames and is intended for low-latency Raspberry Pi monitoring without
an external VLC loop.

For VLC or another generic player, enable `Serve VLC-compatible HTTP/WAV audio`
on the machine that demodulates audio. Open
`http://<raspberry-ip>:21092/audio.wav` from the player, replacing the IP and
port as needed. This mode also carries 48 kHz mono PCM, but uses an ordinary
HTTP WAV stream instead of the FobosAPP UDP framing.

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
- Linux support is still at the early preparation stage. Non-Windows local
  playback now uses an initial Qt5 Multimedia backend, but this path still needs
  real Raspberry Pi testing.
