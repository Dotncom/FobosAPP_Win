# FobosAPP

FobosAPP is an SDR receiver application for RigExpert Fobos SDR hardware.
The current packaged release is Windows-first, with active Linux/Raspberry Pi
and Android USB/network client support work. Version 4.2.2 keeps the stable
real-device, network, and video/image work from the 2.x/3.x line, expands the
receiver backend layer, adds native RTL-SDR/rtl_tcp and optional SoapySDR
backends, improves scan/measurement tooling, keeps the GNSS/QTH laboratory
workflow available, and keeps the desktop, Raspberry Pi, and Android tuning
controls current.

## Windows Release Package

Use the packaged `release/bin` folder as a self-contained runtime directory.
Keep `FobosAPP.exe`, the Qt DLLs, Fobos DLLs, FFTW DLL, libusb DLL, VC runtime
DLLs, and the `platforms/qwindows.dll` plugin together in the same folder tree.

The app stores local settings in `FobosAPP.ini` next to the executable and writes
diagnostic logs to `FobosAPP_diagnostic.log`.

Before replacing a release folder during an update, keep or export
`FobosAPP.ini`. It contains user presets, scan lists, QTH map markers, API keys
for map providers, and UI settings. The desktop app also provides
`Settings... -> Settings backup -> Export settings... / Import settings...` for
making a separate backup file.

DMR voice in 4.2.2 is experimental. Windows packages may include optional
`dmr_voice_backends/fobos_dmr_voice_*.dll` GPL backend modules. See
`THIRD_PARTY_LICENSES.txt` and `licenses/dmr_voice_backend/` before
redistributing AMBE-capable binaries.

GNSS/QTH work in 4.2.2 is a laboratory workflow, not a finished
navigation receiver. See `docs/gnss_preflight_4.1.md` for the recommended test
order, RF notes, and the generated acquisition report files.

## Optional DMR Voice Backends

FobosAPP loads optional DMR voice modules from `dmr_voice_backends/` at runtime.
The app itself uses the stable C ABI in
`FobosDMRVoiceBackend-gpl/include/fobos_dmr_voice_backend.h`; third-party AMBE
implementations are kept as optional Git submodules so users can fetch and build
them only when needed.

Optional backend dependencies:

- mbelib-neo: https://github.com/arancormonk/mbelib-neo
- OpenDMR/softdmr: https://github.com/hicaoc/softdmr

When cloning from Git:

```bash
git clone https://github.com/Dotncom/FobosAPP.git
cd FobosAPP
git submodule update --init --recursive third_party/mbelib-neo third_party/softdmr
```

If you are working from a source zip instead of a Git checkout, clone the
optional dependencies into the same paths:

```bash
git clone https://github.com/arancormonk/mbelib-neo.git third_party/mbelib-neo
git clone https://github.com/hicaoc/softdmr.git third_party/softdmr
```

Build the backend modules after the submodules are present:

```powershell
cmake -S FobosDMRVoiceBackend-gpl -B build\fobos-dmr-voice-backend-gpl-vs `
  -G "Visual Studio 17 2022" -A x64
cmake --build build\fobos-dmr-voice-backend-gpl-vs --config Release
```

Copy the resulting backend libraries into the runtime folder:

```text
release/bin/dmr_voice_backends/fobos_dmr_voice_mbelib.dll
release/bin/dmr_voice_backends/fobos_dmr_voice_opendmr.dll
```

On Linux/Raspberry Pi the same backend project builds shared libraries with
`.so` names, usually `libfobos_dmr_voice_mbelib.so` and
`libfobos_dmr_voice_opendmr.so`; place them in the application runtime
`dmr_voice_backends/` directory.

## Optional DSD-neo DMR Bridge

The Digital Audio dock has a `DMR backend` selector:

- `FobosAPP + mbelib`
- `FobosAPP + OpenDMR/OP25`
- `DSD-neo`
- `GopherTrunk (future)` reserved for later work

The optional DSD-neo bridge is disabled by default. When selected, FobosAPP
forwards the selected DMR discriminator PCM as raw mono PCM16LE over local UDP and
can accept decoded DMR voice audio back over UDP.

Manual DSD-neo command matching the default ports:

```powershell
.\dsd-neo\dsd-neo.exe -fs -i udp:127.0.0.1:7355 -s 48000 -o udp:127.0.0.1:23456 -nm
```

See `docs/dmr_external_backend.md` and `config/dmr_backends.example.json` for
the full bridge contract. DSD-neo is an external optional tool; it is not
required for normal SDR, scan, GNSS, or internal DMR metadata use.

## Main Features

- AM, SAM, NFM, WFM, USB, LSB, DSB, CW, FT8, RTTY, FSK, PSK, ATV, SSTV, APT,
  WEFAX, Meteor LRPT beta, and DMR beta mode selection.
- Digital Audio dock with FT8/RTTY/FSK/PSK decoding work, DMR beta
  sync/activity monitoring, and an optional DSD-neo lab bridge.
- DMR Lab Capture metadata fields and JSON sidecar files for controlled audio
  and Channel IQ recordings.
- Video dock with analog TV, SSTV, NOAA APT, HF WEFAX, and Meteor LRPT beta
  monitor/test modes.
- Correct `HF1 + HF2` direct-sampling spectrum layout from `-Fs/2` through
  zero to `+Fs/2`.
- Stable async Fobos streaming path with safer start, stop, and retune handling.
- Live retuning of frequency, sample rate, FFT length, gains, bandwidth, audio,
  and display settings.
- Visual manual-retune scan by cycling through an explicit center frequency
  list on Standard or Agile APIs. Adjacent centers are auto-spaced by at least
  the current sample rate, and the dwell time is user-adjustable because the
  practical scan rate is limited by host/libusb retune overhead, not by the RF
  hardware alone.
- Resizable/dockable UI, zoomable spectrum and waterfall, Y-axis level display,
  level range controls, and frequency widgets with units, presets, and step
  buttons.
- Network server/client mode with remote control, audio/spectrum streaming,
  channel IQ streaming, full IQ client processing, and observer clients.
- Runtime-selectable receiver backend layer with Standard Fobos API, Fobos SDR
  agile API, native RTL-SDR, rtl_tcp, and optional SoapySDR discovery.

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
powershell -ExecutionPolicy Bypass -File tools\build_windows_local.ps1
powershell -ExecutionPolicy Bypass -File tools\deploy_windows.ps1 `
  -BuildDir build\steamdeck-msvc-release\Release
```

Optional GPL DMR voice backend modules are built separately after the DMR
submodules are present; see `Optional DMR Voice Backends` above.

Create the Windows zip after deploy:

```powershell
powershell -ExecutionPolicy Bypass -File tools\package_windows_release.ps1
```

The application icon is stored in `packaging/icons` and is embedded into the
Windows executable, the Qt runtime resources, Linux desktop installs, and the
Android USB/network client launcher.

Optional Windows code signing requires a real code-signing certificate from a
trusted certificate authority if you want Windows to show a verified publisher.
After the certificate is available as a PFX file, deploy and sign with:

```powershell
$env:FOBOSAPP_CODESIGN_PFX="C:\path\to\certificate.pfx"
$env:FOBOSAPP_CODESIGN_PASSWORD="pfx-password"
powershell -ExecutionPolicy Bypass -File tools\deploy_windows.ps1 -Sign
```

You can also sign an already-built executable directly:

```powershell
powershell -ExecutionPolicy Bypass -File tools\sign_windows.ps1 `
  -Path release\bin\FobosAPP.exe `
  -PfxPath C:\path\to\certificate.pfx `
  -CertPassword pfx-password
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

For future Raspberry Pi DMR voice/backend testing, start from a Git checkout
with the optional DMR submodules, or clone them into the unpacked source tree:

```bash
git submodule update --init --recursive third_party/mbelib-neo third_party/softdmr
# or, inside a source zip tree:
git clone https://github.com/arancormonk/mbelib-neo.git third_party/mbelib-neo
git clone https://github.com/hicaoc/softdmr.git third_party/softdmr
```

Then build the app and the optional backend modules:

```bash
sudo ./tools/install_deps_debian.sh
./tools/prepare_fobos_linux.sh
./tools/build_linux.sh
cmake -S FobosDMRVoiceBackend-gpl -B build/fobos-dmr-voice-backend-gpl \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build/fobos-dmr-voice-backend-gpl -j
```

For a normal runtime layout, copy the built backend `.so` files next to the app:

```bash
mkdir -p build/linux-release/dmr_voice_backends
cp build/fobos-dmr-voice-backend-gpl/libfobos_dmr_voice_*.so \
  build/linux-release/dmr_voice_backends/
./tools/run_linux.sh
```

During development, FobosAPP also searches `build/fobos-dmr-voice-backend-gpl/`
directly, so the copy step is optional for quick local tests.

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

## Android USB/Network Client

An Android USB/network client lives in `android/network-client`. It is a
separate Android Studio project that can talk to the existing FobosAPP network
server over the TCP control channel or run the current direct USB/OTG Fobos
preview path. The current Android client supports remote
start/stop/settings, observer/controller role handling, spectrum/waterfall
display, zoom/pan, tap-to-tune, frequency presets, band-marker overlays, and
48 kHz mono ready-audio playback in network mode. The USB/OTG path can open a
Fobos receiver through Android USB Host, apply basic receiver settings, and
show an experimental live spectrum/waterfall preview on compatible hardware.

Android debug APKs are signed by the normal Android debug key. For a signed
release APK, set these environment variables before running
`./gradlew assembleRelease` from `android/network-client`:

```bash
export FOBOSAPP_ANDROID_KEYSTORE=/path/to/release.keystore
export FOBOSAPP_ANDROID_KEY_ALIAS=fobosapp
export FOBOSAPP_ANDROID_KEYSTORE_PASSWORD=...
export FOBOSAPP_ANDROID_KEY_PASSWORD=...
```

## Current Limitations

- DMR is currently a beta sync/activity monitor, not a complete DMR voice or
  payload decoder.
- The new WEFAX, LRPT, and analog video paths are beta features and need more
  testing with real signals.
- Meteor LRPT currently provides a QPSK IQ monitor, not a final decoded image.
- Full IQ client processing needs a fast LAN and can be too heavy for slower
  network links.
- Linux/Raspberry Pi support is usable for current test builds, but still needs
  broader hardware and distribution testing.
