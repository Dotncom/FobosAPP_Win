# FobosAPP Android Network Client

This is an early Android network-client skeleton for FobosAPP. It is separate
from the desktop Qt Widgets application on purpose: the first Android target is
a lightweight remote client, not a direct USB receiver build.

## What Works In This Skeleton

- Connects to a desktop/Linux/Raspberry FobosAPP instance running in
  `Network Settings -> Server` mode.
- Uses the same TCP control-channel handshake:
  `FOBOSAPP_HELLO 1`, `FOBOSAPP_OK 1`, `FOBOSAPP_PING 1`,
  `FOBOSAPP_PONG 1`.
- Sends server-side processing commands with the existing JSON control schema:
  `start`, `stop`, `settings`, `requestPriority`, and `priorityResponse`.
- Receives server-side `spectrum` frames and draws a simple spectrum/waterfall.
- Lets the user tap the spectrum/waterfall scale to retune the listening
  frequency on the server.
- Receives `audio` frames as base64 `pcm_s16le`, 48 kHz mono, queues them on a
  dedicated audio thread, and plays them with Android `AudioTrack`.
- Supports the multi-client observer/controller role messages at a basic level.
- Uses a collapsible controls panel so the spectrum/waterfall can fill the
  screen, especially in landscape orientation.
- Includes a USB sandbox panel for early direct-receiver work. It can list
  Android-visible USB devices, request permission for a likely Fobos device, and
  safely open/close the device to verify Android USB Host access. It also has
  an experimental OTG session probe that claims the likely streaming interface,
  reports bulk endpoints, reads firmware info with a safe vendor control
  transfer, performs short one-shot bulk read tests, and can start an
  experimental live OTG preview with Android-side FFT frames.

## Current Scope

The normal Android client path still uses server-side processing. The USB
sandbox now has a first direct-receiver preview path, but it is intentionally
limited and diagnostic: 4096-bin FFT maximum, no local audio demodulation yet,
and no full replacement for the desktop receiver pipeline. High sample rates
are exposed for testing, but phone, hub, and USB stack limits still matter.

## Build

Open `android/network-client` in Android Studio, or build from a terminal with
an Android SDK installed:

```bash
cd android/network-client
gradle assembleDebug
```

The project has no third-party Android dependencies. It uses Java, the Android
SDK, and the Android Gradle Plugin declared in `build.gradle`.

## Test Flow

1. Start FobosAPP on the machine with the receiver.
2. Open `Network Settings`.
3. Set `Mode` to `Server`.
4. Set `Processing` to `Server processing (spectrum/audio stream)`.
5. Confirm the control port, default `21090`.
6. Launch the Android client, enter the server IP and port, and press
   `Connect`.
7. After `Control channel ready`, use `Start`, `Stop`, or `Apply`.

For now, keep the Android client in the same trusted LAN/VPN as the server.
The control protocol is intentionally simple and is not encrypted by itself.

## USB Sandbox Test Flow

Use a powered USB-C/OTG hub or dock. The Fobos receiver can draw up to about
5 V / 1 A, so direct phone power is not a reliable test setup.

1. Connect external power to the hub/dock.
2. Connect the hub/dock to the phone.
3. Connect the Fobos receiver to the hub/dock.
4. Open the Android client and expand the controls panel.
5. Press `USB scan`.
6. Confirm that a `16d0:132e` device appears.
7. Press `USB permission/open` and accept the Android permission prompt.
8. Check the log for `openDevice OK` and the detected `bcdDevice` hint.
9. Press `OTG open`.
10. Confirm that the log says `claimed if...` and lists a `bulk IN` endpoint.
11. Press `OTG info` and check that hardware/firmware/build strings are shown.
12. Press `OTG read`. A `-1` result is acceptable before streaming starts; this
    button only verifies that the Android bulk-transfer path is wired.
13. Press `OTG sample test` for a one-shot `OPEN -> START -> bulk read -> STOP`
    experiment at 100 MHz / 8 Msps.
14. Press the top `Net` button to switch it to `OTG`.
15. Press the normal top `Start` button for a short live spectrum/waterfall
    preview. The log should include `preview first FFT frame 4096 bins`.
16. Press the normal top `Stop` button before unplugging during repeated
    experiments.
17. Press `Tools` only when you need the lower-level USB diagnostics:
    scan, permission/open, raw read, one-shot sample test, or explicit close.

Expected Fobos identifiers from the patched desktop libraries:

- VID/PID: `16d0:132e`
- Standard API generation: `bcdDevice 0000`
- Agile API generation: `bcdDevice 0101`
