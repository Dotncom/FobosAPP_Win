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
  safely open/close the device to verify Android USB Host access.

## Current Scope

This first step intentionally uses only server-side processing. The Android app
does not yet process Channel IQ or Full IQ locally, and it does not talk to the
Fobos receiver over Android USB for IQ streaming yet. The USB sandbox is only a
diagnostic proof-of-life layer for the next direct-receiver phase.

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

Expected Fobos identifiers from the patched desktop libraries:

- VID/PID: `16d0:132e`
- Standard API generation: `bcdDevice 0000`
- Agile API generation: `bcdDevice 0101`
