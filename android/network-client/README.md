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

## Current Scope

This first step intentionally uses only server-side processing. The Android app
does not yet process Channel IQ or Full IQ locally, and it does not talk to the
Fobos receiver over Android USB. Those are later phases after the network client
is proven useful.

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
