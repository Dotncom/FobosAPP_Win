# DMR External Decoder Bridge

FobosAPP can use several DMR paths from the `DMR backend` combo in the
`Digital Audio` dock:

- `FobosAPP + mbelib` uses the internal DMR parser with the mbelib voice backend
  path.
- `FobosAPP + OpenDMR/OP25` uses the internal DMR parser with the OpenDMR/OP25
  runtime voice backend preference.
- `DSD-neo` mirrors the selected DMR discriminator PCM to an external decoder
  process for comparison and optional decoded-audio playback.
- `GopherTrunk (future)` is reserved in the UI. A local lab-side virtual
  dibit-source adapter is available under `downloads/gophertrunk` for
  integration experiments.

DSD-FME is no longer the active integration target for this project. Keep new
DMR external-decoder work focused on DSD-neo and GopherTrunk unless a specific
test case proves that another backend is needed.

## DSD-neo Data Flow

FobosAPP exports the narrow selected DMR channel after IQ channel selection and
FM/4FSK discriminator processing:

```text
Fobos IQ -> channel select -> FM/4FSK discriminator -> DmrDecoder
                                                     |
                                                     +-> DSD-neo UDP PCM16LE
```

This intentionally avoids sending full wideband IQ to DSD-neo. The external
decoder receives the same mono signed 16-bit PCM stream that feeds the internal
DMR lab decoder.

## GUI Controls

Open `Digital Audio`, select DMR, then set `DMR backend` to `DSD-neo`.
The DSD-neo row controls the external process:

- `Auto start` starts the configured `dsd-neo` executable.
- Program path defaults to `dsd-neo/dsd-neo.exe` in the Windows release
  layout, resolved relative to the FobosAPP executable folder.
- `UDP in 7355` is the local DSD-neo PCM input port where FobosAPP sends DMR
  PCM.
- `UDP 23456` is the local decoded-audio input where FobosAPP listens.

When the bridge is enabled without auto-start, launch DSD-neo manually and point
it at the same UDP input/output ports.

## Recommended Command

The auto-start path uses the same shape as this manual command:

```powershell
.\dsd-neo\dsd-neo.exe -fs -i udp:127.0.0.1:7355 -s 48000 -o udp:127.0.0.1:23456 -nm
```

If the DMR 4FSK baseband rate in FobosAPP is changed, FobosAPP passes that
selected value through `-s`.

## Format Contract

- Input to DSD-neo: raw PCM16LE mono UDP datagrams.
- Input sample rate: current FobosAPP DMR 4FSK baseband rate, normally 48 kHz.
- DSD-neo output back to FobosAPP: UDP raw PCM16 mono, expected 8 kHz DMR voice.
- FobosAPP upsamples returned DSD-neo voice to 48 kHz before local playback.

## GopherTrunk Virtual Dibit Source

GopherTrunk already exposes the useful DMR boundary we need: a continuous
stream of decoded dibits plus a monotonic `baseIdx`. For FobosAPP this is a
better integration point than pretending to be a generic SDR device, because
FobosAPP owns the Fobos-specific wideband IQ path, channel selection,
discriminator, symbol timing, and slicer.

The local lab adapter is:

```text
downloads/gophertrunk/GopherTrunk-src/GopherTrunk-main/cmd/fobos-dmr-virtual
downloads/gophertrunk/bin/fobos-dmr-virtual.exe
```

It accepts newline-delimited JSON over stdin or one TCP client:

```json
{"baseIdx":0,"dibits":"010310020223332030"}
{"baseIdx":18,"dibits":[0,1,0,3,1,0,0,2]}
{"reset":true}
```

Recommended manual lab command:

```powershell
.\downloads\gophertrunk\bin\fobos-dmr-virtual.exe -listen 127.0.0.1:7460 -audio-udp 127.0.0.1:23456
```

The next FobosAPP-side step is to expose the selected DMR burst stream from
`DmrDecoder::Result` and feed it to this TCP port. Send a `reset` packet on
start, stop, retune, sample-rate change, DMR lock loss, or slicer reset. The
stream should contain full 132-dibit DMR bursts (`54 payload + 24 center +
54 payload`) in over-the-air order. Do not send only the extracted AMBE payload:
GopherTrunk's interleaved decoder needs the center field and cadence to detect
the two-slot phase, 264/288-dibit stride, embedded LC, and talkgroup/radio IDs.

## Licensing Boundary

DSD-neo remains an external program. FobosAPP does not compile it into the core
application and does not require it for normal SDR, scan, GNSS, or internal DMR
metadata tests. Release packages may include instructions or a compatible
external binary only when its license and redistribution requirements are
reviewed for that release.

GopherTrunk remains an external lab tool unless its license and redistribution
requirements are reviewed for a release package. The adapter in `downloads/` is
for local integration experiments and should not be committed as-is without a
deliberate third-party notice/update.
