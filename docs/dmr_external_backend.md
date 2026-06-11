# DMR External Decoder Bridge Skeleton

FobosAPP 4.0 can load optional `fobos_dmr_voice_*.dll` voice backend modules
through the C ABI documented in `FobosDMRVoiceBackend-gpl/include`. This note is
for the next, separate bridge: mirroring controlled DMR lab data to external
decoder programs so captures can be compared against mature implementations.

## Candidate Backends

- `dsd-neo` is the first practical bridge target. Its README documents generic
  TCP PCM16LE input, UDP audio input/output, DMR mono helpers (`-fs -nm`), and a
  DMR trunking workflow.
- `GopherTrunk` is more useful as a scanner/trunking architecture reference. It
  is a larger headless scanner engine with DMR, P25, TETRA, NXDN, and other
  protocol support, but it is not the shortest path for a drop-in DMR verifier.

## Local App Boundary

For external decoder comparison, FobosAPP should export the narrow selected DMR
channel after the same IQ FM/4FSK front-end that feeds `DmrDecoder`:

```text
Fobos IQ -> channel select -> FM/4FSK discriminator -> DmrDecoder
                                                     |
                                                     +-> optional external bridge
```

This is deliberately after channel filtering and frequency control. Sending full
wideband IQ to external tools would duplicate our SDR front-end, make tests less
stable, and create another frequency-control problem.

## Initial Config Shape

The app-side settings should be able to represent:

- backend disabled/internal only
- external process with arguments
- TCP PCM output host/port
- UDP PCM output host/port
- expected PCM sample rate, initially 48000 Hz
- whether external lines are shown in the Digital Audio pane
- whether external results are used only for lab comparison

The example file is `config/dmr_backends.example.json`.

## First Integration Steps

1. Keep using `dmr_lab_replay` for deterministic local regression.
2. Add a small PCM16LE streamer class shared by replay and GUI.
3. Add a GUI checkbox: internal only / mirror PCM to external backend.
4. Compare external CC/TS/SRC/TG against DMR Lab metadata.
5. Only after metadata is stable, decide whether decoded voice audio should be
   routed back into FobosAPP.
