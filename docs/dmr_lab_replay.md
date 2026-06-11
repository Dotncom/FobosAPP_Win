# DMR Lab Replay

`dmr_lab_replay` is a small offline decoder harness for repeatable DMR work.
It replays mono 16-bit PCM WAV audio recordings, or stereo 16-bit channel-IQ
WAV recordings through a lightweight DMR FM discriminator, into the same
`DmrDecoder` used by the desktop app. It captures decoder debug lines and
summarizes observed color codes, timeslots, source IDs, and target/group IDs.

Example:

```powershell
build\Desktop_x86_windows_msvc2022_pe_64bit-Release\dmr_lab_replay.exe `
  --file release\bin\recordings\test.wav `
  --expect-cc 5 `
  --expect-ts 1 `
  --expect-src 129 `
  --expect-tg 379 `
  --json release\bin\dmr_lab_summary.json
```

Clean IQ self-test:

```powershell
build\Desktop_x86_windows_msvc2022_pe_64bit-Release\dmr_lab_replay.exe `
  --synthetic-iq-self-test `
  --synthetic-iq-wav-out release\bin\recordings\synthetic_dmr_iq.wav `
  --synthetic-pcm-wav-out release\bin\recordings\synthetic_dmr_pcm.wav
```

This generates a deterministic, clean DMR-like 4FSK IQ stream with known dibits,
passes it through the offline IQ discriminator, slices the resulting 48 kHz PCM
back into 4800 symbols/s, and reports symbol errors plus the recovered 4FSK
levels. It validates the IQ-to-symbol path; it is not intended to generate valid
AMBE voice or valid service metadata.

If a recording has the app's `fbos` metadata chunk and DMR Lab was enabled
during recording, the tool reads expected values automatically. Command-line
expectations override missing metadata values.

Current scope:

- Supported input: mono 16-bit PCM audio WAV and stereo 16-bit channel-IQ WAV.
- Channel-IQ replay uses a lightweight offline discriminator. It is meant for
  repeatable lab comparison, not as a replacement for the live audio chain.
- Use `--qt-debug` when low-level `voice lc stream/pooled` debug lines should
  be printed during replay.
- Use `--pcm-out output.s16le` to dump the exact 48 kHz PCM stream that was
  passed to `DmrDecoder`; this is the first bridge point for external tools.

Next bridge target:

- Feed the same mono PCM stream to an external decoder process or TCP input.
- Compare the external CC/TS/SRC/TG output against this replay summary.
