# IQ Pipeline Audit Notes

This note tracks the current working theory for DMR/GNSS synchronization failures and the likely optimization points in the receiver path.

## Current Observation

The new `[IQ stream]` diagnostics from the 2026-06-13 GNSS test do not show a catastrophic USB/IQ continuity failure:

1. `blockSamples` stayed fixed at 65536.
2. `blockMin` and `blockMax` stayed fixed at 65536.
3. `measuredRate` stayed very close to 50 MS/s after startup.
4. `lateCallbacks` stayed at 0.
5. `clipPercent` stayed at 0.
6. `nonFinitePercent` stayed at 0.
7. The live queue depth moved but did not grow without bound.

This does not prove that every downstream consumer sees a perfect continuous stream, but it makes a basic Fobos/libusb block-drop problem less likely for this test.

The 2026-06-13 synthetic GPS self-test also found the injected PRN immediately in terms of correctness (`prnMatch true`, metric about 25.8 dB), but the full search still took about 113 seconds. That points to performance/architecture work in acquisition rather than a pure math failure.

## Remaining Suspect Areas

1. Channelizer and resampler correctness:
   - verify frequency-shift sign with real offset test tones, not only synthetic GNSS;
   - verify I/Q order and conjugation assumptions with a known positive/negative offset source;
   - compare FobosAPP channelized output against a trusted external tool.
2. Receiver spurs and interference:
   - the waterfall showed stable narrow lines near the GNSS band;
   - true GPS L1 C/A is spread-spectrum and often below the visible noise floor;
   - fixed narrow peaks are more likely local spurs, USB/power/display noise, or external interference than GPS satellites.
3. GNSS acquisition model:
   - current GPS C/A search uses 4.092 MS/s, PRN/code-phase/Doppler search, and 1 ms correlation power accumulation;
   - the variable name `coherentMs` is misleading because the implementation accumulates correlation power per millisecond;
   - the 160 ms deep scan is computationally expensive and took about 132 seconds on the Steam Deck in the latest test.
4. GNSS signal level:
   - the reported IQ RMS around -55.7 dBFS may be valid, but a bicone antenna without an active GNSS LNA/filter is a hard RF condition;
   - phone GNSS lock is not a fair comparison because phones use assisted GNSS, dedicated RF front ends, filtering, low-noise amplification, and multi-constellation receivers.
5. DMR-specific path:
   - if raw IQ is stable, DMR issues are more likely in channel filtering, 4FSK discriminator shaping, symbol timing, burst framing, or AMBE bit ordering;
   - the DMR path still needs a stage-by-stage offline harness before further voice tuning.

## Optimization Ideas

1. Reduce GNSS Doppler search cost:
   - start with narrower Doppler windows when assisted time/location/known PRNs are available;
   - use a coarse search first, then refine only the top candidates.
2. Cache repeated acquisition data:
   - precompute and cache GPS C/A FFTs per PRN;
   - precompute or recursively update Doppler wipe-off oscillators instead of calling `sin/cos` for every sample.
3. Limit expensive deep scans:
   - default live acquisition should stay short and responsive;
   - deep scans should run as offline/replay jobs with progress and cancellation.
4. Add acquisition dump files:
   - save PRN/Doppler/code-phase grids for a recording; implemented as `recordings/gnss_reports/gnss_acq_*` JSON/CSV artifacts;
   - compare these dumps with GNSS-SDR or another reference implementation.
5. Add spur diagnostics:
   - average the spectrum in GNSS mode;
   - log stable narrow peaks and optionally notch them before acquisition;
   - show expected GPS L1 C/A spread-spectrum bandwidth on the plot.
6. Clarify internal IQ contracts:
   - pass sequence/timing metadata with IQ frames where practical;
   - separate waterfall snapshots from decoder/acquisition streams;
   - keep decoder input paths explicit so DMR/GNSS/video do not silently consume stale or discontinuous snapshots.

## Next Practical Tests

1. Feed a known complex test tone through the live channelizer and verify positive/negative offset sign.
2. Record GNSS IQ with the current bicone setup, then run FobosAPP acquisition offline with the `GPS replay` button and compare with GNSS-SDR.
3. Test with a GNSS active antenna plus bias tee or an external GNSS LNA/filter if available.
4. Repeat DMR with the same IQ stream diagnostics enabled and check whether the raw stream remains stable during voice tests.
