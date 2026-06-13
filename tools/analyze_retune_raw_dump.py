#!/usr/bin/env python3
"""Inspect FobosAPP retune raw IQ dumps.

The companion .json file describes the .f32iq payload saved by the app.
The raw payload is little-endian float32 interleaved I,Q exactly as received
by DataProcessor::handleData before IqBuffer::publish().
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def top_peaks(power_db: np.ndarray, sample_rate: float, count: int) -> list[tuple[float, float]]:
    if power_db.size == 0:
        return []
    guard = max(1, power_db.size // 500)
    candidates = np.argpartition(power_db, -min(count * 8, power_db.size))[-min(count * 8, power_db.size):]
    candidates = candidates[np.argsort(power_db[candidates])[::-1]]
    selected: list[int] = []
    for idx in candidates:
        if all(abs(int(idx) - prev) > guard for prev in selected):
            selected.append(int(idx))
            if len(selected) >= count:
                break
    center = power_db.size // 2
    bin_hz = sample_rate / float(power_db.size)
    return [((idx - center) * bin_hz, float(power_db[idx])) for idx in selected]


def analyze(path: Path, peak_count: int) -> None:
    if path.suffix.lower() == ".f32iq":
        json_path = path.with_suffix(".json")
    else:
        json_path = path
    with json_path.open("r", encoding="utf-8") as handle:
        meta = json.load(handle)

    raw_path = json_path.with_name(meta.get("rawFile", json_path.with_suffix(".f32iq").name))
    sample_rate = float(meta.get("sampleRateHz") or 0.0)
    if sample_rate <= 0.0:
        raise SystemExit("sampleRateHz is missing from metadata")

    raw = np.fromfile(raw_path, dtype="<f4")
    if raw.size < 2:
        raise SystemExit(f"No IQ samples in {raw_path}")

    blocks = meta.get("blocks") or []
    print(f"file: {raw_path}")
    print(f"sampleRateHz: {sample_rate:.3f}")
    print(f"previousCenterHz: {float(meta.get('previousCenterHz', 0.0)):.3f}")
    print(f"requestedCenterHz: {float(meta.get('requestedCenterHz', 0.0)):.3f}")
    print(f"blocksCaptured: {meta.get('blocksCaptured')} status: {meta.get('status')}")
    print()

    offset_floats = 0
    for block in blocks:
        sample_count = int(block.get("sampleCount") or 0)
        float_count = sample_count * 2
        if sample_count <= 0 or offset_floats + float_count > raw.size:
            break
        iq_pairs = raw[offset_floats:offset_floats + float_count].reshape((-1, 2))
        iq = iq_pairs[:, 0].astype(np.float32) + 1j * iq_pairs[:, 1].astype(np.float32)
        window = np.hanning(iq.size).astype(np.float32)
        spectrum = np.fft.fftshift(np.fft.fft(iq * window))
        power_db = 20.0 * np.log10(np.maximum(np.abs(spectrum), 1.0e-20))
        peaks = top_peaks(power_db, sample_rate, peak_count)
        peak_text = ", ".join(f"{hz / 1e6:+.6f} MHz/{db:.1f} dB" for hz, db in peaks)
        print(
            "block {idx:02d} cb {cb:.0f} epoch {epoch:.0f} "
            "centerHint {center:.3f} MHz rms {rms:.6g} phase {phase:+.6f}: {peaks}".format(
                idx=int(block.get("index", 0)),
                cb=float(block.get("callback", 0.0)),
                epoch=float(block.get("epoch", 0.0)),
                center=float(block.get("centerFrequencyHint", 0.0)) / 1e6,
                rms=float(block.get("rms", 0.0)),
                phase=float(block.get("phaseStepMeanRad", 0.0)),
                peaks=peak_text,
            )
        )
        offset_floats += float_count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dump", type=Path, help="Path to .json or .f32iq retune dump")
    parser.add_argument("--peaks", type=int, default=5, help="Number of peaks per block")
    args = parser.parse_args()
    analyze(args.dump, max(1, args.peaks))


if __name__ == "__main__":
    main()
