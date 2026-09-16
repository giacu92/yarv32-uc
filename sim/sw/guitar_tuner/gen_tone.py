#!/usr/bin/env python3
"""Test-tone generator for the guitar tuner: a steady note that detunes slowly.

WHY THIS AND NOT A CHIRP.  The tuner analyses a 1024-sample window at
1953.125 Hz, so one estimate covers 524 ms and one bin is 1.907 Hz.  For the
peak to stay inside a bin for the length of a window the frequency may move
no faster than about 3.6 Hz/s; past that the peak smears over several bins,
loses height, and raises the mean of the search band -- which is exactly what
the estimator's SNR gate measures.  A chirp fast enough to watch is already
several times over that limit (one semitone per second at E4 is 19.6 Hz/s,
5.4x), so it makes the tuner flicker for reasons that have nothing to do with
the tuner.  A note that drifts a few cents per second is the real use case: a
string going out of tune, or a peg being turned.

The default sweeps +/-50 cents -- the full width of the gauge -- over 30
seconds, which at E2 is 0.16 Hz/s, forty times slower than the smear limit.
--verify reports the actual rate against that limit, so a changed duration
cannot silently cross it.

Harmonics matter as much as the frequency.  The tuner deliberately does not
pick the loudest bin, because a bright pickup routinely makes the 2nd or 3rd
partial louder than the fundamental and a tuner that reports the octave is
worse than none; --harmonics bright reproduces that case on real hardware.

Frequency is integrated into phase rather than substituted into sin(2*pi*f*t),
which is the usual mistake: the latter jumps phase whenever f changes and
sprays clicks across the spectrum.

No dependencies beyond the standard library.

Examples:
    ./gen_tone.py                                  # E2, +/-50 cents in 30 s
    ./gen_tone.py --note A2 --cents -20 20 --seconds 45
    ./gen_tone.py --mode steps --dwell 4           # hold each value, 10 steps
    ./gen_tone.py --note E4 --harmonics bright --verify
"""

import argparse
import math
import struct
import wave

# Equal temperament, A4 = 440 Hz.  Standard guitar tuning plus the rest of
# the tuner's range, so a note can be named instead of typed as a frequency.
A4_HZ = 440.0
NOTE_SEMITONES = {
    "C": -9, "C#": -8, "D": -7, "D#": -6, "E": -5, "F": -4,
    "F#": -3, "G": -2, "G#": -1, "A": 0, "A#": 1, "B": 2,
}

# What the firmware does, mirrored here so --verify can report against it.
TUNER_FS = 15625.0 / 8.0      # analysis rate after the CIC decimator
TUNER_N = 1024                # transform size
TUNER_BIN_HZ = TUNER_FS / TUNER_N
TUNER_WINDOW_S = TUNER_N / TUNER_FS
TUNER_SMEAR_LIMIT = TUNER_BIN_HZ / TUNER_WINDOW_S   # Hz/s

HARMONIC_SETS = {
    # Fundamental only: the easy case, and the one that says nothing about
    # whether the fundamental picker works.
    "pure": [1.0],
    # Roughly what a plucked string gives an inch from the soundhole.
    "guitar": [1.0, 0.55, 0.35, 0.22, 0.12],
    # The trap: the third partial is the loudest thing in the spectrum, so a
    # build that picks the largest bin reports an octave and a fifth up.
    "bright": [0.45, 0.50, 1.0, 0.30, 0.15],
}


def note_to_hz(name):
    """'E2' -> 82.407 Hz.  Accepts sharps ('C#3'), not flats."""
    name = name.strip().upper()
    for length in (2, 1):                 # try 'C#' before 'C'
        if name[:length] in NOTE_SEMITONES:
            letter, octave = name[:length], name[length:]
            break
    else:
        raise ValueError("bad note name: %r" % name)
    if not octave.lstrip("-").isdigit():
        raise ValueError("bad octave in %r" % name)
    semitones = NOTE_SEMITONES[letter] + (int(octave) - 4) * 12
    return A4_HZ * (2.0 ** (semitones / 12.0))


def cents_at(t, duration, cents_from, cents_to, mode, dwell):
    """Detuning, in cents, at time t."""
    if mode == "steps":
        steps = max(1, int(round(duration / dwell)))
        k = min(int(t / dwell), steps - 1)
        if steps == 1:
            return cents_from
        return cents_from + (cents_to - cents_from) * k / (steps - 1)
    return cents_from + (cents_to - cents_from) * (t / duration)


def render(f0, duration, rate, cents_from, cents_to, mode, dwell,
           harmonics, amplitude, fade):
    """Phase-continuous synthesis.  Returns a list of floats in [-1, 1]."""
    n = int(duration * rate)
    weights = HARMONIC_SETS[harmonics]
    norm = sum(weights)
    # Phase accumulator per partial: integrating the instantaneous frequency
    # is what keeps the waveform continuous when the frequency moves.
    phases = [0.0] * len(weights)
    out = []

    for i in range(n):
        t = i / rate
        f = f0 * (2.0 ** (cents_at(t, duration, cents_from, cents_to,
                                   mode, dwell) / 1200.0))
        s = 0.0
        for h, w in enumerate(weights):
            phases[h] += 2.0 * math.pi * f * (h + 1) / rate
            s += w * math.sin(phases[h])
        s /= norm

        # Raised-cosine fade, so the file neither starts nor ends on a step.
        if fade > 0.0:
            if t < fade:
                s *= 0.5 * (1.0 - math.cos(math.pi * t / fade))
            elif t > duration - fade:
                s *= 0.5 * (1.0 - math.cos(math.pi * (duration - t) / fade))

        out.append(amplitude * s)
        # Keep the accumulators bounded on long files.
        if i % rate == 0:
            phases = [p % (2.0 * math.pi) for p in phases]

    return out


def write_wav(path, samples, rate):
    frames = bytearray()
    for s in samples:
        v = int(round(max(-1.0, min(1.0, s)) * 32767.0))
        frames += struct.pack("<h", v)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(bytes(frames))


def goertzel(samples, rate, freq):
    """Magnitude at one frequency.  Cheaper than a DFT and enough to check
    that the file contains what it claims to."""
    n = len(samples)
    k = 2.0 * math.cos(2.0 * math.pi * freq / rate)
    s1 = s2 = 0.0
    for x in samples:
        s0 = x + k * s1 - s2
        s2, s1 = s1, s0
    return math.sqrt(s1 * s1 + s2 * s2 - k * s1 * s2) / n


def verify(samples, rate, f0, duration, cents_from, cents_to, mode, dwell):
    """Measure the detuning the file actually carries, at three points, by
    scanning the fundamental's neighbourhood.  This is a check on the
    generator, not on the tuner."""
    print("verification (generator's own, by Goertzel scan):")
    win = int(TUNER_WINDOW_S * rate)

    # Where to look.  In steps mode the windows go in the MIDDLE of a
    # dwell: a 524 ms window that straddles a step average two frequencies
    # and reports something that matches neither, which is the measurement
    # being right about a question nobody asked.
    if mode == "steps":
        steps = max(1, int(round(duration / dwell)))
        picks = sorted({0, steps // 2, steps - 1})
        centres = [(k + 0.5) * dwell for k in picks]
    else:
        centres = [0.15 * duration, 0.5 * duration, 0.85 * duration]

    for t in centres:
        start = int(t * rate - win / 2.0)
        start = max(0, min(start, len(samples) - win))
        t_mid = (start + win / 2.0) / rate
        want = cents_at(t_mid, duration, cents_from, cents_to, mode, dwell)
        chunk = samples[start:start + win]

        best_c, best_m = 0.0, -1.0
        c = want - 15.0
        while c <= want + 15.0:            # 0.5-cent grid around the target
            m = goertzel(chunk, rate, f0 * (2.0 ** (c / 1200.0)))
            if m > best_m:
                best_m, best_c = m, c
            c += 0.5
        flag = "" if abs(best_c - want) <= 1.0 else "   <-- off"
        print("  t = %5.1f s: want %+6.1f cents, measured %+6.1f%s" %
              (t_mid, want, best_c, flag))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-o", "--out", default="tone.wav")
    p.add_argument("--note", default="E2", help="note name, e.g. E2 A2 D3 G3 B3 E4")
    p.add_argument("--freq", type=float, help="explicit frequency, overrides --note")
    p.add_argument("--seconds", type=float, default=30.0)
    p.add_argument("--rate", type=int, default=48000)
    p.add_argument("--cents", type=float, nargs=2, default=(-50.0, 50.0),
                   metavar=("FROM", "TO"))
    p.add_argument("--mode", choices=("ramp", "steps"), default="ramp",
                   help="ramp: continuous drift. steps: hold each value")
    p.add_argument("--dwell", type=float, default=3.0,
                   help="seconds per value in steps mode")
    p.add_argument("--harmonics", choices=sorted(HARMONIC_SETS), default="guitar")
    p.add_argument("--amplitude", type=float, default=0.35,
                   help="0..1; leaves headroom so a speaker does not clip")
    p.add_argument("--fade", type=float, default=0.25, help="fade in/out, seconds")
    p.add_argument("--verify", action="store_true")
    a = p.parse_args()

    f0 = a.freq if a.freq else note_to_hz(a.note)
    label = "%.3f Hz" % f0 if a.freq else "%s (%.3f Hz)" % (a.note.upper(), f0)

    # The rate check, which is the whole reason this is not a chirp.
    span_hz = abs(f0 * (2.0 ** (a.cents[1] / 1200.0) - 2.0 ** (a.cents[0] / 1200.0)))
    rate_hz_s = span_hz / a.seconds if a.mode == "ramp" else 0.0

    print("note        %s" % label)
    print("detune      %+.1f to %+.1f cents over %.1f s (%s)" %
          (a.cents[0], a.cents[1], a.seconds, a.mode))
    print("harmonics   %s %s" % (a.harmonics, HARMONIC_SETS[a.harmonics]))
    if a.mode == "ramp":
        print("sweep rate  %.3f Hz/s  (tuner smears past %.1f Hz/s -- %s)" %
              (rate_hz_s, TUNER_SMEAR_LIMIT,
               "fine" if rate_hz_s < TUNER_SMEAR_LIMIT / 2.0 else "TOO FAST"))
    else:
        print("sweep rate  0 (held), %d steps of %.1f s" %
              (max(1, int(round(a.seconds / a.dwell))), a.dwell))

    samples = render(f0, a.seconds, a.rate, a.cents[0], a.cents[1],
                     a.mode, a.dwell, a.harmonics, a.amplitude, a.fade)
    write_wav(a.out, samples, a.rate)
    print("wrote       %s (%d samples, %.1f s, %d Hz mono 16-bit)" %
          (a.out, len(samples), a.seconds, a.rate))

    if a.verify:
        verify(samples, a.rate, f0, a.seconds, a.cents[0], a.cents[1],
               a.mode, a.dwell)


if __name__ == "__main__":
    main()
