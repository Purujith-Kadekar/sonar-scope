# Expo Talking Points

## One-line summary
A software-defined sonar transmitter and a mini oscilloscope that measures it, both on STM32 boards.

## Signal chain
1. Pots set the pulse parameters (bandwidth, length, amplitude).
2. The Black Pill builds the ping in software.
3. Timer and DMA send each sample to the MCP4921 DAC with no CPU involvement.
4. A Sallen-Key low-pass filter smooths the DAC steps.
5. The Nucleo samples at 1.5 MS/s and measures frequency, period, wavelength and Vpp.
6. The TFT shows the wave on a grid, with the stats below.

## Design decisions
- **Zero-CPU transmit:** three DMA streams per sample (CS low, data, CS high) give exact timing.
- **Three modulations:** LFM chirp, geometric sweep, phase-coded BPSK.
- **Three windows:** Hann, Hamming, Blackman, to reduce edge ripple.
- **Adaptive pings:** the pots change the pulse every 100 ms.
- **Noise-tolerant measurement:** hysteresis crossings with interpolation, averaged over about 2.7 ms.
- **Low-RAM display:** drawn in 8-row bands between captures.

## Numbers
| Item | Value |
|---|---|
| Transmit rate | 200 kS/s |
| Receive rate | 1.5 MS/s |
| Capture length | 4096 samples (about 2.7 ms) |
| Frequency range (demo) | 5 to 30 kHz |
| Pulse length | 2 to 20 ms |
| Ping interval | 100 ms |
| 30 kHz period | 33.3 us |
| 30 kHz wavelength | 5.0 cm in water, 1.14 cm in air |

## Be upfront about
- Frequencies are scaled down from real sonar.
- The receiver is not triggered on the ping yet.
- No transducer is connected.
- Readings are not calibrated against a reference.

## Questions you may get
- **Why not the STM32 internal DAC?** The F411 has none, so an external MCP4921 is used.
- **How is a negative half of the sine produced?** The wave is centred on 1.65 V. The part below that is the "negative" half.
- **Why does the amplitude change between frames?** Pings are short and the receiver is not yet triggered on them. The window also tapers each ping.
- **What would you improve?** Ping-triggered capture, a rail-to-rail op-amp, an input buffer, a matched transducer, and calibration.

## Demo order
1. Show the steady 10 kHz tone with `TEST_CW 1`.
2. Switch to pings and turn the pots.
3. Change modulation and window with the button.
4. Change the scope gain with the encoder.
5. Explain the stats and the limits.
