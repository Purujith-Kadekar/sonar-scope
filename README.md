# SONAR Transmitter and Mini Oscilloscope

A software-defined sonar demo built from two STM32 boards.

- **Transmitter** (Black Pill, STM32F411): builds a windowed, modulated ping in software, plays it out through an MCP4921 DAC using timer + DMA (no CPU involvement), and filters it with a Sallen-Key low-pass filter.
- **Receiver / oscilloscope** (NUCLEO-F446RE): samples the signal at 1.5 MS/s, measures frequency, period, wavelength and Vpp, and draws the wave on a 1.8" ST7735 TFT. A rotary encoder changes the vertical gain.

> Frequencies are scaled down to 5 to 30 kHz for bench components. Real sonar uses roughly 100 to 500 kHz. The architecture is the same.

```
Black Pill --SPI--> MCP4921 --> Sallen-Key LPF --> 1k + Schottky clamp --> Nucleo A0
 (pots, button)      (DAC)       (op-amp)                                  (ADC 1.5 MS/s -> TFT)
```

## Repository layout

```
firmware/
  transmitter_blackpill/transmitter_blackpill.ino
  receiver_nucleo/receiver_nucleo.ino
docs/
  wiring.md          pin tables for both boards
  circuit.md         filter and input-protection circuit
  troubleshooting.md common problems and fixes
  expo_notes.md      talking points for a demo or expo
  media/             test videos
```

## Hardware

| Part | Used for |
|---|---|
| WeAct Black Pill STM32F411CEU6 | Transmitter |
| MCP4921 12-bit SPI DAC | Analog output |
| Rail-to-rail dual op-amp (MCP6002, TLV272 or OPA2342) | Sallen-Key filter |
| 3 x 10 kOhm potentiometers | Simulated environment sensors |
| NUCLEO-F446RE | Receiver / scope |
| 1.8" ST7735 TFT (128 x 160, SPI) | Display |
| Rotary encoder with push switch | Vertical gain |
| 1 kOhm resistor, 1 nF capacitor, 2 x BAT85 Schottky diodes | Nucleo input filter and clamp |

Pin tables are in [docs/wiring.md](docs/wiring.md) and the circuits are in [docs/circuit.md](docs/circuit.md).

## Build and upload (Arduino IDE)

1. Add this URL under **File > Preferences > Additional boards manager URLs**:
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stm_index.json`
2. Open **Tools > Board > Boards Manager**, search for **STM32 MCU based boards**, and install it.
3. No extra libraries are needed.

**Receiver (Nucleo)**
- Board: **Nucleo-64**, part number **Nucleo F446RE**
- Optimize: **Smallest code (-Os)**
- Open `firmware/receiver_nucleo/receiver_nucleo.ino` and upload.

**Transmitter (Black Pill)**
- Board: **Generic STM32F4 series**, part number **BlackPill F411CE**
- Upload method: **STM32CubeProgrammer (DFU)** or **(SWD)**, depending on how you connect it
- Open `firmware/transmitter_blackpill/transmitter_blackpill.ino` and upload.

> These sketches were written for the boards above. If you get a build error, open an issue with the first error line.

## How to use

### Test mode first
The transmitter has `#define TEST_CW 1` at the top. It sends a steady 10 kHz tone with no pings. Use it to check the whole chain:

1. Upload the transmitter with `TEST_CW 1`.
2. Connect the filter output to Nucleo A0 through the 1 kOhm + clamp network.
3. The Nucleo should show about **10.0 kHz**, period about **100 us**.
4. Set `TEST_CW 0` and upload again for the normal pings.

### Transmitter controls
| Input | Action |
|---|---|
| POT1 (PA1) | Bandwidth and centre frequency (5 to 25 kHz bandwidth) |
| POT2 (PA2) | Pulse length (2 to 20 ms) |
| POT3 (PA3) | Amplitude (10 to 100 % of full scale) |
| KEY, short press | Modulation: LFM, geometric sweep, phase-coded (BPSK) |
| KEY, long press | Window: Hann, Hamming, Blackman |
| LED | Flashes 1, 2 or 3 times to confirm the selection |

### Receiver controls
| Input | Action |
|---|---|
| Encoder turn | Vertical gain x0.5, x1, x2, x4, x8 |
| Encoder press | Reset gain to x1 |

### Display
The wave is at the top with a grid. The bottom shows:
- `F:` frequency
- `T:` time period
- `WL:` wavelength (speed of sound divided by frequency)
- `Vpp:` peak-to-peak voltage

Edit `WAVE_SPEED` in the receiver: 1500 m/s for water or 343 m/s for air.

## How it works

### Transmitter
- A timer runs at 200 kS/s. Three timer events per sample trigger three DMA streams: chip-select low, the 16-bit SPI data word, and chip-select high. The DAC updates on the chip-select rising edge.
- The CPU only builds the next ping (from a sine lookup table) and then sleeps.
- The sine is centred on DAC code 2048 (about 1.65 V). The MCU cannot make a negative voltage, so the "negative" half is the part below 1.65 V.
- Pot readings change the pulse every 100 ms. This is the adaptation logic.

### Receiver
- TIM2 triggers ADC1 at 1.5 MS/s (3-cycle sampling plus 12-cycle conversion at 22.5 MHz), and DMA stores 4096 samples per capture.
- Frequency is found by counting mid-level crossings with hysteresis and interpolating the crossing times, then averaging across the capture.
- The display is drawn in 8-row bands to save RAM, and only between captures so SPI activity does not disturb the ADC.

## Known limitations

- The receiver captures continuously and is **not triggered on the ping**. A 2 to 20 ms ping every 100 ms is caught only by chance, and the display shows low amplitude between pings. A trigger on the ping's rising edge is the next step.
- At 1.5 MS/s the ADC needs a low-impedance source. A 1 kOhm filter is at the limit, so a unity-gain buffer in front of A0 improves accuracy.
- The LM358 cannot swing to a 3.3 V rail. Use a rail-to-rail op-amp.
- The MCP4921 needs about 4.5 us to settle, which limits the transmit rate to about 200 kS/s.
- Voltage, frequency and wavelength readings are not calibrated against a reference instrument.
- There is no transducer yet. The output drives the filter only.
- These sketches have not been run through a full test on every board revision. Treat them as a working starting point.

## Testing

Bench test recordings are in [`docs/media/`](docs/media/).

| Video | What it shows |
|---|---|
| [Test 1 - waveform capture](docs/media/test_01.mp4) | Receiver display showing the captured wave with F, T, WL and Vpp readings |
| [Test 2 - encoder gain](docs/media/test_02.mp4) | Rotary encoder changing the vertical gain on the scope |

Test setup: transmitter on the Black Pill in `TEST_CW` mode (steady 10 kHz tone), filter output into Nucleo A0 through the 1 kOhm + clamp network. See [docs/circuit.md](docs/circuit.md).

Expected readings in this mode: about 10.0 kHz, period about 100 us, wavelength about 15 cm in water.

## Troubleshooting

See [docs/troubleshooting.md](docs/troubleshooting.md).

## License

MIT. See [LICENSE](LICENSE).
