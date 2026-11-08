# Circuits

## Nucleo input protection (RC filter and Schottky clamp)

```
                                 +3.3 V
                                   |
                               [Cathode]
                                D1 BAT85
                               [Anode]
                                   |
 filter output --[ R1 1k ]---------+--------- Nucleo A0 (PA0)
                                   |
                               [Cathode]
                                D2 BAT85
                               [Anode]
                                   |
                                  GND        C1 = 1 nF from the A0 node to GND
```

- Corner frequency: 1 / (2 * pi * 1 kOhm * 1 nF) = about 159 kHz. At 30 kHz the loss is about 0.15 dB.
- The signal must stay within 0 to 3.3 V. Keep a DC offset of about 1.65 V.
- Keep R1, C1 and the diodes close to the A0 pin.
- At 1.5 MS/s a 1 kOhm source is at the limit for the ADC. A unity-gain op-amp buffer after the filter improves accuracy.

## Transmitter output

```
Black Pill --SPI--> MCP4921 VOUTA --> R --> Sallen-Key low-pass (op-amp) --> output
```

- The DAC output is 0 to 3.3 V, centred on about 1.65 V.
- Use a rail-to-rail op-amp that runs from 3.3 V (MCP6002, TLV272 or OPA2342). The LM358 cannot reach the top of a 3.3 V rail.
- The op-amp needs a slew rate of at least 2 * pi * f * Vpk. For 30 kHz and 1.65 V peak, that is about 0.3 V/us.
- Put a 100 nF capacitor across each supply pin.
