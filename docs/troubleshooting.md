# Troubleshooting

## Display
| Symptom | Fix |
|---|---|
| White screen | Check TFT VCC and LED on 3V3, and the CS, DC and RST wires. Run a simple colour-fill test sketch to confirm the display works. |
| Image shifted | Adjust the column and row offsets in the init code, or try a different tab variant of the display. |
| Red and blue swapped | Change `0xA0` to `0xA8` in the MADCTL command in `tftInit()`. |
| Colours inverted | Change `tftCmd(0x20...)` to `tftCmd(0x21...)`. |

## Upload
| Symptom | Fix |
|---|---|
| Flash or RAM overflow | The wrong board is selected. Choose **Nucleo F446RE**, not a smaller Nucleo. |
| "NOD_xxx not found" | Wrong board selected, or the ST-LINK drive is not mounted. |
| "No space left on device" | Delete old files on the NODE_F446RE drive, or use the STM32CubeProgrammer (SWD) upload method. |

## Signal
| Symptom | Fix |
|---|---|
| Noisy or no waveform | First run the transmitter with `TEST_CW 1` and check the 10 kHz tone. Then check the filter, ground and offset. |
| Wave looks flat-topped | The signal is clipping at the rails. Reduce the amplitude or check the op-amp supply. |
| Trace sits at the bottom or top | The DC offset is wrong. The filter output should read about 1.65 V on a multimeter. |
| Amplitude changes between frames | The receiver is not triggered on the ping. This is expected until ping triggering is added. |
| Frequency readout wrong | Check `Serial` output with `DEBUG_SERIAL 1`. `fs` should be about 1500000. |

## Transmitter
| Symptom | Fix |
|---|---|
| DMA-TIMEOUT in serial | Set `DEBUG_SERIAL 1`. Check that PA4, PA5, PA7 and PB0 are wired correctly and that the board is a Black Pill F411. |
| Distorted DAC output | The sample rate may be too high for the MCP4921. Keep `FS_HZ` at 200000 or lower. |
