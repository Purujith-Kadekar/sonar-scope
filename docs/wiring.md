# Wiring

## Transmitter: Black Pill (STM32F411CEU6)

| Function | Pin | Connects to |
|---|---|---|
| DAC chip select | PA4 | MCP4921 CS |
| SPI clock | PA5 | MCP4921 SCK |
| SPI data | PA7 | MCP4921 SDI |
| DAC latch | PB0 | MCP4921 LDAC (held low by firmware) |
| POT1 wiper | PA1 | Bandwidth / centre frequency |
| POT2 wiper | PA2 | Pulse duration |
| POT3 wiper | PA3 | Amplitude |
| KEY button | PA0 | On-board |
| LED | PC13 | On-board, active low |

MCP4921: VDD and VREF to 3.3 V, AVSS to GND, VOUTA to the filter input. Put a 100 nF capacitor across VDD and GND close to the chip.

## Receiver: NUCLEO-F446RE

The code uses Arduino names. The MCU pins are the same on the Arduino and Morpho headers.

| Function | Arduino name | MCU pin | Morpho header |
|---|---|---|---|
| Signal input | A0 | PA0 | CN7 pin 28 |
| TFT SCK | D13 | PA5 | CN10 pin 11 |
| TFT SDA (MOSI) | D11 | PA7 | CN10 pin 15 |
| TFT CS | D10 | PB6 | CN10 pin 17 |
| TFT DC (A0) | D9 | PC7 | CN10 pin 19 |
| TFT RESET | D8 | PA9 | CN10 pin 21 |
| Encoder CLK | D5 | PB4 | CN10 pin 27 |
| Encoder DT | D4 | PB5 | CN10 pin 29 |
| Encoder switch | D3 | PB3 | CN10 pin 31 |
| TFT VCC, TFT LED | 3V3 | | |
| TFT GND, encoder GND | GND | | |

Notes:
- Power the TFT from 3V3, not 5V.
- Leave TFT MISO unconnected.
- Check the pin numbers against your board's silkscreen or the Nucleo user manual (UM1724) before wiring.
- All grounds (Black Pill, Nucleo, op-amp, scope probe) must be common.
