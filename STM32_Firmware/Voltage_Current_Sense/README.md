# Voltage / Current Sense Node

DroneCAN node for the voltage/current measurement PCB. It measures pack voltage and total current draw and reports them over CAN as a `uavcan.equipment.power.BatteryInfo` message.

**Known issue:** both the voltage and current readings appear to be scaled incorrectly — see [Calibration](#calibration--known-issue) below before trusting the numbers this board reports.

## Hardware

- MCU: STM32G431CBTx
- **Voltage**: a Texas Instruments AMC1303/AMC130M03-style isolated delta-sigma ADC, read over SPI1 with an interrupt-driven DRDY (data-ready) pin — driver in [`Library/AMC130M03/`](Library/AMC130M03/AMC130M03.h). `TIM1` CH1 generates the 8 MHz CLKIN the ADC needs.
- **Current**: the TI current-sense IC's analog output is read directly by the internal `ADC1` (3 channels, DMA, continuous conversion) — see `current_raw[3]` in [`Core/Src/main.c`](Core/Src/main.c).
- FDCAN1 for DroneCAN
- Red/green status LEDs

Only one pack voltage channel and the sum of the current channels are currently wired up and sent — per-cell voltage and per-channel current are not yet broken out individually.

## Calibration / known issue

The raw-to-real conversions live in [`Core/Inc/main.h`](Core/Inc/main.h) and [`Core/Src/main.c`](Core/Src/main.c):

```c
#define vdiv_r1 604          // voltage divider R1
#define vdiv_r2 10           // voltage divider R2
#define adc_max 4096.0f      // 12-bit ADC full scale
#define current_sense_min -9.2f
#define current_sense_max 115.0f
#define voltage_read 5       // samples averaged before each CAN send
```

- `inverse_voltage_divider()` scales the AMC130M03's volts-at-the-ADC reading back up by `(r1+r2)/r2`.
- `calculate_current()` maps the raw 12-bit ADC count linearly across `[current_sense_min, current_sense_max]` amps.

If you're chasing the known scaling bug, start by checking the actual resistor divider ratio on the board against `vdiv_r1`/`vdiv_r2`, and the current sensor's real output range against `current_sense_min`/`current_sense_max` — these are the two most likely places the calibration has drifted from the hardware.

## Data flow

1. `AMC130M03_DRDY_Callback` (EXTI) kicks off a non-blocking SPI DMA read when the external ADC has new data; `AMC130M03_SPI_CpltCallback` finishes it and stores the result in `g_adc.latest`.
2. `ADC1` continuously DMAs its 3 current channels into `current_raw[]` in the background.
3. `StartMeasurePower` wakes every 10 ms, takes a critical-section snapshot of both, and accumulates running sums.
4. Every `voltage_read` (5) cycles it averages the accumulated samples, converts them with the formulas above, and calls `send_battery_info(voltage, current)` to broadcast over DroneCAN.

## FreeRTOS tasks

| Task | Rate / trigger | Purpose |
|---|---|---|
| `CanRx` | On FDCAN RX interrupt | Drains the hardware RX FIFO into libcanard and processes incoming transfers |
| `CanTx` | Notified by Rx/Status tasks | Flushes libcanard's TX queue onto the bus |
| `CanStatus` | 1 Hz | Runs libcanard's 1 Hz housekeeping |
| `MeasurePower` | 100 Hz (10 ms) | Samples/averages the ADCs and sends `send_battery_info()` every 5th cycle (~20 Hz) |
| `LedBlink` | 1 Hz | Green heartbeat LED |
| `ErrorLED` | 1 Hz poll | Red LED on if either the voltage or current averaging buffer came up empty (`error` bits 0/1), or if `AMC130M03_Init()` failed at boot |

Unlike the other nodes in this repo, this project does not start CAN jitter before `LedBlink`/`CanStatus` — see [`main.c`](Core/Src/main.c) if cross-talk with other boards on the bus becomes an issue (see the note in the [STM32_Firmware README](../README.md#issues--solutions)).

## Building and flashing

Standard STM32CubeMX-generated CMake project:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Flash with ST-Link (STM32CubeProgrammer, OpenOCD, or the STM32 VS Code extension).

## See also

- [STM32_Firmware README](../README.md) — how this project fits with the others
- [Drone_Can library README](../Libraries/Drone_Can/README.md) — the shared CAN stack this node is built on
