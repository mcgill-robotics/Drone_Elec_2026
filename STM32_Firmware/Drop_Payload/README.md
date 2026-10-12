# Drop Payload Node

DroneCAN node for the payload-release PCB. It drives up to three continuous-rotation release servos so the drone can carry and independently drop up to three payloads.

## Hardware

- MCU: STM32G431CBTx
- 3 PWM outputs: `TIM3` CH1, `TIM2` CH2, `TIM2` CH1 (local channels 0, 1, 2 respectively — see `write_servo()` in [`Core/Src/main.c`](Core/Src/main.c))
- FDCAN1 for DroneCAN
- Red/green status LEDs

## DroneCAN configuration

Set in [`Library/can_config.h`](Library/can_config.h):

| Setting | Value |
|---|---|
| `USE_SERVO` | defined |
| `SERVO_COUNT` | 3 |
| `SERVO_CHANNEL_OFFSET` | 0 — reads global CAN servo indices 0–2 |
| `USE_ESC` | commented out (not used) |

See the [Drone_Can README](../Libraries/Drone_Can/README.md) for what the offset/count scheme means and how to wire a new node against the shared library.

## Release logic

Unlike a normal proportional servo, these are driven **bang-bang** (spin / spin-the-other-way / stop) rather than mapped linearly, because the release mechanism is a continuous-rotation servo used as an actuator, not a positioner. For each of the 3 channels, the incoming CAN servo command (range 1000–2000) is compared against thresholds (see [`Core/Inc/main.h`](Core/Inc/main.h)):

| CAN command | Output |
|---|---|
| `> 1650` (`SERVO_UPPER_LIMIT`) | Spin clockwise (PWM pulse `6538`) |
| `< 1350` (`SERVO_LOWER_LIMIT`) | Spin counter-clockwise (PWM pulse `3269`) |
| otherwise | Hold centered/stopped (PWM pulse `4904`) |

This logic lives in the `StartServoUpdate` FreeRTOS task.

## FreeRTOS tasks

| Task | Rate / trigger | Purpose |
|---|---|---|
| `CanRx` | On FDCAN RX interrupt | Drains the hardware RX FIFO into libcanard and processes incoming transfers |
| `CanTx` | Notified by Rx/Status tasks | Flushes libcanard's TX queue onto the bus |
| `CanStatus` | 1 Hz (jittered) | Runs libcanard's 1 Hz housekeeping |
| `ServoUpdate` | 50 Hz | Applies the bang-bang logic above; if a channel stops receiving updates it is stopped and flagged (`ALLOWED_SERVO_FAILS` = 5 missed cycles) |
| `LedBlink` | 1 Hz (jittered) | Green heartbeat LED |
| `ErrorLED` | 2 Hz poll | Red LED on while any servo watchdog bit is set |
| `EscStatus` / `EscUpdate` | — | Stubbed out (bodies commented out); ESC support is not wired up on this board, see `USE_ESC` above |

The random jitter on task start (`introduce_jitter`) staggers CAN traffic between boards on the same bus — see the note in the [STM32_Firmware README](../README.md#issues--solutions).

## Building and flashing

Standard STM32CubeMX-generated CMake project:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Flash with ST-Link (STM32CubeProgrammer, OpenOCD, or the STM32 VS Code extension). Reopen the `.ioc` in STM32CubeMX for pin/peripheral changes, but avoid letting it regenerate outside `USER CODE BEGIN/END` blocks without re-checking the task wiring above.

## See also

- [STM32_Firmware README](../README.md) — how this project fits with the others
- [Drone_Can library README](../Libraries/Drone_Can/README.md) — the shared CAN stack this node is built on
