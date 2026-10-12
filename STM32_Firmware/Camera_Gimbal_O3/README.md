# Camera Gimbal / O3 Air-Unit Node

**Not used in the 2025/26 build.** The code is written but untested — the project ran out of time this year, and the gimbal function is instead handled by [Wing_ESC_Servo](../Wing_ESC_Servo/README.md), with the DJI O3 air unit armed manually. Kept here for a future year to pick up.

DroneCAN node intended for a two-axis camera gimbal PCB. It has two jobs:

1. Drive two PWM servo outputs for gimbal roll/pitch.
2. Watch the flight controller's arm status and tell the DJI O3 air unit over UART when the drone arms, so the O3 can come out of its low-power mode (it overheats quickly without prop wash airflow while unarmed).

## Hardware

- MCU: STM32G431CBTx
- 2 PWM outputs for the gimbal servos: `TIM3` CH1, `TIM2` CH2 (local channels 0, 1 — see `write_servo()` in [`Core/Src/main.c`](Core/Src/main.c); `TIM2` CH1 also exists in the switch statement for a possible 3rd channel but is not started)
- USART1 (half-duplex) to the DJI O3 air unit, wired to the board's otherwise-unused ESC connector — see [`Library/dji_o3/dji_o3.h`](Library/dji_o3/dji_o3.h) / [`dji_o3.c`](Library/dji_o3/dji_o3.c)
- FDCAN1 for DroneCAN
- Red/green status LEDs

## DroneCAN configuration

Set in [`Library/can_config.h`](Library/can_config.h):

| Setting | Value |
|---|---|
| `USE_SERVO` | defined |
| `SERVO_COUNT` | 2 |
| `SERVO_CHANNEL_OFFSET` | 5 — reads global CAN servo indices 5 and 6 |
| `USE_ESC` | defined, but unused by this node's logic (`EscUpdate`/`EscStatus` tasks aren't created in `main()`) |

See the [Drone_Can README](../Libraries/Drone_Can/README.md) for what the offset/count scheme means.

## Gimbal control

CAN servo commands (range 1000–2000) are linearly mapped to a PWM compare value (3269–6538) via `map_int()`, same scheme as [Wing_ESC_Servo](../Wing_ESC_Servo/README.md). Negative commands are mirrored positive before mapping.

## Arming → O3 power state

`StartDJIO3` polls the shared `arm_status` flag (set by the CAN library's arming-status handler, see [`can_arm.h`](../Libraries/Drone_Can/can_node/Inc/can_arm.h)) every 500 ms and calls `DJI_O3_Update(&o3, arm_status)`, which sends the appropriate command over UART1 to wake or idle the air unit.

## FreeRTOS tasks

| Task | Rate / trigger | Purpose |
|---|---|---|
| `CanRx` | On FDCAN RX interrupt | Drains the hardware RX FIFO into libcanard and processes incoming transfers |
| `CanTx` | Notified by Rx/Status tasks | Flushes libcanard's TX queue onto the bus |
| `CanStatus` | 1 Hz (jittered) | Runs libcanard's 1 Hz housekeeping |
| `GimbalUpdate` | 50 Hz | Applies the servo mapping above; stops and flags a channel if updates stop arriving (`ALLOWED_SERVO_FAILS` = 5 missed cycles) |
| `DJIO3` | 2 Hz, after a 5 s boot delay | Pushes the current arm status to the O3 air unit over UART |
| `LedBlink` | 1 Hz (jittered) | Green heartbeat LED |
| `ErrorLED` | 2 Hz poll | Red LED on while any servo or O3 error bit is set |

The random jitter on task start (`introduce_jitter`) staggers CAN traffic between boards on the same bus — see the note in the [STM32_Firmware README](../README.md#issues--solutions).

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
