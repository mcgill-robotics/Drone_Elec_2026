# Water Gun Payload Node

DroneCAN node for the water-gun payload PCB. It drives a water pump and a solenoid valve as a single digital on/off output — there's no PWM here, just "fire" or "don't fire".

This board is physically interchangeable with [Drop_Payload](../Drop_Payload/README.md) — only one of the two payload boards is ever plugged in at a time, since the drone only carries one kind of payload per flight.

## Hardware

- MCU: STM32G431CBTx
- `MOTOR_Pin` / `SOLENOID_Pin` (GPIOA) driven together as plain digital outputs — see `motor_fire()` in [`Core/Src/main.c`](Core/Src/main.c)
- FDCAN1 for DroneCAN
- Red/green status LEDs

## DroneCAN configuration

Set in [`Library/can_config.h`](Library/can_config.h):

| Setting | Value |
|---|---|
| `USE_SERVO` | defined |
| `SERVO_COUNT` | 1 — **do not change in this file**, the firmware is hard-coded for a single channel |
| `SERVO_CHANNEL_OFFSET` | 7 — reads global CAN servo index 7 (the drone only ever carries one water-gun/drop payload) |
| `USE_ESC` | commented out (not used) |

See the [Drone_Can README](../Libraries/Drone_Can/README.md) for what the offset/count scheme means.

## Fire logic

The incoming CAN servo command (range 1000–2000) is compared against a single threshold (`SERVO_UPPER_LIMIT = 1650` in [`Core/Inc/main.h`](Core/Inc/main.h)) in the `StartMotorUpdate` task:

- command `> 1650` → pump + solenoid **on**
- otherwise → pump + solenoid **off**

If CAN updates stop arriving for `ALLOWED_SERVO_FAILS` (5) consecutive 50 Hz cycles, the output is forced off and an error bit is raised.

## FreeRTOS tasks

| Task | Rate / trigger | Purpose |
|---|---|---|
| `CanRx` | On FDCAN RX interrupt | Drains the hardware RX FIFO into libcanard and processes incoming transfers |
| `CanTx` | Notified by Rx/Status tasks | Flushes libcanard's TX queue onto the bus |
| `CanStatus` | 1 Hz (jittered) | Runs libcanard's 1 Hz housekeeping |
| `MotorUpdate` | 50 Hz | Applies the fire logic above, with the watchdog described in the previous section |
| `LedBlink` | 1 Hz (jittered) | Green heartbeat LED |
| `ErrorLED` | 2 Hz poll | Red LED on while the watchdog bit is set |

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
- [Drop_Payload](../Drop_Payload/README.md) — the alternate payload board that shares this CAN servo slot
