# Wing ESC/Servo Node

DroneCAN node for the canbus-to-PWM PCB that lives in each wing and in the tail. It takes motor and servo commands off the CAN bus and converts them to PWM so the ESCs/servos get a clean, locally-generated signal instead of a long PWM wire run from the flight controller.

One firmware image is flashed to both the left and right wing boards — which wing it becomes is set purely by [`Library/can_config.h`](Library/can_config.h) before flashing (see below).

## Hardware

- MCU: STM32G431CBTx
- Drives 3 servo PWM channels (`TIM2` CH1/CH2, `TIM3` CH1) and 2 ESC PWM channels (`TIM4` CH1/CH2)
- FDCAN1 for DroneCAN
- Red/green status LEDs

## DroneCAN configuration

Actuator counts and channel offsets are set in [`Library/can_config.h`](Library/can_config.h):

| Setting | Value | Meaning |
|---|---|---|
| `USE_SERVO` | defined | Node listens for `uavcan.equipment.actuator.ArrayCommand` servo entries |
| `USE_ESC` | defined | Node listens for `uavcan.equipment.esc.RawCommand` |
| `SERVO_COUNT` | 2 | Local servo channels 0–1 |
| `ESC_COUNT` | 2 | Local ESC channels 0–1 |
| `SERVO_CHANNEL_OFFSET` | currently `0` | Which global CAN servo index this board's local channel 0 reads from |
| `ESC_CHANNEL_OFFSET` | currently `2` | Which global CAN ESC index this board's local channel 0 reads from |

The left/right wing boards run the **same firmware** and are told apart only by these two offsets: the left wing uses ESC's 0–1 and servos 0–1, the right wing uses ESC's 2–3 and servos 2–3 (3 servos on the right in some builds, which also means bumping `SERVO_COUNT` to 3 for that board). The values above are whatever was last flashed — before building for a specific wing, set both offsets (and `SERVO_COUNT`/`ESC_COUNT` if needed) to match that wing, then reflash.

Full explanation of the offset scheme and how to wire up a new node against the shared CAN library lives in the [Drone_Can README](../Libraries/Drone_Can/README.md).

## Actuator mapping

Both servo and ESC commands are converted with a straight linear map from the DroneCAN command range to a PWM compare value (see `map_int()` in [`main.c`](Core/Src/main.c)):

| | CAN command range | PWM compare range |
|---|---|---|
| Servo | 1000 – 2000 | 3269 – 6538 |
| ESC | 0 – 8191 | 24285 – 48570 |

`write_servo(channel, pulse)` and `write_esc(channel, pulse)` map the logical channel number to the physical timer/channel pair.

## FreeRTOS tasks

| Task | Rate / trigger | Purpose |
|---|---|---|
| `CanRx` | On FDCAN RX interrupt | Drains the hardware RX FIFO into libcanard and processes incoming transfers |
| `CanTx` | Notified by Rx/Status tasks | Flushes libcanard's TX queue onto the bus |
| `CanStatus` | 1 Hz (jittered) | Runs libcanard's 1 Hz housekeeping (node status, etc.) |
| `EscStatus` | 10 Hz (jittered) | Sends ESC status feedback over CAN |
| `ServoUpdate` | 50 Hz | Re-applies the last received servo command to PWM; zeroes the output and raises an error bit if updates stop arriving (watchdog, see `ALLOWED_SERVO_FAILS`) |
| `EscUpdate` | 200 Hz | Same watchdog pattern as `ServoUpdate`, for the ESC channels |
| `LedBlink` | 1 Hz (jittered) | Green heartbeat LED |
| `ErrorLED` | 2 Hz poll | Red LED on while any servo/ESC watchdog bit is set |

The random jitter on task start (`introduce_jitter`, seeded from the MCU's unique ID) staggers CAN traffic between boards so two boards running identical code don't transmit their periodic messages in lock-step and collide — see the note in the [STM32_Firmware README](../README.md#issues--solutions).

## Building and flashing

This is a standard STM32CubeMX-generated CMake project:

```sh
cmake --preset Debug      # see CMakePresets.json
cmake --build --preset Debug
```

Flash with ST-Link (STM32CubeProgrammer, OpenOCD, or the STM32 VS Code extension). The `.ioc` file can be reopened in STM32CubeMX if pin/peripheral changes are needed, but **do not** let CubeMX regenerate code outside the `USER CODE BEGIN/END` blocks without re-checking the FreeRTOS task wiring above.

## See also

- [STM32_Firmware README](../README.md) — how this project fits with the others
- [Drone_Can library README](../Libraries/Drone_Can/README.md) — the shared CAN stack this node is built on
