# CANBUS Basic Node Test

FreeRTOS + DroneCAN test firmware used to validate the CAN stack itself, in particular **dynamic node ID allocation (DNA)** — the handshake a DroneCAN node goes through to be assigned a node ID by the allocator on the bus, before it's allowed to transmit anything else.

This project carries its own embedded copy of the CAN library (`Library/Canard`, `Library/can_node`, `Library/UAVCAN_DSDL`) rather than referencing the shared [`../../STM32_Firmware/Libraries/Drone_Can`](../../STM32_Firmware/Libraries/Drone_Can/README.md) — it predates that shared library being factored out, so treat it as a known-good reference snapshot rather than something to keep in sync.

## What it does

- Brings up FDCAN1 and the CAN stack, then runs the same `CanRx` → acquire node ID → process/flush loop described in the [Drone_Can README](../../STM32_Firmware/Libraries/Drone_Can/README.md).
- While waiting for a node ID, the red LED is held on; once DNA succeeds it turns off — so a quick visual check tells you whether allocation is working on the bus.
- `StartCanStatus`, `StartCanTx`, `StartEscStatus`, and `StartLedBlink` tasks exist with the same shapes as the production nodes, but the ESC-specific PWM output code is commented out (`esc_raw_to_pwm` / `esc_set_output` in [`Core/Src/main.c`](Core/Src/main.c)) since this build isn't tied to a specific actuator PCB.

## Hardware

- MCU: STM32G4
- FDCAN1 only — no PWM outputs are actually driven (TIM4 is initialized but not started, left over from an earlier ESC test)

## Building and flashing

Standard STM32CubeMX-generated CMake project — build and flash with ST-Link the same way as the production firmware (see the [STM32_Firmware README](../../STM32_Firmware/README.md)).

## See also

- [STM32_Test_Code README](../README.md) — how this fits with the other test firmware
- [Drone_Can library README](../../STM32_Firmware/Libraries/Drone_Can/README.md) — the current shared CAN stack (this project's library code predates it)
