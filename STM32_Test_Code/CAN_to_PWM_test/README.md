# CAN-to-PWM Hardware Test

Bare-metal (no FreeRTOS, no DroneCAN stack) sanity check for the canbus-to-PWM PCB — the same board that [Wing_ESC_Servo](../../STM32_Firmware/Wing_ESC_Servo/README.md) runs in production. It doesn't test any CAN or DroneCAN logic; it just toggles every output pin on the board so you can confirm with a multimeter/scope/LED that the board was assembled correctly before trusting it with real firmware.

## What it does

[`Core/Src/main.c`](Core/Src/main.c) does all of its work in `main()`'s infinite loop — toggle every output, wait 10 seconds, repeat:

```c
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);  // ESC 1
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);  // ESC 2
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);  // SERVO 1
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);  // SERVO 2
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15); // SERVO 3
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11); // RED LED
HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12); // Green LED
HAL_Delay(10000);
```

Each pin flips state every 10 seconds, so with a multimeter or LED you can walk the board and confirm every ESC/servo output and both status LEDs are wired and switching correctly.

## Hardware

- MCU: STM32G431 (same canbus-to-PWM board as [Wing_ESC_Servo](../../STM32_Firmware/Wing_ESC_Servo/README.md))
- No CAN, no RTOS — this is deliberately the simplest possible firmware so a wiring fault can't be mistaken for a software bug

## Building and flashing

Standard STM32CubeMX-generated CMake project — build and flash with ST-Link the same way as the production firmware (see the [STM32_Firmware README](../../STM32_Firmware/README.md)).

## See also

- [STM32_Test_Code README](../README.md) — how this fits with the other test firmware
- [Wing_ESC_Servo](../../STM32_Firmware/Wing_ESC_Servo/README.md) — the production firmware for this same PCB
