# Motor / Payload Hardware Test

Bare-metal (no FreeRTOS, no DroneCAN stack) sanity check for the water-gun payload PCB — the same board that [Water_Gun_Payload](../../STM32_Firmware/Water_Gun_Payload/README.md) runs in production. It exercises the pump/solenoid outputs directly so you can confirm the board was assembled correctly before trusting it with real firmware.

## What it does

[`Core/Src/main.c`](Core/Src/main.c) does all of its work in `main()`'s infinite loop: drive all four outputs high, hold for 10 seconds, drop them low, hold for 5 seconds, repeat.

```c
// high for 10s
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
HAL_Delay(10000);
// low for 5s
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,  GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
HAL_Delay(5000);
```

With the pump/solenoid (or a load/LED standing in for them) connected, this confirms the output drive stages on the board actually switch — the on/off timing is only there to make the cycling visible/audible, it has no relationship to the real fire logic in [Water_Gun_Payload](../../STM32_Firmware/Water_Gun_Payload/README.md).

## Hardware

- MCU: STM32G431 (same payload board as [Water_Gun_Payload](../../STM32_Firmware/Water_Gun_Payload/README.md))
- No CAN, no RTOS — deliberately the simplest possible firmware so a wiring fault can't be mistaken for a software bug

## Building and flashing

Standard STM32CubeMX-generated CMake project — build and flash with ST-Link the same way as the production firmware (see the [STM32_Firmware README](../../STM32_Firmware/README.md)).

## See also

- [STM32_Test_Code README](../README.md) — how this fits with the other test firmware
- [Water_Gun_Payload](../../STM32_Firmware/Water_Gun_Payload/README.md) — the production firmware for this same PCB
