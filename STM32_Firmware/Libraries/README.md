# Libraries

Shared code referenced by every STM32 project in [`STM32_Firmware`](../README.md).

- [**Drone_Can**](Drone_Can/README.md) — the DroneCAN/libcanard stack and device helpers (ESC, servo, battery, pitot, arming) that every node firmware builds against.
- [**DSDL_Generator**](DSDL_Generator/README.md) — the offline tooling that turns DSDL message definitions into the generated `.h`/`.c` files consumed by Drone_Can's `UAVCAN_DSDL` folder. You only need this if you're adding a new DroneCAN message type, not for day-to-day firmware work.
