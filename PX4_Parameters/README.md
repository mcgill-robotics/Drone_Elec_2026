## PX4 Parameters

Exported PX4 parameter files (`.params`), used to version-control the flight controller's tuning/configuration instead of relying on whatever is currently flashed to a given autopilot.

1. `6C mini params.params` are from the small test platform from 2023/24 named "little T"
2. `V6X VTOL params.params` are from the main drone of 2025/26 "Benu" (can also find parameters from previous year drone in commit history)

### Usage

- **Export** (back up current tuning): QGroundControl → Vehicle Setup → Parameters → Tools → Save to file, then commit the resulting `.params` file here.
- **Load** (restore tuning onto a board): QGroundControl → Vehicle Setup → Parameters → Tools → Load from file, pick the relevant `.params` file above, then reboot the flight controller.

These files only make sense alongside the servo/motor channel mapping each [STM32_Firmware](../STM32_Firmware/README.md) node expects on the corresponding `PWM AUX`/actuator output — see that README for which CAN servo/ESC index each board listens on.