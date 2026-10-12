# Drone electrical 2025/26 repo

Electrical/firmware repo for the 2025/26 drone, "Benu". There are 4 sections to this repository:
<br>
1. [**PX4_Parameters**](PX4_Parameters/README.md) revision controlling the settings of the flight controller software PX4 we use for flying the drone
2. [**STM32_Firmware**](STM32_Firmware/README.md) is the firmware for the pcb's with ic's onboard
3. [**STM32_Test_Code**](STM32_Test_Code/README.md) is simple firware to ensure all board features are working after assembly

Every PCB's firmware is a DroneCAN node talking over a shared CAN bus — see the [Drone_Can library README](STM32_Firmware/Libraries/Drone_Can/README.md) for how that bus and library work, and the [STM32_Firmware README](STM32_Firmware/README.md) for a per-board breakdown.
