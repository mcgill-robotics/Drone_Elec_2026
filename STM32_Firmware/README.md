# STM32 Firmware
All firmware code used in 2025/26 drone "Benu"
<br>

## Libraries
This folder includes the dronecan library such that it can be referenced by all the stm32 projects in this directory. It includes two files: 

1. **Drone_Can** The library file itself which is a mix of existing dronecan library and custom scripts. See it's README file for more information.

2. **DSDL_Generator** this is a series of cloned repos from drone can which allow the DSDL files to be generated. These files include all the definitions for different dronecan devices (ie. servo, motor, battery, etc.). The output files produced from running the python script in a virtual environment (venv) are copied into the Drone_Can library under UAVCAN_DSDL

**Note:** dronecan and uavcan will be used interchangably. UAVCAN version 0 has been renamed to dronecan and UAVCAN version 1 has been renamed cyphal. PX4 has limited support for cyphal.
<br>

## STM32 Projects
All of these devices use the dronecan library and an stm32g4 chip. The library was built around using freeRTOS to ensure all task occur with deterministic timing. 

### Camera_Gimbal_O3
**Note:** this project is not currently used for 2025/26 as we ran out of time. The code is written but untested. For this year, the gimbal is controlled using the Wing_ESC_Servo code and the O3 air unit is armed manually.
<br>

This code has two purposes:
1. Control two PWM servo outputs to drive roll the pitch of a camera gimbal
2. Recieve the arming status from flight controller and if armed tell the dji o3 air unit connected over UART tx to the unused esc connector on the pcb. This is needed as the O3 airunit overheats quickly without the airflow from the propellors, so goes into a low power mode until armed.

This uses servo array location 5 and 6. The PX4 pulishes an array of servos ranging from 0 to 7.

### Drop_Payload

This uses a canbus to pwm circuit board to release the payload after reaching the target. Code is nearly identical to Wing_ESC_Servo.

This uses servo array location 7 as the drone can only carry one payload, but more could be added by doing something like servo[7] > 1500 means drop payload 2 and servo[7] < 1500 means drop payload 1. The PX4 pulishes an array of servos ranging from 0 to 7.

### Voltage_Current_Sense

Uses the voltage measurement pcb to measure voltage of each battery and current draw from each of the 50v series pairs. The current measurement is done by reading the analog output from 0 to 3.3v of the Texas Instruments current measurement device, averaging it, and sending it over canbus. The voltage measurement uses a Texas Instuments isolated delta-sigma ADC to safely read the voltage of the batteries, averages and scales it, then sends it over canbus. Currently only sending one pack voltage and the sum of currents has been setup, not each individual cell.

**Note:** both voltage and current measurements seem to be scaled incorrectly.

### Water_Gun_Payload

Uses the water_gun pcb to control a water pump and water solenoid. When the servo input command is greater than 1650 it turns on the water pump and solenoid. It uses the same servo 7 input command as the drop_payload as only one will ever be plugged in at once.

### Wing_ESC_Servo

Uses the canbus to PWM pcb to control the servos and motors in the drone's wing and tail. It recieves the motor and servo comands over canbus and converts that to pwm for improved signal integrity and simplified wiring. 

This code works for both left and right wing by simply change a couple values in the configuration file Library/can_config.h. The left wing uses 2 esc's starting at 0, and 2 servos starting at 0. The right uses 2 esc's starting at 2, and 3 servos starting at 2.

### Pitot Tube

Not currently present as the pitot tube ic got broken.

Receives data over i2c from pitot tube ic then relays over canbus.

## Issues (& solutions)
1. An issue occured using freeRTOS where if two devices running identical code are on the same bus they send messages at the exact same time so one always overwrites the other. To solve this a radomized delay is added to the start of each freertos task which interfaces with canbus.