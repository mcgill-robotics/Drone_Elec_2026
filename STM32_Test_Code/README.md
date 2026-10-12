# Test Code

Small bare-metal/minimal firmware used to validate a single PCB or subsystem in isolation, before trusting it to run the full DroneCAN firmware from [STM32_Firmware](../STM32_Firmware/README.md). Each one targets the same hardware as a specific production node — see its README for exactly what it exercises and how to read the result.

1. [**CAN_to_PWM_test**](CAN_to_PWM_test/README.md) verifies that all functionality of the canbus to pwm pcb works (the board behind [Wing_ESC_Servo](../STM32_Firmware/Wing_ESC_Servo/README.md)).
2. [**CANBUS_Basic_Node2**](CANBUS_Basic_Node2/README.md) tests dronecan including dynamic node allocation.
3. [**Motor_Payload_Test**](Motor_Payload_Test/README.md) verifies the functionality of the watergun payload pcb (the board behind [Water_Gun_Payload](../STM32_Firmware/Water_Gun_Payload/README.md)).