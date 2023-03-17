# FOV-DeviceFirmware

## Codebase 

This codebase will include all of the firmware files relating to the FOV device firmware. 
This files should generally be atomic (do a specfic thing), and we will outline here what 
their functionality is. 


`NonBlockingMQTTClientOnOneCore`
- This file runs the MQTT client on one core
    - This is to ensure that its non-blocking. 
    - Note that we also run a simple print statement with delay on the other core + in the `void loop()` func. 


`NonBlockingMotorsOnOneCore`
- This file runs the stepper motors in a preset pattern on one core
    - We also run random print + delay code on the other core. 


`MQTTClientAndSimpleMotorMovementNonBlockingOnTwoCores`
- This file will run the AWS Client code and very simple stepper motor code on two cores at the same time 
    - This should all be running concurrently. 


`MQTTClientAndStepperMotorBlockingButOnTwoCoresButHardcodedSpeed`
- This file runs the AWS Client code and the real time stepper motor code; its uses `.runSpeedToPosition()` which is blocking, 
however since we are using two cores, its ok! 
    - There are currently issues with the adaptive speed control - so going to leave this file as one that uses 
    hardcoded speed values but is otherwise working! 


`StepperMotorsSpeedTest`
- This file runs the stepper motors using `.runSpeedToPosition()` on one core, and progressively lowers the stepper motor speed. 
    - TLDR; we get a timeout error once the speed drops below `2000`


`StepperMotorMovementMethods`
- This file is to experiment using the different methods available to move the stepper motors 
    - (i.e. `.run()`, `.runSpeedToPosition()`, etc.)


`VibesAndMQTTClientAndStepperMotorBlockingButOnTwoCoresAdaptiveSpeed`
- This file should essentially be our _real thing_. It will have adaptive speed + the vibrations working at the same time on one core. With the AWS client on the other core. 

## Resources 

- StepperMotor Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
- Great library guide: https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
