# FOV-DeviceFirmware

## Codebase 

This codebase will include all of the firmware files relating to the FOV device firmware. 
This files should generally be atomic (do a specfic thing), and we will outline here what 
their functionality is. 

Note: when uploading data files, remember to click the _ESP32 Sketch Data Upload_ button under the _tools_ tab!

**Most important files**

- `AudioTutorialFromProcessingCommandAndVibes.ino`
    - This file is the most up to date version! Audio tutorial is only called after a command from Processing + wifi and internet will try reconnect after connection is lost + the last internet creds are remembered + pwm settings are updated. 
    - Need to get the firmware library working soon! 

`VibesAWSSteppersMultiCore`
- This file runs the AWS Client, steppers and vibrations. 

`AudioVibesAWSSteppersMultiCoreTutorial`
- Same as above, but with the audio tutorial.

---

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


`MQTTClientAndSimpleMotorMovementNonBlockingOnTwoCoresWithRunMethod`
- This file runs the AWS Client code and very simple stepper motor code (running a preset pattern, not using AWS coords) on two cores at the same time 
    - We use the `.run()` method to move the stepper motors, which is non-blocking, and also allows using acceleration. 


`MQTTClientAndSimpleMotorMovementNonBlockingOnTwoCoresWithRunToPositionMethod`
- This file runs the AWS Client code and very simple stepper motor code (running a preset pattern, not using AWS coords) on two cores at the same time 
    - We use the `.runToPosition()` method to move the stepper motors, which is blocking (but doesn't matter as we are using two cores), but also **doesn't allow 
    acceleration**.


`VibesAndMQTTClientAndStepperMotorBlockingButOnTwoCoresAdaptiveSpeed`
- This file should be our base firmware file, running AWS, vibrations, and the steppers. 
- ** Wait maybe not lol** check out vibesawssteppersmutlicore.ino



## Resources 

- StepperMotor Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
- Great library guide: https://hackaday.io/project/183279-accelstepper-the-missing-manual/details
