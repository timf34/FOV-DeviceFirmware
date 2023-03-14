# FOV-DeviceFirmware

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