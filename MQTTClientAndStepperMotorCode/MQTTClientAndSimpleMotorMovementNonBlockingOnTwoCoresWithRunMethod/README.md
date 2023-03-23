This file does two things:

- Runs the AWS client code on one core 
- Runs the steppers motors _following a preset pattern_ on the other core. 
    - Note that it is not using the AWS coordinates. 
    - Also note that I don't think we have the y axis setup somehow... 

Notes:
- Using `delayMicroseconds()` makes using the `.run()` method possible:)