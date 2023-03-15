### Notes on making it work 

There was a memory error resutling from some of the variables in the `moveStepsToPos()` function. 
The variables used to control the speed were global and were causing a memory issue that was fixed
when I hardcoded in a random figure thankfully. 