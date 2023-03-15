The purpose of this file is to investigate what speeds will work when running the stepper motors on one of the cores. 

This is leading on from the fact that we get the following error from our AWS+Stepper motor code when we try to run it 
using adaptive speed control. 

First run:
Runnning the code with speeds from 10000 down to 1000, it works all the way until we reach 1000... then we get the following error 

```
2000
Core 1
Looping
E (61998) task_wdt: Task watchdog got triggered. The following tasks did not reset the watchdog in time:
E (61998) task_wdt:  - IDLE (CPU 0)
E (61998) task_wdt: Tasks currently running:
E (61998) task_wdt: CPU 0: Core0Code
E (61998) task_wdt: CPU 1: IDLE
E (61998) task_wdt: Aborting.

abort() was called at PC 0x400d9201 on core 0

Backtrace:0x40083521:0x3ffbe7cc |<-CORRUPTED

ELF file SHA256: 0000000000000000

Rebooting...
```

So it seems that `1000` is the limit, and that it should at least work at 2000. 