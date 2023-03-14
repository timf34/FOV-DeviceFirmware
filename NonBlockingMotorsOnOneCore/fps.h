#ifndef FPSCounter_h
#define FPSCounter_h

#include "Arduino.h"

class FPSCounter {
  private:
    unsigned long start_time;
    unsigned long end_time;
    float fps;
  
  public:
    FPSCounter();
    void start();
    void stop();
    float getFPS();
};

#endif
