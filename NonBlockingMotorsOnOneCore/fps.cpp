#include "fps.h"

FPSCounter::FPSCounter() {}

void FPSCounter::start() {
  start_time = millis();
}

void FPSCounter::stop() {
  end_time = millis();
  unsigned long execution_time = end_time - start_time;
  fps = 1000.0 / execution_time;
}

float FPSCounter::getFPS() {
  return fps;
}
