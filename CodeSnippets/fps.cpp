// # include <Arduino.h>

// class FPSCounter {
//   private:
//     unsigned long start_time;
//     unsigned long end_time;
//     float fps;
  
//   public:
//     FPSCounter() {}
    
//     void start() {
//       start_time = millis();
//     }
    
//     void stop() {
//       end_time = millis();
//       unsigned long execution_time = end_time - start_time;
//       fps = 1000.0 / execution_time;
//     }
    
//     float getFPS() {
//       return fps;
//     }
// };