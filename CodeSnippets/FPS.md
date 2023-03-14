Code snippet 

```
void loop() {
  unsigned long start_time = millis(); // get the current time
  // Your code snippet here
  unsigned long end_time = millis(); // get the current time again
  unsigned long execution_time = end_time - start_time; // calculate the execution time
  float fps = 1000.0 / execution_time; // calculate the FPS
  Serial.print("FPS: ");
  Serial.println(fps);
}
```


As a class

```
class FPSCounter {
  private:
    unsigned long start_time;
    unsigned long end_time;
    float fps;
  
  public:
    FPSCounter() {}
    
    void start() {
      start_time = millis();
    }
    
    void stop() {
      end_time = millis();
      unsigned long execution_time = end_time - start_time;
      fps = 1000.0 / execution_time;
    }
    
    float getFPS() {
      return fps;
    }
};
```

And class usage

```
FPSCounter fpsCounter;

void setup() {
  Serial.begin(9600);
}

void loop() {
  fpsCounter.start();
  // Your code snippet here
  fpsCounter.stop();
  float fps = fpsCounter.getFPS();
  Serial.print("FPS: ");
  Serial.println(fps);
}
```