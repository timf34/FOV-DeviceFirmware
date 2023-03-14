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