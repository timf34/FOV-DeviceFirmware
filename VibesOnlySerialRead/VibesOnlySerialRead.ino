#include "Arduino.h"

int lastPoss = 0;
int vibeMode;

//////////////////////////
// Vibration Motor Setup//
/////////////////////////

// Vibration Motor in pitch
#define VIB_GPIO1 9
#define PWM1_Ch 0
#define PWM1_Res 8
#define PWM1_Freq 1000

// Vibration motor on body
#define VIB_GPIO2 21
#define PWM2_Ch 1
#define PWM2_Res 8
#define PWM2_Freq 1000

int PWM1_DutyCycle = 0;
int PWM2_DutyCycle = 0;

void pwmPinsSetup()
{

  // Setup Motor 1
  ledcAttachPin(VIB_GPIO1, PWM1_Ch);
  ledcSetup(PWM1_Ch, PWM1_Freq, PWM1_Res);
  // Set up Motor 2
  ledcAttachPin(VIB_GPIO2, PWM2_Ch);
  ledcSetup(PWM2_Ch, PWM2_Freq, PWM2_Res);

  ledcWrite(PWM1_Ch, 0);
  ledcWrite(PWM2_Ch, 0);
}

///////////////////////////////////////////////////////////////////////////
// Vibration Motor On Second Core
///////////////////////////////////////////////////////////////////////////
// Vibration functionality is handled by ESP32's second core to provide multitasking

// vibration response depending on events
void pwmMotor(void *pvParametersm)
{

  // VibeMode = 0 Tackle
  if (vibeMode == 0)
  {
    ledcWrite(PWM2_Ch, 200);
    ledcWrite(PWM1_Ch, 200);
    delay(40);
    ledcWrite(PWM2_Ch, 105);
    ledcWrite(PWM1_Ch, 65);
    delay(350);
    ledcWrite(PWM2_Ch, 0);
    ledcWrite(PWM1_Ch, 0);
  }

  // VibeMode = 1 Top vibrator (home kick/ pass)
  if (vibeMode == 1)
  {
    ledcWrite(PWM1_Ch, 200);
    delay(40);
    ledcWrite(PWM1_Ch, 65);
    delay(200);
    ledcWrite(PWM1_Ch, 0);
  }

  // VibeMode = 2 Bottom vibrator (away kick/ pass)
  if (vibeMode == 2)
  {
    ledcWrite(PWM2_Ch, 200);
    delay(40);
    ledcWrite(PWM2_Ch, 105);
    delay(200);
    ledcWrite(PWM2_Ch, 0);
  }

  // VibeMode = 3 Home ball received
  if (vibeMode == 3)
  {
    ledcWrite(PWM1_Ch, 200);
    delay(40);
    ledcWrite(PWM1_Ch, 65);
    delay(200);
    ledcWrite(PWM1_Ch, 0);
    delay(55);
    ledcWrite(PWM1_Ch, 200);
    delay(40);
    ledcWrite(PWM1_Ch, 65);
    delay(200);
    ledcWrite(PWM1_Ch, 0);
  }

  // VibeMode = 4  Away ball received
  if (vibeMode == 4)
  {
    ledcWrite(PWM2_Ch, 200);
    delay(40);
    ledcWrite(PWM2_Ch, 105);
    delay(200);
    ledcWrite(PWM2_Ch, 0);
    delay(55);
    ledcWrite(PWM2_Ch, 200);
    delay(40);
    ledcWrite(PWM2_Ch, 105);
    delay(200);
    ledcWrite(PWM2_Ch, 0);
    ;
  }

  // VibeMode = Nothing
  if (vibeMode == 5)
  {
    ledcWrite(PWM1_Ch, 200);
    ledcWrite(PWM2_Ch, 200);
    delay(50);
    ledcWrite(PWM1_Ch, 0);
    ledcWrite(PWM2_Ch, 0);
  }

  vibeMode = 10; // Reset VibeMode

  vTaskDelete(NULL);
}

// Multitasking RTOS Cores
TaskHandle_t PWMVibe;

void coreSetup()
{
  xTaskCreatePinnedToCore(
      pwmMotor,    // Function that should be called
      "Motor PWM", // Name of the task (for debugging)
      10000,       // Stack size (bytes)
      NULL,        // Parameter to pass
      1,           // Task priority
      &PWMVibe,    // Task handle
      0            // Select Which Core
  );
}

void tutorial_vibrations(int mode_for_vibrations)
{
  vibeMode = mode_for_vibrations;

  Serial.println("Vibration Mode: ");
  Serial.println(vibeMode);

  coreSetup();
}



/*******************************************************************************************************************************************************************************************************************************************************************************/
// Vibration Variables and Setup
/*******************************************************************************************************************************************************************************************************************************************************************************/
bool vibeOn = true;

#define I2S_DOUT 22 // DIN connection
#define I2S_BCLK 25 // Bit clock
#define I2S_LRC 26  // Left Right Clock

// Variables for main loop
char inBuff[] = "----------------------------------------------------------------";
int bytesread;

void setup()
{
  Serial.begin(9600);

  delay(5000);
  pwmPinsSetup();
}

// Period which board checks for new messages from server
int period = 140;
unsigned long time_now = 0;

void loop()
{
  SerialRead();
}

/*******************************************************************************************************************************************************************************************************************************************************************************/
// User Input Functions
/*******************************************************************************************************************************************************************************************************************************************************************************/

// Process Instruction
void processInstruction(String instruction)
{

  /*
     O Audio On
     S Audio Off
     V number to Mp3
     T string to Mp3
     G general race info
  */
  String value1 = "0";
  String value2 = "0";

  int instructionLength = instruction.length();

  if (instructionLength > 0)
  {
    value1 = instruction.substring(0, 1);
    Serial.print("Value 1: ");
    Serial.println(value1);
  }

  vibeMode = value1.toInt();

  coreSetup();

}

void SerialRead()
{
  if (Serial.available())
  {
    bytesread = Serial.readBytesUntil('\n', inBuff, 64); // Read from serial until CR is read or timeout exceeded
    inBuff[bytesread] = '\0';
    String instruction = String(inBuff);
    processInstruction(instruction);
  }
}


/*******************************************************************************************************************************************************************************************************************************************************************************/
// Additional Functions
/*******************************************************************************************************************************************************************************************************************************************************************************/

// String  var = getValue( StringVar, ',', 2); // if  a,4,D,r  would return D
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length();

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
} // END
