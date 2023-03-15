#include <AccelStepper.h>
#include <MultiStepper.h>

#define ENABLE_Y 19
#define DIR_Y 13
#define STEP_Y 14
#define motorInterfaceType 1
AccelStepper stepper_Y = AccelStepper(motorInterfaceType, STEP_Y, DIR_Y);

#define ENABLE_X 23
#define DIR_X 32
#define STEP_X 33
AccelStepper stepper_X = AccelStepper(motorInterfaceType, STEP_X, DIR_X);

MultiStepper steppers;
long positionMove[2];

#define xConvert (26000 / 102)
#define yConvert (18500 / 64)

float xSpd = 8000;
float ySpd = 8000;

// End Stop Hall Sensor
#define hall_X 36
#define hall_Y 38

#define use_second_core true

// Homing Sequence Variables
long initial_homing_X = -1; // X Axis
long initial_homing_Y = 1;  // Y Axis

bool loopTrue = true; // This will make it move!
bool spdChange = false;

void stepperSetup()
{
    pinMode(ENABLE_X, OUTPUT);
    pinMode(ENABLE_Y, OUTPUT);
    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
    // Configure each stepper
    stepper_X.setMaxSpeed(25000);
    stepper_Y.setMaxSpeed(25000);
    // Then give them to MultiStepper to manage
    steppers.addStepper(stepper_X);
    steppers.addStepper(stepper_Y);
}

void hallSensorsSetup()
{
    pinMode(hall_X, INPUT);
    pinMode(hall_Y, INPUT);
}

void Core0Code(void *pvParameters)
{
    for (;;)
    {
        loopMovement();
    }
}

void Core1Code(void *pvParameters)
{
    for (;;)
    {
        delay(5000);
        Serial.println("Core 1");

    }
}

void setup()
{
    Serial.begin(9600);

    stepperSetup();
    hallSensorsSetup();
    homeSteppers();
    stepper_X.setCurrentPosition(0);
    stepper_Y.setCurrentPosition(0);

    xTaskCreatePinnedToCore(
        Core0Code,   /* Task function. */
        "Core0Code", /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        1,           /* priority of the task */
        NULL,        /* Task handle to keep track of created task */
        0            /* pin task to core 0 */
    );

    if (use_second_core == true)
    {
        xTaskCreatePinnedToCore(
            Core1Code,   /* Task function. */
            "Core1Code", /* name of task. */
            5000,        /* Stack size of task */
            NULL,        /* parameter of the task */
            1,           /* priority of the task */
            NULL,        /* Task handle to keep track of created task */
            0            /* pin task to core 0 */
        );
    }
}

void loop()
{

    Serial.println("Looping");
    delay(5000);
}

// Declare an array of speed_values from 10000 to 1000 at 1000 intervals
int speed_values[10] = {10000, 9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000};


void loopMovement()
{
    // moveStepsToPos(0, 0);
    // vTaskDelay(1);
    // moveStepsToPos(48, 26);
    // vTaskDelay(1);

    // Loop through the speed_values array and pass them to moveStepsToPos call 
    for (int i = 0; i < 10; i++)
    {
        Serial.println(speed_values[i]);
        moveStepsToPos(0, 0, speed_values[i], speed_values[i]);
        vTaskDelay(1);
        moveStepsToPos(48, 26, speed_values[i], speed_values[i]);
        vTaskDelay(1);
    }

}

void speedCalc(float x1, float y1, float x2, float y2)
{                // calculate required stepper speed based on distance the ball has the travel within an alotted time
    float t = 2; // timeframe to complete movement

    float dx = abs(x2 - x1); // distance to next coordinate
    float dy = abs(y2 - y1);
    float sx = dx / t; // in mm/s //speed needed to get to next point within allowed timeframe
    float sy = dy / t;
    xSpd = (sx * (26000 / 103)); // convert speed  mm/s to stepper speed
    ySpd = (sy * (18500 / 65));

    // Top speed is 20000 before motors start jamming
    if (xSpd > 18000)
    {
        xSpd = 18000;
    }
    if (ySpd > 18000)
    {
        ySpd = 18000;
    }
}

void moveStepsToPos(long x, long y, int _xSpd, int _ySpd)
{
    stepper_X.setMaxSpeed(_xSpd);
    stepper_Y.setMaxSpeed(_ySpd);
    stepper_X.setAcceleration(_xSpd * 50);
    stepper_Y.setAcceleration(_ySpd * 50);
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, LOW);
    positionMove[0] = x * xConvert;
    positionMove[1] = y * yConvert;

    steppers.moveTo(positionMove);
    steppers.runSpeedToPosition();

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}

void homeSteppers()
{
    /////////////////////////////////////////////////////////////////////////////////////////
    // X AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    float homingSpd = 15000.0;
    Serial.println("Homing X Axis");
    digitalWrite(ENABLE_X, LOW);
    digitalWrite(ENABLE_Y, HIGH);
    delay(5); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper X is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_X))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_X.moveTo(initial_homing_X); // Set the position to move to
        initial_homing_X--;                 // Decrease by 1 for next move if needed
        stepper_X.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_X.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_X.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_X.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_X = -1;

    while (!digitalRead(hall_X))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_X.moveTo(initial_homing_X);
        stepper_X.run();
        initial_homing_X++;
        delay(1);
    }

    stepper_X.setCurrentPosition(0);
    Serial.println("Homing X Axis Completed");

    /////////////////////////////////////////////////////////////////////////////////////////
    // Y AXIS HOMING
    ////////////////////////////////////////////////////////////////////////////////////////
    Serial.println("Homing Y Axis");
    digitalWrite(ENABLE_Y, LOW);
    digitalWrite(ENABLE_X, HIGH);
    delay(3); // Wait for EasyDriver wake up

    //  Set Max Speed and Acceleration of each Steppers at startup for homing
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper

    // Start Homing procedure of Stepper Motor at startup

    Serial.print("Stepper Y is Homing . . . . . . . . . . . ");

    while (digitalRead(hall_Y))
    { // Make the Stepper move CCW until the switch is activated
        // Serial.println(digitalRead(home_switch));
        stepper_Y.moveTo(initial_homing_Y); // Set the position to move to
        initial_homing_Y--;                 // Decrease by 1 for next move if needed
        stepper_Y.run();                    // Start moving the stepper
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);      // Set the current position as zero for now
    stepper_Y.setMaxSpeed(homingSpd);     // Set Max Speed of Stepper (Slower to get better accuracy)
    stepper_Y.setAcceleration(homingSpd); // Set Acceleration of Stepper
    initial_homing_Y = 1;

    while (!digitalRead(hall_Y))
    { // Make the Stepper move CW until the switch is deactivated
        stepper_Y.moveTo(initial_homing_Y);
        stepper_Y.run();
        initial_homing_Y++;
        delay(1);
    }

    stepper_Y.setCurrentPosition(0);
    Serial.println("Homing Y Axis Completed");

    // moveStepsToPos(1, 1);
    stepper_X.setCurrentPosition(1);
    stepper_Y.setCurrentPosition(1);

    digitalWrite(ENABLE_X, HIGH);
    digitalWrite(ENABLE_Y, HIGH);
}
