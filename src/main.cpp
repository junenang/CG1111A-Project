#include "MeMCore.h"
#include "ultrasonic.h"
#include "IR Sensor.h"
#include "RatioColourSensor.h"

#define TURNING_TIME_MS 290 // The time duration (ms) for turning
#define adjustment 80.0     // course correction multiplier
#define pollRate 60         // course course correction poll rate in ms
MeDCMotor leftMotor(M1);    // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);   // assigning RightMotor to port M2
uint8_t motorSpeed = 255;   // Leave room for the full +/-80 steering correction.
const uint8_t LINE_S1 = 9;  // Line sensor: Port 2
const uint8_t LINE_S2 = 10;
const int IR_TRIGGER = 120;
const int IR_RESET = 40;
const unsigned long IR_CURVE_MS = 110;
const unsigned long DEBUG_INTERVAL_MS = 200;
RatioColourSensor colourSensor; // Create a colour sensor object

// Initialisation
MeBuzzer buzzer;                     // Create a buzzer object to play tones
MeLineFollower lineFollower(PORT_2); // Create a line follower object

bool polling(unsigned long interval)
{
  static unsigned long lastPollTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastPollTime < interval)
  {
    return false;
  }
  lastPollTime = currentTime;
  return true;
}

void pid()
{
  const float target = 10.0f;

  float distance = readUltrasonic();
  // Serial.println("Distance: " + String(distance) + " cm");
  //  No echo or a distant return: continue forward through the opening.
  if (distance == 0 || distance > 15.0f)
  {
    leftMotor.run(-motorSpeed);
    rightMotor.run(motorSpeed);
    return;
  }

  float error = target - distance;
  float correction = constrain(adjustment * error, -80.0f, 80.0f);

  leftMotor.run(-motorSpeed - correction);
  rightMotor.run(motorSpeed - correction);
}

void move()
{
  if (polling(pollRate))
  {
    pid();
  }
}

void goForward(int duration)
{
  leftMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
  rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
  delay(duration);            // Keep going straight for this time duration
  leftMotor.stop();           // Stop left motor
  rightMotor.stop();          // Stop right motor
}
void turnLeft()
{
  leftMotor.run(motorSpeed);  // Positive: wheel turns clockwise
  rightMotor.run(motorSpeed); // Positive: wheel turns clockwise
  delay(TURNING_TIME_MS);     // Keep turning left for this time duration
  leftMotor.stop();           // Stop left motor
  rightMotor.stop();          // Stop right motor
}

void turnRight()
{
  leftMotor.run(-motorSpeed);  // Negative: wheel turns anti-clockwise
  rightMotor.run(-motorSpeed); // Negative: wheel turns anti-clockwise
  delay(TURNING_TIME_MS);      // Keep turning right for this time duration
  leftMotor.stop();            // Stop left motor
  rightMotor.stop();           // Stop right motor
}
void uTurn()
{
  leftMotor.run(motorSpeed);    // Positive: wheel turns clockwise
  rightMotor.run(motorSpeed);   // Negative: wheel turns anti-clockwise
  delay(1.9 * TURNING_TIME_MS); // Keep turning for this time duration
  leftMotor.stop();             // Stop left motor
  rightMotor.stop();            // Stop right motor
}

void doubleLeftTurn()
{
  turnLeft();     // Turn left for 287 ms
  delay(100);     // Wait for 100 ms
  goForward(650); // Move forward for 650 ms
  delay(100);     // Wait for 100 ms
  turnLeft();     // Turn left for 287 ms
}

void doubleRightTurn()
{
  turnRight();    // Turn right for 287 ms
  delay(100);     // Wait for 100 ms
  goForward(650); // Move forward for 750 ms
  delay(100);     // Wait for 100 ms
  turnRight();    // Turn right for 287 ms
}

bool detectBlackLine()
{
  return digitalRead(LINE_S1) == LOW ||
         digitalRead(LINE_S2) == LOW;
}

void celebrate()
{
  // Each of the following "function calls" plays a single tone.
  // The numbers in the bracket specify the frequency and the duration (ms)
  buzzer.tone(392, 200);
  buzzer.tone(523, 200);
  buzzer.tone(659, 200);
  buzzer.tone(784, 200);
  buzzer.tone(659, 150);
  buzzer.tone(784, 400);
  buzzer.noTone();
}

void setup()
{
  Serial.begin(9600);
  setupIR();

  if (!colourSensor.begin())
  {
    leftMotor.stop();
    rightMotor.stop();
    Serial.println(F("No valid colour calibration. Send c to calibrate before driving."));
  }

  Serial.println(F("c=calibrate (motors stop); g=capture, q=cancel during calibration."));
  Serial.println(F("Recalibrate after changing sensor settling time or mounting."));
  delay(3000); // Do nothing for 10000 ms = 10 seconds
  // move();      // The motors keep running until explicitly stopped.
}

void loop()
{
  static bool irTurnTriggered = false;
  static bool irCurveActive = false;
  static unsigned long irCurveStarted = 0;
  static unsigned long lastDebugTime = 0;

  // Handle calibration even when no valid EEPROM record is available.
  if (Serial.available())
  {
    const char command = static_cast<char>(Serial.read());
    if (command == 'c' || command == 'C')
    {
      leftMotor.stop();
      rightMotor.stop();
      irCurveActive = false;
      irTurnTriggered = false;
      if (!colourSensor.calibrate(Serial))
        Serial.println(F("Calibration failed or cancelled; previous model retained."));
      return;
    }
  }

  if (!colourSensor.isCalibrated())
  {
    leftMotor.stop();
    rightMotor.stop();
    return;
  }

  int irReading = readIRRaw();
  const unsigned long now = millis();
  if (now - lastDebugTime >= DEBUG_INTERVAL_MS)
  {
    lastDebugTime = now;
    Serial.print(F("IR raw: "));
    Serial.println(irReading);
  }
  // Separate trigger and reset thresholds prevent noise from rearming turns.
  if (!irCurveActive && irReading <= IR_RESET)
  {
    irTurnTriggered = false;
  }
  // delay(200); // Keep diagnostic output readable: five measurements per second.

  if (detectBlackLine())
  { // line follower
    irCurveActive = false;

    leftMotor.stop();
    rightMotor.stop();

    const auto reading = colourSensor.read();

    if (reading.label == RatioColourSensor::RED)
    {
      turnLeft();
      delay(100);
    }
    else if (reading.label == RatioColourSensor::GREEN)
    {
      turnRight();
      delay(100);
    }
    else if (reading.label == RatioColourSensor::ORANGE)
    {
      uTurn();
      delay(100);
    }
    else if (reading.label == RatioColourSensor::PINK)
    {
      doubleLeftTurn();
      delay(100);
    }
    else if (reading.label == RatioColourSensor::BLUE)
    {
      doubleRightTurn();
      delay(100);
    }
    else if (reading.label == RatioColourSensor::WHITE)
    {
      celebrate();
      exit(0); // Stop the program after celebration
    }
    // delay(1000);
    // uTurn();
    // celebrate();
    // delay(500);
  }
  else if (irCurveActive)
  {
    // Keep checking the line and serial commands while the curve runs.
    if (now - irCurveStarted >= IR_CURVE_MS)
    {
      irCurveActive = false;
      pid();
    }
  }
  else if (irReading > IR_TRIGGER && !irTurnTriggered)
  {
    const float wallDistance = readUltrasonic();
    if (wallDistance > 15.0f || wallDistance == 0)
    {
      Serial.println(F("IR turn triggered"));
      irTurnTriggered = true;
      irCurveActive = true;
      leftMotor.run(-motorSpeed);
      rightMotor.run(motorSpeed - 40);
      irCurveStarted = millis();
    }
    else
    {
      move();
    }
  }
  else
  {
    move();
  }
}
