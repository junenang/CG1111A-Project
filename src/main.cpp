#include "MeMCore.h"
#include "ultrasonic.h"

#define TURNING_TIME_MS 300 // The time duration (ms) for turning
#define adjustment 34.0     // course correction multiplier
#define pollRate 60         // course course correction poll rate in ms
MeDCMotor leftMotor(M1);    // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);   // assigning RightMotor to port M2
uint8_t motorSpeed = 255;
const uint8_t LINE_S1 = 9; // Line sensor: Port 2
const uint8_t LINE_S2 = 10;

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
  const float target = 9.0f;

  float distance = readUltrasonic();
  Serial.println("Distance: " + String(distance) + " cm");
  // No echo or a distant return: continue forward through the opening.
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
  delay(1.85 * TURNING_TIME_MS); // Keep turning for this time duration
  leftMotor.stop();             // Stop left motor
  rightMotor.stop();            // Stop right motor
}

void doubleLeftTurn()
{
  turnLeft();     // Turn left for 287 ms
  delay(100);     // Wait for 100 ms
  goForward(750); // Move forward for 750 ms
  delay(100);     // Wait for 100 ms
  turnLeft();     // Turn left for 287 ms
}

void doubleRightTurn()
{
  turnRight();    // Turn right for 287 ms
  delay(100);     // Wait for 100 ms
  goForward(750); // Move forward for 750 ms
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
  delay(3000); // Do nothing for 10000 ms = 10 seconds
  move();      // The motors keep running until explicitly stopped.
}

void loop()
{
  if (detectBlackLine())
  { // line follower

    leftMotor.stop();
    rightMotor.stop();

    delay(1000);
    uTurn();
    celebrate();
    delay(500);
  }
  else
  {
    move();
  }
}
