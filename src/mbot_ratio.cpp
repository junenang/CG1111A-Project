#include <Arduino.h>
#include <MeMCore.h>
#include "RatioColourSensor.h"

namespace
{
  RatioColourSensor sensor;
  MeRGBLed onboardLeds(7, 2);
  bool indicatorOn = false;

  void stopMotors()
  {
    pinMode(6, OUTPUT);
    analogWrite(6, 0);
    pinMode(5, OUTPUT);
    analogWrite(5, 0);
  }

  void setIndicator(uint8_t r, uint8_t g, uint8_t b)
  {
    onboardLeds.setColor(0, r, g, b);
    onboardLeds.show();
    indicatorOn = r || g || b;
  }

  void prepareScan()
  {
    const bool wasOn = indicatorOn;
    setIndicator(0, 0, 0);
    if (wasOn)
      delay(RatioColourSensor::SETTLE_MS);
  }

  void showColour(RatioColourSensor::Label label)
  {
    switch (label)
    {
    case RatioColourSensor::RED:
      setIndicator(64, 0, 0);
      break;
    case RatioColourSensor::GREEN:
      setIndicator(0, 64, 0);
      break;
    case RatioColourSensor::ORANGE:
      setIndicator(64, 20, 0);
      break;
    case RatioColourSensor::PINK:
      setIndicator(64, 16, 32);
      break;
    case RatioColourSensor::BLUE:
      setIndicator(16, 40, 64);
      break;
    case RatioColourSensor::WHITE:
      setIndicator(48, 48, 48);
      break;
    default:
      setIndicator(0, 0, 0);
      break;
    }
  }

  void testExternalLEDs()
  {
    prepareScan();
    Serial.println(F("External RED"));
    sensor.shineRed();
    delay(1000);
    Serial.println(F("External GREEN"));
    sensor.shineGreen();
    delay(1000);
    Serial.println(F("External BLUE"));
    sensor.shineBlue();
    delay(1000);
    sensor.visibleLedsOff();
    Serial.println(F("Visible LEDs off (Y0 selected)."));
  }
} // namespace

// void setup() {
//   stopMotors();
//   Serial.begin(9600);
//   onboardLeds.setpin(13);
//   setIndicator(0, 0, 0);
//   sensor.setBeforeScan(prepareScan);
//   const bool loaded = sensor.begin();
//   Serial.println(F("mBot RGB-ratio test. Motors stopped. Serial: 9600 baud."));
//   Serial.println(F("l=LED test; c=calibrate; r=one raw/ratio reading and colour."));
//   Serial.println(F("Calibration prompts use g=capture, q=cancel. Uppercase also works."));
//   Serial.println(F("A0=LDR; A2/A3=decoder. Y0 may enable IR if connected."));
//   if (loaded) sensor.printCalibration(Serial);
//   else Serial.println(F("No saved ratio calibration. Send c; r still prints raw ADC."));
// }

// void loop() {
//   stopMotors();
//   if (!Serial.available()) return;
//   char command = static_cast<char>(Serial.read());
//   if (command >= 'A' && command <= 'Z') command += 'a' - 'A';
//   switch (command) {
//     case 'l': testExternalLEDs(); break;
//     case 'c':
//       prepareScan();
//       sensor.visibleLedsOff();
//       if (!sensor.calibrate(Serial))
//         Serial.println(F("No new ratio calibration saved. Check messages above."));
//       break;
//     case 'r': {
//       const RatioColourSensor::Reading reading = sensor.read();
//       RatioColourSensor::printReading(Serial, reading);
//       showColour(reading.label); // Indicator is displayed only after the scan.
//       break;
//     }
//     case '\r': case '\n': case ' ': case '\t': break;
//     default: Serial.println(F("Use l, c or r.")); break;
//   }
// }
