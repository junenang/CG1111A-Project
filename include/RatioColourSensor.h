#ifndef MBOT_RATIO_COLOUR_SENSOR_H
#define MBOT_RATIO_COLOUR_SENSOR_H

#include <Arduino.h>
#include <stdint.h>

// +5 V -> fixed resistor -> A0 -> LDR -> GND.
// A2/A3 select decoder outputs: 0=visible off, 1=green, 2=blue, 3=red.
// Selecting output 0 also enables IR if Y0 is connected to the IR driver.
class RatioColourSensor
{
public:
  static const uint8_t CHANNELS = 3;
  static const uint8_t COLOURS = 7;
  static const uint8_t PIN_A = A2;
  static const uint8_t PIN_B = A3;
  static const uint8_t PIN_LDR = A0;
  static const uint16_t SETTLE_MS = 1000;
  static const uint8_t ADC_SAMPLES = 10;
  static const uint8_t SAMPLE_GAP_MS = 8;
  static const uint8_t CAL_FRAMES = 3;
  // Old nearest-profile test uses bytes 0..149. This test has its own record.
  static const int EEPROM_BASE = 256;

  enum Label : uint8_t
  {
    BLACK,
    WHITE,
    RED,
    GREEN,
    ORANGE,
    PINK,
    BLUE,
    UNKNOWN_COLOUR
  };
  enum Reason : uint8_t
  {
    MATCHED,
    NO_CALIBRATION,
    BAD_ADC,
    BELOW_REFERENCE,
    NEUTRAL_BUT_NOT_WHITE,
    AMBIGUOUS_RATIO
  };

  struct Model
  {
    float black[CHANNELS];
    float white[CHANNELS];
    float blueMinB;
    float greenMinG;
    float pinkMinB;
    float orangeMinG;
    float blackMaxBrightness;
    float whiteMinBrightness;
    float whiteMaxSaturation;
    // Diagnostics only: no nearest-centre distances or acceptance radii.
    float paperRatios[5][CHANNELS];
  };

  struct Reading
  {
    float raw[CHANNELS];
    float scaled[CHANNELS]; // Black=0, white=255; upper values stay unclipped.
    float ratios[CHANNELS]; // Fractions of positive response, sum=1.
    float brightness;       // Mean positive scaled response / 255.
    float saturation;       // (largest - smallest positive response)/largest.
    bool hasRatios;
    Label label;
    Reason reason;
  };

  RatioColourSensor();
  bool begin();
  bool isCalibrated() const;
  void setBeforeScan(void (*callback)());
  void visibleLedsOff();
  void shineRed();
  void shineGreen();
  void shineBlue();
  void readRawRGB(float raw[CHANNELS]);
  Reading read();
  const char *readColour(); // One RGB scan; suitable for later use in main.
  bool calibrate(Stream &console);
  bool loadCalibration();
  void printCalibration(Stream &console) const;
  static void printReading(Stream &console, const Reading &reading);
  static const char *name(Label label);
  static const __FlashStringHelper *reasonText(Reason reason);

  // Pure calculation entry points also used by the automated checks.
  static Reading evaluate(const Model &model, const float raw[CHANNELS]);
  static bool trainModel(Model &model,
                         const float samples[COLOURS][CAL_FRAMES][CHANNELS],
                         Stream *console = nullptr);
  static bool validModel(const Model &model);

private:
  struct Record
  {
    uint32_t magic;
    uint16_t version;
    uint16_t settleMs;
    uint8_t samples;
    uint8_t gapMs;
    uint8_t frames;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pinLdr;
    uint8_t polarity;
    Model model;
    uint16_t checksum;
  };

  Record record_;
  bool ready_;
  void (*beforeScan_)();

  void selectOutput(uint8_t output);
  static bool prompt(Stream &console, Label label);
  static void makeRecord(Record &record);
  static bool validRecord(const Record &record);
  static uint16_t checksum(const Record &record);
  static void printTriple(Stream &console, const float values[CHANNELS],
                          float multiplier = 1.0f);
};

#endif
