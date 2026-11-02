#include "RatioColourSensor.h"

#include <EEPROM.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

namespace
{
  const uint32_t kMagic = 0x52544243UL;
  const uint16_t kVersion = 1;
  const float kMinAdcSpan = 150.0f;     // Retain the current test's reference limit.
  const float kRuleMargin = 0.005f;     // 0.5 percentage point boundary guard.
  const float kDominanceMargin = 0.01f; // Avoid accepting nearly equal channels.
  const float kMinimumTrainingGap = 0.02f;
  const float kNegativeTolerance = 0.15f;

  float maximum(float a, float b) { return a > b ? a : b; }
  float minimum(float a, float b) { return a < b ? a : b; }

  bool validReferences(const RatioColourSensor::Model &model)
  {
    for (uint8_t c = 0; c < 3; ++c)
    {
      if (!isfinite(model.black[c]) || !isfinite(model.white[c]) ||
          model.black[c] < 0 || model.black[c] > 1023 ||
          model.white[c] < 0 || model.white[c] > 1023 ||
          model.black[c] - model.white[c] <= kMinAdcSpan)
        return false;
    }
    return true;
  }

  bool features(const RatioColourSensor::Model &model, const float raw[3],
                RatioColourSensor::Reading &result)
  {
    memset(&result, 0, sizeof(result));
    result.label = RatioColourSensor::UNKNOWN_COLOUR;
    result.reason = RatioColourSensor::NO_CALIBRATION;
    memcpy(result.raw, raw, sizeof(result.raw));
    if (!validReferences(model))
      return false;

    float positive[3];
    float sum = 0;
    bool saturated = false;
    bool belowReference = false;
    for (uint8_t c = 0; c < 3; ++c)
    {
      if (!isfinite(raw[c]) || raw[c] < 0 || raw[c] > 1023)
      {
        result.reason = RatioColourSensor::BAD_ADC;
        return false;
      }
      saturated = saturated || raw[c] <= 1 || raw[c] >= 1022;
      const float response = (model.black[c] - raw[c]) /
                             (model.black[c] - model.white[c]);
      result.scaled[c] = 255.0f * response;
      belowReference = belowReference || response < -kNegativeTolerance;
      // Small negative noise is excluded from proportions. Do not cap at white.
      positive[c] = maximum(0.0f, response);
      sum += positive[c];
    }
    result.brightness = sum / 3.0f;
    const float high = maximum(positive[0], maximum(positive[1], positive[2]));
    const float low = minimum(positive[0], minimum(positive[1], positive[2]));
    result.saturation = high > 0 ? (high - low) / high : 0;
    result.hasRatios = sum > 0.000001f;
    if (result.hasRatios)
    {
      for (uint8_t c = 0; c < 3; ++c)
        result.ratios[c] = positive[c] / sum;
    }
    result.reason = saturated ? RatioColourSensor::BAD_ADC : (belowReference ? RatioColourSensor::BELOW_REFERENCE : RatioColourSensor::MATCHED);
    return result.reason == RatioColourSensor::MATCHED;
  }

  struct Bounds
  {
    float minG, maxG, minB, maxB;
    float minBrightness, maxBrightness, minSat, maxSat;
  };

  bool split(float otherMax, float targetMin, float &boundary,
             Stream *console, const __FlashStringHelper *description)
  {
    if (targetMin - otherMax < kMinimumTrainingGap)
    {
      if (console)
      {
        console->print(F("Calibration failed: insufficient ratio separation for "));
        console->println(description);
        console->print(F("Other maximum / target minimum (%): "));
        console->print(100.0f * otherMax, 1);
        console->print(F(" / "));
        console->println(100.0f * targetMin, 1);
      }
      return false;
    }
    boundary = 0.5f * (otherMax + targetMin);
    return true;
  }
} // namespace

RatioColourSensor::RatioColourSensor() : ready_(false), beforeScan_(nullptr)
{
  memset(&record_, 0, sizeof(record_));
}

bool RatioColourSensor::begin()
{
  digitalWrite(PIN_A, LOW);
  digitalWrite(PIN_B, LOW);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_LDR, INPUT);
  digitalWrite(PIN_LDR, LOW); // Ensure the internal pull-up is disabled.
  visibleLedsOff();
  return loadCalibration();
}

bool RatioColourSensor::isCalibrated() const { return ready_; }
void RatioColourSensor::setBeforeScan(void (*callback)()) { beforeScan_ = callback; }
void RatioColourSensor::selectOutput(uint8_t output)
{
  digitalWrite(PIN_A, (output & 1) ? HIGH : LOW);
  digitalWrite(PIN_B, (output & 2) ? HIGH : LOW);
}
void RatioColourSensor::visibleLedsOff() { selectOutput(0); }
void RatioColourSensor::shineRed() { selectOutput(3); }
void RatioColourSensor::shineGreen() { selectOutput(1); }
void RatioColourSensor::shineBlue() { selectOutput(2); }

void RatioColourSensor::readRawRGB(float raw[CHANNELS])
{
  if (beforeScan_)
    beforeScan_();
  const uint8_t outputs[3] = {3, 1, 2};
  for (uint8_t c = 0; c < 3; ++c)
  {
    selectOutput(outputs[c]);
    delay(SETTLE_MS);
    analogRead(PIN_LDR);
    uint32_t total = 0;
    for (uint8_t s = 0; s < ADC_SAMPLES; ++s)
    {
      total += analogRead(PIN_LDR);
      delay(SAMPLE_GAP_MS);
    }
    raw[c] = static_cast<float>(total) / ADC_SAMPLES;
  }
  visibleLedsOff();
}

RatioColourSensor::Reading RatioColourSensor::read()
{
  float raw[3];
  readRawRGB(raw);
  if (ready_)
    return evaluate(record_.model, raw);
  Reading result = {};
  memcpy(result.raw, raw, sizeof(raw));
  result.label = UNKNOWN_COLOUR;
  result.reason = NO_CALIBRATION;
  return result;
}

const char *RatioColourSensor::readColour() { return name(read().label); }

bool RatioColourSensor::validModel(const Model &model)
{
  if (!validReferences(model))
    return false;
  const float boundaries[] = {model.blueMinB, model.greenMinG, model.pinkMinB,
                              model.orangeMinG, model.whiteMaxSaturation};
  for (uint8_t i = 0; i < 5; ++i)
  {
    if (!isfinite(boundaries[i]) || boundaries[i] <= 0 || boundaries[i] >= 1)
      return false;
  }
  if (model.pinkMinB >= model.blueMinB ||
      model.orangeMinG >= model.greenMinG ||
      !isfinite(model.blackMaxBrightness) || model.blackMaxBrightness <= 0 ||
      !isfinite(model.whiteMinBrightness) ||
      model.whiteMinBrightness <= model.blackMaxBrightness ||
      model.whiteMinBrightness > 1.5f)
    return false;
  for (uint8_t i = 0; i < 5; ++i)
  {
    float sum = 0;
    for (uint8_t c = 0; c < 3; ++c)
    {
      const float r = model.paperRatios[i][c];
      if (!isfinite(r) || r < 0 || r > 1)
        return false;
      sum += r;
    }
    if (fabsf(sum - 1.0f) > 0.01f)
      return false;
  }
  return true;
}

RatioColourSensor::Reading RatioColourSensor::evaluate(const Model &model,
                                                       const float raw[CHANNELS])
{
  Reading result;
  if (!features(model, raw, result))
    return result;
  if (!validModel(model))
  {
    result.reason = NO_CALIBRATION;
    return result;
  }
  if (result.brightness <= model.blackMaxBrightness)
  {
    result.label = BLACK;
    return result;
  }
  // White must be sufficiently balanced, with no excessive red share.
  // This extra ratio check helps keep pale pink separate from white.
  if (result.saturation <= model.whiteMaxSaturation &&
      result.ratios[0] <= 0.35f)
  {
    if (result.brightness >= model.whiteMinBrightness)
    {
      result.label = WHITE;
    }
    else
    {
      result.reason = NEUTRAL_BUT_NOT_WHITE;
    }
    return result;
  }
  const float r = result.ratios[0], g = result.ratios[1], b = result.ratios[2];
  result.reason = AMBIGUOUS_RATIO;
  // These are ordered ratio rules learned during calibration, not distances.
  if (fabsf(b - model.blueMinB) <= kRuleMargin)
    return result;
  if (b > model.blueMinB)
  {
    if (b > r + kDominanceMargin && b > g + kDominanceMargin)
      result.label = BLUE;
  }
  else
  {
    if (fabsf(g - model.greenMinG) <= kRuleMargin)
      return result;
    if (g > model.greenMinG)
    {
      if (g > r + kDominanceMargin && g > b + kDominanceMargin)
        result.label = GREEN;
    }
    else if (r > g + kDominanceMargin && r > b + kDominanceMargin)
    {
      if (fabsf(b - model.pinkMinB) <= kRuleMargin)
        return result;
      if (b > model.pinkMinB)
        result.label = PINK;
      else
      {
        if (fabsf(g - model.orangeMinG) <= kRuleMargin)
          return result;
        result.label = g > model.orangeMinG ? ORANGE : RED;
      }
    }
  }
  if (result.label != UNKNOWN_COLOUR)
    result.reason = MATCHED;
  return result;
}

bool RatioColourSensor::trainModel(Model &model,
                                   const float samples[COLOURS][CAL_FRAMES][CHANNELS], Stream *console)
{
  memset(&model, 0, sizeof(model));
  for (uint8_t c = 0; c < 3; ++c)
  {
    for (uint8_t f = 0; f < CAL_FRAMES; ++f)
    {
      model.black[c] += samples[BLACK][f][c] / CAL_FRAMES;
      model.white[c] += samples[WHITE][f][c] / CAL_FRAMES;
    }
  }
  if (!validReferences(model))
  {
    if (console)
    {
      console->print(F("Calibration failed: each BLACK-WHITE ADC span must exceed "));
      console->println(kMinAdcSpan, 0);
    }
    return false;
  }

  Bounds bounds[COLOURS];
  for (uint8_t label = 0; label < COLOURS; ++label)
  {
    Bounds &v = bounds[label];
    v.minG = v.minB = v.minBrightness = v.minSat = INFINITY;
    v.maxG = v.maxB = v.maxBrightness = v.maxSat = -INFINITY;
    for (uint8_t f = 0; f < CAL_FRAMES; ++f)
    {
      Reading measured;
      if (!features(model, samples[label][f], measured))
      {
        if (console)
          console->println(F("Calibration failed: ADC saturation or reference drift."));
        return false;
      }
      if (label != BLACK && !measured.hasRatios)
        return false;
      v.minG = minimum(v.minG, measured.ratios[1]);
      v.maxG = maximum(v.maxG, measured.ratios[1]);
      v.minB = minimum(v.minB, measured.ratios[2]);
      v.maxB = maximum(v.maxB, measured.ratios[2]);
      v.minBrightness = minimum(v.minBrightness, measured.brightness);
      v.maxBrightness = maximum(v.maxBrightness, measured.brightness);
      v.minSat = minimum(v.minSat, measured.saturation);
      v.maxSat = maximum(v.maxSat, measured.saturation);
      if (label >= RED)
      {
        for (uint8_t c = 0; c < 3; ++c)
          model.paperRatios[label - RED][c] += measured.ratios[c] / CAL_FRAMES;
      }
    }
  }

  float otherMaxB = 0, otherMaxG = 0;
  float colouredMaxBrightness = 0, colouredMinBrightness = INFINITY;
  float colouredMinSat = INFINITY;
  for (uint8_t label = WHITE; label < COLOURS; ++label)
  {
    if (label != BLUE)
      otherMaxB = maximum(otherMaxB, bounds[label].maxB);
    if (label != GREEN && label != BLUE)
      otherMaxG = maximum(otherMaxG, bounds[label].maxG);
    if (label >= RED)
    {
      colouredMaxBrightness = maximum(colouredMaxBrightness, bounds[label].maxBrightness);
      colouredMinBrightness = minimum(colouredMinBrightness, bounds[label].minBrightness);
      colouredMinSat = minimum(colouredMinSat, bounds[label].minSat);
    }
  }
  if (!split(otherMaxB, bounds[BLUE].minB, model.blueMinB, console, F("blue")) ||
      !split(otherMaxG, bounds[GREEN].minG, model.greenMinG, console, F("green")) ||
      !split(maximum(bounds[RED].maxB, bounds[ORANGE].maxB), bounds[PINK].minB,
             model.pinkMinB, console, F("pink")) ||
      !split(bounds[RED].maxG, bounds[ORANGE].minG,
             model.orangeMinG, console, F("red/orange")))
    return false;

  model.blackMaxBrightness = maximum(0.10f, bounds[BLACK].maxBrightness + 0.03f);
  if (colouredMinBrightness <= model.blackMaxBrightness + 0.05f ||
      bounds[WHITE].minBrightness - colouredMaxBrightness < 0.02f ||
      colouredMinSat - bounds[WHITE].maxSat < 0.02f)
  {
    if (console)
      console->println(F("Calibration failed: white/black brightness or white/colour balance overlaps."));
    return false;
  }
  // Allow the white variation observed across the four test rounds.
  model.whiteMinBrightness = minimum(
      0.85f,
      0.5f * (bounds[WHITE].minBrightness + colouredMaxBrightness));

  model.whiteMaxSaturation = maximum(
      0.20f,
      0.5f * (bounds[WHITE].maxSat + colouredMinSat));
  if (!validModel(model))
    return false;

  // The rule order and dominance guards must also recognise the training data.
  for (uint8_t label = 0; label < COLOURS; ++label)
  {
    for (uint8_t f = 0; f < CAL_FRAMES; ++f)
    {
      if (evaluate(model, samples[label][f]).label != static_cast<Label>(label))
      {
        if (console)
        {
          console->print(F("Calibration failed: ratio rules cannot consistently separate "));
          console->println(name(static_cast<Label>(label)));
        }
        return false;
      }
    }
  }
  return true;
}

bool RatioColourSensor::prompt(Stream &console, Label label)
{
  console.print(F("Place "));
  if (label == BLUE)
    console.print(F("LIGHT BLUE"));
  else
    console.print(name(label));
  console.println(F(" paper; send g to capture, or q to cancel."));
  console.println(F("Keep it still until the NEXT paper prompt or completion message."));
  while (true)
  {
    if (console.available())
    {
      const int ch = console.read();
      if (ch == 'g' || ch == 'G')
        return true;
      if (ch == 'q' || ch == 'Q')
        return false;
    }
    delay(1);
  }
}

void RatioColourSensor::makeRecord(Record &record)
{
  memset(&record, 0, sizeof(record));
  record.magic = kMagic;
  record.version = kVersion;
  record.settleMs = SETTLE_MS;
  record.samples = ADC_SAMPLES;
  record.gapMs = SAMPLE_GAP_MS;
  record.frames = CAL_FRAMES;
  record.pinA = PIN_A;
  record.pinB = PIN_B;
  record.pinLdr = PIN_LDR;
  record.polarity = 1;
}

uint16_t RatioColourSensor::checksum(const Record &record)
{
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&record);
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < offsetof(Record, checksum); ++i)
  {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}

bool RatioColourSensor::validRecord(const Record &r)
{
  return r.magic == kMagic && r.version == kVersion && r.settleMs == SETTLE_MS &&
         r.samples == ADC_SAMPLES && r.gapMs == SAMPLE_GAP_MS && r.frames == CAL_FRAMES &&
         r.pinA == PIN_A && r.pinB == PIN_B && r.pinLdr == PIN_LDR && r.polarity == 1 &&
         r.checksum == checksum(r) && validModel(r.model);
}

bool RatioColourSensor::loadCalibration()
{
  ready_ = false;
  if (EEPROM_BASE + static_cast<int>(sizeof(record_)) > EEPROM.length())
    return false;
  EEPROM.get(EEPROM_BASE, record_);
  ready_ = validRecord(record_);
  return ready_;
}

bool RatioColourSensor::calibrate(Stream &console)
{
  Record candidate;
  makeRecord(candidate);
  float samples[COLOURS][CAL_FRAMES][CHANNELS];
  const Label order[COLOURS] = {WHITE, BLACK, RED, GREEN, ORANGE, PINK, BLUE};
  console.println(F("Ratio calibration: fixed mounting, height, shielding and power."));
  const unsigned long captureMs = static_cast<unsigned long>(CAL_FRAMES) * 3UL *
                                  (SETTLE_MS + static_cast<unsigned long>(ADC_SAMPLES) * SAMPLE_GAP_MS);
  console.print(F("Each paper takes about "));
  console.print((captureMs + 999) / 1000);
  console.println(F(" seconds. Do not move it until the next prompt."));
  for (uint8_t i = 0; i < COLOURS; ++i)
  {
    if (!prompt(console, order[i]))
    {
      visibleLedsOff();
      console.println(F("Cancelled; previous ratio calibration retained."));
      return false;
    }
    for (uint8_t f = 0; f < CAL_FRAMES; ++f)
      readRawRGB(samples[order[i]][f]);
    if (order[i] == BLACK)
    {
      console.println(F("White / black / BLACK-WHITE ADC span (R,G,B rows):"));
      for (uint8_t c = 0; c < 3; ++c)
      {
        for (uint8_t f = 0; f < CAL_FRAMES; ++f)
        {
          candidate.model.white[c] += samples[WHITE][f][c] / CAL_FRAMES;
          candidate.model.black[c] += samples[BLACK][f][c] / CAL_FRAMES;
        }
        console.print(candidate.model.white[c], 1);
        console.print(F(" / "));
        console.print(candidate.model.black[c], 1);
        console.print(F(" / "));
        console.println(candidate.model.black[c] - candidate.model.white[c], 1);
      }
      if (!validReferences(candidate.model))
      {
        console.print(F("Calibration failed: each ADC span must exceed "));
        console.println(kMinAdcSpan, 0);
        return false;
      }
    }
  }
  if (!trainModel(candidate.model, samples, &console))
    return false;
  if (EEPROM_BASE + static_cast<int>(sizeof(candidate)) > EEPROM.length())
  {
    console.println(F("Calibration failed: insufficient EEPROM space."));
    return false;
  }
  candidate.checksum = checksum(candidate);
  EEPROM.update(EEPROM_BASE, 0); // Commit the magic byte last.
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&candidate);
  for (size_t i = 1; i < sizeof(candidate); ++i)
    EEPROM.update(EEPROM_BASE + i, bytes[i]);
  EEPROM.update(EEPROM_BASE, bytes[0]);
  // Reuse the candidate buffer to avoid a second record on the AVR's small stack.
  EEPROM.get(EEPROM_BASE, candidate);
  if (!validRecord(candidate))
  {
    console.println(F("Calibration failed: EEPROM verification. Previous RAM model retained."));
    return false;
  }
  record_ = candidate;
  ready_ = true;
  console.println(F("Ratio calibration saved; it will load after reset."));
  printCalibration(console);
  return true;
}

void RatioColourSensor::printTriple(Stream &console, const float values[CHANNELS],
                                    float multiplier)
{
  for (uint8_t c = 0; c < CHANNELS; ++c)
  {
    console.print(values[c] * multiplier, 1);
    if (c < 2)
      console.print(',');
  }
  console.println();
}

void RatioColourSensor::printCalibration(Stream &console) const
{
  if (!ready_)
  {
    console.println(F("No valid ratio calibration."));
    return;
  }
  console.print(F("Ratio EEPROM address / bytes: "));
  console.print(EEPROM_BASE);
  console.print(F(" / "));
  console.println(static_cast<unsigned int>(sizeof(record_)));
  console.print(F("Black raw R,G,B: "));
  printTriple(console, record_.model.black);
  console.print(F("White raw R,G,B: "));
  printTriple(console, record_.model.white);
  for (uint8_t label = RED; label <= BLUE; ++label)
  {
    console.print(name(static_cast<Label>(label)));
    console.print(F(" ratios (%): "));
    printTriple(console, record_.model.paperRatios[label - RED], 100.0f);
  }
  const Model &m = record_.model;
  console.print(F("Learned boundaries (%): blue B>"));
  console.print(100 * m.blueMinB, 1);
  console.print(F("; green G>"));
  console.print(100 * m.greenMinG, 1);
  console.print(F("; pink B>"));
  console.print(100 * m.pinkMinB, 1);
  console.print(F("; orange G>"));
  console.println(100 * m.orangeMinG, 1);
  console.print(F("Brightness black<= / white>= (% of reference): "));
  console.print(100 * m.blackMaxBrightness, 1);
  console.print(F(" / "));
  console.println(100 * m.whiteMinBrightness, 1);
  console.print(F("White maximum saturation (%): "));
  console.println(100 * m.whiteMaxSaturation, 1);
}

void RatioColourSensor::printReading(Stream &console, const Reading &r)
{
  console.print(F("Raw R,G,B: "));
  printTriple(console, r.raw);
  if (r.reason != NO_CALIBRATION)
  {
    console.print(F("Scaled sensor R,G,B: "));
    printTriple(console, r.scaled);
    if (r.hasRatios)
    {
      console.print(F("Ratio R,G,B (%): "));
      printTriple(console, r.ratios, 100.0f);
    }
    else
      console.println(F("Ratio R,G,B: n/a (no positive light response)"));
    console.print(F("Brightness / saturation (%): "));
    console.print(100 * r.brightness, 1);
    console.print(F(" / "));
    console.println(100 * r.saturation, 1);
  }
  console.print(F("Colour = "));
  console.println(name(r.label));
  console.print(F("Reason: "));
  console.println(reasonText(r.reason));
}

const char *RatioColourSensor::name(Label label)
{
  switch (label)
  {
  case BLACK:
    return "black";
  case WHITE:
    return "white";
  case RED:
    return "red";
  case GREEN:
    return "green";
  case ORANGE:
    return "orange";
  case PINK:
    return "pink";
  case BLUE:
    return "blue";
  default:
    return "unknown";
  }
}

const __FlashStringHelper *RatioColourSensor::reasonText(Reason reason)
{
  switch (reason)
  {
  case MATCHED:
    return F("ratio/brightness rule matched");
  case NO_CALIBRATION:
    return F("no matching ratio calibration; send c");
  case BAD_ADC:
    return F("invalid or saturated ADC reading");
  case BELOW_REFERENCE:
    return F("reading well below calibrated black; check reference drift");
  case NEUTRAL_BUT_NOT_WHITE:
    return F("neutral reading is too dim for calibrated white");
  default:
    return F("near a ratio boundary or lacks expected channel dominance");
  }
}
