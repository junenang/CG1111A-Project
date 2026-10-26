#include <Arduino.h>
#include "ultrasonic.h"

const uint8_t US_PIN = 12; // Ultrasonic: Port 1
const unsigned long US_TIMEOUT_US = 5000;
const unsigned long US_INTERVAL_MS = 60;

float readUltrasonic()
{
    static unsigned long lastReadMs = 0;
    static float lastResult = 0;
    static bool firstRead = true;

    unsigned long now = millis();

    // Reuse the previous result between measurements.
    if (!firstRead && now - lastReadMs < US_INTERVAL_MS)
    {
        return lastResult;
    }

    firstRead = false;
    lastReadMs = now;

    pinMode(US_PIN, OUTPUT);
    digitalWrite(US_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(US_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_PIN, LOW);

    pinMode(US_PIN, INPUT);

    unsigned long duration =
        pulseIn(US_PIN, HIGH, US_TIMEOUT_US);

    if (duration == 0)
    {
        lastResult = 0;
        return lastResult;
    }

    float distanceCm = duration / 58.0;
    lastResult = distanceCm;

    return lastResult;
}
