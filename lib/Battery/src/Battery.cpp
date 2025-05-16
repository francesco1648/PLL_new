#include "Battery.h"

/**
 * Reads the current battery voltage.
 * @return Battery voltage in volts.
 */
float Battery::readVoltage() {
  float vpart = analogRead(pin) * (3.3f / 1023.0f);
  return vpart * ((r1 + r2)/r2);
}

/**
 * Estimates the current battery charge.
 * @return Battery charge percentage.
 */
float Battery::chargePercent() {
  float charge = (readVoltage()-BAT_LOW) / (BAT_NOM-BAT_LOW);
  return charge * 100.f;
}

/**
 * Check the battery charge status.
 * @return true if the battery is still charged.
 */
bool Battery::charged() {
  return readVoltage() > BAT_LOW;
}

/** 
 * Check if the Pico is being powered from the USB port.
 * @return true if USB is providing power
 */
bool Battery::USB() {
  return digitalRead(34);
}

Battery battery;
