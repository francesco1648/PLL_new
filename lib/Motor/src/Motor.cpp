#include "Motor.h"

/**
 * Create object and set motor pins.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 * @param invert Invert motor direction, usuful when motors are mounted opposite to one another.
 */
Motor::Motor(byte pwm, byte dir, bool invert)
    : pwm(pwm), dir(dir), invert(invert)
{}

/**
 * Initialize motors.
 */
void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

/**
 * Sets the motor speed.
 * @param value Speed of the motor, ranging from 0 to maximum PWM value.
 */
void Motor::write(int value) {
  int mot = constrain(abs(value), 0, PWM_MAX_VALUE);
  analogWrite(pwm, mot);
  digitalWrite(dir, invert ^ (value < 0));
}
