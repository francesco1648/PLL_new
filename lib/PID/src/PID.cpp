#include "PID.h"

/**
 * Initializes the PID class with necessary parameters. 
 * @param kp Proportional gain.
 * @param ki Integrative gain.
 * @param kd Derivative gain.
 * @param max_output To define the range of the output, from -max_output to max_output.
 * @param alpha Error filte, 1 to disable filtering.
 */
PID::PID(float kp, float ki, float kd, float max_output, float alpha) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->max_output = max_output;
  this->alpha = alpha;

  old_integral = 0;
  old_fe = 0;
  old_error = 0;
  tempo = millis();
}

/**
 * Sets the reference value.
 * @param ref Reference value for the PID.
 */
void PID::updateReferenceValue(float ref) {
  referenceValue = ref;
}

/**
 * Updates the feedback for PID computation.
 * @param fb Feedback value.
 */
void PID::updateFeedback(float fb) {
  feedback = fb;
}

/**
 * Sets the proportional gain.
 * @param kp Value of the proportional gain.
 */
void PID::setKp(float kp) {
  this->kp = kp;
}

/**
 * Sets the integrative gain.
 * @param ki Value of the integrative gain.
 */
void PID::setKi(float ki) {
  this->ki = ki;
}

/**
 * Sets the derivative gain.
 * @param kd Value of the derivative gain.
 */
void PID::setKd(float kd) {
  this->kd = kd;
}

/**
 * Gets output computed from the last iteration of calculate()
 * @return float Output value of the PID controller.
 */
float PID::getOutput() {
  return output;
}

/**
 * Gets the currently used reference value.
 * @return float The currently set reference value.
 */
float PID::getReferenceValue() {
  return referenceValue;
}

/**
 * Resets the PID state, as if it was the first iteration.
 */
void PID::resetState() {
  old_fe = 0.f;
  old_error = 0.f;
  old_integral = 0.f;
}

/**
 * Main PID routine. 
 * Should be executed at a fixed frequency, which will influence the behaviour for a given set of gains.
 */
void PID::calculate() {
  float dt;
  float error, fe, derivative, integral;
  
  dt = (float)(millis() - tempo) / 1000.f; // sarà fatto a frequenza fissata, andrà eliminato e magari aggiunto come parametro al costruttore

  // e[k] = r[k] - y[k], error between setpoint and true position
  error = referenceValue - feedback;

  // e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
  fe = alpha * error + (1 - alpha) * old_fe;
  
  // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
  derivative = (fe - old_fe) / dt;
  
  // e_i[k+1] = e_i[k] + Tₛ e[k], integral
  integral = old_integral + ki * old_error * dt;

  // PID formula:
  // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
  output = referenceValue + kp * error + integral + kd * derivative;

  // Clamp the output
  if (output > max_output)
    output = max_output;
  else if (output < -max_output)
    output = -max_output;
  else // Anti-windup
    old_integral = integral;

  // store the state for the next iteration
  old_error = error;
  old_fe = fe;
  tempo = millis();
}
