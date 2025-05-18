#include "SmartMotor.h"
#include  "PID_AutoTune_v0.h"




/**
 * Create SmartMotor object, creating all necessary objects.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 * @param enc_a Pin A of the encoder.
 * @param enc_b Pin B of the encoder.
 * @param invert Invert motor direction, usuful when motors are mounted opposite to one another.
 * @param pio PIO to use for the encoder. Each PIO can handle up to 4 encoders.
 */
SmartMotor::SmartMotor(byte pwm, byte dir, byte enc_a, byte enc_b, bool invert, PIO pio)
    : motor(pwm, dir, invert),
      encoder(enc_a, enc_b, new MovingAvgFilter<int>(ENC_TR_SAMPLES), invert, pio),
      pid(0.f, 0.f, 0.f, MAX_SPEED, 1.f),
      invert(invert)
{}

/**
 * Initialize SmartMotor and necessary components.
 */
void SmartMotor::begin() {
    motor.begin();
    encoder.begin();
}

/**
 * Update routine, updating the PID and the motor speed.
 * This function will be executed at a fixed rate, defined by DT_PID, and should therefore be called as often as possible.
 */
void SmartMotor::update() {
    unsigned long now = millis();
    if(now - pid_last > DT_PID) {
        pid.updateFeedback(getSpeed());
        pid.calculate();
        motor.write(speedToPower(pid.getOutput()));
        pid_last = now;
    }
}

/**
 * Set the desired speed of the motor.
 * @param value Desired motor speed between -MAX_SPEED and MAX_SPEED.
 */
void SmartMotor::setSpeed(float value) {
    pid.updateReferenceValue(value);
}

/**
 * Get the current speed of the motor.
 * The value is only updated at a fixed rate, defined by DT_ENC, to avoid losing precision.
 * @return float Current speed of the motor between -MAX_SPEED and MAX_SPEED.
 */
float SmartMotor::getSpeed() {
    unsigned long now = millis();
    if(now - enc_last > DT_ENC) {
        speed = (float)(encoder.getSpeed())/100.f;
        enc_last = now;
    }
    return speed;
}

/**
 * Stop the motor.
 * This function will stop the motor and reset the PID.
 */
void SmartMotor::stop() {
    motor.write(0);
    pid.updateReferenceValue(0.f);
    pid.resetState();
}

/**
 * Calibrate the PID controller.
 * This function will set the PID parameters to values that should work for the motor.
 * The method used is based on Åström–Hägglund tuning method, while using Ziegler-Nichols formulas to compute the gains.
 * Only the Kp and Ki gains are computed while the Kd gain is set to 0 since it doesn't have a positive effect on controlling the motor.
 * @param target Target speed to use for calibration.
 */
double input = 0;
double output = 0;
double setpoint = 100;
PID_ATune aTune(&input, &output);

void SmartMotor::calibrate() {
  aTune.SetOutputStep(50);     // quanto spinge ogni step
  aTune.SetControlType(1);     // 0 = PI, 1 = PID
  aTune.SetLookbackSec(10);    // quanto tempo osserva la risposta
  aTune.SetNoiseBand(1);       // tolleranza sul rumore
  //aTune.sampleTime(100);    // ogni quanto valuta
  //aTune.SetRelayStep(50);      // valore step di output

  unsigned long start = millis();
  while (!aTune.Runtime()) {
    input = getSpeed();              // aggiorna lettura
    motor.write(output);             // imposta motore
    delay(100);                      // uguale a sample time
    if (millis() - start > 20000) {  // timeout
      Serial.println("Autotune timeout");
      break;
    }
  }

  // Se tuning riuscito, salva valori
  float Kp = aTune.GetKp();
  float Ki = aTune.GetKi();
  float Kd = aTune.GetKd();

  if (Kp > 0) {
    pid.setKp(Kp);
    pid.setKi(Ki);
    pid.setKd(Kd);
    Serial.printf("PID Calibrato: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
  } else {
    Serial.println("Autotune fallito");
  }

  motor.write(0); // ferma il motore
}
/**
 * Converts speed to power.
 * This function linearly scales the speed to the PWM duty cycle, without taking into account the motor's.
 * @param speed Theretical speed of the motor.
 * @return int PWM value to set the motor to.
 */
int SmartMotor::speedToPower(float speed) {
    return (speed/MAX_SPEED)*PWM_MAX_VALUE;
}
