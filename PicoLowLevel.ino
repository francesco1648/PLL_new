
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "DynamixelSerial.h"
#include "TractionEncoder.h"
#include "MovingAvgFilter.h"
#include "ExpSmoothingFilter.h"
#include "Debug.h"
#include "mcp2515.h"
#include "Display.h"
#include "SmartMotor.h"
#include "Motor.h"
#include "PID.h"
#include "CanWrapper.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"



void okInterrupt();
void navInterrupt();
void sendFeedback();
void handleSetpoint(uint8_t msg_id, const byte* msg_data);

int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;

CanWrapper canW(5, 10000000UL, &SPI);

SmartMotor motorTrLeft(DRV_TR_LEFT_PWM, DRV_TR_LEFT_DIR, ENC_TR_LEFT_A, ENC_TR_LEFT_B, false);
SmartMotor motorTrRight(DRV_TR_RIGHT_PWM, DRV_TR_RIGHT_DIR, ENC_TR_RIGHT_A, ENC_TR_RIGHT_B, true);


#ifdef MODC_YAW
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
#endif


#ifdef MODC_EE
DynamixelMotor motorEEPitch(SERVO_EE_PITCH_ID);
DynamixelMotor motorEEHeadPitch(SERVO_EE_HEAD_PITCH_ID);
DynamixelMotor motorEEHeadRoll(SERVO_EE_HEAD_ROLL_ID);
#endif

//WebManagement wm(CONF_PATH);

Display display;

void setup() {
  Serial.begin(115200);
  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);
  SPI.begin();

  //LittleFS.begin();

  String hostname = WIFI_HOSTBASE+String(CAN_ID);
  //wm.begin(WIFI_SSID, WIFI_PWD, hostname.c_str());

  // CAN initialization
  canW.begin();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY); // switching frequency to 15kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // initializing ADC
  analogReadResolution(12); // set precision to 12 bits, 0-4095 input

  // motor initialization
  motorTrLeft.begin();
  motorTrRight.begin();

  motorTrLeft.calibrate();
  motorTrRight.calibrate();

#if defined MODC_EE
  Serial1.setRX(1);
  Serial1.setTX(0);
  Dynamixel.setSerial(&Serial1);
  Dynamixel.begin(19200);
#endif

  Debug.println("BEGIN", Levels::INFO);

#ifdef MODC_YAW
  encoderYaw.update();
  encoderYaw.readAngle();
  encoderYaw.setZero();
#endif

  // Display initialization
  display.begin();

  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);

}

void loop() {
  int time_cur = millis();
  uint8_t msg_id;
  byte msg_data[8];

  // update motors
  motorTrLeft.update();
  motorTrRight.update();

  // health checks
  if (time_cur - time_bat >= DT_BAT) {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL) Debug.println("Telemetry frequency below required: " + String(1000/time_tel_avg) + " Hz", Levels::WARN);

    if(!battery.charged()) Debug.println("Battery voltage low! " + String(battery.readVoltage()) + "v", Levels::WARN);
  }

  // send telemetry
  if (time_cur - time_tel >= DT_TEL) {
    time_tel_avg = (time_tel_avg + (time_cur - time_tel)) / 2;
    time_tel = time_cur;

    sendFeedback();
  }

  if (canW.readMessage(&msg_id, msg_data)) {

    // Received CAN message with setpoint
    time_data = time_cur;
    handleSetpoint(msg_id, msg_data);
  } else if (time_cur - time_data > CAN_TIMEOUT && time_data != -1) {
    //if we do not receive data for more than a second stop motors
    time_data = -1;
    Debug.println("Stopping motors after timeout.", Levels::INFO);
    motorTrLeft.stop();
    motorTrRight.stop();
  }

  //wm.handle();
  display.handleGUI();
}

/**
 * @brief Handles the setpoint messages received via CAN bus.
 * @param msg_id ID of the received message.
 * @param msg_data Pointer to the message data.
 */
void handleSetpoint(uint8_t msg_id, const byte* msg_data) {
  int16_t servo_data;
  Debug.println("RECEIVED CANBUS DATA");

  switch (msg_id) {
    case MOTOR_SETPOINT:
      float leftSpeed, rightSpeed;
      memcpy(&leftSpeed, msg_data, 4);
      memcpy(&rightSpeed, msg_data + 4, 4);
      motorTrLeft.setSpeed(leftSpeed);
      motorTrRight.setSpeed(rightSpeed);

      Debug.println("TRACTION DATA :\tleft: \t" + String(leftSpeed) + "\tright: \t" + String(rightSpeed));
      break;

    case DATA_EE_PITCH_SETPOINT:
      memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
      motorEEPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
      Debug.print("PITCH END EFFECTOR MOTOR DATA : \t");
      Debug.println(servo_data);
      break;

    case DATA_EE_HEAD_PITCH_SETPOINT:
      memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
      motorEEHeadPitch.moveSpeed(servo_data, SERVO_SPEED);
#endif
      Debug.print("HEAD PITCH END EFFECTOR MOTOR DATA : \t");
      Debug.println(servo_data);
      break;

    case DATA_EE_HEAD_ROLL_SETPOINT:
      memcpy(&servo_data, msg_data, 2);
#ifdef MODC_EE
      motorEEHeadRoll.moveSpeed(servo_data, SERVO_SPEED);
#endif
      Debug.print("HEAD ROLL END EFFECTOR MOTOR DATA : \t");
      Debug.println(servo_data);
      break;
  }
}

/**
 * @brief Sends feedback data over CAN bus.
 *
 * This function sends various feedback data including motor speeds, yaw angle, and end effector positions
 * if the respective modules are enabled.
 *
 * @note The function uses conditional compilation to include/exclude parts of the code based on the presence of specific modules.
 */
void sendFeedback() {

  // send motor data
  float speeds[2] = {motorTrLeft.getSpeed(), motorTrRight.getSpeed()};
  canW.sendMessage(MOTOR_FEEDBACK, speeds, 8);

  // send yaw angle of the joint if this module has one
#ifdef MODC_YAW
  encoderYaw.update();
  float angle = encoderYaw.readAngle();
  canW.sendMessage(JOINT_YAW_FEEDBACK, &angle, 4);
#endif

  // send end effector data (if module has it)
#ifdef MODC_EE
  int pitch = motorEEPitch.readPosition();
  int headPitch = motorEEHeadPitch.readPosition();
  int headRoll = motorEEHeadRoll.readPosition();

  canW.sendMessage(DATA_EE_PITCH_FEEDBACK, &pitch, 4);
  canW.sendMessage(DATA_EE_HEAD_PITCH_FEEDBACK, &headPitch, 4);
  canW.sendMessage(DATA_EE_HEAD_ROLL_FEEDBACK, &headRoll, 4);
#endif
}

void okInterrupt() {
  display.okInterrupt();
}

void navInterrupt() {
  display.navInterrupt();
}