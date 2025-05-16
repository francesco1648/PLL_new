#include "DynamixelSerial.h"

void DynamixelInterface::writeByte(byte b) {
  serialPort->write(b);
  while (serialPort->available() < 1);
  serialPort->read();
}

bool DynamixelInterface::waitBytes(int n) {
  int tc = 0;
  while ((serialPort->available() < n) && (tc < AX_TIME_OUT)) {
    tc++;
    delayMicroseconds(1000);
  }
  
  return tc >= AX_TIME_OUT;
}

void DynamixelInterface::writeMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_UART);
}

void DynamixelInterface::readMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_SIO);
}

void DynamixelInterface::writeBuf(byte id, byte* buf, int len) {
  byte out[10] = {AX_START, AX_START, id, (byte)(len+1)};
  int n = 4;

  for(int i = 0; i < len; i++) out[i+n] = buf[i];
  n+=len;

  byte checksum = 0;
  for(int i = 2; i < n; i++) checksum += out[i];
  out[n++] = ~checksum & 0xFF;

  writeMode();
  serialPort->write(out, n);
  while (serialPort->available() < n);
  for(int i = 0; i < n; i++) serialPort->read();
  readMode();
}

int DynamixelInterface::readWord() {
  int out = -1;
  waitBytes(7);

  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if (in==255 && serialPort->peek()==255) {
      serialPort->read();                            // Start Bytes
      serialPort->read();                            // Ax-12 ID
      serialPort->read();                            // Length
      byte status = serialPort->read();

      if (status!=0) out = -status;
      else out = serialPort->read();
    }
  }
  return out;
}

int DynamixelInterface::readDWord() {
  int out = -1;
  waitBytes(8);

  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if (in == 255 && serialPort->peek() == 255) {
      serialPort->read();  // Start Bytes
      serialPort->read();  // Ax-12 ID
      serialPort->read();  // Length
      byte status = serialPort->read();

      if (status != 0) out = -status;
      else out = serialPort->read() | (serialPort->read() << 8);
    }
  }
  return out;
}

int DynamixelInterface::readStatus() {
  int out = -1;
  waitBytes(6);
  
  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if ( (in == 255) & (serialPort->peek() == 255) ) {
      serialPort->read();                                    // Start Bytes
      serialPort->read();                                    // Ax-12 ID
      serialPort->read();                                    // Length
      out = serialPort->read();
    }
  }
  return out;
}

void DynamixelInterface::setSerial(HardwareSerial *sPort) {
  serialPort = sPort;
}

void DynamixelInterface::begin(long baud) {
  serialPort->begin(baud);
  readMode();
}

void DynamixelInterface::end() {
  serialPort->end();
}

int DynamixelMotor::reset() {
  byte cmd[] = {AX_RESET};
  Dynamixel.writeBuf(id, cmd, 1);
  return Dynamixel.readStatus();
}

int DynamixelMotor::ping() {
  byte cmd[] = {AX_READ_DATA, AX_PING};
  Dynamixel.writeBuf(id, cmd, 2);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setID(byte newID) {
  byte cmd[] = {AX_WRITE_DATA, AX_ID, newID};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setBD(long baud) {
  byte Baud_Rate = (2000000 / baud) - 1;
  
  byte cmd[] = {AX_WRITE_DATA, AX_BAUD_RATE, Baud_Rate};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::move(int Position) {
  byte pos_h = Position >> 8;
  byte pos_l = Position;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, pos_l, pos_h};
  Dynamixel.writeBuf(id, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelMotor::moveSpeed(int Position, int Speed) {
  byte pos_h = Position >> 8;
  byte pos_l = Position;
  byte speed_h = Speed >> 8;
  byte speed_l = Speed;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, pos_l, pos_h, speed_l, speed_h};
  Dynamixel.writeBuf(id, cmd, 6);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setEndless(bool Status) {
  int al_ccw = Status?0:AX_DEFAULT_CCW_AL; // endless mode is enabled when both limits are set to 0
  if(!Status) turn(0, 0); // stop the motor before disabling endless mode

  return setAngleLimit(0, al_ccw);
}

int DynamixelMotor::turn(bool SIDE, int Speed) {
  byte speed_l = Speed >> 8;
  byte speed_h = Speed;
  // side 0 is left
  if (SIDE!=0) speed_h += 4; // ???

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_SPEED_L, speed_l, speed_h};
  Dynamixel.writeBuf(id, cmd, 4);
  return Dynamixel.readStatus();
}

void DynamixelMotor::action() {
  byte cmd[] = {AX_ACTION};
  Dynamixel.writeBuf(AX_BROADCAST_ID, cmd, 1);
}

int DynamixelMotor::enableTorque(bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_TORQUE_ENABLE, Status};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::enableLED(bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_LED, Status};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::readTemperature() {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_TEMPERATURE, 1};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelMotor::readPosition() {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_POSITION_L, 2};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readDWord();
}

int DynamixelMotor::readVoltage() {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_VOLTAGE, 1};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelMotor::setTempLimit(byte Temperature) {
  byte cmd[] = {AX_WRITE_DATA, AX_LIMIT_TEMPERATURE, Temperature};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setVoltageLimit(byte DVoltage, byte UVoltage) {
  byte cmd[] = {AX_WRITE_DATA, AX_DOWN_LIMIT_VOLTAGE, DVoltage, UVoltage};
  Dynamixel.writeBuf(id, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setAngleLimit(int CWLimit, int CCWLimit) {
  byte al_cw_h = CWLimit >> 8;
  byte al_cw_l = CWLimit;
  byte al_ccw_h = CCWLimit >> 8;
  byte al_ccw_l = CCWLimit;

  byte cmd[] = {AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, al_cw_l, al_cw_h, AX_CCW_ANGLE_LIMIT_L, al_ccw_l, al_ccw_h};
  Dynamixel.writeBuf(id, cmd, 7);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setMaxTorque(int MaxTorque) {
  byte torque_h = MaxTorque >> 8;
  byte torque_l = MaxTorque;

  byte cmd[] = {AX_WRITE_DATA, AX_MAX_TORQUE_L, torque_l, torque_h};
  Dynamixel.writeBuf(id, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setSRL(byte SRL) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, SRL};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setRDT(byte RDT) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, (byte)(RDT/2)};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setLEDAlarm(byte LEDAlarm) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_LED, LEDAlarm};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setShutdownAlarm(byte SALARM) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_SHUTDOWN, SALARM};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setCMargin(byte CWCMargin, byte CCWCMargin) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_MARGIN, CWCMargin, AX_CCW_COMPLIANCE_MARGIN, CCWCMargin};
  Dynamixel.writeBuf(id, cmd, 5);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setCSlope(byte CWCSlope, byte CCWCSlope) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_SLOPE, CWCSlope, AX_CCW_COMPLIANCE_SLOPE, CCWCSlope};
  Dynamixel.writeBuf(id, cmd, 5);
  return Dynamixel.readStatus();
}

int DynamixelMotor::setPunch(int Punch) {
  byte punch_h = Punch >> 8;
  byte punch_l = Punch;

  byte cmd[] = {AX_WRITE_DATA, AX_PUNCH_L, punch_l, punch_h};
  Dynamixel.writeBuf(id, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelMotor::moving() {
  byte cmd[] = {AX_READ_DATA, AX_MOVING, 1};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelMotor::lockRegister() {
  byte cmd[] = {AX_WRITE_DATA, AX_LOCK, true};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelMotor::RWStatus() {
  byte cmd[] = {AX_READ_DATA, AX_REGISTERED_INSTRUCTION, 1};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelMotor::readSpeed() {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_SPEED_L, 2};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readDWord();
}

int DynamixelMotor::readLoad() {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_LOAD_L, 2};
  Dynamixel.writeBuf(id, cmd, 3);
  return Dynamixel.readDWord();
}


DynamixelInterface Dynamixel;
