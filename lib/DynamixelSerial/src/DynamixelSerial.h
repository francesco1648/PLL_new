#ifndef DynamixelSerial_h
#define DynamixelSerial_h

// EEPROM AREA
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_MAX_TORQUE_L             14
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18

// RAM AREA
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_SPEED_L             32
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48

// Instruction set
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_ACTION                   5
#define AX_RESET                    6

// Constants
#define AX_BROADCAST_ID             254
#define AX_START                    255

// Configurations
#define AX_DEFAULT_CCW_AL			1023
#define AX_TIME_OUT					10

#include <Arduino.h>
#include <HardwareSerial.h>

class DynamixelInterface {
private:
	HardwareSerial *serialPort;
	void writeByte(byte b);
	bool waitBytes(int n);

	void writeMode();
	void readMode();

public:
	void setSerial(HardwareSerial *sPort);
	void begin(long baud);
	void end();

	void writeBuf(byte id, byte* buf, int len);

	int readStatus();
	int readWord();
	int readDWord();
};

class DynamixelMotor {
private:
	byte id;
public:
	DynamixelMotor(byte id) : id(id) {}

	int reset();
	int ping();

	int setID(byte newID);
	int setBD(long baud);

	int move(int Position);
	int moveSpeed(int Position, int Speed);
	int setEndless(bool Status);
	int turn(bool SIDE, int Speed);

	void action(void);

	int setTempLimit(byte Temperature);
	int setAngleLimit(int CWLimit, int CCWLimit);
	int setVoltageLimit(byte DVoltage, byte UVoltage);
	int setMaxTorque(int MaxTorque);
	int setSRL(byte SRL);
	int setRDT(byte RDT);
	int setLEDAlarm(byte LEDAlarm);
	int setShutdownAlarm(byte SALARM);
	int setCMargin(byte CWCMargin, byte CCWCMargin);
	int setCSlope(byte CWCSlope, byte CCWCSlope);
	int setPunch(int Punch);

	int moving();
	int lockRegister();
	int RWStatus();

	int readTemperature();
	int readVoltage();
	int readPosition();
	int readSpeed();
	int readLoad();

	int enableTorque(bool Status);
	int enableLED(bool Status);
};

extern DynamixelInterface Dynamixel;

#endif
