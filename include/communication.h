#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// CAN bus packet identifiers

#define BATTERY_VOLTAGE                 0x11
#define BATTERY_PERCENT                 0x12
#define BATTERY_TEMPERATURE             0x13
#define MOTOR_SETPOINT                  0x21
#define MOTOR_FEEDBACK			        0x22
#define JOINT_YAW_FEEDBACK              0x32
#define DATA_EE_PITCH_SETPOINT          0x41
#define DATA_EE_HEAD_PITCH_SETPOINT     0x43
#define DATA_EE_HEAD_ROLL_SETPOINT      0x45
#define DATA_EE_PITCH_FEEDBACK          0x42
#define DATA_EE_HEAD_PITCH_FEEDBACK     0x44
#define DATA_EE_HEAD_ROLL_FEEDBACK      0x46

// TODO: update to ROS2 equivalent
#define DATA_PITCH                      0x04 // Deprecated?

#endif
