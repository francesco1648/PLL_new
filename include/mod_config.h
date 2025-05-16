#ifndef module_configuration_h
#define module_configuration_h

// MK1_MOD1, MK1_MOD2, MK2_MOD1, MK2_MOD2 are defined at build time

#if defined(MK1_MOD1)
#define CAN_ID    0x11  // MK1 first module (HEAD)
#define MODC_EE
#define SERVO_EE_PITCH_ID 6
#define SERVO_EE_HEAD_ROLL_ID 4
#define SERVO_EE_HEAD_PITCH_ID 2
#define SERVO_SPEED 200

#elif defined(MK1_MOD2)
#define CAN_ID    0x12  // MK1 second module (MIDDLE)
#define MODC_YAW

#elif defined(MK2_MOD1)
#define CAN_ID    0x21  // MK2 first module (HEAD)

#elif defined(MK2_MOD2)
#define CAN_ID    0x22  // MK2 second module (MIDDLE)
#define MODC_YAW
#endif

#endif