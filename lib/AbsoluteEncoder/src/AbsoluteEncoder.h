#ifndef ABSOLUTE_ENCODER_H
#define ABSOLUTE_ENCODER_H

#include <Arduino.h>
#include "../../Debug/src/Debug.h"
#include "../../../include/definitions.h"
#include "../../../include/mod_config.h"
#include "ams_as5048b.h"

/**
 * Class used as a wrapper for the ams_as5048b library to read data from the absolute encoder.
 */
class AbsoluteEncoder: private AMS_AS5048B {
  public:
    AbsoluteEncoder(byte addr = 0x44);
    void begin();
    void setZero();
    float readAngle();
    void update();

  private:
};

#endif
