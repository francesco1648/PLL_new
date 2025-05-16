#ifndef TRACTION_ENCODER_H
#define TRACTION_ENCODER_H

#include <Arduino.h>
#include <functional>
#include "Filter.h"
#include "../../../include/definitions.h"
#include "../../../include/quadrature_encoder.pio.h"

/**
 * Class used to read data from traction rotary encoders.
 */
class TractionEncoder {
  public:
    TractionEncoder(byte pin_a, byte pin_b, Filter<int> *filter = NULL, bool invert = false, PIO pio = pio0);
    void begin();
    int getSpeed();
    int getSteps();

  private:
    byte pin_a, pin_b;
    bool invert;
    Filter<int> *filter;

    unsigned long last_time;
    int last_steps;

    PIO pio;
    uint sm;

    static uint last_sm;
    static uint offset;
    static bool initialized;
};

#endif
