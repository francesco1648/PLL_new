#include "TractionEncoder.h"


bool TractionEncoder::initialized = false;
uint TractionEncoder::last_sm;
uint TractionEncoder::offset;

/**
 * Creates object.
 * @param pin_a Pin A of the encoder.
 * @param pin_b Pin B of the encoder.
 * @param filter Filter to apply to the encoder's output. NULL if no filter is needed.
 * @param invert Invert the encoder's output.
 * @param pio PIO to use. Each PIO can handle up to 4 encoders.
 */
TractionEncoder::TractionEncoder(byte pin_a, byte pin_b, Filter<int> *filter, bool invert, PIO pio) 
    : pin_a(pin_a), pin_b(pin_b), invert(invert), filter(filter), pio(pio)
{}

/**
 * Sets the encoder's pins and attach the interrupt.
 */
void TractionEncoder::begin() {
    if (!initialized) {
        initialized = true;
        last_sm = 0;
        offset = pio_add_program(pio, &quadrature_encoder_program);
    }
    sm = last_sm++;
    quadrature_encoder_program_init(pio, sm, offset, pin_a, 0);
    last_time = micros();
}


/**
 * Returns the last detected speed.
 * @return The speed in centiRPMs.
 */
int TractionEncoder::getSpeed() {
    unsigned long time = micros();
    int steps = getSteps();
    int delta = steps - last_steps;
    int deltat = time - last_time;
    last_steps = steps;
    last_time = time;

    // check definitions.h to understand what the constant is
    int rpm = delta * ENC_TR_CONVERSION / deltat;

    if(filter != NULL) rpm = filter->filter(rpm);

    return invert?-rpm:rpm;
}

/**
 * Returns the number of steps since the class was initialized.
 * @return The number of steps.
 */
int TractionEncoder::getSteps() {
    return quadrature_encoder_get_count(pio, sm);
}
