#ifndef EXP_SMOOTHING_FILTER_H
#define EXP_SMOOTHING_FILTER_H
#include <Arduino.h>
#include "Filter.h"

/**
 * Class for a exponential smoothing filter implementing the Filter interface.
 * @tparam T type of the variable to be filtered.
 */
template<typename T>
class ExpSmoothingFilter: public Filter<T> {
  public:
    /**
     * Constructor for the exponential smoothing filter.
     * The weight for the filter is given by the parameters a and b, representing the fraction a/b.
     * @param a weight fraction numerator.
     * @param b weight fraction denominator.
     */
    ExpSmoothingFilter(int a, int b) {
        this->a = a;
        this->b = b;
        previous = 0;
    }

    /**
     * Computes the filtered output using an exponential filter, giving the passed value a weight of a/b.
     * @param value to be filtered.
     * @return T filtered output.
     */
    T filter(T value) {
        return previous = (value*a + previous*(b-a))/b;
    }

  private:
    T previous;
    int a, b;
};

#endif
