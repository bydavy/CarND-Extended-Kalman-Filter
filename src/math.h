#ifndef CUSTOM_MATH_H
#define CUSTOM_MATH_H

#include <stdlib.h>

/**
 * Pi, the ratio of a circle's circumference to its diameter
 *
 * @return pi
 */
constexpr double pi() { return std::atan(1)*4; }

constexpr double two_pi() { return 2*pi(); }

#endif //CUSTOM_MATH_H
