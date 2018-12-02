#ifndef __POTENTIOMETER_H
#define __POTENTIOMETER_H

#include "mbed.h"

class Potentiometer
{
public:
  Potentiometer(PinName);

  double get_angle(void);

private:
  constexpr static double MIN_VALUE = 1.0 / 11.0;
  constexpr static double MAX_VALUE = 1.0;
  constexpr static double CENTOR_VALUE = (MAX_VALUE + MIN_VALUE) / 2.0;
  constexpr static double MIN_ANGLE = -2.0 * M_PI / 3.0;// [rad]
  constexpr static double MAX_ANGLE = 2.0 * M_PI / 3.0;// [rad]
  constexpr static double RIGHT_ANGLE_OHM = 3400;// [ohm]
  constexpr static double CENTOR_ANGLE = 0.0;// [rad]
  constexpr static double ANGLE_OFFSET = 0.0;// [rad]
  constexpr static double R_CONST = 1000.0;// [ohm]
  constexpr static double R_VARIABLE = 10000.0;// [ohm]

  AnalogIn sensor;
  double angle;
};

#endif// __POTENTIOMETER_H
