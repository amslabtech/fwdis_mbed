#include "potentiometer.h"

Potentiometer::Potentiometer(PinName pin)
:sensor(pin)
{

}

double Potentiometer::get_angle(void)
{
  double value = sensor;
  double rv = (1.0 / value - 1) * R_CONST;
  double angle = (rv - R_VARIABLE / 2.0) / RIGHT_ANGLE_OHM * M_PI / 2.0;
  return angle;
}
