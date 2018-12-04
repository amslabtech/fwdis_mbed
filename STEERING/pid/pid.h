#ifndef __PID_H
#define __PID_H

#include "mbed.h"

class PID
{
public:
  PID(void);

  void set_gain(double, double, double);// kp, ki, kd
  void set_dt(double);
  void set_input_limit(double, double);// min, max
  void set_output_limit(double, double);// min, max
  void set_integral_max(double);
  void set_set_point(double);
  double calculate(double);

private:
  double kp;
  double ki;
  double kd;
  double error;
  double previous_error;
  double input_max;
  double input_min;
  double output_max;
  double output_min;
  double integral;
  double integral_max;
  double dt;
  double set_point;
  double pv;
};

#endif// __PID_H
