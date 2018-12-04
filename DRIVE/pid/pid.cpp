#include "pid.h"

PID::PID()
{
  kp = 0;
  ki = 0;
  kd = 0;
  dt = 0;
  input_min = 0;
  input_max = 0;
  output_min = 0;
  output_max = 0;
  integral_max = 0;
}

void PID::set_gain(double _kp, double _ki, double _kd)
{
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

void PID::set_dt(double _dt)
{
  dt = _dt;
}

void PID::set_input_limit(double _min, double _max)
{
  input_min = _min;
  input_max = _max;
}

void PID::set_output_limit(double _min, double _max)
{
  output_min = _min;
  output_max = _max;
}

void PID::set_integral_max(double _integral_max)
{
  integral_max = _integral_max;
}


void PID::set_set_point(double _set_point)
{
  set_point = _set_point;
}

double PID::calculate(double _pv)
{
  pv = _pv;
  error = set_point - pv;

  double p_value = kp * error;

  integral += error * dt;
  if(integral > 0 && integral > integral_max){
    integral = integral_max;
  }else if(integral < 0 && integral < -integral_max){
    integral = -integral_max;
  }
  double i_value = ki * integral;

  double d = (error - previous_error) / dt;
  double d_value = kd * d;

  double output = p_value + i_value + d_value;
  if(output > output_max){
    output = output_max;
  }else if(output < output_min){
    output = output_min;
  }

  previous_error = error;

  return output;
}
