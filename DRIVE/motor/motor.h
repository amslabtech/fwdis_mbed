#ifndef __MOTOR_H
#define __MOTOR_H

#include "mbed.h"

class Motor{
public:
  Motor(PinName, PinName);
  void set_duty(float);
  void stop(void);
  void brake(void);
private:
  static const int PWM_PERIOD = 10;//[us]
  PwmOut pin_a;
  PwmOut pin_b;
};

#endif

