#include "motor.h"

Motor::Motor(PinName _pin_a, PinName _pin_b)
:pin_a(_pin_a), pin_b(_pin_b)
{
  pin_a.period_us(PWM_PERIOD);
  pin_b.period_us(PWM_PERIOD);
}

void Motor::set_duty(float duty)
{
  if(duty > 1.0f){
    duty = 1.0f;
  }else if(duty < -1.0f){
    duty = -1.0f;
  }
  if(duty > 0){
    pin_a = duty;
    pin_b = 0;
  }else if(duty < 0){
    pin_a = 0;
    pin_b = -duty;
  }else{
    stop();
  }
}

void Motor::stop(void)
{
  pin_a = 0;
  pin_b = 0;
}

void Motor::brake(void)
{
  pin_a = 1;
  pin_b = 1;
}

