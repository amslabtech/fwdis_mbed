#ifndef __STEERING_H
#define __STEERING_H

#include "mbed.h"
#include "Sabertooth.h"
#include "stm32_rotary_encoder.h"

class Steering
{
public:
  Steering(void);

private:
  STM32RotaryEncoder1 encoder_fr;
  STM32RotaryEncoder3 encoder_fl;
  STM32RotaryEncoder4 encoder_rl;
  STM32RotaryEncoder8 encoder_rr;

};

#endif __STEERING_H
