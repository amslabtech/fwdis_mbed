#ifndef __STEERING_H
#define __STEERING_H

#include <vector>
#include <string>

#include "mbed.h"
#include "Sabertooth.h"
#include "stm32_rotary_encoder.h"
#include "potentiometer.h"

class Steering
{
public:
  Steering(void);

  void test(void);
  void start_control(void);
  void set_angular_velocity(double, double, double, double);
  std::string get_pulses(void);
  double get_angle(int);

private:
  Sabertooth st_f;// 128
  Sabertooth st_r;// 129
  STM32RotaryEncoder1 encoder_fr;// fA
  STM32RotaryEncoder3 encoder_fl;// fB
  STM32RotaryEncoder4 encoder_rl;// rA
  STM32RotaryEncoder8 encoder_rr;// rB
  std::vector<Potentiometer> potentiometers;// fr, fl, rr, rl
  std::vector<double> offsets;// fr, fl, rr, rl
  std::vector<double> angles;// fr, fl, rr, rl
  Thread *_thread;

  const int ENCODER_PULSE4;

  static const PinName TX = PA_0;
  static const int ID_F = 128;
  static const int ID_R = 129;
  static const int BAUDRATE = 9600;
  constexpr static const double INTERVAL = 0.010;// [s]
  constexpr static const double GEAR_RATIO = 56.1;
  static const int ENCODER_PULSE = 500;
  constexpr static const double VOLTAGE = 24;// [V]
  constexpr static const double RAD_P_V = 34.35;//[rad/s/V]
  constexpr static const double MAX_W = 810;// [rad/s]
  static const PinName PM_FR = PC_0;
  static const PinName PM_FL = PC_1;
  static const PinName PM_RR = PC_4;
  static const PinName PM_RL = PC_5;

  static void thread_starter(void const *);
  void thread_worker();
  std::vector<double> target_w;// fr, fl, rr, rl
  std::vector<double> current_w;// fr, fl, rr, rl
  std::vector<int> pulse;// fr, fl, rr, rl

};

#endif //__STEERING_H
