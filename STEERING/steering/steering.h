#ifndef __STEERING_H
#define __STEERING_H

#include <vector>
#include <string>

#include "mbed.h"
#include "Sabertooth.h"
#include "stm32_rotary_encoder.h"
#include "potentiometer.h"
#include "pid.h"
#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"

class Steering
{
public:
  Steering(void);

  void test(void);
  void start_control(void);
  void set_steering_angle(double, double, double, double);
  std::string get_pulses(void);
  double get_angle(int);
  void get_odom_data(fwdis_msgs::FourWheelDriveIndependentSteering&);

private:
  int valtage_to_command(double);
  int omega_to_command(double);
  void set_speed(int, int);// 0~3:fr, fl, rr, rl

  Sabertooth st_f;// 130
  Sabertooth st_r;// 131
  STM32RotaryEncoder1 encoder_fr;// fA
  STM32RotaryEncoder3 encoder_fl;// fB
  STM32RotaryEncoder4 encoder_rl;// rA
  STM32RotaryEncoder8 encoder_rr;// rB
  std::vector<Potentiometer> potentiometers;// fr, fl, rr, rl
  std::vector<double> offsets;// fr, fl, rr, rl
  std::vector<double> angles;// fr, fl, rr, rl
  std::vector<double> target_s;// fr, fl, rr, rl
  std::vector<double> target_w;// fr, fl, rr, rl
  std::vector<double> current_s;// fr, fl, rr, rl
  std::vector<double> current_w;// fr, fl, rr, rl
  std::vector<int> pulse;// fr, fl, rr, rl
  std::vector<PID> pid;// fr, fl, rr, rl
  std::vector<int> sum_pulses;// fr, fl, rr, rl
  std::vector<bool> negative_flag;// fr, fl, rr, rl
  Thread *_thread;
  fwdis_msgs::FourWheelDriveIndependentSteering fwdis_steer;

  const int ENCODER_PULSE4;

  static const PinName TX = PA_0;
  static const int ID_F = 130;
  static const int ID_R = 131;
  static const int BAUDRATE = 9600;
  constexpr static const double INTERVAL = 0.010;// [s]
  constexpr static const double GEAR_RATIO = 56.1;// 51 * 1.1
  static const int ENCODER_PULSE = 500;
  constexpr static const double VOLTAGE = 24;// [V]
  constexpr static const double RAD_P_V = 34.348;//[rad/s/V]
  constexpr static const double MAX_W = 733;// [rad/s]
  constexpr static const double MAX_ANGLE = M_PI * 2.0 / 3.0;// [rad/s]
  static const PinName PM_FR = PC_0;
  static const PinName PM_FL = PC_1;
  static const PinName PM_RR = PC_4;
  static const PinName PM_RL = PC_5;

  static void thread_starter(void const *);
  void thread_worker();

};

#endif //__STEERING_H
