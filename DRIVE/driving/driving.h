#ifndef __DRIVING_H
#define __DRIVING_H

#include <vector>
#include <string>

#include "mbed.h"
#include "Sabertooth.h"
#include "stm32_rotary_encoder.h"
#include "pid.h"
#include <ros.h>
#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"

class Driving
{
public:
  Driving(void);

  void test(void);
  void start_control(void);
  void set_angular_velocity(double, double, double, double);
  std::string get_pulses(void);
  void get_odom_data(fwdis_msgs::FourWheelDriveIndependentSteering&);

private:
  int voltage_to_command(double);
  int omega_to_command(double);
  void set_speed(int, int);// 0~3:fr, fl, rr, rl

  Sabertooth st_f;// 128
  Sabertooth st_r;// 129
  STM32RotaryEncoder1 encoder_fr;// fA
  STM32RotaryEncoder3 encoder_fl;// fB
  STM32RotaryEncoder4 encoder_rl;// rA
  STM32RotaryEncoder8 encoder_rr;// rB
  Thread *_thread;
  std::vector<double> target_w;// fr, fl, rr, rl
  std::vector<double> current_w;// fr, fl, rr, rl
  std::vector<int> pulse;// fr, fl, rr, rl
  std::vector<PID> pid;// fr, fl, rr, rl
  std::vector<int> sum_pulses;
  std::vector<bool> negative_flag;//fr, fl, rr, rl
  std::vector<double> outputs;//fr, fl, rr, rl
  fwdis_msgs::FourWheelDriveIndependentSteering fwdis_drive;

  const int ENCODER_PULSE4;

  static const PinName TX = PA_0;
  static const int ID_F = 128;
  static const int ID_R = 129;
  static const int BAUDRATE = 9600;
  constexpr static const double INTERVAL = 0.010;// [s]
  constexpr static const double GEAR_RATIO = 23.1;// 21*1.1
  static const int ENCODER_PULSE = 500;
  constexpr static const double VOLTAGE = 24;// [V]
  constexpr static const double RAD_P_V = 34.348;//[rad/s/V]
  constexpr static const double MAX_W = 733;//810;// [rad/s]

  static void thread_starter(void const *);
  void thread_worker();

};

#endif //__DRIVING_H
