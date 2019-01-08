#include "steering.h"

Steering::Steering(void)
:st_f(TX, ID_F, BAUDRATE), st_r(TX, ID_R, BAUDRATE)
, ENCODER_PULSE4(ENCODER_PULSE * 4)
{
  encoder_fr.initialize();
  encoder_fl.initialize();
  encoder_rl.initialize();
  encoder_rr.initialize();
  for(int i=0;i<4;i++){
    target_s.push_back(0);
    target_w.push_back(0);
    current_s.push_back(0);
    current_w.push_back(0);
    pulse.push_back(0);
    offsets.push_back(0);
    angles.push_back(0);
    sum_pulses.push_back(0);
    negative_flag.push_back(0);
  }
  Potentiometer _pm_fr(PM_FR);
  Potentiometer _pm_fl(PM_FL);
  Potentiometer _pm_rr(PM_RR);
  Potentiometer _pm_rl(PM_RL);
  potentiometers.push_back(_pm_fr);
  potentiometers.push_back(_pm_fl);
  potentiometers.push_back(_pm_rr);
  potentiometers.push_back(_pm_rl);
  for(int i=0;i<4;i++){
    //angles[i] = get_angle(i);
  }
  PID _pid_fr;
  PID _pid_fl;
  PID _pid_rr;
  PID _pid_rl;
  pid.push_back(_pid_fr);
  pid.push_back(_pid_fl);
  pid.push_back(_pid_rr);
  pid.push_back(_pid_rl);
  for(int i=0;i<4;i++){
    pid[i].set_gain(300.0, 1000.0, 0);
    pid[i].set_dt(INTERVAL);
    pid[i].set_input_limit(-MAX_ANGLE, MAX_ANGLE);
    pid[i].set_output_limit(-MAX_W / 2.0, MAX_W / 2.0);// for debug
    pid[i].set_integral_max(0.01);
    pid[i].set_set_point(target_s[i]);
  }
}

void Steering::test(void)
{
  for(int i=0;i<4;i++){
    //set_speed(i, omega_to_command(target_w[i]));
    //set_speed(i, omega_to_command(20));
  }
}

void Steering::start_control(void)
{
  st_f.InitializeCom();
  st_r.InitializeCom();
  st_f.ShutdownMotors();
  st_r.ShutdownMotors();
  _thread = new Thread(&Steering::thread_starter, this);
}

void Steering::thread_starter(void const *p)
{
  Steering *instance = (Steering*)p;
  instance->thread_worker();
}

void Steering::thread_worker()
{
  encoder_fr.reset();
  encoder_fl.reset();
  encoder_rr.reset();
  encoder_rl.reset();
  while(1){
    //test();
    pulse[0] = -encoder_fr.get_pulse();
    pulse[1] = encoder_fl.get_pulse();
    pulse[2] = encoder_rr.get_pulse();
    pulse[3] = encoder_rl.get_pulse();
    encoder_fr.reset();
    encoder_fl.reset();
    encoder_rr.reset();
    encoder_rl.reset();
    for(int i=0;i<4;i++){
      current_w[i] = 2.0 * M_PI * (double)pulse[i] / ENCODER_PULSE4 / INTERVAL;// motor omega
      angles[i] += current_w[i] / GEAR_RATIO * INTERVAL;// steering theta
      double output = pid[i].calculate(angles[i]);
      if((angles[i] < -MAX_ANGLE && output < 0) || (angles[i] > MAX_ANGLE && output > 0)){
        output= 0 ;
      }
      set_speed(i, omega_to_command(output));

      sum_pulses[i] = angles[i] * 1000;
      //sum_pulses[i] = omega_to_command(target_w[i]) * 1000;
      //sum_pulses[i] = output * 1000;
      //sum_pulses[i] = target_s[i] * 1000;
      //sum_pulses[i] = (target_s[i] - angles[i]) * 1000;
    }
    fwdis_steer.front_right_steering_angle = angles[0];
    fwdis_steer.front_left_steering_angle = angles[1];
    fwdis_steer.rear_right_steering_angle = angles[2];
    fwdis_steer.rear_left_steering_angle = angles[3];

    Thread::wait(INTERVAL*1000);
  }
}

void Steering::set_steering_angle(double s_fr, double s_fl, double s_rr, double s_rl)
{
  target_s[0] = s_fr;
  target_s[1] = s_fl;
  target_s[2] = s_rr;
  target_s[3] = s_rl;
  for(int i=0;i<4;i++){
    pid[i].set_set_point(target_s[i]);
    if(target_s[i] < 0){
      negative_flag[i] = true;
    }else{
      negative_flag[i] = false;
    }
  }
  //for debug
  target_w[0] = s_fr;
  target_w[1] = s_fl;
  target_w[2] = s_rr;
  target_w[3] = s_rl;
}

// for debug
std::string Steering::get_pulses(void)
{
  std::string str = "";
  for(int i=0;i<pulse.size();i++){
    str += std::to_string(sum_pulses[i]) + ", ";
  }
  return str;
}

double Steering::get_angle(int id)
{
  double angle = potentiometers[id].get_angle() - offsets[id];
  return angle;
}

int Steering::valtage_to_command(double v)
{
  return (int)(v / VOLTAGE * 127);
}

int Steering::omega_to_command(double w)
{
  return (int)(w / MAX_W * 127);
}

void Steering::set_speed(int id, int speed)
{
  switch(id){
    case 0:
      st_f.SetSpeedMotorA(speed);
      break;
    case 1:
      st_f.SetSpeedMotorB(speed);
      break;
    case 2:
      st_r.SetSpeedMotorA(speed);
      break;
    case 3:
      st_r.SetSpeedMotorB(speed);
      break;
  }
}

void Steering::get_odom_data(fwdis_msgs::FourWheelDriveIndependentSteering& data)
{
  data = fwdis_steer;
}
