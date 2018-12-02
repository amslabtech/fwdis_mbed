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
    target_w.push_back(0);
    current_w.push_back(0);
    pulse.push_back(0);
    offsets.push_back(0);
  }
  Potentiometer _pm_fr(PM_FR);
  Potentiometer _pm_fl(PM_FL);
  Potentiometer _pm_rr(PM_RR);
  Potentiometer _pm_rl(PM_RL);
  potentiometers.push_back(_pm_fr);
  potentiometers.push_back(_pm_fl);
  potentiometers.push_back(_pm_rr);
  potentiometers.push_back(_pm_rl);
}

void Steering::test(void)
{
  st_f.SetSpeedMotorA((int)(target_w[0] / MAX_W * 127));
  st_f.SetSpeedMotorB((int)(target_w[1] / MAX_W * 127));
  st_r.SetSpeedMotorA((int)(target_w[2] / MAX_W * 127));
  st_r.SetSpeedMotorB((int)(target_w[3] / MAX_W * 127));
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
  while(1){
    test();
    pulse[0] = encoder_fr.get_pulse();
    pulse[1] = encoder_fl.get_pulse();
    pulse[2] = encoder_rr.get_pulse();
    pulse[3] = encoder_rl.get_pulse();
    /*
    encoder_fr.reset();
    encoder_fl.reset();
    encoder_rr.reset();
    encoder_rl.reset();
    */
    Thread::wait(INTERVAL*1000);
  }
}

void Steering::set_angular_velocity(double w_fr, double w_fl, double w_rr, double w_rl)
{
  target_w[0] = w_fr;
  target_w[1] = w_fl;
  target_w[2] = w_rr;
  target_w[3] = w_rl;
}

// for debug
std::string Steering::get_pulses(void)
{
  std::string str = "";
  for(int i=0;i<pulse.size();i++){
    str += std::to_string(pulse[i]) + ", ";
  }
  return str;
}

double Steering::get_angle(int id)
{
  double angle = potentiometers[id].get_angle() - offsets[id];
  return angle;
}
