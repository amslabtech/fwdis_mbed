#include "driving.h"

Driving::Driving(void)
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
    sum_pulses.push_back(0);
    negative_flag.push_back(false);
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
    pid[i].set_gain(0.040, 0.0, -0.00001);
    pid[i].set_dt(INTERVAL);
    pid[i].set_input_limit(0, MAX_W);
    pid[i].set_output_limit(0, VOLTAGE);
    pid[i].set_integral_max(0.01);
    pid[i].set_set_point(0);
  }
}

void Driving::test(void)
{
  /*
  st_f.SetSpeedMotorA((int)(target_w[0] / MAX_W * 127));
  st_f.SetSpeedMotorB((int)(target_w[1] / MAX_W * 127));
  st_r.SetSpeedMotorA((int)(target_w[2] / MAX_W * 127));
  st_r.SetSpeedMotorB((int)(target_w[3] / MAX_W * 127));
  */
  /*
  for(int i=0;i<4;i++){
    set_speed(i, omega_to_command(target_w[i]));
  }
  */
}

void Driving::start_control(void)
{
  st_f.InitializeCom();
  st_r.InitializeCom();
  st_f.ShutdownMotors();
  st_r.ShutdownMotors();
  _thread = new Thread(&Driving::thread_starter, this);
}

void Driving::thread_starter(void const *p)
{
  Driving *instance = (Driving*)p;
  instance->thread_worker();
}

void Driving::thread_worker()
{
  while(1){
    //test();
    pulse[0] = encoder_fr.get_pulse();
    pulse[1] = encoder_fl.get_pulse();
    pulse[2] = -encoder_rr.get_pulse();
    pulse[3] = encoder_rl.get_pulse();
    encoder_fr.reset();
    encoder_fl.reset();
    encoder_rr.reset();
    encoder_rl.reset();
    for(int i=0;i<4;i++){
      current_w[i] = 2.0 * M_PI * (double)pulse[i] / ENCODER_PULSE4 / INTERVAL;// motor omega
      if(negative_flag[i]){
        current_w[i] = -current_w[i];
      }
      double output = pid[i].calculate(current_w[i]);
      if(negative_flag[i]){
        output = -output;
      }
      if(negative_flag[i]){
        set_speed(i, omega_to_command(-target_w[i]));
      }else{
        set_speed(i, omega_to_command(target_w[i]));
      }
      //set_speed(i, voltage_to_command(output));
      //set_speed(i, omega_to_command(231));
      //sum_pulses[i] += pulse[i];
      //sum_pulses[i] = output * 1000.0;
      //sum_pulses[i] = target_w[i] * 1000.0;
      sum_pulses[i] = current_w[i] / GEAR_RATIO * 1000.0;
      //sum_pulses[i] = (target_w[i] - current_w[i]) * 1000.0;
    }
    Thread::wait(INTERVAL*1000);
  }
}

void Driving::set_angular_velocity(double w_fr, double w_fl, double w_rr, double w_rl)
{
  target_w[0] = w_fr * GEAR_RATIO;
  target_w[1] = w_fl * GEAR_RATIO;
  target_w[2] = w_rr * GEAR_RATIO;
  target_w[3] = w_rl * GEAR_RATIO;
  for(int i=0;i<4;i++){
    if(target_w[i] < 0){
      target_w[i] = -target_w[i];
      negative_flag[i] = true;
    }else{
      negative_flag[i] = false;
    }
    pid[i].set_set_point(target_w[i]);
  }
}

// for debug
std::string Driving::get_pulses(void)
{
  std::string str = "";
  for(int i=0;i<4;i++){
    str += std::to_string(sum_pulses[i]) + ", ";
  }
  return str;
}

int Driving::voltage_to_command(double v)
{
  return (int)(v / VOLTAGE * 127);
}

int Driving::omega_to_command(double w)
{
  return (int)(w / MAX_W * 127) / 2.5;
}

void Driving::set_speed(int id, int speed)
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
