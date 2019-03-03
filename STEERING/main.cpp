#include "mbed.h"
#include "rtos.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"
#include "steering.h"

Steering steering;

ros::NodeHandle nh;

std_msgs::String mbed_log;
ros::Publisher mbed_log_pub("/mbed/log", &mbed_log);

std_msgs::Float32 mbed_val;
ros::Publisher mbed_val_pub("/mbed/val", &mbed_val);

fwdis_msgs::FourWheelDriveIndependentSteering fwdis_steer;
ros::Publisher fwdis_steer_pub("/odom/steer", &fwdis_steer);

fwdis_msgs::FourWheelDriveIndependentSteering fwdis;

std_msgs::Float32 duty_fr;
ros::Publisher duty_fr_pub("/tinypower/steer/duty0", &duty_fr);

std_msgs::Float32 duty_fl;
ros::Publisher duty_fl_pub("/tinypower/steer/duty1", &duty_fl);

std_msgs::Float32 duty_rr;
ros::Publisher duty_rr_pub("/tinypower/steer/duty2", &duty_rr);

std_msgs::Float32 duty_rl;
ros::Publisher duty_rl_pub("/tinypower/steer/duty3", &duty_rl);

void reset_callback(const std_msgs::Empty& msg)
{
  NVIC_SystemReset();
}

void start_callback(const std_msgs::Empty& msg)
{
  steering.start_control();
}

void fwdis_callback(const fwdis_msgs::FourWheelDriveIndependentSteering& msg)
{
  fwdis = msg;
  steering.set_steering_angle(fwdis.front_right_steering_angle, fwdis.front_left_steering_angle, fwdis.rear_right_steering_angle, fwdis.rear_left_steering_angle);
}

ros::Subscriber<std_msgs::Empty> reset_sub("/mbed/reset", reset_callback);
ros::Subscriber<std_msgs::Empty> start_sub("/mbed/start", start_callback);
ros::Subscriber<fwdis_msgs::FourWheelDriveIndependentSteering> fwdis_sub("/fwdis/command", fwdis_callback);

void work(void const *args)
{
  mbed_log.data = "";
  while(true){
    /*
    mbed_log.data = steering.get_pulses().c_str();
    mbed_log_pub.publish(&mbed_log);
    mbed_val.data = steering.get_angle(0);
    mbed_val_pub.publish(&mbed_val);
    */
    Thread::wait(500);
  }
}

void odom(void const *args)
{
  while(true){
    steering.get_odom_data(fwdis_steer);
    fwdis_steer_pub.publish(&fwdis_steer);
    duty_fr.data = steering.get_voltage_fr() / 24.0 * 100;
    duty_fr_pub.publish(&duty_fr);
    duty_fl.data = steering.get_voltage_fl() / 24.0 * 100;
    duty_fl_pub.publish(&duty_fl);
    duty_rr.data = steering.get_voltage_rr() / 24.0 * 100;
    duty_rr_pub.publish(&duty_rr);
    duty_rl.data = steering.get_voltage_rl() / 24.0 * 100;
    duty_rl_pub.publish(&duty_rl);
    Thread::wait(50);
  }
}

int main()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(mbed_log_pub);
  nh.advertise(mbed_val_pub);
  nh.advertise(fwdis_steer_pub);
  nh.subscribe(start_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(fwdis_sub);
  nh.advertise(duty_fr_pub);
  nh.advertise(duty_fl_pub);
  nh.advertise(duty_rr_pub);
  nh.advertise(duty_rl_pub);

  Thread thread(work);
  Thread thread2(odom);

  while(1){
    nh.spinOnce();
    Thread::wait(10);
  }
  //return 0;
}
