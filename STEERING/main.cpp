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

  Thread thread(work);
  Thread thread2(odom);

  while(1){
    nh.spinOnce();

    Thread::wait(100);
  }
  //return 0;
}
