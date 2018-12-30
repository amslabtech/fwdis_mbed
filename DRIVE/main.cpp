#include "mbed.h"
#include "rtos.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "fwdis_msgs/FourWheelDriveIndependentSteering.h"
#include "driving.h"

Driving driver;

ros::NodeHandle nh;

std_msgs::String mbed_log;
ros::Publisher mbed_log_pub("/mbed/log", &mbed_log);

fwdis_msgs::FourWheelDriveIndependentSteering fwdis;

void reset_callback(const std_msgs::Empty& msg)
{
  NVIC_SystemReset();
}

void start_callback(const std_msgs::Empty& msg)
{
  driver.start_control();
}

void fwdis_callback(const fwdis_msgs::FourWheelDriveIndependentSteering& msg)
{
  fwdis = msg;
  driver.set_angular_velocity(fwdis.front_right_wheel_velocity, fwdis.front_left_wheel_velocity, fwdis.rear_right_wheel_velocity, fwdis.rear_left_wheel_velocity);
}

ros::Subscriber<std_msgs::Empty> reset_sub("/mbed/reset", reset_callback);
ros::Subscriber<std_msgs::Empty> start_sub("/mbed/start", start_callback);
ros::Subscriber<fwdis_msgs::FourWheelDriveIndependentSteering> fwdis_sub("/fwdis/velocity", fwdis_callback);

void work(void const *args)
{
  mbed_log.data = "";
  while(true){
    mbed_log.data = driver.get_pulses().c_str();
    mbed_log_pub.publish(&mbed_log);
    Thread::wait(500);
  }
}

int main()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(mbed_log_pub);
  nh.subscribe(start_sub);
  nh.subscribe(reset_sub);
  nh.subscribe(fwdis_sub);

  Thread thread(work);

  while(1){
    nh.spinOnce();

    Thread::wait(1000);
  }
  //return 0;
}
