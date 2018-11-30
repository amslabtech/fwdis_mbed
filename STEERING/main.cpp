#include "mbed.h"
#include "rtos.h"

#include <ros.h>
#include <std_msgs/Float32.h>;

#include "Sabertooth.h"
#include "stm32_rotary_encoder.h"
#include "four_wheel_drive_independent_steering/FourWheelDriveIndependentSteering.h"
#include "steering.h"


ros::NodeHandle nh;
std_msgs::Float32 msg;

ros::Publisher pub("chatter", &msg);

std_msgs::Float32 hoge;
ros::Publisher hoge_pub("/hoge", &hoge);

four_wheel_drive_independent_steering::FourWheelDriveIndependentSteering fwdis;


void huga_callback(const std_msgs::Float32& msg)
{
    hoge.data = msg.data;
}

ros::Subscriber<std_msgs::Float32> huga_sub("/huga", huga_callback);

void work(void const *args)
{
    hoge.data = 3.14;
    while(true){
        hoge_pub.publish(&hoge);
        Thread::wait(500);
    }
}

int main()
{
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(pub);
    nh.advertise(hoge_pub);
    nh.subscribe(huga_sub);

    Thread thread(work);

    Sabertooth st(PA_0, 128, 9600);
    st.InitializeCom();

    while(1){
        msg.data = 123;

        st.SetSpeedMotorA(22);
        Thread::wait(500);
        st.SetSpeedMotorA(-22);
        Thread::wait(500);
        pub.publish(&msg);
        nh.spinOnce();

        Thread::wait(1000);
    }
    //return 0;
}
