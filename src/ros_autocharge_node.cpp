
#include "ros_autocharge_action"

#include <signal.h>
#include <cmath>

#include "yaml-cpp/yaml.h"


using namespace std;



void mySigIntHandler(int sig)
{
    ROS_INFO("close ros_autocharge!\r\n");
    ros::shutdown();
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"ros_autocharge_action",ros::init_options::NoSigintHandler); 
    ros::NodeHandle nh;
    signal(SIGINT, mySigIntHandler);

    ROSAutoCharge Charge;
    ros::spin();

    exit(0) ;
}