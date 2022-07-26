
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

    // YAML::Node node = YAML::Load("[[1,2],[2,3]]");
    // for (YAML::const_iterator i = node.begin(); i != node.end(); ++i) {
    //     for(size_t k = 0; k < i->size(); ++k)
    //     {
    //         std::cout << fixed << setprecision(2) << (float)((*i)[k].as<float>()) << ",";
    //     }
    //     std::cout << endl;
    //     // std::cout << node[i].as<int>() << "\n";
    // }

    ROSAutoCharge Charge;
    ros::spin();


    // ChargerUART uart("/dev/ttyUSB0", 115200);
    // ChargerUART::ChargeDataTypedef chargedata;
    // ros::Rate r(20);
    // while(ros::ok())
    // {
    //     uart.PortLoop();
    //     uart.getChargerData(chargedata);
    //     uart.setSwitchState(1);
    //     ros::spinOnce();
    //     r.sleep();
    // }

    // int tmp;
    // double len;
    // Point p = {0, 0};;
    // Line l1 = {{0, 3},{0, 5}};
    // Line l2 = {{0, 0},{0, 4}};
    // tmp = CalCrossPoint(l1, l2, p, len);
    // printf("result:%d Point:%0.2f, %0.2f Length %0.2f\r\n", tmp, p.x, p.y, len);

    // ros::shutdown();
    exit(0) ;
}