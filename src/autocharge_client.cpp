#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <ros_autocharge_action/action1Action.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/utils.h" //include this to use tf2::getYaw()
#include <signal.h>

typedef actionlib::SimpleActionClient<ros_autocharge_action::action1Action> Client;
Client *ac_;
ros_autocharge_action::action1Goal goal;
ros_autocharge_action::action1ResultConstPtr result;



void activeCallback()
{
    ROS_INFO("Active!");
}

void doneCallback(const actionlib::SimpleClientGoalState &state, const ros_autocharge_action::action1ResultConstPtr &result)
{
    ROS_INFO("Action finished: %s",state.toString().c_str());
    ROS_INFO("Result: %d", result->result);
    ros::shutdown();
}


void feedbackCallback(const ros_autocharge_action::action1FeedbackConstPtr &feedback)
{
    tf2::Quaternion quat;
    tf2::convert(feedback->poseonreflector.orientation, quat);
    ROS_INFO("Feedback:%d x:%0.4f y:%0.4f angle:%0.4f int:%d", feedback->step, \
            feedback->poseonreflector.position.x, feedback->poseonreflector.position.y, tf2::getYaw(quat), feedback->reflector_intensity);
    // if(feedback->Step == 5 || feedback->Step == 6)
    // {
    //     goal.startmode = 2;
    //     ac_->sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    // }
}



void mySigIntHandler(int sig)
{
    ROS_INFO("close ros_autocharge!\r\n");
    // ac_->cancelAllGoals();
    goal.startmode = 1;
    ac_->sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    ros::shutdown();
    // exit(0);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"action_test_client",ros::init_options::NoSigintHandler); 
    ros::NodeHandle nh;
    Client ac("AutoCharge_Server", true);
    ac_ = &ac;
    ros::Duration t(10);
    ros::Rate r(1);
    signal(SIGINT, mySigIntHandler);

    // ac.waitForServer();
    while(!ac.isServerConnected() && ros::ok())
    {
        ROS_INFO("Waiting for action server to start.");
        r.sleep();
    }

    
    goal.startmode = 1;
    goal.length = 0.17;
    goal.distance = 1.0;
    ac.sendGoal(goal, doneCallback, activeCallback, feedbackCallback);
    

    while(ros::ok())
    {
        
        r.sleep();
    }

    return 0;
}