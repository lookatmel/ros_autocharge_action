#include "ros_autocharge_action"
#include <cmath>
#include "yaml-cpp/yaml.h"

using namespace std;


ROSAutoCharge::ROSAutoCharge()
    :as(NULL)
{
    ros::NodeHandle nh_private("~");
	nh_private.param<std::string>("laser_topic", laser_topic_, "/scan");
	nh_private.param<std::string>("cmd_vel", cmd_vel_topic_, "/cmd_vel"); 
	nh_private.param<std::string>("pannel_id", pannel_frame_id_, "/pannel"); 
	nh_private.param<std::string>("base_id", base_frame_id_, "/base_link"); 
    nh_private.param<std::string>("laser_id", laser_frame_id_, "/laser"); 


    nh_private.param<float>("visual_angle", visual_angle_, VISUAL_ANGLE); 
    nh_private.param<float>("round_distance", round_distance_, ROUND_DISTANCE); 
    nh_private.param<float>("round_min_distance", round_min_distance_, ROUND_MIN); 
    nh_private.param<float>("round_angle_max", round_angle_max_, ROUND_ANGLE_MAX); 
    nh_private.param<float>("pannel_length", pannel_length_, PANNEL_LENGTH); 
    nh_private.param<float>("obstacle_distance", obstacle_distance_, OBSTACLE_DISTANCE); 
    nh_private.param<float>("obstacle_angle_min", obstacle_angle_min_, OBSTACLE_ANGLE_MIN); 
    nh_private.param<float>("obstacle_angle_max", obstacle_angle_max_, OBSTACLE_ANGLE_MAX); 
    nh_private.param<float>("obstacle_deadzone", obstacle_deadzone_, OBSTACLE_DRADZONE); 
    nh_private.param<float>("pretouch_distance", pretouch_distance_, PRETOUCH_DISTANCE); 
    nh_private.param<float>("moveback_distance", moveback_distance_, MOVEBACK_DISTANCE);
    nh_private.param<std::string>("movebase_polygon", movebase_polygon_str_, "[[0.29,-0.22],[0.29,0.22],[-0.29,0.22],[-0.29,-0.22]]"); 

    YAML::Node node = YAML::Load(movebase_polygon_str_);
    for (YAML::const_iterator i = node.begin(); i != node.end(); ++i)
    {
        float tmp[2];
        float *tmpaddr;
        if(i->size() != 2)
        {
            ROS_ERROR("polygon param error!");
            ros::shutdown();
            return;
        }
        cout << "[";
        for(size_t k = 0; k < 2; ++k)
        {
            tmp[k] = (*i)[k].as<float>();
            cout << fixed << setprecision(2) << (*i)[k].as<float>() << ",";
        }
        cout << "\b";
        cout << "],";
        tmpaddr = new float[2]{tmp[0], tmp[1]};
        movebase_polygon_.push_back(tmpaddr);
    }
    cout << "\b \b\r\n" << endl;
    for(auto i = movebase_polygon_.begin(); i != movebase_polygon_.end(); ++i)
    {
        cout << "[" << (float)(*i)[0] << "," << (float)(*i)[1] << "],";
    }
    cout << "\b\r\n" << endl;

    nh_private.param<std::string>("uart_port", UARTPort_, "/dev/autocharge_usb");
    nh_private.param<int>("uart_baudrate", UARTBaudrate_, 115200);  

    cmd_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);


    uart_ = new ChargerUART(UARTPort_, UARTBaudrate_);
    as = new AutoCharge_Server(nh, "AutoCharge_Server", boost::bind(&ROSAutoCharge::executeCB, this, _1), false);
    as->start();
    ROS_INFO("Created!");
}

ROSAutoCharge::~ROSAutoCharge()
{
    laser_scan_sub.shutdown();
    cmd_pub.shutdown();
    if(as != NULL)
    {
        delete as;
    }
    if(uart_ != NULL)
    {
        delete uart_;
    }
}


void ROSAutoCharge::start()
{
    search_time.fromSec(0);
    last_scan_time.fromSec(0);
    last_time.fromSec(0);

    pannel_overtime.fromSec(0);
    obstacle_overtime.fromSec(0);
    scan_overtime.fromSec(0);
    motion_waittime.fromSec(0);
    stable_waittime.fromSec(0);
    scan_overtime.fromSec(0);

    x = 0; y = 0; yaw = 0;
    x_temp = 0; y_temp = 0; yaw_temp = 0;
    state = 0; step = STEP_INIT; tf_rev = 0; have_obstacle_ = 0; wait_stable = 0, moveback = 0;
    scan_ready = 0;

    laser_scan_sub = nh.subscribe(laser_topic_, 100, &ROSAutoCharge::LaserScanCallback, this);
    last_time = ros::Time::now();
}

void ROSAutoCharge::stop()
{
    publishZeroSpeed();
    step = STEP_CANCEL;
}



void ROSAutoCharge::cancel()
{
    if((step == STEP_AIMING && fabs(x) < 0.5) || step == STEP_PRETOUCH || step == STEP_TOUCH)
    {
        cancel_time = ros::Time::now();
        step = STEP_MOVEBACK;
    }
    else
    {
        step = STEP_CANCEL;
    }

}

void ROSAutoCharge::shutdown()
{
    laser_scan_sub.shutdown();
}

unsigned int ROSAutoCharge::getStep()
{
    return step;
}

void ROSAutoCharge::executeCB(const ros_autocharge_action::action1GoalConstPtr &goal)
{
    ros_autocharge_action::action1GoalConstPtr newgoal = goal;
    ROS_INFO("Got a Goal:%d", newgoal->startmode);
    start();
    setLSpeedAccParam(0.1,0.2);
    setASpeedAccParam(3.14/8, 3.14/1);
    uart_->ensurePortState();

    ros::Rate r(20);
    while(1)
    {
        uart_->PortLoop();
        if(!ros::ok() || !as->isActive())
        {
            publishZeroSpeed();
            break;
        }

        if(as->isPreemptRequested())
        {
            if(as->isNewGoalAvailable())
            {
                newgoal = as->acceptNewGoal();
                if(newgoal->startmode == 2)
                {
                    stop();
                    ROS_INFO("Action Stop!");
                }
                else
                {
                    cancel();
                    ROS_INFO("Action Canncel!");
                }
                
            }
            else
            {
                stop();
                ROS_INFO("Action Stop!");
            }
        }

        loop();
        feedback.Step = getStep();
        as->publishFeedback(feedback);

        switch(getStep())
        {
            case STEP_SUCCESS :
                as->setSucceeded(result);
                ROS_INFO("Action is Succeed!");
                break;
            case STEP_CANCEL :
                as->setPreempted(result);
                break;
            case STEP_ERROR :
                as->setAborted(result);
                ROS_INFO("Action is Failed!");
                break;
            default:break;
        }


        r.sleep();
    }
    shutdown();
}


float ROSAutoCharge::getAccSpeed(float facc , float fdec , float bacc , float bdec , float target , float now_set, float real_value, double period)
{
    if(period <= 0) return now_set;

    double new_set , err_temp;
    double facc_ = facc * period;
    double fdec_ = fdec * period;
    double bacc_ = bacc * period;
    double bdec_ = bdec * period;
    err_temp = target - now_set;
    
    if(target >= 0)
    {
        if(now_set >= 0)
        {
            if(err_temp > facc_)
            {
                if(now_set - real_value > 200)
                {
                    new_set = now_set;
                }
                else
                {
                    new_set = now_set + facc_;
                }
            }
            else if(err_temp < -fdec_)
            {
                new_set = now_set - fdec_;
            }
            else{new_set = target;}
        }
        else
        {
            if(err_temp >= bdec_)
            {
                if(-now_set <= bdec_)
                {
                    new_set = 0;
                }
                else
                {
                    new_set = now_set + bdec_;
                }
            }
            else
            {
                new_set = 0;
            }
        }
    }
    else
    {
        if(now_set <= 0)
        {
            if(err_temp < -bacc_)
            { 
                if(now_set - real_value < -200)
                {
                    new_set = now_set;
                }
                else
                {
                    new_set = now_set - bacc_;
                }
            }
            else if(err_temp > bdec_)
            {
                new_set = now_set + bdec_;
            }
            else{new_set = target;}
        }
        else
        {
            if(err_temp <= -fdec_)
            {
                if(now_set <= fdec_)
                {
                    new_set = 0;
                }
                else
                {
                    new_set = now_set - fdec_;
                }
            }
            else
            {
                new_set = 0;
            }
        }
    }
    
    
    return (float)new_set;
}




void ROSAutoCharge::LaserScanCallback(const sensor_msgs::LaserScan &scan)
{
    // ROS_INFO("Scan Min Angle:%0.100f , Max Angle:%0.100f", scan_angle_min, scan_angle_max);
    static tf::StampedTransform laser_transform;
    static double laser_roll, laser_pitch, laser_yaw, laser_x, laser_y, laser_z;
    static vector<vector<float>>polygon;
    static vector<float>obstacle_thr;
    static unsigned int angle_division, scan_min_index, scan_max_index;
    static float scan_angle_min = 0, scan_angle_max = 0;
    static float laser_range_max = 20.0;
    if((scan_angle_min == 0 && scan_angle_max == 0) || (scan_angle_min != scan.angle_min || scan_angle_max != scan.angle_max))
    {
        try
        {
            auto laser_tf_now = ros::Time::now();
            laser_listener.waitForTransform(base_frame_id_, laser_frame_id_, laser_tf_now, ros::Duration(0.2));
            laser_listener.lookupTransform(base_frame_id_, laser_frame_id_, laser_tf_now, laser_transform);
            laser_x = laser_transform.getOrigin().getX();
            laser_y = laser_transform.getOrigin().getY();
            laser_z = laser_transform.getOrigin().getZ();
            tf::Matrix3x3(laser_transform.getRotation()).getRPY(laser_roll, laser_pitch, laser_yaw);
            ROS_INFO("Got Laser tf x:%0.2f y:%0.2f z:%0.2f roll:%0.2f pitch:%0.2f yaw:%0.2f \r\n", laser_x, laser_y, laser_z, laser_roll, laser_pitch, laser_yaw);
        }
        catch(tf::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }

        // scan_angle_min = std::max(scan.angle_min, obstacle_angle_min_);
        // scan_angle_max = std::min(scan.angle_max, obstacle_angle_max_);
        // angle_division = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        // scan_min_index = scan_angle_min > scan.angle_min ? (scan_angle_min - scan.angle_min) / scan.angle_increment + 1 : 0;
        // scan_max_index = scan_angle_max < scan.angle_max ? angle_division - (scan.angle_max - scan_angle_max) / scan.angle_increment + 1 : angle_division;
        // ROS_INFO("Scan Min Angle:%0.4f , Max Angle:%0.4f", scan_angle_min * 180 / M_PI, scan_angle_max * 180 /M_PI);
        // ROS_INFO("Scan Division:%d Scan Min Index:%d , Max Index:%d", angle_division, scan_min_index, scan_max_index);
        // ROS_INFO("Scan size:%ld Inc:%0.9f", scan.ranges.size(), scan.angle_increment);

        scan_angle_min = scan.angle_min;
        scan_angle_max = scan.angle_max;
        angle_division = (scan.angle_max - scan.angle_min) / scan.angle_increment;
        ROS_INFO("Scan Min Angle:%0.4f , Max Angle:%0.4f , Division:%d , Scan size:%ld , Inc:%0.5f ", \
                    scan_angle_min * 180 / M_PI, scan_angle_max * 180 /M_PI, angle_division, scan.ranges.size(), scan.angle_increment);
        polygon.clear();
        obstacle_thr.clear();
        if(isEqual(fabs(laser_roll / M_PI), 0))
        {
            for(auto i = movebase_polygon_.begin(); i != movebase_polygon_.end(); ++i)
            {
                polygon.push_back(vector<float>((float)(*i)[0] + laser_x, -((float)(*i)[1] + laser_y)));
            }
        }
        else
        {
            for(auto i = movebase_polygon_.begin(); i != movebase_polygon_.end(); ++i)
            {
                polygon.push_back(vector<float>((float)(*i)[0] + laser_x, (float)(*i)[1] + laser_y));
            }
        }

        cout << "obstacle polygon:";
        for(auto i = polygon.cbegin(); i != polygon.cend(); ++i)
        {
            cout << "[" << *(i->cbegin()) << "," <<  *(i->cbegin() + 1) << "]";
        }
        cout << "\r\n" << endl;

        for(int i = 0; i < scan.ranges.size(); ++i)
        {
            float angle_tmp = scan.angle_min + i * scan.angle_increment;
            Line l1 = {{0, 0}, {laser_range_max * cos(angle_tmp), laser_range_max * sin(angle_tmp)}};
            Line l2;
            Point crosspoint;
            double length;
            for (auto j = polygon.cbegin(); j != polygon.cend(); ++j)
            {
                l2.p1.x = *(j->cbegin());
                l2.p1.y = *(j->cbegin() + 1);
                if(j + 1 != polygon.cend())
                {
                    l2.p2.x = *((j + 1)->cbegin());
                    l2.p2.y = *((j + 1)->cbegin() + 1);
                }
                else
                {
                    l2.p2.x = *(polygon.cbegin()->cbegin());
                    l2.p2.y = *(polygon.cbegin()->cbegin() + 1);
                }

                int tmp = CalCrossPoint(l1, l2, crosspoint, length);
                if(tmp == CROSS || tmp == COLLINEATION)
                {
                    obstacle_thr.push_back(length + obstacle_distance_);
                    break;
                }
                else if(j + 1 == polygon.cend())
                {
                    obstacle_thr.push_back(0);
                    break;
                }
            }
        }
        ROS_INFO("laser size:%d , obstacle size:%d", scan.ranges.size(), obstacle_thr.size());
        for(auto i = obstacle_thr.cbegin(); i != obstacle_thr.cend(); ++i)
        {
            printf("%0.5f\r\n", *i); 
        }
        
        
        scan_ready = 1;
    }

    bool obstacle_temp = 0;
    // for(int i = scan_min_index; i <= scan_max_index; i++)
    // {
    //     if(scan.ranges[i] < obstacle_distance_)
    //     {
    //         obstacle_temp |= 1;
    //     }
    // }
    int obstacle_cnt = 0;
    for(auto j = obstacle_thr.cbegin(); j != obstacle_thr.cend(); ++j)
    {
        if(scan.ranges[obstacle_cnt] < *j)
        {
            obstacle_temp |= 1;
        }
        obstacle_cnt++;
    }


    have_obstacle_ = obstacle_temp;
    if(have_obstacle_)
    {
        ROS_INFO("have obstacle!");
    }
    last_scan_time = ros::Time::now();
}


void ROSAutoCharge::loop()
{
    ChargerUART::ChargeDataTypedef chargedata;
    bool chargedata_flag = false;
    try
    {
        tf_now = ros::Time::now();
        tf_rev = listener.waitForTransform(pannel_frame_id_, base_frame_id_, tf_now, ros::Duration(0.2));
        listener.lookupTransform(pannel_frame_id_, base_frame_id_, tf_now, transform_);
        if(tf_rev)
        {
            x_temp = transform_.getOrigin().x();
            y_temp = transform_.getOrigin().y();
            yaw_temp = tf::getYaw(transform_.getRotation());
            if((fabs(x_temp - transform_.getOrigin().x()) > 0.1 || fabs(y_temp - transform_.getOrigin().y()) > 0.1) \
                && !(x == 0 && y == 0 && yaw == 0))
            {
                // tf_last = tf_now;
                tf_rev = 0;
            }
            else
            {
                tf_last = tf_now;
                x = x_temp;
                y = y_temp;
                yaw = yaw_temp;
            }
            // ROS_INFO("X:%0.3f Y:%0.3f Angle:%0.3f", x, y, yaw * 180 / 3.14159);
        }
    }
    catch(tf::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }

    chargedata_flag = uart_->getChargerData(chargedata);
    
    
    pannel_overtime = ros::Time::now() - tf_last;
    period = ros::Time::now() - last_time;
    // ROS_INFO("Period:%0.5f", period.toSec());

    switch(step)
    {
        double stime;
        case STEP_INIT:
            if(!scan_ready)
            {
                scan_overtime += period;
                if(scan_overtime.toSec() > 10)
                {
                    StepChange(STEP_ERROR);
                    ROS_INFO("Fail to get Laserscan!");
                }
            }
            else if(have_obstacle_)
            {
                obstacle_overtime += period;
                if(obstacle_overtime.toSec() > 10)
                {
                    StepChange(STEP_ERROR);
                    ROS_INFO("Charge fail because of obstacle!");
                }
            }
            else if(tf_rev != 1)
            {
                pannel_overtime += period;
                if(pannel_overtime.toSec() > 2)
                {
                    StepChange(STEP_SEARCH);
                    search_time = ros::Time::now();
                    ROS_INFO("Charge Searching!");
                }
            }
            else if(tf_rev == 1)
            {
                StepChange(STEP_ROUND);
                ROS_INFO("Charge Around!");
            }
            setLSpeed(0);
            setASpeed(0);
            break;
        case STEP_SEARCH:
            if(tf_rev)
            {
                setAllSpeedZeroWithAcc();
                StepChange(STEP_ROUND);
                ROS_INFO("Charge Around!");
                break;
            }
            stime = (ros::Time::now() - search_time).toSec();
            if(stime > 40)
            {
                setLSpeed(0);
                setASpeedWithAcc(0);
                StepChange(STEP_ERROR);
            }
            else
            {
                setLSpeed(0);
                setASpeedWithAcc(-0.8);
            }
            break;
        case STEP_ROUND:
            float rpoint_distance;
            if(have_obstacle_)
            {
                setLSpeedWithAcc(0);
                setASpeedWithAcc(0);
                setWaitStable(2);
                ROS_INFO("Have Obstacle!");
                break;
            }

            if((tf_rev != 1))    //search pannel
            {
                pannel_overtime += period;
                if(pannel_overtime.toSec() > 10)
                {
                    StepChange(STEP_ERROR);
                    setLSpeedWithAcc(0);
                    setASpeedWithAcc(0);
                    ROS_INFO("Charge Error!");
                    break;
                }
                else
                {
                    round_angle_max_ = fabs(yaw) > 1.4 ? fabs(yaw) - 0.1745329 : 1.4 - 0.1745329;
                    if(yaw > 0)
                    {
                        setLSpeedWithAcc(0);
                        setASpeedWithAcc(-0.1);
                    }
                    else
                    {
                        setLSpeedWithAcc(0);
                        setASpeedWithAcc(0.1);
                    }
                }
                break;
            }

            if(!isStable())
            {
                setAllSpeedZeroWithAcc();
                break;
            }

            rpoint_distance = sqrt(pow(round_distance_ - fabs(x), 2) + pow(fabs(y), 2));
        
            if(fabs(x) < round_min_distance_)  // too close
            {
                StepChange(STEP_ERROR);
                setAllSpeedZeroWithAcc();
                ROS_INFO("Charge Error!");
                break;
            }
            else if((fabs(y) < 0.01) && x - (-round_distance_) > -0.1)  //next step  || rpoint_distance < 0.05
            {
                setAllSpeedZeroWithAcc();
                StepChange(STEP_SPIN);
                setWaitStable(2);
                pannel_overtime.fromSec(0);
                ROS_INFO("Charge SPIN!");
                break;
            }

            if(y > 0)
            {
                target_angle = std::max(atan2((0 - y), (-round_distance_ - x)), -round_angle_max_);
            }
            else
            {
                target_angle = std::min(atan2((0 - y), (-round_distance_ - x)), round_angle_max_);
            }

            ROS_INFO("Target Angle:%0.4f X:%0.4f Y: %0.4f Angle:%0.4f Limit:%0.4f", target_angle * 180 / M_PI, x, y, yaw * 180 / M_PI, round_angle_max_ * 180 / M_PI);

            target_aspeed = (target_angle - yaw) * 0.5;
            setASpeedWithAcc(target_aspeed);
            if(fabs(target_angle - yaw) > 0.17)
            {
                setLSpeedWithAcc(0);
            }
            else
            {
                if(rpoint_distance < 0.3)
                {
                    setLSpeedWithAcc(0.03);
                }
                else
                {
                    setLSpeedWithAcc(0.2);
                }
            }

            break;
        case STEP_SPIN :
            if(!isStable())
            {
                setAllSpeedZeroWithAcc();
                ROS_INFO("Wait Stable!");
                break;
            }

            if((fabs(x) > obstacle_deadzone_) && have_obstacle_)
            {
                setAllSpeedZeroWithAcc();
                ROS_INFO("Have Obstacle!");
                break;
            }

            if(tf_rev != 1)
            {
                pannel_overtime += period;
                if(pannel_overtime.toSec() > 1)
                {
                    if(yaw > 0)
                    {
                        setLSpeedWithAcc(0);
                        setASpeedWithAcc(-0.1);
                    }
                    else if(yaw < 0)
                    {
                        setLSpeedWithAcc(0);
                        setASpeedWithAcc(0.1);
                    }
                    ROS_INFO("TF ERROR!");
                }
                break;
            }
            else
            {
                pannel_overtime.fromSec(0);
            }

            ROS_INFO("SPINNING X:%0.4f Y: %0.4f Angle:%0.4f", x, y, yaw * 180 / M_PI);
            if(fabs(yaw) < (2 * M_PI / 180))
            {
                motion_waittime += period;
                setAllSpeedZeroWithAcc();
                if(motion_waittime.toSec() > 2)
                {
                    StepChange(STEP_AIMING);
                    motion_waittime.fromSec(0);
                    ROS_INFO("Charge Aiming!");
                }
                break;
            }
            setLSpeedWithAcc(0);
            setASpeedWithAcc(-yaw * 0.5);
            break;
        case STEP_AIMING:
            if((fabs(x) > obstacle_deadzone_) && have_obstacle_)
            {
                setAllSpeedZeroWithAcc();
                setWaitStable(2);
                ROS_INFO("Have Obstacle!");
                break;
            }

            if(tf_rev != 1)
            {
                setAllSpeedZeroWithAcc();
                setWaitStable(2);
                ROS_INFO("TF ERROR!");
                break;
            }

            if(!isStable())
            {
                setAllSpeedZeroWithAcc();
                break;
            }

            if(fabs(x) <= pretouch_distance_ + 0.001)
            {
                StepChange(STEP_PRETOUCH);
                setAllSpeedZero();
                ROS_INFO("Charge Reach! Distance:%0.4f", x);
                break;
            }

            target_angle = -y * 3;
            target_aspeed = (target_angle - yaw) * 0.2;
            if(target_aspeed > 0.20) target_aspeed = 0.20;
            else if(target_aspeed < -0.20) target_aspeed = -0.20;
            setASpeedWithAcc(target_aspeed);
            if(fabs(target_angle - yaw) > 0.17)
            {
                setLSpeedWithAcc(0);
            }
            else
            {
                if(fabs(x) - pretouch_distance_ < 0.1)
                    setLSpeedWithAcc(0.04);
                else if(fabs(x) - pretouch_distance_ < 0.05)
                    setLSpeedWithAcc(0.02);
                else
                    setLSpeedWithAcc(0.05);
            }
            ROS_INFO("Target Distance X:%0.4f Y: %0.4f Angle:%0.4f", x, y, yaw * 180 / M_PI);
            break;
        case STEP_PRETOUCH:
            // setAllSpeedZero();
            // setASpeedWithAcc(-yaw * 0.8);
            setLSpeed(0.005);
            target_angle = -y * 3;
            target_aspeed = (target_angle - yaw) * 0.2;
            if(target_aspeed > 0.20) target_aspeed = 0.20;
            else if(target_aspeed < -0.20) target_aspeed = -0.20;
            setASpeedWithAcc(target_aspeed);
            if(chargedata_flag == true && chargedata.chargecheck == true)
            {
                StepChange(STEP_TOUCH);
            }
            ROS_INFO("Reach X:%0.4f Y: %0.4f Angle:%0.4f", x, y, yaw * 180 / M_PI);
            break;
        case STEP_TOUCH:
            setAllSpeedZero();
            uart_->setSwitchState(1);
            ROS_INFO("Touch X:%0.4f Y: %0.4f Angle:%0.4f", x, y, yaw * 180 / M_PI);
            break;
        case STEP_MOVEBACK:
            setLSpeedWithAcc(-0.1);
            setASpeedWithAcc(0);
            if(ros::Time::now() - cancel_time > ros::Duration(moveback_distance_ / fabs(-0.1)))
            {
                setLSpeedWithAcc(0);
                StepChange(STEP_CANCEL);
            }
            break;
        case STEP_CANCEL :
            setAllSpeedZero();
            break;
        case STEP_ERROR:
            setAllSpeedZero();
            break;
        default:break;
    }

    publishSpeed();

    last_time = ros::Time::now();
}


void ROSAutoCharge::setWaitStable(unsigned int sec)
{
    last_stable_time = ros::Time::now();
    wait_stable = 1;
    stable_waittime.fromSec(sec);
}

bool ROSAutoCharge::isStable()
{
    if(ros::Time::now() > last_stable_time + stable_waittime)
    {
        return 1;
    }
    return 0;
}


void ROSAutoCharge::StepChange(StepEnum nextstep)
{
    motion_waittime.fromSec(0);
    obstacle_overtime.fromSec(0);
    pannel_overtime.fromSec(0);

    step = nextstep;
}


// protoal

ChargerUART::~ChargerUART()
{
    if(se_.isOpen())
    {
        se_.close();
    }
}

ChargerUART::ChargerUART()
{
    state_ = INIT;
    uart_port_ = "/dev/ttyUSB0";
    baudrate_ = 9600;
    retry_interval_ = ros::Duration(2);
    retry_count_ = 10;
}

ChargerUART::ChargerUART(std::string uart_port, uint32_t baudrate)
{
    state_ = INIT;
    uart_port_ = uart_port;
    baudrate_ = baudrate;
    retry_interval_ = ros::Duration(2.0);
    retry_count_ = 10;
}

ChargerUART::ChargerUART(std::string uart_port, uint32_t baudrate, float retry_interval, uint32_t retry_count)
{
    state_ = INIT;
    uart_port_ = uart_port;
    baudrate_ = baudrate;
    retry_interval_ = ros::Duration(retry_interval);
    retry_count_ = retry_count;
}

void ChargerUART::setRetry()
{
    retry_time_ = ros::Time::now();
    state_ = RETRY;
}



bool ChargerUART::setSwitchState(bool state)
{
    if(state_ != OPENED) return 0;
    if(!se_.isOpen()) return 0;
    size_t size;
    uint8_t *temp;
    uint8_t switch_on[5] = {0xED, 0xDE, 0x11, 0x00, 0xDC};
    uint8_t switch_off[5] = {0xED, 0xDE, 0x10, 0x00, 0xDB};

    if(state)
    {
        size = sizeof(switch_on);
        temp = switch_on;
    }
    else
    {
        size = sizeof(switch_off);
        temp = switch_off;
    }
    
    // se_.flush();
    if(se_.write(temp, size) < 1) return 0;
    return 1;
}



bool ChargerUART::getChargerData(ChargeDataTypedef &data)
{
    const static uint32_t match_len = 10;
    int32_t data_len ,pack_len = 0, rest_len = 0, position = 0;
    uint8_t data_pack[100];
    uint16_t sensor_data_tmp;
    if(state_ != OPENED) return 0;
    if(!se_.isOpen()) return 0;
    
    if(se_.available() >= match_len)
    {
        uint32_t check = 0;
        char str_buff[200];
        uint32_t str_len = 0;
        data_len = se_.read(data_pack, std::min(se_.available(),(size_t)sizeof(data_pack)));

        rest_len = data_len;
        while(1)
        {
            for( ; position <= rest_len - match_len; ++position)
            {
                check = 0;
                for(uint32_t i = 0; i < match_len - 1; ++i)
                {
                    check += data_pack[position + i];
                }
                if(data_pack[position] == 0xED && data_pack[position + 1] == 0xDE && (uint8_t)check == data_pack[position + 9])
                {
                    data.chargecheck = data_pack[position + 4];
                    data.button1 = data_pack[position + 5];
                    data.button2 = data_pack[position + 6];
                    data.button3 = data_pack[position + 7];
                    data.button4 = data_pack[position + 8];
                    // ROS_INFO("Charge:%d B1: %d B2:%d B3:%d B4 %d", data.chargecheck, data.button1, data.button2, data.button3, data.button4);
                    // se_.flushInput();
                    return 1;
                }
                // ROS_INFO("Check Error,Skip %d Byte", position);
            }
            se_.flushInput();
            for(size_t k = 0; k < data_len; ++k)
            {
                printf("%02X ",data_pack[k]);
            }
            printf("\r\n");
            ROS_WARN("Recevice Error!");
            return 0;
        }
    }
    return 0;
}


void ChargerUART::PortLoop()
{
    switch(state_)
    {
        case IDLE:
            break;
        case RETRY:
            if(retry_count_ != 0)
            {
                if(ros::Time::now() - retry_time_ < retry_interval_)
                {
                    break;
                }
                retry_count_--;
                se_.close();
            }
            else
            {
                state_ = ERROR;
                break;
            } //continue to init, dont break;
        case INIT:
            if(se_.isOpen()) 
            {
                state_ = OPENED;
                ROS_INFO("Port is Opened!");
                break;
            }
            try
            {
                se_.setPort(uart_port_);
                se_.setBaudrate(baudrate_);
                serial::Timeout timeout = serial::Timeout::simpleTimeout(200);
                se_.setTimeout(timeout);
                if(!se_.isOpen())
                {
                    se_.open();
                }
                else
                {
                    state_ = OPENED;
                    ROS_INFO("Port is Opened!");
                }
            }
            catch(serial::IOException& e)
            {
                ROS_ERROR_STREAM("Unable to open port:"<< e.what()); 
                setRetry();
            }
            break;
        case OPENED:
            if(!se_.isOpen())
            {
                setRetry();
            }
            break;
        case ERROR:
            break;
        default:break;
    }
}


inline bool isEqual(double num1, double num2)
{
    return (std::fabs(num1 - num2) <= EPS);
}

bool PointIsONLine(Point p, Line l)
{
    return(((p.x >= l.p1.x && p.x <= l.p2.x)||(p.x <= l.p1.x && p.x >= l.p2.x)) && ((p.y >= l.p1.y && p.y <= l.p2.y)||(p.y <= l.p1.y && p.y >= l.p2.y)));
}

int CalCrossPoint(Line L1, Line L2, Point& P, double& length)
{
    double   A1,   B1,   C1,   A2,   B2,   C2; 
    A1   =   L1.p2.y   -   L1.p1.y; 
    B1   =   L1.p1.x   -   L1.p2.x; 
    C1   =   L1.p2.x   *   L1.p1.y   -   L1.p1.x   *   L1.p2.y; 
    A2   =   L2.p2.y   -   L2.p1.y; 
    B2   =   L2.p1.x   -   L2.p2.x; 
    C2   =   L2.p2.x   *   L2.p1.y   -   L2.p1.x   *   L2.p2.y; 
    if(isEqual(A1 * B2, B1 * A2))
    {
        if(isEqual((A1 + B1) * C2, (A2 + B2) * C1)) 
        {
            double distance = 0, tmp;
            if(L1.p1.x != 0 || L1.p1.y != 0)
            {
                tmp = sqrt(pow(L1.p1.x, 2) + pow(L1.p1.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L1.p1.x;
                    P.y = L1.p1.y;
                }
            }
            if((L1.p2.x != 0 || L1.p2.y != 0))
            {
                tmp = sqrt(pow(L1.p2.x, 2) + pow(L1.p2.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L1.p2.x;
                    P.y = L1.p2.y;
                }
            }
            if((L2.p1.x != 0 || L2.p1.y != 0))
            {
                tmp = sqrt(pow(L2.p1.x, 2) + pow(L2.p1.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L2.p1.x;
                    P.y = L2.p1.y;
                }
            }
            if((L2.p2.x != 0 || L2.p2.y != 0))
            {
                tmp = sqrt(pow(L2.p2.x, 2) + pow(L2.p2.y, 2));
                if((tmp <= distance && tmp != 0) || distance == 0)
                {
                    distance = tmp;
                    P.x = L2.p2.x;
                    P.y = L2.p2.y;
                }
            }
            length = distance;
            return COLLINEATION;
        }
        else
        {
            length = 0;
            return PARALLEL;
        }
    }
    else
    {
        P.x = (B2 * C1 - B1 * C2) / (A2 * B1 - A1 * B2);
        P.y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);
        if(PointIsONLine(P, L1) && PointIsONLine(P, L2))
        {
            length = sqrt(pow(P.x, 2) + pow(P.y, 2));
            return CROSS; 
        }
        else
        {
            length = 0;
            return NONE;
        }
        
    }
}
