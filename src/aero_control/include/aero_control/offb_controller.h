#ifndef OFFB_CONTROLLER_H
#define OFFB_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class OffBController{
public:
    OffBController();
    void Init_Parameters();

private:
    ros::NodeHandle core;
    ros::NodeHandle nh;

    mavros_msgs::State currentState;
    mavros_msgs::SetMode setMode;
    geometry_msgs::PoseStamped setPosition;
    mavros_msgs::CommandBool armCmd;

    //subscriber
    ros::Subscriber state_sub;
    ros::Subscriber takeoff_sub;
    ros::Subscriber landing_sub;

    //publisher
    ros::Publisher local_pos_pub;

    //service
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    void StateCallback(const mavros_msgs::State::ConstPtr& msg);
    void TakeOffCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void LandingCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

};
#endif