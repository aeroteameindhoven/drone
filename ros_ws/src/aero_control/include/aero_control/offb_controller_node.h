#ifndef OFFB_CONTROLLER_NODE_H
#define OFFB_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class OffBControllerNode{
public:
    OffBControllerNode();
    void Init_Parameters();

private:
    ros::NodeHandle core;

    mavros_msgs::State current_state;
    mavros_msgs::SetMode set_mode;
    geometry_msgs::PoseStamped setPosition;
    mavros_msgs::CommandBool arm_cmd;

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