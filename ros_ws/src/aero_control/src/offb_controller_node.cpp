#include "aero_control/offb_controller_node.h"



void OffBControllerNode::StateCallback (const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void OffBControllerNode::TakeOffCallback (const geometry_msgs::PoseStamped::ConstPtr& msg){
    setPosition = *msg;
}

void OffBControllerNode::LandingCallback (const geometry_msgs::PoseStamped::ConstPtr& msg){
    setPosition = *msg;
}

OffBControllerNode::OffBControllerNode()
{
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffBControllerNode::StateCallback);
    ros::Subscriber takeoff_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("aero_takeoff/takeoff", 10, &OffBControllerNode::TakeOffCallback);
    ros::Subscriber landing_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("aero_landing/landing", 10, &OffBControllerNode::LandingCallback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

}

