#include "aero_control/offb_controller.h"


void OffBController::StateCallback (const mavros_msgs::State::ConstPtr& msg){
    currentState = *msg;
}

void OffBController::TakeOffCallback (const geometry_msgs::PoseStamped::ConstPtr& msg){
    setPosition = *msg;
}

void OffBController::LandingCallback (const geometry_msgs::PoseStamped::ConstPtr& msg){
    setPosition = *msg;
}

OffBController::OffBController()
{

    ros::Subscriber state_sub = core.subscribe<mavros_msgs::State>
            ("mavros/state", 10, &OffBController::StateCallback, this);
    ros::Subscriber takeoff_sub = core.subscribe<geometry_msgs::PoseStamped>
            ("aero_takeoff/takeoff", 10, &OffBController::TakeOffCallback, this);
    ros::Subscriber landing_sub = core.subscribe<geometry_msgs::PoseStamped>
            ("aero_landing/landing", 10, &OffBController::LandingCallback, this);

    ros::Publisher local_pos_pub = core.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = core.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = core.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !currentState.connected){
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

    mavros_msgs::SetMode offbSetMode;
    offbSetMode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool armCmd;
    armCmd.request.value = true;

    ros::Time lastRequest = ros::Time::now();

    while(ros::ok()){
        if( currentState.mode != "OFFBOARD" &&
            (ros::Time::now() - lastRequest > ros::Duration(5.0))){
            if( set_mode_client.call(offbSetMode) &&
                offbSetMode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            lastRequest = ros::Time::now();
        } else {
            if( !currentState.armed &&
                (ros::Time::now() - lastRequest > ros::Duration(5.0))){
                if( arming_client.call(armCmd) &&
                    armCmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                lastRequest = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    new OffBController();
    ros::spin();

    return 0;
}