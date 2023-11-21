/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state_uav0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state_uav0 = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nhuav0;

    ros::Subscriber state_sub_uav0 = nhuav0.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub_uav0 = nhuav0.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav0 = nhuav0.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client_uav0 = nhuav0.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state_uav0.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_uav0;
    pose_uav0.pose.position.x = 0;
    pose_uav0.pose.position.y = 5;
    pose_uav0.pose.position.z = 10;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_uav0.publish(pose_uav0);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode_uav0;
    offb_set_mode_uav0.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_uav0;
    arm_cmd_uav0.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state_uav0.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client_uav0.call(offb_set_mode_uav0) &&
                offb_set_mode_uav0.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state_uav0.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client_uav0.call(arm_cmd_uav0) &&
                    arm_cmd_uav0.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub_uav0.publish(pose_uav0);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}