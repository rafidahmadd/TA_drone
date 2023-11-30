#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#define FLIGHT_ALTITUDE 3
#define WP_1_X 16
#define WP_1_Y 0
#define WP_2_X 16
#define WP_2_Y -12

// Untuk Follow Aruco
// cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

// ros::Publisher vel_pub_uav1;
// ros::Publisher vel_pub_uav2;
// geometry_msgs::TwistStamped velocity_msg_uav1;
// geometry_msgs::TwistStamped velocity_msg_uav2;

// const int target_marker_id = 1;

// void imageCallback_uav1(const sensor_msgs::ImageConstPtr& msg10) {
//     try {
//         cv::Mat image_uav1 = cv_bridge::toCvShare(msg10, "bgr8")->image;

//         // Convert the image to grayscale
//         cv::Mat gray;
//         cv::cvtColor(image_uav1, gray, cv::COLOR_BGR2GRAY);

//         // Detect ArUco markers
//         std::vector<int> markerIds;
//         std::vector<std::vector<cv::Point2f>> markerCorners;
//         cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);

//         // Process marker data and calculate control commands
//         if (!markerIds.empty()) {
//             // Search for the target marker ID
//             auto it_uav1 = std::find(markerIds.begin(), markerIds.end(), target_marker_id);

//             if (it_uav1 != markerIds.end()) {
//                 int markerIndex = std::distance(markerIds.begin(), it_uav1);
//                 cv::aruco::drawDetectedMarkers(image_uav1, markerCorners, markerIds);

//                 // Calculate control commands based on target marker position
//                 double markerX = (markerCorners[markerIndex][0].x + markerCorners[markerIndex][2].x) / 2;
//                 double markerY = (markerCorners[markerIndex][0].y + markerCorners[markerIndex][2].y) / 2;

//                 double linear_velocity_uav1 = 0.1 * (markerY - image_uav1.rows / 2);
//                 double angular_velocity_uav1 = 0.1 * (markerX - image_uav1.cols / 2);

//                 velocity_msg_uav1.twist.linear.x = linear_velocity_uav1;
//                 velocity_msg_uav1.twist.angular.z = angular_velocity_uav1;

//                 vel_pub_uav1.publish(velocity_msg_uav1);
//             }
//         }

//         // Display the image with detected markers
//         cv::imshow("ArUco Marker Detection", image_uav1);
//         cv::waitKey(1);

//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// void imageCallback_uav2(const sensor_msgs::ImageConstPtr& msg11) {
//     try {
//         cv::Mat image_uav2 = cv_bridge::toCvShare(msg11, "bgr8")->image;

//         // Convert the image to grayscale
//         cv::Mat gray;
//         cv::cvtColor(image_uav2, gray, cv::COLOR_BGR2GRAY);

//         // Detect ArUco markers
//         std::vector<int> markerIds;
//         std::vector<std::vector<cv::Point2f>> markerCorners;
//         cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);

//         // Process marker data and calculate control commands
//         if (!markerIds.empty()) {
//             // Search for the target marker ID
//             auto it_uav2 = std::find(markerIds.begin(), markerIds.end(), target_marker_id);

//             if (it_uav2 != markerIds.end()) {
//                 int markerIndex = std::distance(markerIds.begin(), it_uav2);
//                 cv::aruco::drawDetectedMarkers(image_uav2, markerCorners, markerIds);

//                 // Calculate control commands based on target marker position
//                 double markerX = (markerCorners[markerIndex][0].x + markerCorners[markerIndex][2].x) / 2;
//                 double markerY = (markerCorners[markerIndex][0].y + markerCorners[markerIndex][2].y) / 2;

//                 double linear_velocity_uav2 = 0.1 * (markerY - image_uav2.rows / 2);
//                 double angular_velocity_uav2 = 0.1 * (markerX - image_uav2.cols / 2);

//                 velocity_msg_uav2.twist.linear.x = linear_velocity_uav2;
//                 velocity_msg_uav2.twist.angular.z = angular_velocity_uav2;

//                 vel_pub_uav2.publish(velocity_msg_uav2);
//             }
//         }

//         // Display the image with detected markers
//         cv::imshow("ArUco Marker Detection", image_uav2);
//         cv::waitKey(1);

//     } catch (cv_bridge::Exception& e) {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

//---

mavros_msgs::State current_state;
mavros_msgs::State current_state_uav1;
mavros_msgs::State current_state_uav2;

geometry_msgs::PoseStamped current_position;
geometry_msgs::PoseStamped current_position_uav1;
geometry_msgs::PoseStamped current_position_uav2;

sensor_msgs::NavSatFix current_gps_position;
sensor_msgs::NavSatFix current_gps_position_uav1;
sensor_msgs::NavSatFix current_gps_position_uav2;

geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped pose_uav1;
geometry_msgs::PoseStamped pose_uav2;


// Subscriber Define
ros::Subscriber global_pos_sub;
ros::Subscriber global_pos_sub_uav1;
ros::Subscriber global_pos_sub_uav2;
ros::Subscriber local_pos_sub;
ros::Subscriber local_pos_sub_uav1;
ros::Subscriber local_pos_sub_uav2;
ros::Subscriber state_sub;
ros::Subscriber pos_sub;
ros::Subscriber state_sub_uav1;
ros::Subscriber state_sub_uav2;
ros::Subscriber init_local_pos_sub;
ros::Subscriber init_local_pos_sub_uav1;
ros::Subscriber init_local_pos_sub_uav2;
ros::Subscriber local_vel_sub;
ros::Subscriber local_vel_sub_uav1;
ros::Subscriber local_vel_sub_uav2;

// Publisher Define
ros::Publisher local_pos_pub;
ros::Publisher local_pos_pub_uav1;
ros::Publisher local_pos_pub_uav2;
ros::Publisher local_vel_pub;
ros::Publisher local_vel_pub_uav1;
ros::Publisher local_vel_pub_uav2;
ros::ServiceClient arming_client;
ros::ServiceClient arming_client_uav1;
ros::ServiceClient arming_client_uav2;
ros::ServiceClient set_mode_client;
ros::ServiceClient set_mode_client_uav1;
ros::ServiceClient set_mode_client_uav2;
ros::ServiceClient land_client;
ros::ServiceClient land_client_uav1;
ros::ServiceClient land_client_uav2;


void state_cb(const mavros_msgs::State::ConstPtr& msg1){
    current_state = *msg1;
}

void state_cb_uav1(const mavros_msgs::State::ConstPtr& msg2){
    current_state_uav1 = *msg2;
}

void state_cb_uav2(const mavros_msgs::State::ConstPtr& msg3){
    current_state_uav2 = *msg3;
}

//void poseCallbackUAV0(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//    pose_uav0 = *msg;
//}

//void poseCallbackUAV1(const geometry_msgs::PoseStamped::ConstPtr& msg) {
//    pose_uav1 = *msg;
//}

//void poseCallback0(const geometry_msgs::PoseStamped::ConstPtr& fmsg1) {
    // Mendapatkan informasi posisi dari pesan
//    geometry_msgs::Point position0 = fmsg1->pose.position;
    //ROS_INFO("Position: x=%f, y=%f, z=%f", position0.x, position0.y, position0.z);
//}

//void poseCallback1(const geometry_msgs::PoseStamped::ConstPtr& fmsg2) {
    // Mendapatkan informasi posisi dari pesan
//    geometry_msgs::Point position1 = fmsg2->pose.position;
    //ROS_INFO("Position: x=%f, y=%f, z=%f", position1.x, position1.y, position1.z);
//}
void follow0(const geometry_msgs::PoseStamped::ConstPtr& msg4){
current_position = *msg4;

}

void follow1(const geometry_msgs::PoseStamped::ConstPtr& msg5){
current_position_uav1 = *msg5;

}

void follow2(const geometry_msgs::PoseStamped::ConstPtr& msg6){
current_position_uav2 = *msg6;

}

void gps0(const sensor_msgs::NavSatFix::ConstPtr& msg7){
current_gps_position = *msg7;

}

void gps1(const sensor_msgs::NavSatFix::ConstPtr& msg8){
current_gps_position_uav1 = *msg8;

}

void gps2(const sensor_msgs::NavSatFix::ConstPtr& msg9){
current_gps_position_uav2 = *msg9;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multidronecoba_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_uav1;
    ros::NodeHandle nh_uav2;

    // subs and pubs for uavs

    // image_transport::ImageTransport it_uav1(nh_uav1);
    // image_transport::Subscriber sub_uav1 = it_uav1.subscribe("iris1/usb_cam/image_raw/compressed", 1, imageCallback_uav1);
    // vel_pub_uav1 = nh_uav1.advertise<geometry_msgs::TwistStamped>("uav1/mavros/setpoint_velocity/cmd_vel", 1);
    
    // image_transport::ImageTransport it_uav2(nh_uav2);
    // image_transport::SubscDICT_6X6_250riber sub_uav2 = it_uav2.subscribe("iris2/usb_cam/image_raw/compressed", 1, imageCallback_uav2);
    // vel_pub_uav2 = nh_uav2.advertise<geometry_msgs::TwistStamped>("uav2/mavros/setpoint_velocity/cmd_vel", 1);

    // ros::Publisher pub = node_handle.advertise<message_type>(topic_name, queue_size);
    
    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("mavros/cmd/land");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //ros::Subscriber sub = nh_uav0.subscribe("uav0/mavros/local_position/pose", 10, poseCallback0);
    local_pos_sub = nh.subscribe("mavros/local_position/odom", 10, follow0);
    global_pos_sub = nh.subscribe("mavros/global_position/global", 10, gps0);
    //global_pos_sub_uav0 = nh_uav0.subscribe("uav0/mavros/odometry/in", 10, gps0);

    // state_sub_uav1 = nh_uav1.subscribe<mavros_msgs::State>
    //         ("uav1/mavros/state", 10, state_cb_uav1);
    // local_pos_pub_uav1 = nh_uav1.advertise<geometry_msgs::PoseStamped>
    //         ("uav1/mavros/setpoint_position/local", 10);
    // arming_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandBool>
    //         ("uav1/mavros/cmd/arming");
    // land_client_uav1 = nh_uav1.serviceClient<mavros_msgs::CommandTOL>
    //         ("uav1/mavros/cmd/land");
    // set_mode_client_uav1 = nh_uav1.serviceClient<mavros_msgs::SetMode>
    //         ("uav1/mavros/set_mode");
    // //ros::Subscriber sub = nh_uav0.subscribe("uav0/mavros/local_position/pose", 10, poseCallback1);
    // local_pos_sub_uav1 = nh_uav1.subscribe("uav1/mavros/local_position/odom", 10, follow1);
    // global_pos_sub_uav1 = nh_uav1.subscribe("uav1/mavros/global_position/global", 10, gps1);
    // //global_pos_sub_uav1 = nh_uav1.subscribe("uav1/mavros/odometry/in", 10, gps1);
    
    // state_sub_uav2 = nh_uav2.subscribe<mavros_msgs::State>
    //         ("uav2/mavros/state", 10, state_cb_uav2);
    // local_pos_pub_uav2 = nh_uav2.advertise<geometry_msgs::PoseStamped>
    //         ("uav2/mavros/setpoint_position/local", 10);
    // arming_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandBool>
    //         ("uav2/mavros/cmd/arming");
    // land_client_uav2 = nh_uav2.serviceClient<mavros_msgs::CommandTOL>
    //         ("uav2/mavros/cmd/land");
    // set_mode_client_uav2 = nh_uav2.serviceClient<mavros_msgs::SetMode>
    //         ("uav2/mavros/set_mode");
    // local_pos_sub_uav2 = nh_uav2.subscribe("uav2/mavros/local_position/odom", 10, follow2);
    // global_pos_sub_uav2 = nh_uav2.subscribe("uav2/mavros/global_position/global", 10, gps2);
    // // global_pos_sub_uav2 = nh_uav2.subscribe("uav2/mavros/odometry/in", 10, gps2);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(40.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected && current_state_uav1.connected && current_state_uav2.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCU...");
    }

   

//uav0 mode and commands
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

//uav1 mode and commands
    mavros_msgs::CommandBool arm_cmd_uav1;
    arm_cmd_uav1.request.value = true;
    
    mavros_msgs::SetMode offb_set_mode_uav1; 
    offb_set_mode_uav1.request.custom_mode = "AUTO.TAKEOFF";
    
    mavros_msgs::CommandTOL land_cmd_uav1;
    land_cmd_uav1.request.yaw = 0;
    land_cmd_uav1.request.latitude = 0;
    land_cmd_uav1.request.longitude = 0;
    land_cmd_uav1.request.altitude = 0;

//uav2 mode and commands
    mavros_msgs::CommandBool arm_cmd_uav2;
    arm_cmd_uav2.request.value = true;

    mavros_msgs::SetMode offb_set_mode_uav2;
    offb_set_mode_uav2.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandTOL land_cmd_uav2;
    land_cmd_uav2.request.yaw = 0;
    land_cmd_uav2.request.latitude = 0;
    land_cmd_uav2.request.longitude = 0;
    land_cmd_uav2.request.altitude = 0;


    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm

    // while(ros::ok() && !current_state_uav0.armed && 
    // !current_state_uav1.armed && !current_state_uav2.armed){
    // if( !current_state_uav0.armed && !current_state_uav1.   armed && !current_state_uav2.armed &&
    // (ros::Time::now() - last_request > ros::Duration(5.0))){
        // if( arming_client_uav2.call(arm_cmd_uav2) && arm_cmd_uav2.response.success && 
            // arming_client_uav0.call(offb_set_mode_uav0) && arm_cmd_uav0.response.success &&
            // arming_client_uav1.call(arm_cmd_uav1) && arm_cmd_uav1.response.success){
            // ROS_INFO("Persiapan");
            // }
        // last_request = ros::Time::now();
    // } else {
        // if( current_state_uav0.mode != "AUTO.TAKEOFF" && current_state_uav1.mode != "AUTO.TAKEOFF" && current_state_uav2.mode != "AUTO.TAKEOFF" 
        // && (ros::Time::now() - last_request > ros::Duration(5.0))){
            // if( set_mode_client_uav2.call(offb_set_mode_uav2) && offb_set_mode_uav2.response.mode_sent &&
                // set_mode_client_uav0.call(offb_set_mode_uav0) && offb_set_mode_uav0.response.mode_sent && 
                // set_mode_client_uav1.call(offb_set_mode_uav1) && offb_set_mode_uav1.response.mode_sent){
                // ROS_INFO("Mode Offboard");
            // }
            // last_request = ros::Time::now();
            // }
        // }
    // ros::spinOnce();
    // rate.sleep();
    // }        





    // while(ros::ok() && !current_state_uav0.armed && 
    // !current_state_uav1.armed && !current_state_uav2.armed){
    // if( current_state_uav0.mode != "OFFBOARD" && current_state_uav1.mode != "OFFBOARD" && current_state_uav2.mode != "OFFBOARD" 
    // && (ros::Time::now() - last_request > ros::Duration(5.0))){
        // if( set_mode_client_uav2.call(offb_set_mode_uav2) && offb_set_mode_uav2.response.mode_sent &&
            // set_mode_client_uav0.call(offb_set_mode_uav0) && offb_set_mode_uav0.response.mode_sent && 
            // set_mode_client_uav1.call(offb_set_mode_uav1) && offb_set_mode_uav1.response.mode_sent){
            // ROS_INFO("Mode Offboard");
        // }
        // last_request = ros::Time::now();
    // } else {
        // if( !current_state_uav0.armed && !current_state_uav1.armed && !current_state_uav2.armed &&
            // (ros::Time::now() - last_request > ros::Duration(5.0))){
            // if( arming_client_uav2.call(arm_cmd_uav2) && arm_cmd_uav2.response.success && 
                // arming_client_uav0.call(arm_cmd_uav0) && arm_cmd_uav0.response.success &&
                // arming_client_uav1.call(arm_cmd_uav1) && arm_cmd_uav1.response.success){
                // ROS_INFO("Persiapan");
                // }
            // last_request = ros::Time::now();
            // }
        // }
        // local_pos_pub_uav0.publish(pose_uav0);
        // local_pos_pub_uav1.publish(pose_uav1);
        // local_pos_pub_uav2.publish(pose_uav2);

    // ros::spinOnce();
    // rate.sleep();
    // }


    // while(ros::ok() && !current_state_uav0.armed && 
    // !current_state_uav1.armed && !current_state_uav2.armed){
    // if( current_state_uav0.mode != "OFFBOARD" && current_state_uav1.mode != "OFFBOARD" && current_state_uav2.mode != "OFFBOARD" 
    // && (ros::Time::now() - last_request > ros::Duration(5.0))){
        // if( set_mode_client_uav2.call(offb_set_mode_uav2) && offb_set_mode_uav2.response.mode_sent &&
            // set_mode_client_uav0.call(offb_set_mode_uav0) && offb_set_mode_uav0.response.mode_sent && 
            // set_mode_client_uav1.call(offb_set_mode_uav1) && offb_set_mode_uav1.response.mode_sent){
            // ROS_INFO("Take Off");
        // }
        // last_request = ros::Time::now(); 
        // }
// 
    // local_pos_pub_uav0.publish(pose_uav0);
    // local_pos_pub_uav1.publish(pose_uav1);
    // local_pos_pub_uav2.publish(pose_uav2);
    // ros::spinOnce();
    // rate.sleep();
    // }   


// Takeoff
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+0.5;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-1,5;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("takeoff setinggi 3 meter");
    for(int i = 0; ros::ok() && i < 10*40; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("ketinggian sudah 3 meter");

      // Menuju Waypoint B
    pose.pose.position.x = WP_1_X;
    pose.pose.position.y = WP_1_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Menuju Waypoint Pertama");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Waypoint Pertama Tercapai");

    // UAV0 Naik ke 4 meter
    pose.pose.position.x = WP_1_X;
    pose.pose.position.y = WP_1_Y;
    pose.pose.position.z = 4;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Master Naik ke 4 Meter");
    for(int i = 0; ros::ok() && i < 10*15; ++i){
      
      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Master sudah di ketinggian 4 Meter");

    // UAV1 menuju ke patok B
    pose.pose.position.x = WP_1_X;
    pose.pose.position.y = WP_1_Y;
    pose.pose.position.z = 4;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y -1;
    // pose_uav1.pose.position.z = 2;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose.pose.position.y -1;
    // pose_uav2.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("UAV1 mengambil data");
    

    for(int i = 0; ros::ok() && i < 10*30; ++i){
      
      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Data berhasil diambil oleh UAV1");  

    double uav0_x = current_position.pose.position.x;
    double uav1_x = current_position_uav1.pose.position.x;
    double uav2_x = current_position_uav2.pose.position.x;

    double uav0_y = current_position.pose.position.y;
    double uav1_y = current_position_uav1.pose.position.y;
    double uav2_y = current_position_uav2.pose.position.y;
    
    double uav0_z = current_position.pose.position.z;
    double uav1_z = current_position_uav1.pose.position.z;
    double uav2_z = current_position_uav2.pose.position.z;

    double gps_x = current_gps_position.longitude;
    double gps_uav1_x = current_gps_position_uav1.longitude;
    double gps_uav2_x = current_gps_position_uav1.longitude;

    double gps_y = current_gps_position.latitude;
    double gps_uav1_y = current_gps_position_uav1.latitude;
    double gps_uav2_y = current_gps_position_uav2.latitude;
    
    

    ROS_INFO("Mengirim data dari UAV1 ke Master");

    
    // for(int i = 0; ros::ok() && i < 10*30; ++i){
    //local_pos_sub_uav0 = nh_uav0.subscribe<geometry_msgs::PoseStamped>
    //        ("uav0/mavros/local_position/pose", 10, follow0);
    //local_pos_sub_uav1 = nh_uav1.subscribe<geometry_msgs::PoseStamped>
    //        ("uav1/mavros/local_position/pose", 10, follow1);
    //local_pos_sub_uav2 = nh_uav2.subscribe<geometry_msgs::PoseStamped>
    //        ("uav2/mavros/local_position/pose", 10, follow2);        
    

    //ROS_INFO("Posisi uav0 y =%f", uav0_y);
    //ROS_INFO("Posisi uav1 y =%f", uav1_y);
    //ROS_INFO("Posisi uav2 y =%f", uav2_y);
    //ROS_INFO("Posisi uav0 z =%f", uav0_z);
    //ROS_INFO("Posisi uav1 z =%f", uav1_z);
    //ROS_INFO("Posisi uav2 z =%f", uav2_z);
    global_pos_sub;
    global_pos_sub_uav1;
    global_pos_sub_uav2;
        
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // ROS_INFO("Posisi gps uav0 y =%f", gps_uav0_y);
    // ROS_INFO("Posisi gps uav1 y =%f", gps_uav1_y);
    // ROS_INFO("Posisi gps uav2 y =%f", gps_uav2_y);
    // ROS_INFO("Posisi gps uav0 x =%f", gps_uav0_x);
    // ROS_INFO("Posisi gps uav1 x =%f", gps_uav1_x);
    // ROS_INFO("Posisi gps uav2 x =%f", gps_uav2_x);
    
    ROS_INFO("Posisi gps uav0 y =%f", uav0_y);
    ROS_INFO("Posisi gps uav1 y =%f", uav1_y);
    ROS_INFO("Posisi gps uav2 y =%f", uav2_y);
    ROS_INFO("Posisi gps uav0 x =%f", uav0_x);
    ROS_INFO("Posisi gps uav1 x =%f", uav1_x);
    ROS_INFO("Posisi gps uav2 x =%f", uav2_x);

    // if (uav0_y == uav1_y) {
    //     ROS_INFO("Patok Tidak Berubah");
    // } else {
    //     ROS_ERROR("Patok Berubah!!");
    // }

    
    

    ROS_INFO("Data Berhasil Diterima");
    
    // UAV0 dan UAV1 Kembali ke 3 meter
    pose.pose.position.x = WP_1_X;
    pose.pose.position.y = WP_1_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Assembly Formasi");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Formasi Berhasil Terbentuk");

    // Menuju Waypoint C
    pose.pose.position.x = WP_2_X;
    pose.pose.position.y = WP_2_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Menuju Waypoint Kedua");
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Waypoint Kedua Tercapai");

     // UAV0 Naik ke 4 meter
    pose.pose.position.x = WP_2_X;
    pose.pose.position.y = WP_2_Y;
    pose.pose.position.z = 4;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Master Naik ke 4 Meter");
    for(int i = 0; ros::ok() && i < 10*15; ++i){
      
      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Master sudah di ketinggian 4 Meter");

    // UAV2 menuju ke patok C
    pose.pose.position.x = WP_2_X;
    pose.pose.position.y = WP_2_Y;
    pose.pose.position.z = 4;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose.pose.position.y+1;
    pose_uav2.pose.position.z = 2;

    ROS_INFO("UAV2 mengambil data");
    for(int i = 0; ros::ok() && i < 10*30; ++i){
      
      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Data berhasil diambil oleh UAV2");

    ROS_INFO("Mengirim data dari UAV2 ke Master");

    // ROS_INFO("Posisi gps uav0 y =%f", gps_uav0_y);
    // ROS_INFO("Posisi gps uav1 y =%f", gps_uav1_y);
    // ROS_INFO("Posisi gps uav2 y =%f", gps_uav2_y);
    // ROS_INFO("Posisi gps uav0 x =%f", gps_uav0_x);
    // ROS_INFO("Posisi gps uav1 x =%f", gps_uav1_x);
    // ROS_INFO("Posisi gps uav2 x =%f", gps_uav2_x);

    ROS_INFO("Posisi gps uav0 y =%f", uav0_y);
    ROS_INFO("Posisi gps uav1 y =%f", uav1_y);
    ROS_INFO("Posisi gps uav2 y =%f", uav2_y);
    ROS_INFO("Posisi gps uav0 x =%f", uav0_x);
    ROS_INFO("Posisi gps uav1 x =%f", uav1_x);
    ROS_INFO("Posisi gps uav2 x =%f", uav2_x);

    // if (uav0_y == uav2_y){
    //     ROS_INFO("Patok Tidak Berubah");
    // } else {
    //     ROS_ERROR("Patok Berubah!!");
    // }
    
    ROS_INFO("Data diterima");

    // UAV0 dan UAV1 Kembali ke 3 meter
    pose.pose.position.x = WP_2_X;
    pose.pose.position.y = WP_2_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y+1;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y-2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Assembly Formasi");
    for(int i = 0; ros::ok() && i < 10*20; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("Formasi Berhasil Terbentuk");

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // pose_uav1.pose.position.x = pose.pose.position.x;
    // pose_uav1.pose.position.y = pose.pose.position.y + 1;
    // pose_uav1.pose.position.z = pose.pose.position.z;

    // pose_uav2.pose.position.x = pose_uav1.pose.position.x;
    // pose_uav2.pose.position.y = pose_uav1.pose.position.y - 2;
    // pose_uav2.pose.position.z = pose_uav1.pose.position.z;

    ROS_INFO("Kembali ke Home Position");
    //send setpoints for 10 seconds
    for(int i = 0; ros::ok() && i < 10*30; ++i){

      local_pos_pub.publish(pose);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("UAV sudah berada di Home");

    while (ros::ok()) {
        // Your soft landing control logic goes here
        // You may use altitude data from altimeter or other sensors to control the descent speed

        // Example: Set the target position to move the drone down
        pose.header.stamp = ros::Time::now();
        pose.pose.position.z -= 0.1;  // Decrease altitude by 0.1 meters per iteration
        

        // pose_uav1.header.stamp = ros::Time::now();
        // pose_uav1.pose.position.z -= 0.1;  // Decrease altitude by 0.1 meters per iteration
        
        // pose_uav2.header.stamp = ros::Time::now();
        // pose_uav2.pose.position.z -= 0.1;  // Decrease altitude by 0.1 meters per iteration
        
        for(int i = 0; ros::ok() && i < 10*2; ++i){

            local_pos_pub.publish(pose);
            // local_pos_pub_uav1.publish(pose_uav1);
            // local_pos_pub_uav2.publish(pose_uav2);

            ros::spinOnce();
            rate.sleep();
        }

        // Disarm the drone when soft landing is complete
        if (current_state.armed && pose.pose.position.z <= 0.0) {
            ROS_INFO("Soft landing complete. Disarming...");

            for(int i = 0; ros::ok() && i < 10*30; ++i){

            local_pos_pub.publish(pose);
            // local_pos_pub_uav1.publish(pose_uav1);
            // local_pos_pub_uav2.publish(pose_uav2);
            
            ros::spinOnce();
            rate.sleep();
        }


            arm_cmd.request.value = false;
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                 ROS_INFO("Disarmed");
                 break;  // Exit the loop when disarmed
             } else {
                 ROS_ERROR("Failed to disarm");
                 return -1;
             }
        }

        

                // Disarm the drone when soft landing is complete
        // if (current_state_uav1.armed && pose_uav1.pose.position.z <= 0.0) {
        //     ROS_INFO("Soft landing complete. Disarming...");
        //     offb_set_mode_uav1.request.custom_mode = "AUTO.LAND";


        //     if (set_mode_client_uav1.call(offb_set_mode_uav1) && offb_set_mode_uav1.response.mode_sent) {
        //     ROS_INFO("UAV1 Landing initiated");
        //     } else {
        //      ROS_ERROR("Failed to set LAND mode");
        //     return -1;
        //     }

        //     for(int i = 0; ros::ok() && i < 10*10; ++i){

        //     local_pos_pub_uav1.publish(pose_uav1);

        //     ros::spinOnce();
        //     rate.sleep();
        //     }

            // arm_cmd_uav0.request.value = false;
            // if (arming_client_uav1.call(arm_cmd_uav1) && arm_cmd_uav1.response.success) {
            //     ROS_INFO("Disarmed");
            //     break;  // Exit the loop when disarmed
            // } else {
            //     ROS_ERROR("Failed to disarm");
            //     return -1;
            // }
        // }

        

        // Disarm the drone when soft landing is complete
        // if (current_state_uav2.armed && pose_uav2.pose.position.z <= 0.0) {
        //     ROS_INFO("Soft landing complete. Disarming...");
        //     offb_set_mode_uav2.request.custom_mode = "AUTO.LAND";

        //     if (set_mode_client_uav2.call(offb_set_mode_uav2) && offb_set_mode_uav2.response.mode_sent) {
        //     ROS_INFO("UAV0 Landing initiated");
        // } else {
        //      ROS_ERROR("Failed to set LAND mode");
        //     return -1;
        // }

        //     for(int i = 0; ros::ok() && i < 10*10; ++i){

        //     local_pos_pub_uav2.publish(pose_uav2);

        //     ros::spinOnce();
        //     rate.sleep();
        // }

        //     // arm_cmd_uav2.request.value = false;
        //     // if (arming_client_uav2.call(arm_cmd_uav2) && arm_cmd_uav2.response.success) {
        //     //     ROS_INFO("Disarmed");
        //     //     break;  // Exit the loop when disarmed
        //     // } else {
        //     //     ROS_ERROR("Failed to disarm");
        //     //     return -1;
        //     // }
        // }
    }


    // offb_set_mode_uav0.request.custom_mode = "AUTO.LAND";
    // offb_set_mode_uav1.request.custom_mode = "AUTO.LAND";
    // offb_set_mode_uav2.request.custom_mode = "AUTO.LAND";

    // if (set_mode_client_uav0.call(offb_set_mode_uav0) && offb_set_mode_uav0.response.mode_sent) {
    //     ROS_INFO("UAV0 Landing initiated");
    // } else {
    //     ROS_ERROR("Failed to set LAND mode");
    //     return -1;
    // }

    // if (set_mode_client_uav1.call(offb_set_mode_uav1) && offb_set_mode_uav1.response.mode_sent) {
    //     ROS_INFO("UAV1 Landing initiated");
    // } else {
    //     ROS_ERROR("Failed to set LAND mode");
    //     return -1;
    // }

    // if (set_mode_client_uav2.call(offb_set_mode_uav2) && offb_set_mode_uav2.response.mode_sent) {
    //     ROS_INFO("UAV2 Landing initiated");
    // } else {
    //     ROS_ERROR("Failed to set LAND mode");
    //     return -1;
    // }



    // ROS_INFO("Landing at Home");
    // //send setpoints for 10 seconds
    // for(int i = 0; ros::ok() && i < 10*30; ++i){

    //   local_pos_pub_uav0.publish(pose_uav0);
    //   local_pos_pub_uav1.publish(pose_uav1);
    //   local_pos_pub_uav2.publish(pose_uav2);

    //   ros::spinOnce();
    //   rate.sleep();
    // }
    // --- ROS_INFO("mendarat");
 /* As to land all UAV simultaneously the not landing condition should be checked combined instead of 
   individual UAV as In that case Only first land and other entered into failsafe model*/
    // --- while (! (land_client_uav0.call(land_cmd_uav0) && land_cmd_uav0.response.success && 
    // ---            land_client_uav0.call(land_cmd_uav1) && land_cmd_uav1.response.success &&
    // ---            land_client_uav0.call(land_cmd_uav2) && land_cmd_uav2.response.success))
        /*(!(land_client_uav0.call(land_cmd_uav0) && land_cmd_uav0.response.success)) &&
            !(land_client_uav1.call(land_cmd_uav1) &&
                land_cmd_uav1.response.success) &&
             (!(land_client_uav2.call(land_cmd_uav2) &&
            land_cmd_uav2.response.success))*/
     
    // ---   local_pos_pub_uav0.publish(pose_uav0);  
    // ---   local_pos_pub_uav1.publish(pose_uav1);  
    // ---   local_pos_pub_uav2.publish(pose_uav2);
    // ---   ROS_INFO("mendarat");
    // ---   ros::spinOnce();
    // ---   rate.sleep();
    // --- }
    // return 0;



}
