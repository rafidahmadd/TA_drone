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
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>

#define FLIGHT_ALTITUDE 3
#define WP_1_X 4
#define WP_1_Y 0
#define WP_2_X 4
#define WP_2_Y -3

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_position;
sensor_msgs::NavSatFix current_gps_position;
geometry_msgs::PoseStamped pose;

// Subscriber Define
ros::Subscriber global_pos_sub;
ros::Subscriber local_pos_sub;
ros::Subscriber state_sub;
ros::Subscriber pos_sub;
ros::Subscriber init_local_pos_sub;
ros::Subscriber local_vel_sub;

// Publisher Define
ros::Publisher local_pos_pub;
ros::Publisher local_vel_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::ServiceClient land_client;

int serial_port;

void state_cb(const mavros_msgs::State::ConstPtr& msg1) {
    current_state = *msg1;
}

void follow(const geometry_msgs::PoseStamped::ConstPtr& msg2) {
    current_position = *msg2;
}

void gps(const sensor_msgs::NavSatFix::ConstPtr& msg3) {
    current_gps_position = *msg3;
}

void send_string_to_serial(const std::string& data) {
    std::string utf8_data = data;
    write(serial_port, utf8_data.c_str(), utf8_data.length());
}

void send_gps_to_serial(const sensor_msgs::NavSatFix& gps) {
    std::ostringstream oss;
    oss << "GPS Position: Lat=" << gps.latitude << ", Lon=" << gps.longitude << ", Alt=" << gps.altitude << "\n";
    send_string_to_serial(oss.str());
}

void landDrone(ros::NodeHandle& nh)
{
    // Membuat service client untuk landing
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    // Mengecek apakah service tersedia sebelum memanggilnya
    if (land_client.waitForExistence(ros::Duration(5.0)))
    {
        if (land_client.call(land_cmd) && land_cmd.response.success)
        {
            ROS_INFO("Landing command sent successfully.");
        }
        else
        {
            ROS_ERROR("Failed to send landing command.");
        }
    }
    else
    {
        ROS_ERROR("Landing service not available.");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "multidronecoba_node");
    ros::NodeHandle nh;

    // Serial port setup
    serial_port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port == -1) {
        send_string_to_serial("Unable to open serial port\n");
        return -1;
    }
    struct termios options;
    tcgetattr(serial_port, &options);
    cfsetispeed(&options, B57600);
    cfsetospeed(&options, B57600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_port, TCSANOW, &options);

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    global_pos_sub = nh.subscribe("mavros/global_position/global", 10, gps);

    ros::Rate rate(40.0);

    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
        send_string_to_serial("Connecting to FCU...\n");
    }

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    global_pos_sub;
    std::ostringstream oss;
    send_gps_to_serial(current_gps_position);
    // oss << "Posisi gps uav0 y =" << current_gps_position.longitude << "\n";
    // send_string_to_serial(oss.str());
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 x =" << current_gps_position.latitude << "\n";
    // send_string_to_serial(oss.str());

    send_string_to_serial("Takeoff setinggi 3 meter\n");
    for (int i = 0; ros::ok() && i < 10 * 50; ++i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    send_string_to_serial("ketinggian sudah 3 meter\n");

    pose.pose.position.x = WP_1_X;
    pose.pose.position.y = WP_1_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    send_string_to_serial("Menuju Waypoint 1\n");
    for (int i = 0; ros::ok() && i < 10 * 50; ++i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    send_string_to_serial("Waypoint 1 tercapai, gambar diambil\n");
    send_gps_to_serial(current_gps_position);
    int result1 = system("python3 /home/mfproject2/catkin_ws/src/drone_pkg/scripts/capture_camera1.py");

    global_pos_sub;
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 y =" << current_gps_position.longitude << "\n";
    // send_string_to_serial(oss.str());
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 x =" << current_gps_position.latitude << "\n";
    // send_string_to_serial(oss.str());

    pose.pose.position.x = WP_2_X;
    pose.pose.position.y = WP_2_Y;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    send_string_to_serial("Menuju waypoint 2\n");
    for (int i = 0; ros::ok() && i < 10 * 50; ++i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    send_string_to_serial("Waypoint 2 tercapai, gambar diambil\n");
    send_gps_to_serial(current_gps_position);
    int result2 = system("python3 /home/mfproject2/catkin_ws/src/drone_pkg/scripts/capture_camera2.py");

    global_pos_sub;
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 y =" << current_gps_position.longitude << "\n";
    // send_string_to_serial(oss.str());
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 x =" << current_gps_position.latitude << "\n";
    // send_string_to_serial(oss.str());

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    send_string_to_serial("Kembali ke home position\n");
    for (int i = 0; ros::ok() && i < 10 * 50; ++i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    send_string_to_serial("Home position tercapai\n");

    landDrone(nh);
    
    global_pos_sub;
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 y =" << current_gps_position.longitude << "\n";
    // send_string_to_serial(oss.str());
    // oss.str("");
    // oss.clear();
    // oss << "Posisi gps uav0 x =" << current_gps_position.latitude << "\n";
    // send_string_to_serial(oss.str());

    close(serial_port);
    return 0;
}
