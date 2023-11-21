#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Convert the image to grayscale
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds);

        // Draw the markers on the image
        cv::aruco::drawDetectedMarkers(image, markerCorners, markerIds);

        // Display the image with detected markers
        cv::imshow("ArUco Marker Detection", image);
        cv::waitKey(1);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_marker_detector");
    ros::NodeHandle nh_uav1;
    ros::NodeHandle nh_uav2;
    image_transport::ImageTransport it1(nh_uav1);
    image_transport::ImageTransport it2(nh_uav2);
    image_transport::Subscriber sub1 = it1.subscribe("/uav1/mavros/camera/image_captured", 1, imageCallback);
    image_transport::Subscriber sub2 = it2.subscribe("/uav2/mavros/camera/image_captured", 1, imageCallback);

    ros::spin();
    return 0;
}
