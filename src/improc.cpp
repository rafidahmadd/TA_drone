#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

cv::Mat capturedImage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        capturedImage = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_capture");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/your/camera/topic", 1, imageCallback);


bool compareImages(const cv::Mat& img1, const cv::Mat& img2) {
    cv::Mat diff;
    cv::absdiff(img1, img2, diff);
    cv::cvtColor(diff, diff, cv::COLOR_BGR2GRAY);
    cv::threshold(diff, diff, 50, 255, cv::THRESH_BINARY);
    return cv::countNonZero(diff) == 0;
}

// ...

// Setelah Anda mendapatkan dua gambar (capturedImage dan referenceImage)
if (compareImages(capturedImage, referenceImage)) {
    ROS_INFO("Images are the same");
} else {
    ROS_INFO("Images are different");
}

    // ... Add image capture logic and processing here

    ros::spin();
    return 0;
}
