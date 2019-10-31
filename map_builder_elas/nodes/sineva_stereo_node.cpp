#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include "elas.h"
#include "stereomatch.h"

ros::Publisher k_depth_pub;

template <class Type>  
Type stringToNum(const std::string& str)
{  
    std::istringstream iss(str); 
    Type num;  
    iss >> num;  
    return num;      
}  

class ImageProcessor
{
public:
    ImageProcessor(float fx, float baseline): fx(fx), baseline(baseline){}
    void processStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight,const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgPose);

private:
    float fx, fy, cx, cy;
    float baseline;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sineva_stereo_node");
    ros::NodeHandle nh;

    std::string focal_length, base_line, leftTopic, rightTopic, poseTopic;
    nh.param<std::string>("sineva_stereo_node/focal_length", focal_length, "361.3");
    nh.param<std::string>("sineva_stereo_node/base_line", base_line, "0.12");
    nh.param<std::string>("sineva_stereo_node/left_topic", leftTopic, "/mynteye/left_rect/image_rect");
    nh.param<std::string>("sineva_stereo_node/right_topic", rightTopic, "/mynteye/right_rect/image_rect");
    nh.param<std::string>("sineva_stereo_node/pose_topic", poseTopic, "/rovio/pose_with_covariance_stamped");

    ImageProcessor imageProcesser(stringToNum<float>(focal_length),stringToNum<float>(base_line));
    //depth publisher
    k_depth_pub = nh.advertise<sensor_msgs::Image>("sineva_depth", 1);
    //subscribe to left and right image
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, leftTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, rightTopic, 1);
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> pose_sub(nh, poseTopic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, 
    geometry_msgs::PoseWithCovarianceStamped> SyncPol;
    message_filters::Synchronizer<SyncPol> sync(SyncPol(10), left_sub, right_sub, pose_sub);
    sync.registerCallback(boost::bind(&ImageProcessor::processStereo, &imageProcesser, _1, _2, _3));
    ros::spin();

    return 0;
}

void ImageProcessor::processStereo(const sensor_msgs::ImageConstPtr& msgLeft, const sensor_msgs::ImageConstPtr& msgRight, const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgPose)
{
    // convert the ros image message to cv::Mat using cvshared
    cv::Mat leftImage, rightImage;
    double x,y,z,qx,qy,qz,qw;
    try
    {
        leftImage = cv_bridge::toCvShare(msgLeft)->image;
        rightImage = cv_bridge::toCvShare(msgRight)->image;
        x = msgPose->pose.pose.position.x;
        y = msgPose->pose.pose.position.y;
        z = msgPose->pose.pose.position.z;
        qx = msgPose->pose.pose.orientation.x;
        qy = msgPose->pose.pose.orientation.y;
        qz = msgPose->pose.pose.orientation.z;
        qw = msgPose->pose.pose.orientation.w;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(leftImage.channels() == 3)
    {
        cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
        cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);
    }

    Eigen::Quaterniond q( qw, qx, qy, qz);
    Eigen::Isometry3d T(q);
    T.pretranslate( Eigen::Vector3d(x,y,z)); 
    StereoMatch stereoMatch;

    cv::Mat disparityImage = stereoMatch.run(leftImage, rightImage, T, baseline, fx);
    sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", disparityImage).toImageMsg();
    k_depth_pub.publish(depthMsg);
}
