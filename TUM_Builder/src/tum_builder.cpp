#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgPose)
{
    ROS_INFO("I received pose data !!");
    std::string filename = "tum_mynt.txt";
    double x,y,z,qx,qy,qz,qw;
    double timestamp;

    x = msgPose->pose.pose.position.x;
    y = msgPose->pose.pose.position.y;
    z = msgPose->pose.pose.position.z;
    qx = msgPose->pose.pose.orientation.x;
    qy = msgPose->pose.pose.orientation.y;
    qz = msgPose->pose.pose.orientation.z;
    qw = msgPose->pose.pose.orientation.w;
    timestamp = msgPose->header.stamp.toSec();

    ofstream f;
    f.open(filename,ios::app);
    f << std::fixed;
    f << setprecision(6) << timestamp << " " << setprecision(9) << x << " "
    << y << " " << z << " " << qx << " " << qy << " "<< qz << " " << qw << std::endl; 
    f.close();
}
void poseStampedCallback(const geometry_msgs::PoseStampedConstPtr& msgPose)
{
    ROS_INFO("I received pose data !!");
    std::string filename = "maplab_loc_trajectory_TMI.txt";
    double x,y,z,qx,qy,qz,qw;
    double timestamp;

    x = msgPose->pose.position.x;
    y = msgPose->pose.position.y;
    z = msgPose->pose.position.z;
    qx = msgPose->pose.orientation.x;
    qy = msgPose->pose.orientation.y;
    qz = msgPose->pose.orientation.z;
    qw = msgPose->pose.orientation.w;
    timestamp = msgPose->header.stamp.toSec();

    ofstream f;
    f.open(filename,ios::app);
    f << std::fixed;
    f << setprecision(6) << timestamp << " " << setprecision(9) << x << " "
    << y << " " << z << " " << qx << " " << qy << " "<< qz << " " << qw << std::endl; 
    f.close();
}
void positionCallback(const geometry_msgs::PointStampedConstPtr& msgPoint)
{
    ROS_INFO("Position received!!");
    std::string filename = "MH_03_TUM.txt";
    double x,y,z,qx,qy,qz,qw;
    double timestamp;

    x = msgPoint->point.x;
    y = msgPoint->point.y;
    z = msgPoint->point.z;
    qx = 0.0;
    qy = 0.0;
    qz = 0.0;
    qw = 0.0;
    timestamp = msgPoint->header.stamp.toSec();
    ofstream fp;
    fp.open(filename,ios::app);
    fp << std::fixed;
    fp << setprecision(6) << timestamp << " " << setprecision(9) << x << " "
    << y << " " << z << " " << qx << " " << qy << " "<< qz << " " << qw << std::endl; 
    fp.close();
}
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "tum_builder");
 
   ros::NodeHandle n;
 
   std::string poseTopic = "/rovio/pose_with_covariance_stamped";
   std::string positionTopic = "/leica/position";
   std::string poseStampedTopic = "/maplab_rovio/T_M_I";
   
   ros::Subscriber pose_sub = n.subscribe(poseTopic, 1, poseCallback);
   ros::Subscriber point_sub = n.subscribe(positionTopic, 1, positionCallback);
   ros::Subscriber pointStamp_sub = n.subscribe(poseStampedTopic, 1, poseStampedCallback);

 
   ros::spin();
 
   return 0;
}
