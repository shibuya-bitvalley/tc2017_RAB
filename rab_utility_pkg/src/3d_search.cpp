#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){  
   ROS_INFO("inside callback");
}

int main (int argc, char** argv){
   ros::init (argc, argv, "human_search");
   ros::NodeHandle nh;
   ros::Rate loop_rate(10);
   ros::Subscriber sub;
   sub = nh.subscribe ("/hokuyo3d/hokuyo_cloud2", 1, cloud_callback);
   ros::spin();
}
