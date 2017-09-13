#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
   ros::init(argc, argv, "tf_broadcaster_right_urg");
   ros::NodeHandle node;
   

   tf::TransformBroadcaster br;
   tf::Transform transform;
   
   ros::Rate rate(10.0);
   while(node.ok()){
      transform.setOrigin(tf::Vector3(0.0, 1.0, 0.0) );
      transform.setRotation(tf::Quaternion(0, 0, 0.707, 0.707) );
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "right_laser", "right_lrf") );
      rate.sleep();
   }
   
   return 0;
}

   