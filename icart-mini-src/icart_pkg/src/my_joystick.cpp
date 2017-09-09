#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopIcart{
 public:
   TeleopIcart();
 private:
   void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
   ros::NodeHandle nh;
   int vel_linear, vel_angular;
   double l_scale_, a_scale_;
   ros::Publisher vel_pub_;
   ros::Subscriber joy_sub_;
};

// コンストラクタ
TeleopIcart::TeleopIcart(): vel_linear(1), vel_angular(3){
   // パラメータの初期化
   nh.param("axis_linear", vel_linear, vel_linear);
   nh.param("axis_angular", vel_angular, vel_angular);
   nh.param("scale_angular", a_scale_, a_scale_);
   nh.param("scale_linear", l_scale_, l_scale_);
   
   // 購読するトピックの定義
   joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopIcart::joyCallback, this);
   // 配布するトピックの定義
   vel_pub_ = nh.advertise<geometry_msgs::Twist>("ypspur_ros/cmd_vel", 1);
}

// 購読用コールバック関数
void TeleopIcart::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
   // 配布するメッセージを用意する
   geometry_msgs::Twist twist;
   twist.linear.x = l_scale_*joy->axes[vel_linear];
   twist.angular.z = a_scale_*joy->axes[vel_angular];
   
   ROS_INFO_STREAM("(" << joy->axes[vel_linear] << " " << joy->axes[vel_angular] << ")");
   // メッセージを配布する
   vel_pub_.publish(twist);
}

int main(int argc, char** argv){   
   ros::init(argc, argv, "teleop_icart");
   TeleopIcart TeleopIcart;
   ros::spin();
}
