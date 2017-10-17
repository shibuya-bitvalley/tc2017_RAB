#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct MyPose{
   double x;
   double y;
   double yaw;
};

int main(int argc, char** argv){
   // 起動後に5秒間スリープ
   ROS_INFO("waypoint launched! Wait 5 secs.");
   sleep(5);
   MyPose way_point[] = {
     {  9.167,  1.031, 0.036},
     { 11.929,  1.130, 0.117},
     { 15.270,  1.522, 0.101},
     { 18.749,  1.873, 0.005},
     { 21.650,  1.886, 0.082},
     { 24.588,  2.127, 0.001},
     { 27.197,  2.129, 0.122},
     { 30.312,  2.512, 0.007},
     { 31.898,  2.524,-0.712},
     { 32.553,  1.959,-1.427},
     { 32.854, -0.114,-1.591},
     { 32.788, -3.333,-1.469},
     { 33.068, -6.086,-1.535},
     { 33.211,-10.068,-1.426},
     { 33.691,-13.357,-1.562},
     { 33.723,-16.911,-1.535},
     { 33.821,-19.625,-1.488},
     { 33.981,-21.567,-1.479},
     { 34.204,-24.005,-1.565},
     { 34.222,-27.040,-1.434},
     { 34.651,-30.150,-1.644},
     { 34.438,-33.057,-1.451},
     { 34.824,-36.259,-1.506},
     { 35.027,-39.378,-1.588},
     { 34.983,-41.961,-1.462},
     { 35.381,-45.602,-1.482},
     { 35.610,-48.176,-1.460},
     { 36.011,-51.785,-1.561},
     { 36.045,-55.272,-1.560},
     { 36.083,-58.849,-1.433},
     { 36.571,-62.365,-1.601},
     { 36.489,-65.131,-1.875},
     { 36.245,-65.910,-3.106},
     { 35.113,-65.951,-2.975},
     { 33.189,-66.274,-3.117},
     { 29.203,-66.372,-3.119},
     { 25.760,-66.450,-3.056},
     { 22.098,-66.762,-3.117},
     { 18.203,-66.856,-3.036},
     { 13.624,-67.341,-3.095},
     {  9.550,-67.533,-3.106},
     {  7.376,-67.610,-3.061},
     {  4.348,-67.855,-3.094},
     {  0.364,-68.043,-3.101},
     { -3.416,-68.196,-3.089},
     { -6.040,-68.335,-3.088},
     { -8.574,-68.471, 2.215},
     { -9.336,-67.457, 1.607},
     { -9.446,-64.380, 1.665},
     { -9.885,-59.730, 1.565},
     { -9.872,-57.555, 1.668},
     {-10.158,-54.621, 1.642},
     {-10.341,-52.046, 1.607},
     {-10.442,-49.241, 1.577},
     {-10.461,-46.161, 1.692},
     {-10.654,-44.582, 1.566},
     {-10.643,-42.362, 1.727},
     {-11.112,-39.389, 1.613},
     {-11.416,-32.127, 1.629},
     {-11.651,-28.103, 1.582},
     {-11.692,-24.436, 1.644},
     {-11.781,-23.216, 1.807},
     {-12.420,-20.566, 1.556},
     {-12.367,-16.985, 1.624},
     {-12.648,-11.650, 1.584},
     {-12.704, -7.575, 1.637},
     {-12.998, -3.146, 1.455},
     {-12.686, -0.462, 0.074},
     { -9.114, -0.199, 0.075},
     { -4.502,  0.148, 0.078},
     {  0.788,  0.564, 0.070},
     {  4.722,  0.841, 0.072},
     {999, 999, 999}};
   ros::init(argc, argv, "wp_navigation");
   
   // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。          
   // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。                       
   MoveBaseClient ac("move_base", true);
    
   // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。          
   // この例では５秒間待つ(ブロックされる)                                             
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }
   
   ROS_INFO("The server comes up");
   move_base_msgs::MoveBaseGoal goal;
   // map(地図)座標系(第６週のプログラムとの大きな変更点）                             
   goal.target_pose.header.frame_id = "map";
   // 現在時刻                                                                         
   goal.target_pose.header.stamp = ros::Time::now();
      
   int i = 0;
   while (ros::ok()) {
      // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標                 
      goal.target_pose.pose.position.x =  way_point[i].x;
      goal.target_pose.pose.position.y =  way_point[i].y;
      
      if (goal.target_pose.pose.position.x == 999) break;
      
      goal.target_pose.pose.orientation
	= tf::createQuaternionMsgFromYaw(way_point[i].yaw);
      
      ROS_INFO("Sending goal: No.%d", i+1);
      // サーバーにgoalを送信                                                           
      ac.sendGoal(goal);
      
      // 結果が返ってくるまで60.0[s] 待つ。ここでブロックされる。                       
      bool succeeded = ac.waitForResult(ros::Duration(60.0));
      
      // 結果を見て、成功ならSucceeded、失敗ならFailedと表示                            
      actionlib::SimpleClientGoalState state = ac.getState();
      
      if(succeeded) {
	 ROS_INFO("Succeeded: No.%d (%s)",i+1, state.toString().c_str());
      }
      else {
	 ROS_INFO("Failed: No.%d (%s)",i+1, state.toString().c_str());
      }
      i++;
   }
   return 0;
}
