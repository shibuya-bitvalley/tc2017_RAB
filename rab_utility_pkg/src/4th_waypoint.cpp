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
      { 19.1, 1.49,  0.0 * M_PI},
      { 32.4, 2.31,  -0.5 * M_PI}, // 2辺目突入
      { 33.2, -11.7, -0.5 * M_PI},
      { 34.0, -25.9, -0.5 * M_PI},
      { 35.0, -38.1, -0.5 * M_PI},
      { 35.5, -45.6, -0.5 * M_PI},
      { 35.9, -56.7, -0.5 * M_PI},
      { 36.3, -65.6, -1.0 * M_PI}, // 教室エリア突入
      { 27.2, -66.2, -1.0 * M_PI},
      { 15.7, -67.0, -1.0 * M_PI},
      { 3.93, -67.7, -1.0 * M_PI},
      { -3.3, -67.9, -1.0 * M_PI},
      { -9.5, -68.6,  0.5 * M_PI}, // 4辺目突入
      { -9.9, -59.3,  0.5 * M_PI},
      {-10.5, -50.8,  0.5 * M_PI},
      {-10.8, -39.7,  0.5 * M_PI},
      {-11.8, -28.3,  0.5 * M_PI},
      {-12.6, -9.41,  0.5 * M_PI},
      {-13.4, -0.27,  0.0 * M_PI}, // last turn
      { -4.7,  0.45,  0.0 * M_PI},
      { 5.03,  1.00,  0.0 * M_PI},
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
