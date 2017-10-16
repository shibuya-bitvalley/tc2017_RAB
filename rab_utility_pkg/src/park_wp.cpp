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
   // 起動後に10秒間スリープ
   ROS_INFO("waypoint launched! Wait 10 secs.");
   sleep(10);
   MyPose way_point[] = {
      { 18.429 , 4.215 , 0.373 },
      { 27.828 , 7.819 , 0.420 },
      { 36.811 ,11.240 , 0.452 },
      { 46.963 ,16.177 , 0.432 },
      { 56.226 ,21.454 , 0.472 },
      { 68.530 ,26.750 , 2.002 },
      { 63.910 ,36.780 , 2.053 },
      { 59.391 ,45.409 , 2.202 },
      { 53.301 ,53.729 , 2.267 },
      { 47.934 ,60.137 , 2.305 },
      { 41.420 ,67.347 ,-2.823 },
      { 31.447 ,64.068 ,-2.520 },
      { 20.195 ,56.008 ,-2.490 },
      { 10.587 ,48.683 ,-2.483 },
      { -0.748 ,39.928 ,-2.440 },
      { -8.385 ,33.486 ,-2.444 },
      {-14.794 ,28.120 ,-2.418 },
      {-27.196 ,17.176 ,-2.399 },
      {-34.746 ,10.247 ,-2.414 },
      {-40.388 , 5.223 ,-2.490 },
      {-44.998 , 1.714 , 2.343 },
      {-49.372 , 6.200 , 2.301 },
      {-53.577 ,10.893 , 2.284 },
      {-58.747 ,16.863 , 2.304 },
      {-64.748 ,23.514 ,-0.348 },
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
   // map(地図)座標系                             
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
