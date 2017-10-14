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
<<<<<<< HEAD
      { 26.6, 10.2, 0.0 * M_PI},

=======
      { 13.8139343262,23.3812561035 ,1.3944182342 },
      { 16.8216896057,40.2569389343 ,1.4784300397 },
      { 18.6961421967,60.4928817749 ,1.49628469   },
      { 20.2949428558,81.9102172852 ,2.3031130838 },
      { 10.9216060638,92.3354797363 ,-3.102905342 },
      { -4.9806923866,91.7199554443 ,-3.0139466213},
      {-19.7912845612,89.8191070557 ,-2.0899815781},
      {-27.1875209808,76.8768692017 ,-1.4385496664},
      {-25.4863376617,64.0882339478 ,-1.3867688389},
      {-23.8420333862,55.2542266846 ,-1.4262096329},
      {-22.0647068024,43.047542572  ,-1.3759790182},
      {-18.4042110443,24.4964790344 ,1.6244588091 },
      {-18.5467624664,27.1503734589 ,-1.3863387562},
      {-15.0494022369,8.4056587219  ,-1.2862833913},
      {-10.9169044495,-5.7251086235 ,-1.461590551 },
      { -9.4787883759,-18.8415813446,-2.8609535669},
      {-19.923494339 ,-21.852230072 ,-2.918451144 },
      {-34.916885376 ,-25.254535675 ,-2.9590894996},
      {-54.498298645 ,-28.8684177399,2.156869203  },
      {-60.7969665527,-19.3807983398,1.7845000044 },
>>>>>>> ba4035192dc632a3f9c513bf35800348d6e5f60a
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
<<<<<<< HEAD
   // map(地図)座標系(第６週のプログラムとの大きな変更点）                             
=======
   // map(地図)座標系                             
>>>>>>> ba4035192dc632a3f9c513bf35800348d6e5f60a
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
