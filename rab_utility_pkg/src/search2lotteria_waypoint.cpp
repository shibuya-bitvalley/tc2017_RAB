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
      {-115.291610718,294.69128418  ,-1.5320037263},
      {-114.446990967,272.929504395 ,-2.4651297996},
      {-128.79800415 ,261.408111572 ,-3.0575810925},
      {-158.116485596,258.939208984 ,-3.0423359269},
      {-196.158203125,255.150863647 ,-3.0547011293},
      {-234.610809326,251.801223755 ,2.4713526126 },
      {-257.977050781,270.322357178 ,1.0939543174 },
      {-247.77722168 ,290.066375732 ,1.5771411974 },
      {-247.90788269 ,310.65927124  ,1.6595110183 },
      {-249.59161377 ,329.588623047 ,0.9296556232 },
      {-240.717285156,341.479431152 ,1.6503970343 },
      {-242.006729126,357.644104004 ,-3.0648948705},
      {-258.082672119,356.408691406 ,3.0939273016 },
      {-282.515808105,357.574188232 ,-1.5422563056},
      {-281.399383545,318.466949463 ,-1.5350752658},
      {-280.371429443,289.701934814 ,-1.5445825061},
      {-279.961975098,274.085723877 ,-0.7747925226},
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
