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
      {-97.4639663696,215.012161255 ,0.7476049707 },
      {-91.4315185547,220.605041504 ,-0.0016817092},
      {-67.4415206909,220.564697266 ,0.3300239291 },
      {-45.2336387634,228.172042847 ,1.6705246975 },
      {-46.010761261 ,235.938583374 ,1.4781483742 },
      {-44.5219192505,251.962463379 ,1.8243500283 },
      {-49.7419319153,272.106781006 ,2.0558772064 },
      {-59.3058052063,290.251586914 ,1.748495025  },
      {-63.7324485779,314.899780273 ,2.9014094276 },
      {-78.5200424194,318.52142334  ,-2.60656047  },
      {-98.4043579102,306.736053467 ,-2.5220318177},
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
