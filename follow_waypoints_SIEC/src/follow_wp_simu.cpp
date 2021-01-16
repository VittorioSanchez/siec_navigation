#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  bool nav_loop = true;
  int goal_id = 0;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  while(nav_loop){
  
  	if (goal_id==0){
  		goal.target_pose.pose.position.x = 11.912637710571289;
  		goal.target_pose.pose.position.y = -32.74427795410156;
  		goal.target_pose.pose.orientation.z = -0.004608063482785251;
  		goal.target_pose.pose.orientation.w = 0.9999893828191071;
  	}
  	else if (goal_id==1){
  		goal.target_pose.pose.position.x = 45.542396545410156;
  		goal.target_pose.pose.position.y = -32.70924377441406;
  		goal.target_pose.pose.orientation.z = 0.7071067966408575;
  		goal.target_pose.pose.orientation.w = 0.7071067657322372;
  	}
  	else if (goal_id==2){
  		goal.target_pose.pose.position.x = 43.51060104370117;
  		goal.target_pose.pose.position.y = -16.24467658996582;
  		goal.target_pose.pose.orientation.z = -0.9991785335828385;
  		goal.target_pose.pose.orientation.w = 0.040524782877253356;
  	}
  	else if (goal_id==3){
  		goal.target_pose.pose.position.x = 7.21849250793457;
  		goal.target_pose.pose.position.y = -7.977362632751465;
  		goal.target_pose.pose.orientation.z = -0.7318630912103079;
  		goal.target_pose.pose.orientation.w = 0.6814516972787524;
  	}


  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);

  	ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Hooray, the base moved 1 meter forward");
    		if (goal_id==3) {
    			goal_id=0;
    			ROS_INFO("go to  0");
    		}
    		else{
    			goal_id = goal_id+1;
    			ROS_INFO("go to %d", goal_id);
    		}
    	}
  	else {
    		ROS_INFO("The base failed to move forward 1 meter for some reason");
    		nav_loop = false;
    	}
	
  }
  return 0;
}
