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
  		goal.target_pose.pose.position.x = 2.0;
  		goal.target_pose.pose.position.y = 2.0;
  	}
  	else{
  		goal.target_pose.pose.position.x = 4.0;
  		goal.target_pose.pose.position.y = 4.0;
  	}
  	goal.target_pose.pose.orientation.w = 1.0;

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);

  	ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Hooray, the base moved 1 meter forward");
    		if (goal_id==0) {
    			goal_id=1;
    			ROS_INFO("go to  2 2");
    		}
    		else{
    			goal_id=0;
    			ROS_INFO("go to  4 4");
    		}
    	}
  	else {
    		ROS_INFO("The base failed to move forward 1 meter for some reason");
    		nav_loop = false;
    	}
	
  }
  return 0;
}
