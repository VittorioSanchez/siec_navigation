#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <actionlib_msgs/GoalID.h>

#include <thread> 
#include <mutex>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;


mutex MUT_state;
int state = 0;

mutex MUT_wp_list;
geometry_msgs::PoseArray wp_list;
actionlib_msgs::GoalID cancel_wp;


 /* thread sub_pub (pub_sub);
  thread navigation (nav);*/
  
  
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic to publish
    wp_cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 50);
    //Topic to subscribe
    wp_status_hmi_sub = n.subscribe("/hmi_wp_status", 1000, &SubscribeAndPublish::callback_hmi_status, this);
    wp_sub = n.subscribe("/list_wp", 50, &SubscribeAndPublish::callback_wp_list, this);
    cout << "init";
    ros::spin();
  }
  
  
  
  void callback_hmi_status(const std_msgs::Int16::ConstPtr& data)
  {
  	MUT_state.lock();
	state = data->data;
	if (state == 1){
		ROS_INFO("Waiting to receive a route...");
	} 
	else {
		ROS_INFO("Stop waypoint following");
		wp_list.poses.clear();	
		wp_cancel_pub.publish(cancel_wp);
	}
	MUT_state.unlock();		
  }
  
  void callback_wp_list(const geometry_msgs::PoseArray::ConstPtr& data)
  {
  	MUT_wp_list.lock();
	wp_list.header = data->header;
	wp_list.poses = data->poses;
	cout<<wp_list;
	MUT_wp_list.unlock();	
  }




private:
  ros::NodeHandle n; 
  ros::Publisher wp_cancel_pub;
  ros::Subscriber wp_status_hmi_sub;
  ros::Subscriber wp_sub;
};//End of class SubscribeAndPublish

void pub_sub(){
	SubscribeAndPublish SAPObject;
	cout<<"out sapobject";
}

void nav(){
  ROS_INFO("In thread nav");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  while (true){
  MUT_wp_list.lock();
  for (unsigned i=0; ((i<wp_list.poses.size()) and (state==1)) ; i++){
  	ROS_INFO("sending goal : %d",i);
    	cout << "sending goal :" << wp_list.poses[i];
  	cout << '\n';
	goal.target_pose.pose.position.x = wp_list.poses[i].position.x;
  	goal.target_pose.pose.position.y = wp_list.poses[i].position.y;
  	goal.target_pose.pose.orientation.z = wp_list.poses[i].orientation.z;
  	goal.target_pose.pose.orientation.w = wp_list.poses[i].orientation.w;

  	ROS_INFO("Sending goal");
  	ac.sendGoal(goal);

  	ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    		ROS_INFO("Hooray, the car moved to the %d waypoint",i);
    	}
  	else {
    		ROS_INFO("The car failed to move for some reason");
    	}	
  }
  MUT_wp_list.unlock();
  
  }
  }



int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  //SubscribeAndPublish SAPObject;
  thread navigation (nav);
  thread sub_pub (pub_sub);

  while (true){
  	NULL;
  }	
  
  return 0;
}
