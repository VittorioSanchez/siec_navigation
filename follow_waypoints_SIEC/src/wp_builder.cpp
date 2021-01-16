#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


using namespace std;


/*double x = 0.0;
double y = 0.0;
double z = 0.0;

double ox = 0.0;
double oy = 0.0;
double oz = 0.0;
double ow = 0.0;*/

int state = 0;

geometry_msgs::PoseStamped wp_rcvd;
geometry_msgs::PoseArray wp_list;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic to publish
    wp_pub = n.advertise<geometry_msgs::PoseArray>("/list_wp", 50);
    //Topic to subscribe
    wp_hmi_sub = n.subscribe("/hmi_wp", 1000, &SubscribeAndPublish::callback_hmi_wp, this);
    wp_status_hmi_sub = n.subscribe("/hmi_wp_status", 1000, &SubscribeAndPublish::callback_hmi_status, this);
    cout << "init";
    ros::spin();
  }
  
  void callback_hmi_status(const std_msgs::Int16::ConstPtr& data)
  {
	state = data->data;
	if (state == 0){
		ROS_INFO("Ready to build wp path");
	}
	else if (state == 2){
		ROS_INFO("Deleting last wp received");
		wp_list.poses.pop_back();
	}
	else if (state == 1){
		ROS_INFO("List finished, sending route to follow_wp...");
		wp_pub.publish(wp_list);
	} 
	else if (state == 3){
		ROS_INFO("Deleting wp list");
		wp_list.poses.clear();
	}		
  }
  void callback_hmi_wp(const geometry_msgs::PoseStamped::ConstPtr& data)
  {
  	wp_list.header.frame_id = "map";
  	if (state == 0){
  		wp_rcvd.pose = data->pose;
  		wp_list.poses.push_back(wp_rcvd.pose);
  	}
  }

private:
  ros::NodeHandle n; 
  ros::Publisher wp_pub;
  ros::Subscriber wp_hmi_sub;
  ros::Subscriber wp_status_hmi_sub;
};//End of class SubscribeAndPublish



int main(int argc, char** argv){

  ros::init(argc, argv, "wp_builder");

  
  SubscribeAndPublish SAPObject;

}

