#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <tf/tf.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Constants.hpp>

using namespace std;
using namespace GeographicLib;

/*double x = 0.0;
double y = 0.0;
double z = 0.0;

double ox = 0.0;
double oy = 0.0;
double oz = 0.0;
double ow = 0.0;*/
#define PI (3.141592653589793)

int state = 0;
bool first_wp = false;
bool gps_init = false; 

double origin_lat = 0.0;
double origin_long = 0.0;
double origin_heading = -0.60;

geometry_msgs::PoseStamped wp_rcvd;
geometry_msgs::PoseArray wp_list;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
   
  geometry_msgs::PoseStamped origin_pose;
  ROS_INFO("Waiting for a message to initialize the origin GPS location...");
  /*boost::shared_ptr<geometry_msgs::PoseStamped const> sharedPtr;
  sharedPtr = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("local_xy_origin", ros::Duration(10));
    if (sharedPtr == NULL)
        ROS_INFO("No point clound messages received);
    else
        origin_pose = *sharedPtr;
  float origin_lat = origin_pose.pose.position.y;
  float origin_long = origin_pose.pose.position.x;
  ROS_INFO("Received origin: lat %lf, long %lf",origin_lat, origin_long);*/
    //Topic to publish
    wp_pub = n.advertise<geometry_msgs::PoseArray>("/list_wp", 50);
    //Topic to subscribe
    wp_hmi_sub = n.subscribe("/hmi_wp", 1000, &SubscribeAndPublish::callback_hmi_wp, this);
    wp_status_hmi_sub = n.subscribe("/hmi_wp_status", 1000, &SubscribeAndPublish::callback_hmi_status, this);
    gps_init_sub = n.subscribe("/local_xy_origin", 1000, &SubscribeAndPublish::callback_gps_init, this);
    
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
  
  void callback_gps_init(const geometry_msgs::PoseStamped::ConstPtr& data)
  {
  	
  	ROS_INFO("Waiting for a message to initialize the origin GPS location...");
	gps_init = true;
	origin_lat = data->pose.position.y;
  	origin_long = data->pose.position.x;
  	ROS_INFO("Received origin: lat %lf, long %lf",origin_lat, origin_long);	
		
  }
  
  void callback_hmi_wp(const geometry_msgs::PoseStamped::ConstPtr& data)
  {
  	if (gps_init){
  	wp_list.header.frame_id = "map";
  	if (state == 0){
  		wp_rcvd.pose = data->pose;
  		Geodesic geod(Constants::WGS84_a(), Constants::WGS84_f());
  		double s12, azi1, azi2;
  		ROS_INFO("The origin is lat: %f , long: %f", origin_lat, origin_long);
  		ROS_INFO("The goal is lat: %f , long: %f", wp_rcvd.pose.position.x, wp_rcvd.pose.position.y);
  		//ROS_INFO("The distance from the origin to the goal is %f", s12);
        	geod.Inverse(origin_lat, origin_long, wp_rcvd.pose.position.x, wp_rcvd.pose.position.y, s12, azi1, azi2);
        	ROS_INFO("The distance from the origin to the goal is %f", s12);
        	ROS_INFO("The azimuth from the origin to the goal is %f °", azi1);
        	azi1 = azi1*(PI/180);
        	ROS_INFO("The azimuth from the origin to the goal is %f rad", azi1);
        	azi1 = azi1 - origin_heading;
        	ROS_INFO("The azimuth from the origin to the goal corrected is %f rad", azi1);
  		float x = cos(azi1) * s12;
  		float y = sin(azi1) * s12;
  		wp_rcvd.pose.position.x = x;
  		wp_rcvd.pose.position.y = -y;
  		ROS_INFO("The translation from the origin to the goal is (x,y): (%lf,%lf)", x, y);
  		
        	
        	
  		if (wp_list.poses.size() == 0){
  			first_wp = true;
  			wp_list.poses.push_back(wp_rcvd.pose);
  		}
  		else {
  			first_wp = false;
  			float x1 = wp_list.poses.back().position.x;
  			float y1 = wp_list.poses.back().position.y;
  			float x2 = wp_rcvd.pose.position.x;
  			float y2 = wp_rcvd.pose.position.y;
  			float comp_x = x2-x1;
  			float comp_y = y2-y1;
  			
  			float th = atan2(comp_y, comp_x);
  			/*ROS_INFO("th is %lf",th);
  			ROS_INFO("x1 is %lf",x1);
  			ROS_INFO("y1 is %lf",y1);
  			ROS_INFO("x2 is %lf",x2);
  			ROS_INFO("y2 is %lf",y2);
  			ROS_INFO("y2-y1 is %lf",(y2-y1));
  			ROS_INFO("x2-x1 is %lf",(x2-x1));
  			ROS_INFO("cos_ang is %lf",cos_ang);*/
  			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  			wp_rcvd.pose.orientation = odom_quat;
  			if (wp_list.poses.size() == 1){
  				wp_list.poses.front().orientation = odom_quat;
  			}
  			wp_list.poses.push_back(wp_rcvd.pose);
  		}
  		
  		
  	}
  	}
  	else {
  		ROS_INFO("gps not init");
  	}
  }

private:
  ros::NodeHandle n; 
  ros::Publisher wp_pub;
  ros::Subscriber wp_hmi_sub;
  ros::Subscriber wp_status_hmi_sub;
  ros::Subscriber gps_init_sub;
};//End of class SubscribeAndPublish



int main(int argc, char** argv){

  ros::init(argc, argv, "wp_builder");
  

  
  
  SubscribeAndPublish SAPObject;
  

}

