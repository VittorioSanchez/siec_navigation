#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;


double x = 0.0;
double y = 0.0;
double z = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
    //Topic you want to publish
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/laserscan", 50);

    //Topic you want to subscribe
    odom_sub = n.subscribe("/gps_odom", 1000, &SubscribeAndPublish::callback_odom, this);
    laser_sub = n.subscribe("/simulator/laserscan", 1000, &SubscribeAndPublish::callback_laser, this);//n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
    cout << "init";
    ros::spin();
  }
  
  void callback_laser(const sensor_msgs::LaserScan::ConstPtr& data)
  {
  /*float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities*/

	sensor_msgs::LaserScan laser;
	laser.header.frame_id = "velodyne";
	laser.header.stamp = ros::Time::now();
	laser.angle_min = data->angle_min;
	laser.angle_max = data->angle_max;
	laser.angle_increment = data->angle_increment;
	laser.time_increment = data->time_increment;
	laser.scan_time = data->scan_time;
	laser.range_min = data->range_min;
	laser.range_max = data->range_max;
	laser.ranges = data->ranges;
	laser.intensities = data->intensities;
	//cout<<"coucou";
	laser_pub.publish(laser);
		
  }
  void callback_odom(const nav_msgs::Odometry::ConstPtr& data)
  {
  ros::Rate r(100);
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  geometry_msgs::TransformStamped odom_trans;
  
  odom.header.stamp = ros::Time::now();
  x = data->pose.pose.position.x;
  y = data->pose.pose.position.y;
  z = 0.0;
  
  vx = data->twist.twist.linear.x;
  vy = data->twist.twist.linear.y;
  vth = data->twist.twist.angular.z;


    //first, we'll publish the transform over tf

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = data->pose.pose.orientation;

    //send the transform
   broadcaster.sendTransform(odom_trans);


    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = data->pose.pose.orientation;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
  
    odom_pub.publish(odom);
    

    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.2)),
        ros::Time::now(),"base_link", "velodyne"));
    r.sleep();
}

private:
  ros::NodeHandle n; 
  ros::Publisher odom_pub;
  ros::Subscriber odom_sub;
  ros::Publisher laser_pub;
  ros::Subscriber laser_sub;
  tf::TransformBroadcaster broadcaster;

};//End of class SubscribeAndPublish



int main(int argc, char** argv){

  ros::init(argc, argv, "robot_tf_publisher");

  
  SubscribeAndPublish SAPObject;

}

