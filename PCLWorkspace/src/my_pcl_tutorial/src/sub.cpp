#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){	
	ROS_INFO("inside callback");
}
/*Calback functions get called when message arrives*/

int main (int argc, char** argv) {

	ros::init (argc, argv, "my_pcl_tutorial");
/**
	NodeHandle is main access point to communicate with ross system
	First constructed fully initialize the node, last NodeHandle destructed closes down the node.
*/	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	sub = nh.subscribe ("/camera/depth_registered/points", 10, cloud_callback);	//calls cloud_callback() when recieving new message
	
	ros::spin();	//Loop

}
