#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  /*printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);*/
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("sub", 1);

  PointCloud2::Ptr msg (new PointCloud2);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

}

int main(int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
/**
	NodeHandle is main access point to communicate with ross system
	First constructed fully initialize the node, last NodeHandle destructed closes down the node.
*/	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	sub = nh.subscribe ("sub.cpp", 10, callback);	//calls cloud_callback() when recieving new message
	
	ros::spin();	//Loop
}