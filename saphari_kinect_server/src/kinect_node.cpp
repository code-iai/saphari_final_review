#include "saphari_kinect_server/driver.h"
#include <ros/node_handle.h>


// main function
int main (int argc, char **argv)
{
	
  // set up ROS
  ros::init (argc, argv, "kinect_server_node");
  ros::NodeHandle nh ("~");
  

  saphari_kinect_server::OpenNINode f(nh);
  ros::spin();

  return 0;

}
