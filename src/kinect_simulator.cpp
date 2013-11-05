#include "ros/ros.h"
#include "std_msgs/String.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <string>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
	ros::init (argc, argv, "kinect_sim");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<PointCloud> ("/camera/depth/points", 1);

	PointCloud::Ptr outgoing_pcd_data = PointCloud::Ptr (new PointCloud);
	
	ros::Rate loop_rate(20);
	
	while (nh.ok())
	{
		outgoing_pcd_data->header.stamp = ros::Time::now ();
		//std::ostringstream ss;
		//ss<<count;
		std::string fileName = "../data/capture7.pcd";
				
		pcl::io::loadPCDFile (fileName , *outgoing_pcd_data);
		
		outgoing_pcd_data->header.frame_id = "/camera_depth_frame";
		
		pub.publish (outgoing_pcd_data);
		ros::spinOnce ();
		//loop_rate.sleep ();
	}	
}

