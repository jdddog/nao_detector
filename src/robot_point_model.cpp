#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <stdio.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <ros/console.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <tf/transform_broadcaster.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sstream>
#include <urdf/model.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

using namespace std;



/*
PointCloudXYZ get_cloud()
{

}

vector<PointCloudXYZ> import_model(string urdf_file, string stl_dir)
{
	//Import urdf file
	urdf::Model model;

	if (!model.initFile(urdf_file))
	{
		ROS_ERROR("Failed to parse urdf file");
	}

	ROS_INFO("Successfully parsed urdf file");

	vector<Link> links;

	//std::vector<boost::shared_ptr<Link>> links;
	//model.get_links(&links);

	for(int i = 0; i < links.size(); i++)
	{
		Link link;

		pcl::PolygonMesh mesh;
		pcl::PointCloud<PointCloudXYZ> cloud;
		pcl::io::loadPolygonFileSTL(stl_file_names[i], mesh);
		pcl_conversions::fromPCL(mesh.cloud, cloud);
		clouds.push_back(cloud);
	}

	return clouds;
}

Link import_link(Link link)
{
	if link.links

}*/


//PointCloudXYZ get_robot_model()
//{


//}

struct Part
{
	string tf_name;
	PointCloudXYZ cloud;
};

PointCloudXYZ get_cloud(std::string path, std::string mesh_name)
{
	pcl::PolygonMesh mesh;
	PointCloudXYZ cloud;
	pcl::io::loadPolygonFileSTL(path + mesh_name, mesh);
	pcl::fromPCLPointCloud2(mesh.cloud, cloud);
	return cloud;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robot_point_model");

	urdf::Model model;
	//Part root;
	vector<Part> body_parts;

	if (!model.initFile("/home/jamie/catkin_ws/src/nao_detector/urdf/nao_h25_v32.urdf"))
	{
		ROS_ERROR("Failed to parse urdf file");
	}

	ROS_INFO("Successfully parsed urdf file");

	typedef std::map<std::string, boost::shared_ptr<urdf::Link> >::iterator it_type;

	//urdf::Link root_link = model.getRoot();


	for(it_type iterator = model.links_.begin(); iterator != model.links_.end(); iterator++)
	{










		std::cout << "Link Name: " << part.link_name << " \n";


		boost::shared_ptr<urdf::Geometry> geo = iterator->second->visual->geometry;

		if (geo->type == urdf::Geometry::MESH)
		{
			boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(geo);
			std::string path = mesh->filename;
			std::cout << "Path before: " << path << " \n";

			int last = path.find_last_of("/");
			path.replace(0, last + 1, "");
			std::cout << "Last: " << last << " \n";



			int pos = path.find(".mesh");
			if ( pos != string::npos ) {
				path.replace(pos, 4, ".stl" );   // 5 = length( $name )
			}


			Part part;
			part.tf_name = iterator->first;
			std::string filename = path.substr(0, path.size()-1);
			part.cloud = get_cloud("/home/jamie/", filename);
			std::cout << "Path after: " << path << " \n";

			body_parts.push_back(part);

		}

	}






	/*ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("/nao/joint/angles", 1000);
	ros::Rate loop_rate(10);

	vector<PointCloudXYZ> robot_clouds;




	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	*/

	return 0;
}
