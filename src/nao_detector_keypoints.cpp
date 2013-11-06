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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <stdio.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <ros/console.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <tf/transform_broadcaster.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sstream>
#include <ros/package.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::FPFHSignature33> Features;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;


//PCDMatching matcher;
const float min_scale = 0.005f; //0.0005 
const int nr_octaves = 6; //4 
const int nr_scales_per_octave = 4; //5 
const float min_contrast = 0.005f; //1 
const int k_sift = 20;
const double r_sift = 0.5; 


void plane_segmentation(pcl::PointCloud<pcl::PointXYZRGBA> &source, pcl::PointCloud<pcl::PointXYZRGBA> &segmented_output)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented (new pcl::PointCloud<pcl::PointXYZRGBA>);
	*input_cloud = source;
	
	
	std::cout << "segmentation..." << std::flush;
	// fit plane and keep points above that plane
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.06); //distance from ground

	//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	seg.setInputCloud (input_cloud);
	seg.segment (*inliers, *coefficients);

	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	
	extract.setInputCloud(input_cloud);
	extract.setIndices (inliers);
	extract.setNegative (true);

	extract.filter (*segmented);
	std::cout << "OK" << std::endl;

	std::cout << "clustering..." << std::flush;
	// euclidean clustering
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud (segmented);
	std::cout << "1\n";

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> clustering;
	clustering.setClusterTolerance (0.02); // 2cm
	clustering.setMinClusterSize (1000);
	clustering.setMaxClusterSize (250000);
	clustering.setSearchMethod (tree);
	clustering.setInputCloud(segmented);
	std::cout << "2\n";
	clustering.extract (cluster_indices);
	std::cout << "3\n";

	if (cluster_indices.size() > 0)//use largest cluster
	{
		std::cout << cluster_indices.size() << " clusters found";
		if (cluster_indices.size() > 1)
		  std::cout <<" Using largest one...";
		std::cout << std::endl;
		pcl::IndicesPtr indices (new std::vector<int>);
		*indices = cluster_indices[0].indices;
		extract.setInputCloud (segmented);
		extract.setIndices (indices);
		extract.setNegative (false);

		extract.filter (*segmented);
	}	
	
	segmented_output = *segmented;
	
}

void ApplyPassThroughFilter(pcl::PointCloud<pcl::PointXYZRGBA> &cloudData, std::string name, float minimum, float maximum)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	*inputCloud = cloudData;
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (inputCloud);
	pass.setFilterFieldName (name);
	pass.setFilterLimits (minimum, maximum);
	pass.filter (cloudData);
}

void keypoints(pcl::PointCloud<pcl::PointXYZRGBA> &source, pcl::PointCloud<pcl::PointWithScale> &result)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointWithScale> features;
	*input_cloud = source;

	pcl::SIFTKeypoint<pcl::PointXYZRGBA, pcl::PointWithScale> sift_detect;
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>); 
	sift_detect.setSearchMethod(tree);

	//Detection parameters
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	
	//Set input
	sift_detect.setInputCloud(input_cloud);
	
	//Detect keypoints
	sift_detect.compute(features);	
	std::cout << "No of SIFT points in the result are " << features.points.size () << std::endl;
	
	result = features;
}

ros::Publisher pub;

void callback(const PointCloudRGBA::ConstPtr &msg)
{	
	std::cout << "starting\n";
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZRGBA>);
	*input_cloud = *msg;
	
	ApplyPassThroughFilter(*input_cloud, "z", 0.5, 4.0);
	ApplyPassThroughFilter(*input_cloud, "y", 0.0, 1.0);
	
	plane_segmentation(*input_cloud, *segmented);
	std::cout << "finished\n";
	
	pcl::PointCloud<pcl::PointWithScale> features;
	//keypoints(*segmented, features);
	/// and publish the message
	
	pub.publish(segmented);
	

}

int main(int argc, char** argv)
{
	// initialize the node for the algorithm
	ros::init(argc, argv, "nao_detector_kp");
	ros::NodeHandle nh;
	pub = nh.advertise<sensor_msgs::PointCloud2>("/nao_detector/plane_segmented/", 1);
	
	ros::Subscriber sub = nh.subscribe<PointCloudRGBA>("/camera/depth_registered/points", 1, callback);
	ros::spin();
}

