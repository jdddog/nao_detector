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
#include <pcl/segmentation/sac_segmentation.h>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

const float default_feature_radius = 0.02f;
const float default_normal_radius = 0.02f;
const float minimum_sample_distance = 0.04f;
const float maximum_correspondence_distance = 0.01f*0.01f;
int number_of_iasac_iterations = 600;
float filterOffset = 0.3f;
const float depth_limit = 0.01f;
const float voxel_grid_size = 0.005f;
//should be lower than this to be a match.
const float template_threshold = 0.0002f;
// matched coordinates and filter limits.
float x, y, z;
float depthFilter=8;
int filecount=0;
bool kidnapped=true;



class FeatureCloud
{
	public:
	
	// Initialize the feature cloud using the kd-tree & default normal and feature radius values.
    FeatureCloud () : _search_technique (new SearchMethod),
		defaultRadiusForNormals (default_feature_radius),
		defaultRadiusForFeatures (default_normal_radius) {}

    ~FeatureCloud () {}

    // Accept the point cloud data and initialize the features and normal extraction.
    void setInputCloud (PointCloud::Ptr cloudToBeSet)
    {
		_cloud = cloudToBeSet;
		processInput ();
    }

    // In case the point cloud data is present in a file.
    void loadInputCloud (const std::string &fileName)
    {
		_cloud = PointCloud::Ptr (new PointCloud);
		pcl::io::loadPCDFile (fileName, *_cloud);
		processInput ();
    }

	public:
    // Compute the surface normals and local features
    void processInput ()
    {
		computeSurfaceNormals ();
		computeLocalFeatures ();
    }

    // Computation of the surface normals opitimized for OpenMP.
    void computeSurfaceNormals ()
    {
		_cloud_normals = SurfaceNormals::Ptr (new SurfaceNormals);
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;		
		
		norm_est.setInputCloud (_cloud);
		norm_est.setSearchMethod (_search_technique);
		norm_est.setRadiusSearch (defaultRadiusForNormals);		
		
		norm_est.compute(*_cloud_normals);
    }

    // Computation of the local features optimitzed for OpenMP.
    void computeLocalFeatures ()
    {
		_cloud_features = LocalFeatures::Ptr (new LocalFeatures);
		pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
		
		fpfh_est.setInputCloud (_cloud);
		fpfh_est.setInputNormals (_cloud_normals);
		fpfh_est.setSearchMethod (_search_technique);
		fpfh_est.setRadiusSearch (defaultRadiusForFeatures);
		
		fpfh_est.compute (*_cloud_features);
    }

	// Getters for the PCD data structures.
    PointCloud::Ptr getPointCloud () const { return (_cloud); }

    SurfaceNormals::Ptr getSurfaceNormals () const { return (_cloud_normals); }
   
    LocalFeatures::Ptr getLocalFeatures () const { return (_cloud_features); }

	private:
		//structures for holding normals and features.
		PointCloud::Ptr _cloud;
		SurfaceNormals::Ptr _cloud_normals;
		LocalFeatures::Ptr _cloud_features;
		SearchMethod::Ptr _search_technique;

		//default radii to be taken for normals and features.
		float defaultRadiusForNormals;
		float defaultRadiusForFeatures;
};

class TemplateAlignment
{
	public:
    struct Result
    {
		float fitness_score;
		Eigen::Matrix4f final_transformation;
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    // Set the given cloud as the target to which the templates will be aligned
    void setTargetCloud (FeatureCloud &target_cloud) {	target_ = target_cloud;  }
    
	void setTemplates (std::vector<FeatureCloud> templates) { templates_ = templates; }
	
    // Add the given cloud to the list of template clouds
    void addTemplateCloud (FeatureCloud &template_cloud) { templates_.push_back (template_cloud); }
    
    std::vector<FeatureCloud> getTemplates() { return templates_; }
    
    int getTemplateCount () { return templates_.size(); }
        
    // Align the given template cloud to the target specified by setTargetCloud ()
    void align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {	
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
	
		sac_ia_.setMinSampleDistance (minimum_sample_distance);
		sac_ia_.setMaxCorrespondenceDistance (maximum_correspondence_distance);
		sac_ia_.setMaximumIterations (number_of_iasac_iterations);
		
		sac_ia_.setInputCloud (template_cloud.getPointCloud ());
		sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());	
		
		sac_ia_.setInputTarget (target_.getPointCloud ());
		sac_ia_.setTargetFeatures (target_.getLocalFeatures ());
		
		pcl::PointCloud<pcl::PointXYZ> registration_output;
		sac_ia_.align (registration_output);
		
		result.fitness_score = (float) sac_ia_.getFitnessScore (maximum_correspondence_distance);
		result.final_transformation = sac_ia_.getFinalTransformation();
		
		int size = registration_output.size();
		x = registration_output.points[size/2].x;
		y = registration_output.points[size/2].y;
		z = registration_output.points[size/2].z;
    }
	
    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    // Modify this based on the joint angles of the robot.
    void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
		//clear out prior results
		results.resize (templates_.size());		
		
		for (size_t i = 0; i < templates_.size (); ++i)
		{			
			align (templates_[i], results[i]);
			if(results[i].fitness_score < template_threshold)
				break;
		}
    }
	
    // Align all of template clouds to the target cloud to find the one with best alignment score
    int findBestAlignment (TemplateAlignment::Result &result)
    {		
		// Align all of the templates to the target cloud
		std::vector<Result, Eigen::aligned_allocator<Result> > results;
		alignAll (results);
	
		// Find the template with the best (lowest) fitness score
		float lowest_score = std::numeric_limits<float>::infinity ();
		int best_template = 0;
		for (size_t i = 0; i < results.size (); ++i)
		{			
			const Result &r = results[i];
			
			if (r.fitness_score < lowest_score)
			{
				lowest_score = r.fitness_score;
				best_template = (int) i;
				if(lowest_score > template_threshold)
					kidnapped = true;
			}
		}
		// Output the best alignment
		result = results[best_template];				
		return (best_template);
    }
	
	private:
		// A list of template clouds and the target to which they will be aligned
		std::vector<FeatureCloud> templates_;
		FeatureCloud target_;
};

TemplateAlignment template_align;

void callback(const PointCloud::ConstPtr& msg)
{	
	// create a modifiable reference to the incoming data.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>); 
	
	// create pointers for the filters
	pcl::PassThrough<pcl::PointXYZ> pass;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	
	TemplateAlignment::Result best_alignment;
	FeatureCloud target_cloud;
	double result;
	
	*cloud = *msg;
	
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.5, 3.5);
	pass.filter (*cloud);
			
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-0.5, 0.0);
	pass.filter (*cloud);
	
	// downsample via voxel grid
	vox_grid.setInputCloud (cloud);
	vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	vox_grid.filter (*tempCloud);
	cloud = tempCloud;
	pcl::io::savePCDFileBinary ("/home/rishi/filtered.pcd", *cloud);
	cout<<"\nvfiltering completed";
	target_cloud.setInputCloud (cloud);
	
	template_align.setTargetCloud (target_cloud);
	int best_index = template_align.findBestAlignment (best_alignment);
	//FeatureCloud &template_cloud = template_align.getTemplates()[best_index];
	
	cout << "\nFound : X Y Z -> " <<x<<" : " <<y<<" : "<<z<<"";
	
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	// tf axes are skewed w.r.t. live capture (X->Z, Y->X, Z->Y)
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
	
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "nao_robot"));	
}

int main(int argc, char** argv)
{
	// initialize the node for the algorithm
	ros::init(argc, argv, "nao_detector_algo");
	ros::NodeHandle nh;
	
	std::vector<FeatureCloud> object_templates;
	FeatureCloud template_cloud;
	
	// location of the config file containing paths of all the templates.	
	std::ifstream input_stream ("../data/object_templates.txt");
	object_templates.resize (0);
	
	std::string filePath;
	std::getline(input_stream, filePath);
	
	while (input_stream.good ())
	{
		std::getline (input_stream, filePath);
		
		if (filePath.empty () || filePath.at (0) == '#')	continue;
		
		// template input cloud
		template_cloud.loadInputCloud (filePath);
		object_templates.push_back (template_cloud);
	}
	
	input_stream.close ();
	for (size_t i = 0; i < object_templates.size (); ++i)
	{
		template_align.addTemplateCloud (object_templates[i]);
	}
	///camera/depth_registered/points
	ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth_registered/points", 1, callback);	
	ros::spin();
}

