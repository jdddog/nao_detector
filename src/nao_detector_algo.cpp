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
#include <ros/package.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::Normal> Normals;
typedef pcl::PointCloud<pcl::FPFHSignature33> Features;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

float finalX=0, finalY=0, finalZ=0;
bool kidnapped=true;
double threshold=0.00002;

float const normalRadius=0.01f;
float const featureRadius=0.01f;
float const minimumSampleDistance=0.05f;
float const maximumCorrespondenceDistance=0.01f;
float const numberOfIterations=1;
float const leafSize=0.005f;

class PCDRegistration
{
	private:
	
		// rawData & search technique to be used, along with the calculated normals and features.
		PointCloud::Ptr rawData;
		Normals::Ptr normals;
		Features::Ptr features;
		SearchMethod::Ptr search;
		
		// radius to be used for normal and feature estimation for this cloud.
		float radius_ne;
		float radius_fe;
		
	public:
	
		PCDRegistration()	:
		search (new SearchMethod)	{}
	
		~PCDRegistration() {}
	
		void setRawData(PointCloud::Ptr cloudPointer) {	
			rawData = cloudPointer;
			estimateNormalsAndFeatures();	
		}
	
		void setRawDataFromFile(const std::string &input_file) {
			rawData = PointCloud::Ptr (new PointCloud);
			pcl::io::loadPCDFile (input_file, *rawData);
			estimateNormalsAndFeatures();
		}
	
		// Get a pointer to the raw PCD
		PointCloud::Ptr getRawPCD () const { return rawData;  }

		// Get a pointer to the normals
		Normals::Ptr getNormals () const { return normals;   }

		// Get a pointer to the feature descriptors
		Features::Ptr getFeatures () const { return features;   }
	
		void estimateNormalsAndFeatures()
		{
			// OpenMP optimized normal and feature estimation classes
			pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimation;
			pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimationFPFH;
						
			normals = Normals::Ptr (new Normals);
						
			normalEstimation.setInputCloud (rawData);
			normalEstimation.setSearchMethod (search);
			normalEstimation.setKSearch (10);
			//normalEstimation.setRadiusSearch (normalRadius);
			normalEstimation.compute (*normals);
						
			features = Features::Ptr (new Features);
						
			featureEstimationFPFH.setInputCloud (rawData);
			featureEstimationFPFH.setInputNormals (normals);
			featureEstimationFPFH.setSearchMethod (search);
			featureEstimationFPFH.setKSearch (10);
			//featureEstimationFPFH.setRadiusSearch (featureRadius);
			featureEstimationFPFH.compute (*features);
		}
};

// struct for storing results of the matching
struct PCDMatching_Results
{
	float score;
	float coordinates[3];
};

class PCDMatching
{
	private:
	
		//list of registered templates for matching and the target cloud to be match these.
		std::vector<PCDRegistration> templateList;
		std::vector<PCDMatching_Results> templateResults;
		PCDRegistration targetCloudData;
		
		//SAC-IA and parameters - the matching technique being used for the process
		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacia;
		
	public:
		
		PCDMatching()
		{
			sacia.setMinSampleDistance (minimumSampleDistance);
			sacia.setMaxCorrespondenceDistance (maximumCorrespondenceDistance*maximumCorrespondenceDistance);
			sacia.setMaximumIterations (numberOfIterations);
		}
		
		~PCDMatching() {}
		
		void setTargetCloudData (PCDRegistration &targetCloudDataReference)
		{
			targetCloudData = targetCloudDataReference;
			sacia.setInputTarget (targetCloudData.getRawPCD());
			sacia.setTargetFeatures (targetCloudData.getFeatures());
		}
		
		void setTemplateList(std::vector<PCDRegistration> &incomingTemplateList) 	{	templateList = incomingTemplateList;	}
		
		void alignTemplate(PCDRegistration &templateCloudDataReference, int templateIndex)
		{
			sacia.setInputCloud(templateCloudDataReference.getRawPCD());
			sacia.setSourceFeatures(templateCloudDataReference.getFeatures());
			
			pcl::PointCloud<pcl::PointXYZ> output;
			sacia.align (output);
			
			std::ostringstream ss;
			ss<<templateIndex;
			std::string number = "template"+ss.str()+".pcd";
			
			pcl::io::savePCDFileBinary (number, output);
			
			int size = output.size();
			templateResults[templateIndex].coordinates[0] = output.points[size/2].x;
			templateResults[templateIndex].coordinates[1] = output.points[size/2].y;
			templateResults[templateIndex].coordinates[2] = output.points[size/2].z;
			templateResults[templateIndex].score = (float)sacia.getFitnessScore(maximumCorrespondenceDistance);
		}
		
		void calculate()
		{
			templateResults.resize (templateList.size ());
			for (int current=0;current<templateList.size();current++)
			{
				alignTemplate(templateList[current], current);
			}
		}
		
		std::vector<PCDMatching_Results> getTemplateResults()	{	return templateResults;   }
};

void ApplyColorFiltering(pcl::PointCloud<pcl::PointXYZRGBA> &cloudData, pcl::PointCloud<pcl::PointXYZ> &colorFilteredData)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
	
	*inputCloud = cloudData;
	
	filteredCloud->width    = inputCloud->width;
	filteredCloud->height   = inputCloud->height;
	filteredCloud->is_dense = false;
	filteredCloud->points.resize (inputCloud->width * inputCloud->height);
	
	int width = filteredCloud->width;
	for(int i=0;i<inputCloud->points.size();i++)
	{		
		if(inputCloud->points[i].r > 240 && inputCloud->points[i].g > 240 && inputCloud->points[i].b > 240)
		{				
			filteredCloud->points[i].x = inputCloud->points[i].x;
			filteredCloud->points[i].y = inputCloud->points[i].y;
			filteredCloud->points[i].z = inputCloud->points[i].z;
			
			int count=0;
			for(int j=i+width;j<inputCloud->points.size();j+=width)
			{
				if(inputCloud->points[i].r > 200 && inputCloud->points[i].g > 200 && inputCloud->points[i].b > 200)
				{
					filteredCloud->points[j].x = inputCloud->points[j].x;
					filteredCloud->points[j].y = inputCloud->points[j].y;
					filteredCloud->points[j].z = inputCloud->points[j].z;
				}
				
				count++;
				if(count>40)
					break;
			}
		}
	}	
	colorFilteredData = *filteredCloud;	
	//pcl::io::savePCDFileBinary ("/home/rishi/color_filtered.pcd", *filteredCloud);
}

void ApplyPassThroughFilter(pcl::PointCloud<pcl::PointXYZ> &cloudData)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
	*inputCloud = cloudData;
	pcl::PassThrough<pcl::PointXYZ> pass;	
	pass.setInputCloud (inputCloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.5, 2.5);
	pass.filter (cloudData);
	
	//pcl::io::savePCDFileBinary ("/home/rishi/passthrough_filtered.pcd", cloudData);
}

void ApplyVoxelGridFilter(pcl::PointCloud<pcl::PointXYZ> &cloudData)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
	*inputCloud = cloudData;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;	
	vox_grid.setInputCloud (inputCloud);
	vox_grid.setLeafSize (leafSize, leafSize, leafSize);
	vox_grid.filter (cloudData);
	
	//pcl::io::savePCDFileBinary ("/home/rishi/voxel_filtered.pcd", cloudData);
}

PCDMatching matcher;

void callback(const PointCloudRGBA::ConstPtr &msg)
{	
	// create a modifiable reference to the incoming data.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr initialCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>);
		
	// assign the incoming message
	*initialCloud = *msg;
	
	ApplyColorFiltering(*initialCloud, *inputCloud);	
	
	ApplyPassThroughFilter(*inputCloud);
	
	ApplyVoxelGridFilter(*inputCloud);
	
	std::cout<<"Filters applied\n";	
	
	PCDRegistration cloudCapture;
	cloudCapture.setRawData(inputCloud);
	matcher.setTargetCloudData(cloudCapture);
	matcher.calculate();
		
	std::vector<PCDMatching_Results> results = matcher.getTemplateResults();
	finalX =0, finalY=0, finalZ =0;
	for(int i=0;i<results.size();i++)
	{
		finalX +=results[i].coordinates[0];
		finalY +=results[i].coordinates[1];
		finalZ +=results[i].coordinates[2];
	}
	finalX /= results.size();
	finalY /= results.size();
	finalZ /= results.size();
	std::cout<<"Location <X,Y,Z> : "<<finalX<<","<<finalY<<","<<finalZ<<"\n";
	
	// broadcast the values
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(finalX, finalY, finalZ));
	transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_depth_optical_frame", "nao_robot"));	
}

int main(int argc, char** argv)
{
	std::vector<PCDRegistration> templates;
	
	// initialize the node for the algorithm
	ros::init(argc, argv, "nao_detector_algo");
	ros::NodeHandle nh;

    std::string data_path = ros::package::getPath("nao_detector") + "/data/";
	std::cout << "data_path: " << data_path << "\n";
	
    std::string object_templates_file = data_path + "object_templates.txt";
	std::cout << "object_templates_file: " << object_templates_file << "\n";
	
	std::ifstream templateListFile ((char*)object_templates_file.c_str());
	templates.resize (0);
	std::string fileName;
		
	
	while (templateListFile.good ())
	{
		std::getline (templateListFile, fileName);		
		
		if (fileName.empty ())	continue;
		
		std::string pcd_file_path = data_path + fileName;
		std::cout << "pcd_file_path: " << pcd_file_path << "\n";
		
		PCDRegistration templateData;
		templateData.setRawDataFromFile(pcd_file_path);
		templates.push_back(templateData);
	}
	templateListFile.close();
	
	std::cout<<"Templates Loaded, total count is "<<templates.size()<<"\n";
	
	matcher.setTemplateList(templates);	
	
	ros::Subscriber sub = nh.subscribe<PointCloudRGBA>("/camera/depth_registered/points", 1, callback);	
	ros::spin();
}

