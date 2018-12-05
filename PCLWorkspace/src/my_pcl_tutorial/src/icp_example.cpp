#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile ("/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/src/my_pcl_tutorial/src/1539950563.428160477.pcd", *cloud_in)==-1)
	{
		PCL_ERROR ("Couldn't read first file! \n");
		return (-1);
	}

  pcl::PointCloud<pcl::PointXYZ>::Ptr cleaned_cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> index;

  pcl::removeNaNFromPointCloud(*cloud_in, *cleaned_cloud_in, index);
  pcl::io::savePCDFileASCII ("cleaned.pcd", *cleaned_cloud_in);

  *cloud_out = *cleaned_cloud_in;

	std::cout << "cloud_in size: " << cloud_in->points.size() << std::endl;
  std::cout << "cleaned_cloud_in size: " <<cleaned_cloud_in->points.size() << std::endl;
  std::cout << "cloud_out size: " << cloud_out ->points.size() << std::endl;

  for(size_t i = 0; i < cloud_out->points.size(); ++i){
    cloud_out->points[i].x = cloud_out->points[i].x + 0.005f;
  }
       
  pcl::io::savePCDFileASCII ("output.pcd", *cloud_out);
  std::cout << "Transformed " << cleaned_cloud_in->points.size() << " data points:" << std::endl;


 	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;       
	icp.setMaximumIterations(10);
	icp.setMaxCorrespondenceDistance(0.01);
	icp.setInputSource(cleaned_cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	Eigen::Matrix4f transformation;
	icp.align(Final, transformation);

	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	
	std::cout << "Transformation: " << transformation << std::endl;

return (0);
}
