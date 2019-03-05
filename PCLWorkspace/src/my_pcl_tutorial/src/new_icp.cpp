#include <iostream>
#include <string>
#include <sstream>
#include <pcl/console/time.h> // TicToc
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;
using std::vector;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
//typedef vector < PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud <PointXYZ>::Ptr > > PointT;
double voxel_grid_size(0.01);
int icp_iterations(1);
double max_corr_distance(1); // 1 mm
int iterations_count(0);
double cloud_shift(0);
bool frameChanger = false;

/*******************************************************************
 *  PRINTS THE RIGID-TRANSFORMATION IN A HUMAN READABLE WAY
 *******************************************************************/


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
  if (event.getKeySym() == "n" && event.keyDown()){
    frameChanger = true;
  }
}

int main(int argc, char *argv[])
{

  if (argc > 0)
  {
    pcl::console::parse_argument(argc, argv, "--iterations", icp_iterations);
    pcl::console::parse_argument(argc, argv, "-i", icp_iterations);
    if (icp_iterations < 1)
    {
      PCL_WARN("Number of maximum iterations must be >= 1\n");
      icp_iterations = 1;
    }

    pcl::console::parse_argument(argc, argv, "--voxel_grid_size", voxel_grid_size);
    pcl::console::parse_argument(argc, argv, "-vgs", voxel_grid_size);

    pcl::console::parse_argument(argc, argv, "--max_corr_distance", max_corr_distance);
    pcl::console::parse_argument(argc, argv, "-mcd", max_corr_distance);
    max_corr_distance = fabs(max_corr_distance);

    pcl::console::parse_argument(argc, argv, "--cloud_shift", cloud_shift);
    pcl::console::parse_argument(argc, argv, "-cs", cloud_shift);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer("ICP COMPARISION");
  // Create two vertically separated viewports
  int v1(0);
  int v2(1);
  viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

  // The color we will be using
  float bckgr_gray_level = 0.0; // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  /**
   *  MAIN CONTENT 
   */

  for (int i = 1; i < 8; i++)
  {
    /**
     * CREATE FILE1 & FILE2
     * DECLARE POINT CLOUDS
     */
    stringstream sstream;
    sstream << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_" << i << ".pcd";
    string file1 = sstream.str();
    sstream.str(string());
    sstream.clear();

    //stringstream ss2;
    stringstream ss2;
    ss2 << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_" << i + 1 << ".pcd";
    string file2 = ss2.str();
    ss2.str(string());
    ss2.clear();

    // The point clouds we will be using
    PointCloudT::Ptr originalCloud(new PointCloudT);  // Original point cloud
    PointCloudT::Ptr targCloud(new PointCloudT);  // Target point cloud
    PointCloudT::Ptr transCloud(new PointCloudT); // ICP output point cloud

    pcl::console::TicToc time;
    time.tic();


    //LOAD FILES AND REMOVE NaN's

    cout << "Loading input data" << endl;
    if (pcl::io::loadPCDFile(file1, *originalCloud) == -1)
    {
      PCL_ERROR("Failed to load first file! \n"); // In case of file not being read
      return (-1);
    }

    //Initializes a vector called vect1
    vector<int> vect1;

    //removeNaNFromPointCloud is a function that removes NaN values from PointCloud ex removing them from *originalCloud and mapping its values to *cleaned_originalCloud using the vect1 vector
    pcl::removeNaNFromPointCloud(*originalCloud, *originalCloud, vect1);


    cout << "frame_" << i <<" Loaded Successfully" << endl;

    cout << "\nLoading input data" << endl;

    if (pcl::io::loadPCDFile(file2, *targCloud) == -1)
    {
      PCL_ERROR("Failed to load second file! \n"); // In case of file not being read
      return (-1);
    }

    //Initializes a vector called vect2
    vector<int>vect2;

    pcl::removeNaNFromPointCloud(*targCloud, *targCloud,vect2);

    cout << "frame_" << i+1 <<" Loaded Successfully\n" << endl;


      // VOXELIZATION 
    

    if (voxel_grid_size > 0)
    {
      // Reduces data size -> Improves alignment speed
      cout << "Filtering scan and CAD clouds" << endl
           << "----------------------------" << endl;
      pcl::console::TicToc time;
      time.tic();
      pcl::VoxelGrid<PointT> voxel_grid;
      voxel_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
      voxel_grid.setInputCloud(originalCloud);
      voxel_grid.filter(*originalCloud);
      voxel_grid.setInputCloud(targCloud);
      voxel_grid.filter(*targCloud);
      cout << "Filtered 2 clouds in " << time.toc() << " ms " << endl;
      cout << "Scan cloud size: " << originalCloud->size() << endl;
      cout << "CAD cloud size: " << targCloud->size() << endl
           << endl;
    }
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix (0, 0) = cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = cos (theta);

  
    std::cout << "Applying this rigid transformation to: originalCloud -> transCloud" << std::endl;

    // Executing the transformation
    pcl::transformPointCloud (*originalCloud, *transCloud, transformation_matrix);
    
    /**
      * ICP ALGORITHM
    **/

    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(icp_iterations);
    icp.setInputSource(transCloud);
    icp.setInputTarget(targCloud);
    icp.align(*transCloud);
    icp.setMaximumIterations(20); // We set this variable to 1 for the next time we will call .align () function
    cout << "Applied " << icp_iterations << " ICP iteration(s) in " << time.toc() << " ms" << endl;

    if (icp.hasConverged())
    {
      cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
      cout << "\nICP transformation " << icp_iterations << " : transCloud -> originalCloud" << endl;
      transformation_matrix = icp.getFinalTransformation().cast<double>();
      //print4x4Matrix(transformation_matrix);
      Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
      cout << "trans %n" << transformation_matrix << endl;
    }
    else
    {
      PCL_ERROR("\nICP has not converged.\n");
      return (-1);
    }

    /**
      * ADDING COLOR TO THE POINT CLOUDS AND VIEWING THEM
    **/

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> originalCloud_color_h(originalCloud, (int)255 * txt_gray_lvl, (int)255 * txt_gray_lvl,(int)255 * txt_gray_lvl);
    viewer.removePointCloud("originalCloud_v1");
    viewer.addPointCloud(originalCloud, originalCloud_color_h, "originalCloud_v1", v1);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> targCloud_color_h(targCloud, 20, 180, 20);
    viewer.removePointCloud("targCloud_v1");
    viewer.addPointCloud(targCloud, targCloud_color_h, "targCloud_v1", v1);
    viewer.removePointCloud("targCloud_v2");
    viewer.addPointCloud(targCloud, targCloud_color_h, "targCloud_v2", v2);


    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> transCloud_color_h(transCloud, 180, 20, 20);
    viewer.removePointCloud("transCloud_v2");
    viewer.addPointCloud(transCloud, transCloud_color_h, "transCloud_v2", v2);
   

    // Set background color
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

   
    // Text descriptions
    viewer.addText("White: Input point cloud\nGreen: Source point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText("White: Source point cloud\nRed: ICP point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    stringstream ss;
    ss << icp_iterations;
    string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024); // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);

  /**
    * Main Loop
  **/

    // Display the visualiser
    if(viewer.wasStopped() == true){
      viewer.spinOnce();
    }

    while (!viewer.wasStopped()) //
    {
      viewer.spinOnce();

      // The user pressed "space" :
      if (frameChanger)
      {
        // The Iterative Closest Point algorithm
        time.tic();
        icp.align(*transCloud);
        cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << endl;

        if (icp.hasConverged())
        {
          printf("\033[11A"); // Go up 11 lines in terminal output.
          printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
          cout << "\nICP transformation " << ++icp_iterations << " : transCloud -> originalCloud" << endl;
          
          transformation_matrix = icp.getFinalTransformation().cast<double>();
          
          std::cout<<"\ntrans %n \n"<<transformation_matrix<<std::endl;

          ss.str("");
          ss << icp_iterations;
          string iterations_cnt = "ICP iterations = " + ss.str();
          viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
          viewer.updatePointCloud(transCloud, transCloud_color_h, "transCloud_v2");

        }
        else
        {
          PCL_ERROR("\nICP has not converged.\n");
          return (-1);
        }


      }

      frameChanger = false;
      
    }
    icp_iterations =1;
  }


  return (0);
}
