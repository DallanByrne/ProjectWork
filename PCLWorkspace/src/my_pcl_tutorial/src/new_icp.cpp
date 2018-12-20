#include <iostream>
#include <string>
#include <sstream>
#include <pcl/console/time.h>   // TicToc
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
double voxel_grid_size (0.0005);
int icp_iterations (1);
double max_corr_distance (0.001);  // 1 mm
int iterations_count (0);
double cloud_shift (0);
bool next_iteration = false;

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

int
main (int argc, char* argv[])
{
  stringstream sstream;
  stringstream ss2;

  for (int i = 1; i < 9; i++){
    //stringstream ss;
    if(i = 0){
      sstream << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_" << i << ".pcd";
    }
    else{

    }
    string file1 = sstream.str();
    sstream.clear();

    //stringstream ss2; 
    ss2 << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_" << i+1 << ".pcd";
    string file2 = ss2.str();
    ss2.clear();

        // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    
    if (argc > 0)
    {
      pcl::console::parse_argument (argc, argv, "--iterations", icp_iterations);
      pcl::console::parse_argument (argc, argv, "-i", icp_iterations);
      if (icp_iterations < 1)
      {
        PCL_WARN("Number of maximum iterations must be >= 1\n");
        icp_iterations = 1;
      }

      pcl::console::parse_argument (argc, argv, "--voxel_grid_size", voxel_grid_size);
      pcl::console::parse_argument (argc, argv, "-vgs", voxel_grid_size);

      pcl::console::parse_argument (argc, argv, "--max_corr_distance", max_corr_distance);
      pcl::console::parse_argument (argc, argv, "-mcd", max_corr_distance);
      max_corr_distance = fabs (max_corr_distance);

      pcl::console::parse_argument (argc, argv, "--cloud_shift", cloud_shift);
      pcl::console::parse_argument (argc, argv, "-cs", cloud_shift);
    }
    pcl::console::TicToc time;
    time.tic ();
    ///usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_1.pcd
    if(pcl::io::loadPCDFile (file1, *cloud_in)==-1)
    {
      PCL_ERROR ("Couldn't read first file! \n");// In case of file not being read
      return (-1);
    }
    
      //Initializes a vector called index	
    std::vector<int> index;
    
    //removeNaNFromPointCloud is a function that removes NaN values from PointCloud ex removing them from *cloud_in and mapping its values 		  to *cleaned_cloud_in using the index vector 
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, index);


    cout << "Loading input data" << endl;
    ///usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/Recordings/frame_2.pcd
    if(pcl::io::loadPCDFile (file2, *cloud_tr)==-1)
    {
      PCL_ERROR ("Couldn't read first file! \n");// In case of file not being read
      return (-1);
    }
    
      //Initializes a vector called index	
    std::vector<int> ind;
    
    //removeNaNFromPointCloud is a function that removes NaN values from PointCloud ex removing them from *cloud_in and mapping its values 		  to *cleaned_cloud_in using the index vector 
    pcl::removeNaNFromPointCloud(*cloud_tr, *cloud_tr, ind);


    cout << "Loading input data" << endl;
    /*if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
    {
      PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
      return (-1);
    }
    std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;
  */
    // Defining a rotation matrix and translation vector
    if (voxel_grid_size > 0)
    {
      // Down-sample the data for faster alignment
      cout << "Filtering scan and CAD clouds" << endl << "----------------------------" << endl;
      pcl::console::TicToc time;
      time.tic ();
      pcl::VoxelGrid<PointT> voxel_grid;
      voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      voxel_grid.setInputCloud (cloud_in);
      voxel_grid.filter (*cloud_in);
      voxel_grid.setInputCloud (cloud_tr);
      voxel_grid.filter (*cloud_tr);
      cout << "Filtered 2 clouds in " << time.toc () << " ms " << endl;
      cout << "Scan cloud size: " << cloud_in->size () << endl;
      cout << "CAD cloud size: " << cloud_tr->size () << endl << endl;
    }
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix (0, 0) = cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = cos (theta);

    // A translation on Z axis (0.4 meters)
    //transformation_matrix (2, 3) = 0.4;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
      // We backup cloud_icp into cloud_tr for later use

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (icp_iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_tr);
    icp.align (*cloud_icp);
    icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied " << icp_iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
      std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
      std::cout << "\nICP transformation " << icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
      transformation_matrix = icp.getFinalTransformation ().cast<double>();
      print4x4Matrix (transformation_matrix);
      Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation ();
      std::cout<<"trans %n"<<transformation_matrix<<std::endl;
    }
    else
    {
      PCL_ERROR ("\nICP has not converged.\n");
      return (-1);
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("ICP COMPARISION");
    // Create two vertically separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                              (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_tr, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nGreen: Matrix transformed point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << icp_iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();

      // The user pressed "space" :
      if (next_iteration)
      {
        // The Iterative Closest Point algorithm
        time.tic ();
        icp.align (*cloud_icp);
        std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;
        //transformation_matrix = icp.getFinalTransformation().cast<double>;
        //print4x4Matrix (transformation_matrix);
        if (icp.hasConverged ())
        {
          printf ("\033[11A");  // Go up 11 lines in terminal output.
          printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
          std::cout << "\nICP transformation " << ++icp_iterations << " : cloud_icp -> cloud_in" << std::endl;
          //Eigen::Matrix4f transformation = icp.getFinalTransformation ();  // WARNING /!\ This is not accurate! For "educational" purpose only!
          transformation_matrix = icp.getFinalTransformation ().cast<double>(); 
          print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
          //std::cout<<"\ntrans %n \n"<<transformation<<std::endl;
          
          ss.str ("");
          ss << icp_iterations;
          std::string iterations_cnt = "ICP iterations = " + ss.str ();
          viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
          viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
        }
        else
        {
          PCL_ERROR ("\nICP has not converged.\n");
          return (-1);
        }
      }
      next_iteration = false;
      //pcl::io::savePCDFileASCII ("output.pcd", cloud);
    }    


  }

  return (0);
}