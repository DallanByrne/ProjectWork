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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <limits>       // std::numeric_limits


using namespace std;
using std::vector;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

double voxGSize(0.01);
int icp_iterations(5);
double max_corr_distance(0.05); // 1 mm
int iterations_count(0);
double cloud_shift(0);
bool frameChanger = false;
//Crop Box
float minX = -0.5, minY = -1, minZ = 0.5;
float maxX = +0.9, maxY = +1, maxZ = +1.6;

/*
    --- Keyboard Input ---
*/

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
  if (event.getKeySym() == "space" && event.keyDown()){
    frameChanger = true;
  }
}

/*
    --- Crop_Box ---
*/
void focusField(pcl::PointCloud<pcl::PointXYZ>::Ptr body){
    pcl::PointCloud<pcl::PointXYZ>::Ptr bodyFiltered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::CropBox<pcl::PointXYZ> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(body);
    boxFilter.filter(*bodyFiltered);

    //*body = *bodyFiltered;
}


/*
    --- Segmentor ---
*/
void planeErase(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
    }
}


/*
    --- Euclidean Cluster Extraction ---
*/
void extractSearch(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
    pcl::PCDWriter writer;
    /*
        -- KDTree --
    */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.05); // 5cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25100);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "clusterTemp/cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
    cout << j << " objects created!" << endl;

}

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

void vizual(Eigen::Matrix4d matrix){

    pcl::visualization::PCLVisualizer viz("Transformed item");
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/clusterFiles/cloud_cluster_2.pcd", *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    
    pcl::transformPointCloud (*cloud, *transformed_cloud, matrix);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viz.addPointCloud (cloud, cloud_color_handler, "cloud_frame3");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
    viz.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    viz.addCoordinateSystem (1.0, "cloud_frame3", 0);
    viz.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_frame3");
    viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

    viz.addPointCloud(cloud);
    while (!viz.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viz.spinOnce ();
    }

}

int main(int argc, char *argv[]){

    if (argc > 0)
    {
        pcl::console::parse_argument(argc, argv, "--iterations", icp_iterations);
        pcl::console::parse_argument(argc, argv, "-i", icp_iterations);
        if (icp_iterations < 1)
        {
        PCL_WARN("Number of maximum iterations must be >= 1\n");
        icp_iterations = 1;
        }

        pcl::console::parse_argument(argc, argv, "--voxGSize", voxGSize);
        pcl::console::parse_argument(argc, argv, "-vgs", voxGSize);

        pcl::console::parse_argument(argc, argv, "--max_corr_distance", max_corr_distance);
        pcl::console::parse_argument(argc, argv, "-mcd", max_corr_distance);
        max_corr_distance = fabs(max_corr_distance);

        pcl::console::parse_argument(argc, argv, "--cloud_shift", cloud_shift);
        pcl::console::parse_argument(argc, argv, "-cs", cloud_shift);
    }

    pcl::visualization::PCLVisualizer viewer("Object ICP");
    int v1(0);
    int v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    float bckgr_gray_level = 0.0; // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    stringstream objectIn;
    cout << "Insert object for comparison: ";
    //cloud_cluster_2.pcd

    objectIn << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/clusterFiles/cloud_cluster_2.pcd";
    string fileIn = objectIn.str();
    objectIn.str(string());

    Eigen::Matrix4d final_matrix = Eigen::Matrix4d::Identity();


    //int i = 305;
    int i = 1;
    double fitness;
    //float unfit = 

    while(i < 70){

        stringstream sstream;
        sstream << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/src/my_pcl_tutorial/src/BF/kinektMotion/frame" << i << ".pcd";
        string filetarget = sstream.str();
        sstream.str(string());
        sstream.clear();

        PointCloudT::Ptr originalCloud(new PointCloudT);  // Original point cloud
        PointCloudT::Ptr targCloud(new PointCloudT);  // Target point cloud
        PointCloudT::Ptr transCloud(new PointCloudT); // ICP output point cloud

        PointCloudT::Ptr tempCloud1(new PointCloudT);
        PointCloudT::Ptr tempCloud(new PointCloudT);    //Temperary Cloud point ... Extract From 

        pcl::console::TicToc time;
        time.tic();


        //Load filetarget and remove NaN's
        cout << "Loading input data" << endl;
        if (pcl::io::loadPCDFile(fileIn, *originalCloud) == -1)
        {
            PCL_ERROR("Failed to load first file! \n"); // In case of file not being read
            return (-1);
        }

        vector<int> vect1;

        //removeNaNFromPointCloud is a function that removes NaN values from PointCloud ex removing them from *originalCloud and mapping its values to *cleaned_originalCloud using the vect1 vector
        pcl::removeNaNFromPointCloud(*originalCloud, *originalCloud, vect1);

        cout << "cloud_cluster_2.pcd loaded successfully" << endl;

        /*
            -- Extract from Temp
        */
        if(pcl::io::loadPCDFile(filetarget, *tempCloud) == -1)
        {
            PCL_ERROR("Failed to load first file! \n"); // In case of file not being read
            return (-1);
        }
        vector<int> v;
        pcl::removeNaNFromPointCloud(*tempCloud, *tempCloud, v);
        focusField(tempCloud);
        planeErase(tempCloud);
        extractSearch(tempCloud);

        sstream << "/usr/local/home/u180120/nethome/ProjectWork/PCLWorkspace/clusterTemp/cloud_cluster_0.pcd";
        string obj = sstream.str();
        sstream.str(string());
        sstream.clear();

        /*
            -- Load file2 and remove NaN's
        */
        cout << "\nLoading input data" << endl;
        if (pcl::io::loadPCDFile(obj, *targCloud) == -1)
        {
            PCL_ERROR("Failed to load second file! \n"); // In case of file not being read
            return (-1);
        }

        vector<int>vect2;

        pcl::removeNaNFromPointCloud(*targCloud, *targCloud,vect2);

        cout << "frame" << i <<" Loaded Successfully\n" << endl; 


        /*
        ---  Matrix Declaration  ---
        */
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    
        std::cout << "Applying this rigid transformation to: originalCloud -> transCloud" << std::endl;

        // Executing the transformation
        pcl::transformPointCloud (*originalCloud, *transCloud, transformation_matrix);
        

        /*
        ---  ICP ALGORITHM ---
        */

        time.tic();
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setMaximumIterations(icp_iterations);
        icp.setInputSource(transCloud);
        icp.setInputTarget(targCloud);
        icp.align(*transCloud);
        icp.setMaximumIterations(25); // We set this variable to 1 for the next time we will call .align () function
        icp.setMaxCorrespondenceDistance (0.06);
        //icp.setTransformationEpsilon (1e-8);
        // Set the euclidean distance difference epsilon (criterion 3)
        //icp.setEuclideanFitnessEpsilon (0.9);

        cout << "Applied " << icp_iterations << " ICP iteration(s) in " << time.toc() << " ms" << endl;

        if (icp.hasConverged())
        {
            cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
            cout << "\nICP transformation " << icp_iterations << " : transCloud -> originalCloud" << endl;
            transformation_matrix = icp.getFinalTransformation().cast<double>();
            //cout << "trans %n" << transformation_matrix << endl;
            print4x4Matrix(transformation_matrix);
            
            if(i == 1 ){
                final_matrix = icp.getFinalTransformation().cast<double>();
            }
            else{
                final_matrix = final_matrix * icp.getFinalTransformation().cast<double>();
            }

        }
        else
        {
            PCL_ERROR("\nICP has not converged.\n");
            return (-1);
        }

        /*
        --  Add color
        */

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
        //viewer.setSize(1280, 1024); // Visualiser window size

        // Register keyboard callback :
        viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);
            
        if(viewer.wasStopped() == true){
            viewer.spinOnce();
        }
        
        //viewer.addPointCloud (cloud_f);
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
                    
                    //std::cout<<"\ntrans %n \n"<<transformation_matrix<<std::endl;
                    print4x4Matrix(transformation_matrix);

                    ss.str("");
                    ss << icp_iterations;
                    string iterations_cnt = "ICP iterations = " + ss.str();
                    viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                    viewer.updatePointCloud(transCloud, transCloud_color_h, "transCloud_v2");
                    viewer.updatePointCloud(targCloud, targCloud_color_h, "targCloud_v1");
                    viewer.updatePointCloud(originalCloud, originalCloud_color_h, "originalCloud_v1");

                    final_matrix = final_matrix * icp.getFinalTransformation().cast<double>();

                }
                else
                {
                    PCL_ERROR("\nICP has not converged.\n");
                    return (-1);
                }
                frameChanger = false;

            }
        }

        fitness = fitness + icp.getFitnessScore();
        /*
        if(i == 1 ){
            final_matrix = icp.getFinalTransformation().cast<double>();
        }
        else{
            final_matrix = final_matrix * icp.getFinalTransformation().cast<double>();
        }
        */
       fileIn = obj;
            
        i++;
    }

    std::cout << "Loop Completed!"<<endl;
    //std::cout<<"\ntrans %n \n"<<final_matrix<<std::endl;
    print4x4Matrix(final_matrix);

    cout <<"\nICP Fitness Score: " << fitness/70 << endl;

    vizual(final_matrix);


    return 0;
}
