#include <elapse_timer.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>
#include <iostream>

int main (int, char** argv)
{
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) // load the file
    {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "Load successful. Number of points: " << cloud->points.size () << std::endl;

    if (!argv[2])
    {
        PCL_ERROR("Search radius is not provided\n");
        return -1;
    }
    double search_radius = std::atof(argv[2]);

    // Create the normal estimation class, and pass the input dataset to it:
    /// pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne; /// Uncomment this in order to run as single thread
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>); // Output datasets

    std::cout << "Starting to compute normals with search radius " << search_radius << " meters ..." << std::endl; // DEBUG DELETE
    ne.setRadiusSearch(search_radius); // Use all neighbors in a sphere of radius 3cm
    prodrone::ElapseTimer execution_time;
    ne.compute(*cloud_normals); // Compute the features
    std::cout << "Computation finished. Elapsed time: " << execution_time.elapsed() << " sec." << std::endl; // DEBUG DELETE

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 230, 20, 20); // Red
    viewer.addPointCloud(cloud, cloud_color_handler, "transformed_cloud");

    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
    return 0;
}