#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/pfh.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

int main(int, char** argv)
{
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file");
        return(-1);
    }

    std::cout << "Loaded " << cloud->points.size() << " points." << std::endl;

    // Compute the normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimation.setRadiusSearch(0.03);
    normal_estimation.compute(*cloud_with_normals);
    std::cout << "Normals computed." << std::endl; // DEBUG DELETE

    // Setup the feature computation
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_estimation;
    // Provide the original point cloud(without normals)
    pfh_estimation.setInputCloud(cloud);
    // Provide the point cloud with normals
    pfh_estimation.setInputNormals(cloud_with_normals);

    // pfh_estimation.setInputWithNormals(cloud, cloud_with_normals); PFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    pfh_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_features (new pcl::PointCloud<pcl::PFHSignature125>);
    pfh_estimation.setRadiusSearch(0.01);
    // Actually compute the spin images
    std::cout << "Starting PFH estimation." << std::endl; // DEBUG DELETE
    pfh_estimation.compute(*pfh_features);
    std::cout << "Finished PFH estimation." << std::endl; // DEBUG DELETE

    std::cout << "output points.size(): " << pfh_features->points.size() << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    pcl::PFHSignature125 descriptor = pfh_features->points[0];
    std::cout << descriptor << std::endl;

    return 0;
}