#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

int main (int argc, char** argv)
{
    std::string filename = argv[1];
    std::cout << "Reading " << filename << std::endl;

    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read(filename, *cloud); // Remember to download the file first!

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
        << " data points (" << pcl::getFieldsList (*cloud) << "). \n";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height
        << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    pcl::PCDWriter writer;
    writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered,
                  Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

    return (0);
}