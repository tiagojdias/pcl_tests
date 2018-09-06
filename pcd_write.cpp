#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Fill in the cloud data
    cloud.width    = 30;
    cloud.height   = 30;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        cloud.points[i].x = 1024 * rand() / (RAND_MAX + 100.0f);
        cloud.points[i].y = 1024 * rand() / (RAND_MAX + 100.0f);
        cloud.points[i].z = 1024 * rand() / (RAND_MAX + 100.0f);
        cloud.points[i].intensity = rand() % 1024;
    }

    pcl::io::savePCDFileASCII ("test_pcd_intensity.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;

//    for(size_t i = 0; i < cloud.points.size(); ++i)
//        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    return (0);
}