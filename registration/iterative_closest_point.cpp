#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <elapse_timer.hpp>
#include <pcl/visualization/pcl_visualizer.h>


int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the CloudIn data
    cloud_in->width    = 500;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_in->points[i].x = 1024 * rand() /(RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() /(RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() /(RAND_MAX + 1.0f);
    }
    std::cout << "Saved " << cloud_in->points.size() << " data points to input:" << std::endl;

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        std::cout << "    " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z << std::endl;

    *cloud_out = *cloud_in;

    std::cout << "size:" << cloud_out->points.size() << std::endl;

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

    std::cout << "Transformed " << cloud_in->points.size() << " data points:" << std::endl;
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
        std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    prodrone::ElapseTimer e("Aligning ", true);
    icp.align(Final);
    e.printElapsed();

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    /// Uncomment below if you want to transform back the input cloud
    pcl::transformPointCloud(*cloud_in, *cloud_in, icp.getFinalTransformation());

    pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(cloud_in, 255, 255, 255);
    viewer.addPointCloud(cloud_in, source_cloud_color_handler, "original_cloud");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(cloud_out, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_out, transformed_cloud_color_handler, "transformed_cloud");
    viewer.addCoordinateSystem(1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return(0);
}