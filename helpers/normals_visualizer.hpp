#ifndef NORMALS_VISUALIZER_HPP
#define NORMALS_VISUALIZER_HPP

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//#include <pcl/point_types.h>
//#include <pcl/point_representation.h>

#include <stdio.h>
#include <string>

namespace prodrone
{

class NormalsVisualizer
{


public:
    NormalsVisualizer(){}

    void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
    {
        pcl::visualization::PCLVisualizer viewer("Viewer"); // visualize normals
        viewer.setBackgroundColor (0.0, 0.0, 0.5);
        viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 230, 20, 20); // Red
        viewer.addPointCloud(cloud, cloud_color_handler, "transformed_cloud");

        while (! viewer.wasStopped())
        {
            viewer.spinOnce ();
        }

        viewer.close();
    }
};

} /// namespace prodrone

#endif /// NORMALS_VISUALIZER_HPP 
