/*
 * Software License Agreement(BSD License)
 *
 * Point Cloud Library(PCL) - www.pointclouds.org
 * Copyright(c) 2009-2011, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * * Neither the name of Willow Garage, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */


#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>

#include <elapse_timer.hpp>
#include <normals_visualizer.hpp>


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        throw std::runtime_error("Required arguments: filename.pcd");
    }

    std::string fileName = argv[1];
    std::cout << "Reading " << fileName << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *cloud_in) == -1) // load the file
    {
        PCL_ERROR("Couldn't read file");
        return(-1);
    }

    std::cout << "Loaded " << cloud_in->points.size() << " points." << std::endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_in, *cloud, indices);
    std::cout << "After removing NaN points, " << cloud->points.size() << " points left." << std::endl;


//    for (int i = 0; i < cloud->points.size(); i++)
//    {
//        if (!pcl::isFinite<pcl::PointXYZ>(cloud->points[i]))
//        {
//            PCL_WARN("point[%d] is not finite (%d, %d, %d), removing it\n", i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//            cloud->points.erase(cloud->begin() + i);
//        }
//    }
//

    // Compute the normals
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch(0.02);

    prodrone::ElapseTimer t_0;
    normal_estimation.compute(*cloud_with_normals);
    t_0.printElapsed();

    for (int i = 0; i < cloud_with_normals->points.size(); i++)
    {
        if (!pcl::isFinite<pcl::Normal>(cloud_with_normals->points[i]))
        {
            PCL_WARN("normals[%d] is not finite, removing it\n", i);
//            cloud_with_normals->points.erase(cloud_with_normals->begin() + i);
        }
    }

//    prodrone::NormalsVisualizer nv;
//    nv.visualize(cloud, cloud_with_normals);

    // Setup the feature computation
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    // Provide the original point cloud(without normals)
    fpfh_estimation.setInputCloud(cloud);
    // Provide the point cloud with normals
    fpfh_estimation.setInputNormals(cloud_with_normals);

    // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    fpfh_estimation.setSearchMethod(tree);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh_estimation.setRadiusSearch(0.02);


    prodrone::ElapseTimer t_1;
    // Actually compute the spin images
    fpfh_estimation.compute(*pfh_features);
    t_1.printElapsed();

    std::cout << "output points.size(): " << pfh_features->points.size() << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    pcl::FPFHSignature33 descriptor = pfh_features->points[0];
    std::cout << descriptor << std::endl;

    return 0;
}
