## Updated for Ubuntu 16.04 by ProDrone Install Scripts
cmake_minimum_required(VERSION 3.2.2)
project(prodrone)

## C++11 Flags
##
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS -pthread)
set(CMAKE_BUILD_TYPE "RELEASE") # This flag sets PCL and OMPL libraries to use high level optimization binaries

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(helpers)

### Basic
# pcd_write_test
add_executable(pcd_write_test basic/pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
# matrix_transform
add_executable (matrix_transform basic/matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})
# bilateral_filter_main
add_executable (bilateral_filter_main basic/bilateral_filter_main.cpp)
target_link_libraries (bilateral_filter_main ${PCL_LIBRARIES})


### Features
# estimating_surface_normals
add_executable(estimating_surface_normals features/estimating_surface_normals.cpp)
target_link_libraries (estimating_surface_normals ${PCL_LIBRARIES})
# normal_estimation_using_integral_images
add_executable(normal_estimation_using_integral_images features/normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})
# point_feature_histograms
add_executable(point_feature_histograms features/point_feature_histograms.cpp)
target_link_libraries (point_feature_histograms ${PCL_LIBRARIES})
# fast_point_feature_histograms
add_executable(fast_point_feature_histograms features/fast_point_feature_histograms.cpp)
target_link_libraries (fast_point_feature_histograms ${PCL_LIBRARIES})
# narf_keypoint_extraction
add_executable(narf_keypoint_extraction features/narf_keypoint_extraction.cpp)
target_link_libraries (narf_keypoint_extraction ${PCL_LIBRARIES})
# narf_feature_extraction
add_executable(narf_feature_extraction features/narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})
# rops_feature
add_executable(rops_feature features/rops_feature.cpp)
target_link_libraries (rops_feature ${PCL_LIBRARIES})


### Range images
# range_image_creation
add_executable(range_image_creation range_images/range_image_creation.cpp)
target_link_libraries (range_image_creation ${PCL_LIBRARIES})
# range_image_border_extraction
add_executable(range_image_border_extraction range_images/range_image_border_extraction.cpp)
target_link_libraries (range_image_border_extraction ${PCL_LIBRARIES})


### Filters
# passthrough
add_executable(passthrough filters/passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})
# voxel_grid
add_executable(voxel_grid filters/voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})
# statistical_removal
add_executable(statistical_removal filters/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
# extract_indices
add_executable(extract_indices filters/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})


### Trees
# kdtree_search
add_executable(kdtree_search trees/kdtree_search.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})
# point_cloud_compression
add_executable(point_cloud_compression trees/point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})
# octree_search
add_executable(octree_search trees/octree_search.cpp)
target_link_libraries (octree_search ${PCL_LIBRARIES})
# octree_change_detection
add_executable(octree_change_detection trees/octree_change_detection.cpp)
target_link_libraries (octree_change_detection ${PCL_LIBRARIES})


### Recognition
# correspondence_grouping
#add_executable(correspondence_grouping recognition/correspondence_grouping.cpp)
#target_link_libraries (correspondence_grouping ${PCL_LIBRARIES})
# implicit_shape_model
add_executable(implicit_shape_model recognition/implicit_shape_model.cpp)
target_link_libraries (implicit_shape_model ${PCL_LIBRARIES})
# global_hypothesis_verification - has errors during compilation
#add_executable(global_hypothesis_verification recognition/global_hypothesis_verification.cpp)
#target_link_libraries (global_hypothesis_verification ${PCL_LIBRARIES})

### Registration
# iterative_closest_point
add_executable(iterative_closest_point registration/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
# pairwise_incremental_registration
add_executable(pairwise_incremental_registration registration/pairwise_incremental_registration.cpp)
target_link_libraries (pairwise_incremental_registration ${PCL_LIBRARIES})
# interactive_icp
add_executable(interactive_icp registration/interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})
# normal_distributions_transform
add_executable(normal_distributions_transform registration/normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})
# alignment_prerejective
add_executable(alignment_prerejective registration/alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})

# test_app
add_executable(test_app registration/test_app.cpp)
target_link_libraries (test_app ${PCL_LIBRARIES})


### Segmentation
#planar_segmentation
add_executable(planar_segmentation segmentation/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
#cylinder_segmentation
add_executable(cylinder_segmentation segmentation/cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})
