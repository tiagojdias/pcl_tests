## Updated for Ubuntu 16.04 by ProDrone Install Scripts
cmake_minimum_required(VERSION 3.2.2)
project(prodrone)

## C++11 Flags
##
add_compile_options(-std=c++11)
add_compile_options(-pthread)
add_compile_options(-Werror=return-type)                   # error on missing return type
add_compile_options(-Werror=format)                        # error on wrong printf formats
add_compile_options(-Werror=parentheses)                   # error when using ambiguous syntax

set(STRICT_GCC_FLAGS "\
    -Wunused-but-set-parameter \
    -Wunused-but-set-variable \
    -Wunused-function \
    -Wunused-variable \
    -Wuninitialized \
    -Wmaybe-uninitialized \
    -Wno-endif-labels \
    -Wmissing-field-initializers \
    -Wunused-parameter
")


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(helpers)

### Basic
# pcd_write_test
add_executable(pcd_write_test basic/pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
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

