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

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

# Find "set (srcs", and add a new entry there, e.g.,
set (srcs
    src/conditional_removal.cpp
    src/bilateral.cpp
)

# Find "set (incs", and add a new entry there, e.g.,
set (incs
    include pcl/${SUBSYS_NAME}/conditional_removal.h
    include pcl/${SUBSYS_NAME}/bilateral.h
)

# Find "set (impl_incs", and add a new entry there, e.g.,
set (impl_incs
    include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
    include/pcl/${SUBSYS_NAME}/impl/bilateral.hpp
)