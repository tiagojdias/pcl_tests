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