cmake_minimum_required(VERSION 3.5)
project(lvio_lite)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


include(cmake/packages.cmake)

#definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()


# Generate Library
add_library(${PROJECT_NAME}
        src/laser_mapping.cc
        src/pointcloud_preprocess.cc
        src/options.cc
        src/utils.cc
        )


target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})


# build
add_executable(run_mapping_online src/run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )

add_executable(run_location_online src/run_location_online.cc)
target_link_libraries(run_location_online
        ${PROJECT_NAME} gflags
        )

add_executable(renderer src/renderer.cc)
target_link_libraries(renderer
        ${PROJECT_NAME}
        ${OpenCV_LIBRARIES}
        )

# add_executable(test_split src/test/test_split.cc)
# target_link_libraries(test_split
#         ${PROJECT_NAME} gflags
#         )

# add_executable(test_load src/test/test_load.cc)
# target_link_libraries(test_load
#         ${PROJECT_NAME} gflags
#         )