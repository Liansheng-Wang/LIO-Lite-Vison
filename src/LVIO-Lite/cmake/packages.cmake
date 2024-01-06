list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})

if (CUSTOM_TBB_DIR)
    set(TBB2018_INCLUDE_DIR "${CUSTOM_TBB_DIR}/include")
    set(TBB2018_LIBRARY_DIR "${CUSTOM_TBB_DIR}/lib/intel64/gcc4.7")
    include_directories(${TBB2018_INCLUDE_DIR})
    link_directories(${TBB2018_LIBRARY_DIR})
endif ()

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        pcl_ros
        tf
        message_generation
        eigen_conversions
        cv_bridge
        )


add_message_files(
        FILES
        Pose6D.msg
        KeyFrame.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
        sensor_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
        DEPENDS EIGEN3 PCL
        INCLUDE_DIRS
)


find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        include
        3rdparty
)
