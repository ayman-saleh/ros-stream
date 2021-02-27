cmake_minimum_required(VERSION 2.8)
project(ros-stream)


add_compile_options(-std=c++11)
find_package(PkgConfig)

# Configure Gstreamer packages
# Gstreamer implements standard pkg-config metadata
# pkg_check_modules must be used to interface with metadata
# cannot use find_package
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
                               gstreamer-sdp-1.0>=1.4
                               gstreamer-video-1.0>=1.4
                               gstreamer-app-1.0>=1.4) 

# Query ROS Specifc Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
)

# Query 3d Party Packages
find_package(
  OpenCV
)
find_package(
  CUDA
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES deepstream
  CATKIN_DEPENDS roscpp sensor_msgs image_transport
)

###########
## Build ##
###########

# Query deepstream and gstreamer include dirs
# TODO: Find a better way to include deepstream libaries
set(NVGS_INCLUDE_DIRS /opt/nvidia/deepstream/deepstream/sources/includes)
set(NVDS_LIBS /opt/nvidia/deepstream/deepstream-5.0/lib)
set(DEEPSTREAM_INCLUDE_DIRS /opt/nvidia/deepstream/deepstream-5.0/sources/apps/apps-common/includes)


include_directories(
  include/wvm-ros-stream
  ${catkin_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${NVGS_INCLUDE_DIRS}
  ${NVDS_LIBS}
  ${DEEPSTREAM_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
) 

## Compile Wavemaker Rosstream library
add_library(rosstream_plugin src/rosstream-plugin.cpp src/rosstreamer-lib.cpp)

## Link libs library
target_link_libraries(rosstream_plugin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
  ${CUDA_LIBS}
  ${CUDA_LIBRARIES}
  ${GST_LIBRARIES}
  ${DEEPSTREAM_INCLUDE_DIRS}
  ${NVDS_LIBS}/libnvds_meta.so
  ${NVDS_LIBS}/libnvdsgst_meta.so
  ${NVDS_LIBS}/libnvbufsurface.so
  ${NVDS_LIBS}/libnvdsgst_helper.so
  ${NVDS_LIBS}/libnvbufsurftransform.so
)