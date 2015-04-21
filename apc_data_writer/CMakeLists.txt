cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(Eigen3)
if(EIGEN3_FOUND)
  message(STATUS "EIGEN3 INCLUDE DIRS ${EIGEN3_INCLUDE_DIR}")
  include_directories(${EIGEN3_INCLUDE_DIR})
endif()

find_package(PCL 1.7.0 REQUIRED)
if(PCL_FOUND)
  message(STATUS "PCL INCLUDE DIRS ${PCL_INCLUDE_DIRS}")
  message(STATUS "PCL LIBS REQUIRED ${PCL_LIBRARIES}")
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

find_package(OpenCV 2.4 REQUIRED)
if(OpenCV_FOUND)
  message(STATUS "OPENCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
  message(STATUS "OPENCV LIBS REQUIRED ${OpenCV_LIBS}")
  include_directories(${OpenCV_INCLUDE_DIRS})
  link_directories(${OpenCV_LIBRARY_DIRS})
endif()

rosbuild_add_boost_directories()

rosbuild_add_executable(apc_data_writer include/data_writer.h src/data_writer.cpp src/main.cpp)
rosbuild_link_boost(apc_data_writer thread system filesystem program_options signals )
target_link_libraries(apc_data_writer ${PCL_LIBRARIES} ${OpenCV_LIBS})


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
