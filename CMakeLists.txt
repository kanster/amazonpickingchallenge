cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()
rosbuild_gensrv()
rosbuild_genmsg()

project( uts_recogniser )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/dependencies/")
set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CMAKE_SOURCE_DIR}/dependencies/" )



find_package(PkgConfig REQUIRED)

# check for OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
    message(STATUS "Enabling OpenMP support")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(STATUS "Disabling OpenMP support")
endif()

find_package(FLANN REQUIRED)
if(FLANN_FOUND)
    message("== FLANN INCLUDE DIRS ${FLANN_INCLUDE_DIRS}")
    message("== FLANN LIBS REQUIRED ${FLANN_LIBRARIES}")
    include_directories( ${FLANN_INCLUDE_DIRS} )
    link_directories( ${FLANN_LIBRARY_DIRS} )
    add_definitions( ${FLANN_DEFINITIONS} )
endif()


pkg_check_modules(EIGEN REQUIRED eigen3)
message( "== EIGEN INCLUDE DIRS ${EIGEN_INCLUDE_DIRS}")
include_directories( ${EIGEN_INCLUDE_DIRS} )

find_package(PCL 1.7.1 REQUIRED)
if(PCL_FOUND)
    message("== PCL INCLUDE DIRS ${PCL_INCLUDE_DIRS}")
    message("== PCL LIBS REQUIRED ${PCL_LIBRARIES}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


pkg_check_modules(V4R REQUIRED v4r)
set( V4R_LIBRARIES "-lv4rSegmenterLight -lv4rTomGine")
message ( "== V4R LIBS REQUIRED ${V4R_LIBRARIES}")
message ( "== V4R INCLUDE DIRS ${V4R_INCLUDE_DIRS}")
include_directories( ${V4R_INCLUDE_DIRS} )
message ( "== V4R LIB DIRS ${V4R_LIBRARY_DIRS}")
link_directories( ${V4R_LIBRARY_DIRS} )

pkg_check_modules(FLANN REQUIRED flann)
set( FLANN_LIBRARIES "-lflann_cpp")
message ( "== FLANN LIBS REQUIRED ${FLANN_LIBRARIES}")
message ( "== FLANN INCLUDE DIRS ${FLANN_INCLUDE_DIRS}")
include_directories( ${FLANN_INCLUDE_DIRS} )
message ( "== FLANN LIB DIRS ${FLANN_LIBRARY_DIRS}")
link_directories( ${FLANN_LIBRARY_DIRS} )

pkg_check_modules(SIFTFAST REQUIRED siftfast)
set( SIFTFAST_LIBRARIES "-lsiftfast" )
message ( "== SIFTFAST LIBS REQUIRED ${SIFTFAST_LIBRARIES}" )
message ( "== SIFTFAST INCLUDE DIRS ${SIFTFAST_INCLUDE_DIRS}")
include_directories( ${SIFTFAST_INCLUDE_DIRS} )
message ( "== SIFTFAST LIB DIRS ${SIFTFAST_LIBRARY_DIRS}")
link_directories( ${SIFTFAST_LIBRARY_DIRS} )

pkg_search_module(GL REQUIRED gl)
pkg_search_module(GLU REQUIRED glu)
if(NOT GL_FOUND)
    message(FATAL "libgl not found, required by v4rTomGine")
endif(NOT GL_FOUND)

find_package(OpenCV 2.4 REQUIRED)
if(OpenCV_FOUND)
    message("== OPENCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
    message("== OPENCV LIBS REQUIRED ${OpenCV_LIBS}")
endif()

include_directories(${PROJECT_SOURCE_DIR})


set( SOURCE_H
    ${PROJECT_SOURCE_DIR}/include/json_parser.hpp
    ${PROJECT_SOURCE_DIR}/include/utils.h
    ${PROJECT_SOURCE_DIR}/include/feature_detector.h
    ${PROJECT_SOURCE_DIR}/include/feature_cluster.h
    ${PROJECT_SOURCE_DIR}/include/feature_matcher.h
    ${PROJECT_SOURCE_DIR}/include/levmar_pose_estimator.h
    ${PROJECT_SOURCE_DIR}/include/projection_filter.h
    ${PROJECT_SOURCE_DIR}/include/mask_generator.h
    ${PROJECT_SOURCE_DIR}/include/cloud_segmenter.h
    ${PROJECT_SOURCE_DIR}/include/svd_pose_estimator.h
    ${PROJECT_SOURCE_DIR}/include/uts_recogniser.h
    ${PROJECT_SOURCE_DIR}/include/rgb_recogniser.h
    ${PROJECT_SOURCE_DIR}/include/rgbd_recogniser.h
)

set( SOURCE_CPP
    ${PROJECT_SOURCE_DIR}/src/utils.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_detector.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_cluster.cpp
    ${PROJECT_SOURCE_DIR}/src/feature_matcher.cpp
    ${PROJECT_SOURCE_DIR}/src/levmar_pose_estimator.cpp
    ${PROJECT_SOURCE_DIR}/src/projection_filter.cpp
    ${PROJECT_SOURCE_DIR}/src/mask_generator.cpp
    ${PROJECT_SOURCE_DIR}/src/cloud_segmenter.cpp
    ${PROJECT_SOURCE_DIR}/src/svd_pose_estimator.cpp
    ${PROJECT_SOURCE_DIR}/src/uts_recogniser.cpp
    ${PROJECT_SOURCE_DIR}/src/rgb_recogniser.cpp
    ${PROJECT_SOURCE_DIR}/src/rgbd_recogniser.cpp
)

rosbuild_add_boost_directories()

rosbuild_add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp ${SOURCE_CPP} ${SOURCE_H})
rosbuild_link_boost(${PROJECT_NAME} thread system filesystem program_options signals )
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${SIFTFAST_LIBRARIES} ${V4R_LIBRARIES} )
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_SOURCE_DIR}/dependencies/libANN.a
    ${CMAKE_SOURCE_DIR}/dependencies/liblevmar.a
)


rosbuild_add_executable( pseudo_request src/pseudo_request.cpp )

#rosbuild_add_executable(thread_test test/multi_reader_single_writer.cpp)
#rosbuild_link_boost(thread_test thread system filesystem program_options signals )

