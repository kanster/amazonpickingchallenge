cmake_minimum_required(VERSION 2.4.6)
project( uts_recogniser )

if(DEFINED CATKIN_DEVEL_PREFIX)
  set(USE_CATKIN true)
else()
  set(USE_CATKIN false)
endif()

################################### ros_build ####################################
if(NOT USE_CATKIN)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -fpermissive -O3")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#rosbuild_find_ros_package(actionlib_msgs)
#include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
#genaction()

rosbuild_init()
#rosbuild_gensrv()
#rosbuild_genmsg()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

################################### catkin #########################################
else()

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  actionlib_msgs
  roscpp
  rospy
  message_filters
  image_geometry
  pcl_ros
  pcl_conversions
  image_transport
  cv_bridge
  apc_msgs
)

find_package(Boost REQUIRED COMPONENTS thread system filesystem)

include_directories(include)

endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/dependencies/")
set( ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${CMAKE_CURRENT_SOURCE_DIR}/dependencies/" )

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

#find_package(LAPACK REQUIRED)
#if(LAPACK_FOUND)
#    message("== LAPACK INCLUDE DIRS ${LAPACK_INCLUDE_DIRS}")
#    message("== LAPACK LIBS REQUIRED ${LAPACK_LIBRARIES}")
#    include_directories( ${LAPACK_INCLUDE_DIRS} )
#    link_directories( ${LAPACK_LIBRARY_DIRS} )
#    add_definitions( ${LAPACK_DEFINITIONS} )
#endif()

find_package(BLAS REQUIRED)
if(BLAS_FOUND)
    message("== BLAS INCLUDE DIRS ${BLAS_INCLUDE_DIRS}")
    message("== BLAS LIBS REQUIRED ${BLAS_LIBRARIES}")
    include_directories( ${BLAS_INCLUDE_DIRS} )
    link_directories( ${BLAS_LIBRARY_DIRS} )
    add_definitions( ${BLAS_DEFINITIONS} )
endif()

pkg_check_modules(EIGEN REQUIRED eigen3)
message( "== EIGEN INCLUDE DIRS ${EIGEN_INCLUDE_DIRS}")
include_directories( ${EIGEN_INCLUDE_DIRS} )

pkg_check_modules(MATIO REQUIRED matio)
set( MATIO_LIBRARIES "-lmatio -lz -lhdf5" )
message("== MATIO LIBS REQUIRED ${MATIO_LIBRARIES}" )
message("== MATIO INCLUDE DIRS ${MATIO_INCLUDE_DIRS}")
include_directories( ${MATIO_INCLUDE_DIRS} )
message("== MATIO LIB DIRS ${MATIO_LIBRARY_DIRS}")
link_directories( ${MATIO_LIBRARY_DIRS} )



pkg_check_modules(EBLEARN REQUIRED eblearn)
set( EBLEARN_LIBRARIES "-leblearn -lidx -leblearntools" )
message("== EBLEARN LIBS REQUIRED ${EBLEARN_LIBRARIES}" )
message("== EBLEARN INCLUDE DIRS ${EBLEARN_INCLUDE_DIRS}")
include_directories( ${EBLEARN_INCLUDE_DIRS} )
message("== EBLEARN LIB DIRS ${EBLEARN_LIBRARY_DIRS}")
link_directories( ${EBLEARN_LIBRARY_DIRS} )


find_package(PCL 1.7.0 REQUIRED)
if(PCL_FOUND)
    message("== PCL INCLUDE DIRS ${PCL_INCLUDE_DIRS}")
    message("== PCL LIBS REQUIRED ${PCL_LIBRARIES}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()


find_package(OpenCV 2.4 REQUIRED)
if(OpenCV_FOUND)
    message("== OPENCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
    message("== OPENCV LIBS REQUIRED ${OpenCV_LIBS}")
endif()

include_directories(${PROJECT_SOURCE_DIR})


set( KDES_CPP
  ${PROJECT_SOURCE_DIR}/src/kdes/libkdes.cpp
)

set( KDES_H
  ${PROJECT_SOURCE_DIR}/include/kdes/helpfun.hpp
  ${PROJECT_SOURCE_DIR}/include/kdes/libkdes.h
  ${PROJECT_SOURCE_DIR}/include/kdes/mat.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/mat4.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/mat5.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/mat73.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/matioConfig.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/matio.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/matio_private.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/matio_pubconf.h
  ${PROJECT_SOURCE_DIR}/include/kdes/matio/mattest.h
)

# build kdes library
set( KDES_LIB kdes )
add_library(${KDES_LIB} ${KDES_CPP} ${KDES_H} )
target_link_libraries( ${KDES_LIB} ${OpenCV_LIBS} ${Boost_LIBRARIES} ${MATIO_LIBRARIES} )


set( HELPFUN_H
  ${PROJECT_SOURCE_DIR}/include/helpfun/feature_detector.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/feature_cluster.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/feature_matcher.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/levmar_pose_estimator.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/mask_generator.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/projection_filter.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/svd_pose_estimator.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/utils.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/rgb_recogniser.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/rgbd_recogniser.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/eblearn_recogniser.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/kd_recogniser.h
  ${PROJECT_SOURCE_DIR}/include/helpfun/json_parser.hpp
)

set( HELPFUN_CPP
  ${PROJECT_SOURCE_DIR}/src/helpfun/feature_detector.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/feature_cluster.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/feature_matcher.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/levmar_pose_estimator.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/mask_generator.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/projection_filter.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/svd_pose_estimator.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/rgb_recogniser.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/rgbd_recogniser.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/eblearn_recogniser.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/kd_recogniser.cpp
  ${PROJECT_SOURCE_DIR}/src/helpfun/utils.cpp
)


# build helpfun library
set( HELPFUN_LIB helpfun )
add_library( ${HELPFUN_LIB} STATIC ${HELPFUN_H} ${HELPFUN_CPP} )
target_link_libraries( ${HELPFUN_LIB} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${EBLEARN_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/libANN.a
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/liblevmar.a
  ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/libsiftfast.a )

if (USE_CATKIN)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/uts_recogniser.cpp ${PROJECT_SOURCE_DIR}/include/uts_recogniser.h)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${HELPFUN_LIB} ${KDES_LIB} ${catkin_LIBRARIES})

add_executable(uts_init ${PROJECT_SOURCE_DIR}/src/uts_init.cpp )
target_link_libraries(uts_init ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BLAS_LIBRARIES} ${catkin_LIBRARIES})

add_executable( pseudo_request src/pseudo_request.cpp )
target_link_libraries(pseudo_request ${catkin_LIBRARIES})

add_executable( json_generator src/json_generator.cpp )

else()

rosbuild_add_boost_directories()

rosbuild_add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/uts_recogniser.cpp ${PROJECT_SOURCE_DIR}/include/uts_recogniser.h)
rosbuild_link_boost(${PROJECT_NAME} thread system filesystem program_options signals )
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BLAS_LIBRARIES})# ${LAPACK_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${HELPFUN_LIB} ${KDES_LIB} )

rosbuild_add_executable(uts_init ${PROJECT_SOURCE_DIR}/src/uts_init.cpp )
rosbuild_link_boost(uts_init thread system filesystem program_options signals )
target_link_libraries(uts_init ${PCL_LIBRARIES} ${OpenCV_LIBS} ${BLAS_LIBRARIES})


rosbuild_add_executable( pseudo_request src/pseudo_request.cpp )

rosbuild_add_executable( json_generator src/json_generator.cpp )

endif()

#rosbuild_add_executable(kd_test ${PROJECT_SOURCE_DIR}/src/kd_test.cpp)
#rosbuild_link_boost(kd_test thread system filesystem program_options signals )
#target_link_libraries(kd_test ${OpenCV_LIBS} ${BLAS_LIBRARIES})# ${LAPACK_LIBRARIES})
#target_link_libraries(kd_test ${HELPFUN_LIB} ${KDES_LIB} )

#rosbuild_add_executable(eblearn_test ${PROJECT_SOURCE_DIR}/src/eblearn_test.cpp)
#target_link_libraries(eblearn_test ${OpenCV_LIBS} ${EBLEARN_LIBRARIES} ${HELPFUN_LIB})
#rosbuild_link_boost(eblearn_test thread system filesystem program_options signals regex )


