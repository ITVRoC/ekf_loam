cmake_minimum_required(VERSION 2.8.3)
project(ekf_loam)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
  adaptive_filter  
)

find_package(Boost REQUIRED COMPONENTS 
  timer
  thread
  serialization
  chrono
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package (Eigen3 CONFIG)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  CATKIN_DEPENDS adaptive_filter
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
	${EIGEN3_LIBRARY_DIRS}
)

add_executable(PreTreatment src/PreTreatment.cpp)
#add_dependencies(PreTreatment ${catkin_EXPORTED_TARGETS} cloud_msgs)
target_link_libraries(PreTreatment ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(LiDAROdometry src/LiDAROdometry.cpp)
#add_dependencies(PreTreatment ${catkin_EXPORTED_TARGETS} cloud_msgs)
target_link_libraries(LiDAROdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(LiDARMapping src/LiDARMapping.cpp)
target_link_libraries(LiDARMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} gtsam)

add_executable(IntegrationTransform src/IntegrationTransform.cpp)
target_link_libraries(IntegrationTransform ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(TransformOdometry src/TransformOdometry.cpp)
target_link_libraries(TransformOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
