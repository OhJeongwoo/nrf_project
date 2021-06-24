cmake_minimum_required(VERSION 2.8.3)
project(sensor_decoder)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  rosbag
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  cv_bridge
  message_filters
  tf
  tf2
  inertiallabs_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)
find_package(PCL 1.8 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES 
  LaneMsg.msg ObstacleMsg.msg PCAN.msg PCANArray.msg Mobileye.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs
	DEPENDS Eigen
	opencv_highgui opencv_imgproc
)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
 )

#add_library({LIB_NAME} lib/{LIB_NAME}.cpp)

set (SOURCES
 #${CMAKE_CURRENT_SOURCE_DIR}/lib/{LIB_NAME}.cpp
)

set (HEADERS
 #${CMAKE_CURRENT_SOURCE_DIR}/include/{LIB_NAME}.h
)

add_executable(pcan_reader src/pcan_reader.cpp ${HEADERS} ${SOURCES})
target_link_libraries(pcan_reader ${catkin_LIBRARIES})
add_dependencies(pcan_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(local_map_publisher src/local_map_publisher.cpp ${HEADERS} ${SOURCES})
target_link_libraries(local_map_publisher ${catkin_LIBRARIES} ${CV_LIBRARIES})
add_dependencies(local_map_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_image_save src/test_image_save.cpp ${HEADERS} ${SOURCES})
target_link_libraries(test_image_save ${catkin_LIBRARIES} ${CV_LIBRARIES})
add_dependencies(test_image_save ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(driving_video_generator src/driving_video_generator.cpp ${HEADERS} ${SOURCES})
target_link_libraries(driving_video_generator ${catkin_LIBRARIES} ${CV_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(driving_video_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(video_recorder src/video_recorder.cpp ${HEADERS} ${SOURCES})
target_link_libraries(video_recorder ${catkin_LIBRARIES} ${CV_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(video_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(pc_collector src/pc_collector.cpp ${HEADERS} ${SOURCES})
target_link_libraries(pc_collector ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pc_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(decision_manual src/decision_manual.cpp ${HEADERS} ${SOURCES})
target_link_libraries(decision_manual ${catkin_LIBRARIES})
add_dependencies(decision_manual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(pcan_topic_generator src/pcan_topic_generator.cpp ${HEADERS} ${SOURCES})
#target_link_libraries(pcan_topic_generator ${catkin_LIBRARIES})
#add_dependencies(pcan_topic_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/pcan_decoder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


