cmake_minimum_required(VERSION 3.0.2)
project(camera_lidar_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  avs_lecture_msgs
  darknet_ros_msgs
  darknet_ros 
  message_generation
  message_filters
  avs_lecture_msgs
  visualization_msgs
)

find_package(OpenCV REQUIRED)
catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/CameraLidarFusion.cfg
  cfg/Homework4.cfg
)

add_message_files(
  FILES 
  FusedObjectArray.msg
  FusedObject.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  DEPENDS OpenCV
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
#  src/camera_lidar_fusion.cpp
#  src/CameraLidarFusion.cpp
 src/synced_yolo_data.cpp
 src/SyncedYoloData.cpp
 #src/homework4_node.cpp
 #src/Homework4.cpp
 #src/ObjectEkf.cpp

)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

#add_executable(synced_yolo_data
#  src/synced_yolo_data.cpp
#  src/SyncedYoloData.cpp
#)
#add_dependencies(synced_yolo_data
#  ${catkin_EXPORTED_TARGETS}
#)
#target_link_libraries(synced_yolo_data
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)

add_executable(homework4
 src/homework4_node.cpp
 src/Homework4.cpp
 src/ObjectEkf.cpp
)
add_dependencies(homework4
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)
target_link_libraries(homework4
  ${catkin_LIBRARIES}
)


catkin_install_python(
  PROGRAMS
    src/${PROJECT_NAME}/pub_camera_info.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
