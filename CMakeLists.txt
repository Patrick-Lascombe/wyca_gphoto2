cmake_minimum_required(VERSION 2.8.3)
project(wyca_gphoto2)

find_package(catkin REQUIRED genmsg actionlib_msgs COMPONENTS
  message_generation
  roscpp
  self_test
  sensor_msgs
  std_msgs
  std_srvs
  actionlib_msgs
  actionlib
)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(gphoto2 REQUIRED libgphoto2)

add_service_files(FILES
  Capture.srv
  GetConfig.srv
  SetConfig.srv
  DownloadPictures.srv
  GetPicturePathList.srv
  DeletePictures.srv
)

add_action_files(DIRECTORY action
  FILES
  Trigger.action
  SetFocus.action
)

generate_messages(DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wyca_gphoto2
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs self_test actionlib_msgs
  DEPENDS OpenCV gphoto2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${gphoto2_INCLUDE_DIRS}
)

add_executable(photo_node src/wyca_gphoto2.cpp)
#add_executable(photo_node_sim src/photo_node/photo_node_sim.cpp)

add_dependencies(photo_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(photo_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(photo_node
  usb-1.0
  ${catkin_LIBRARIES}
  ${gphoto2_LIBRARIES}
)

#target_link_libraries(photo_node_sim
#  gphoto2_ros
#  usb-1.0
#  ${catkin_LIBRARIES}
#  ${gphoto2_LIBRARIES}
#)

install(TARGETS photo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
