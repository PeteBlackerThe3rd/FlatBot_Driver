cmake_minimum_required(VERSION 2.8.3)
project(smart_fusion_sensor)

# Enable c++11 compiler
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  message_generation
  image_transport
  cv_bridge
  dp_ptu47_msgs
  dynamic_reconfigure
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  OpenCV
  gnc_tool_kit
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   #Progress.msg
   #Vertex.msg
   #Triangle.msg
   #ScanMesh.msg
 )

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   PerformScan.srv
   SetMapFix.srv
   RegisterLocalMap.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   dp_ptu47_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
#generate_dynamic_reconfigure_options(
#  cfg/cost_map_gen.cfg
#)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(

#  dp_ptu47_msgs_DIR /home/pete/ptu47_wss/build/dp_ptu47_msgs/catkin_generated/installspace

  INCLUDE_DIRS include
  LIBRARIES gncTK  #system_lib_LIBRARIES
  CATKIN_DEPENDS message_runtime roscpp rospy tf tf2 tf2_ros std_msgs sensor_msgs geometry_msgs dp_ptu47_msgs 
  
  DEPENDS #system_lib system_lib_LIBRARIES
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
#  /usr/include/opencv2
)

# -------------------------------------------------------
# The smart 2 rover fusion sensor node
# -------------------------------------------------------
add_executable(fusion_sensor
	src/sensor_test.cpp
	src/smart_rover_sensor.cpp 
)
target_link_libraries(fusion_sensor ${catkin_LIBRARIES} ${PCL_LIBRARIES} GL GLU GLEW glfw gncTK) 
add_dependencies(fusion_sensor smart_fusion_sensor_generate_messages_cpp)

# -------------------------------------------------------
# Node to subscribe to an image topic and re-publish
# either the left or right half of it
# (Used to extract the left camera image from ZED)
# -------------------------------------------------------
add_executable(half_image_pub
	src/half_image_pub.cpp 
)
target_link_libraries(half_image_pub ${catkin_LIBRARIES} ${PCL_LIBRARIES} GL GLU GLEW glfw gncTK) 
add_dependencies(half_image_pub smart_fusion_sensor_generate_messages_cpp)

# -------------------------------------------------------
# node to publish an image from a file at a user specified frequency
# -------------------------------------------------------
add_executable(publish_image_file
	src/publish_image.cpp)
target_link_libraries(publish_image_file ${catkin_LIBRARIES} ${PCL_LIBRARIES} )

# -------------------------------------------------------
# Node to publish an obj file as a marker message and
# custom textured message
# -------------------------------------------------------
add_executable(publish_mesh
	src/publish_mesh.cpp 
)
target_link_libraries(publish_mesh ${catkin_LIBRARIES} ${PCL_LIBRARIES} GL GLU GLEW glfw gncTK) 
add_dependencies(publish_mesh smart_fusion_sensor_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS 
		fusion_sensor
		half_image_pub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add test based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



