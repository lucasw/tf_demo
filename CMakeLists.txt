cmake_minimum_required(VERSION 3.0.2)
project(tf_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ddynamic_reconfigure
  geometry_msgs
  roscpp
  roslint
  rospy
  tf2_ros
  visualization_msgs
)

set(ROSLINT_CPP_OPTS "--extensions=cpp,h,hpp" "--filter=-whitespace/braces")
roslint_cpp()

roslint_python()
## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
#  INCLUDE_DIRS include
#  LIBRARIES rqt_dr_single
  CATKIN_DEPENDS
  ddynamic_reconfigure
  geometry_msgs
  roscpp
  tf2_ros
  visualization_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(old_tf_to_new_tf
  src/old_tf_to_new_tf.cpp
)

target_link_libraries(old_tf_to_new_tf ${catkin_LIBRARIES})

install(TARGETS
  old_tf_to_new_tf
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
   scripts/buffer_core_demo.py
   scripts/dueling_transforms.py
   scripts/flip_flop.py
   scripts/flyer.py
   scripts/old_tf_to_new_tf.py
   scripts/tf_path.py
   scripts/tracking_arm.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
