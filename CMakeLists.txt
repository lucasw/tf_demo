cmake_minimum_required(VERSION 2.8.3)
project(tf_demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roslint
  rospy
  tf2_ros
)

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
#  CATKIN_DEPENDS rospy rqt_gui_py
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rqt_dr_single
#   src/${PROJECT_NAME}/rqt_dr_single.cpp
# )

## Declare a cpp executable
# add_executable(rqt_dr_single_node src/rqt_dr_single_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rqt_dr_single ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
# target_link_libraries(rqt_dr_single_node
#   ${catkin_LIBRARIES}
# )

install(PROGRAMS
   scripts/old_tf_to_new_tf.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
