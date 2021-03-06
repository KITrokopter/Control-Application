cmake_minimum_required(VERSION 2.8.3)
project(control_application)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  rosconsole
  message_generation
)

include_directories(~/matlab/extern/include)

# Use C++ 11
# I'll just leave this here in case we want to use it
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CurrentPosition.msg
  quadcopter_movement.msg
  quadcopter_position.msg
  quadcopter_is_tracked.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  BuildFormation.srv
  CalculateCalibration.srv
  Rotation.srv
  SetQuadcopters.srv
  Shutdown.srv
  StartCalibration.srv
  TakeCalibrationPicture.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

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
#  LIBRARIES control_application
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(control_application
#   src/${PROJECT_NAME}/control_application.cpp
# )

set(MATLAB_PATH "~/matlab"
  CACHE PATH "Path to matlab installation")

# Adds the matlab path to the library search directories
# LINK_DIRECTORIES(${MATLAB_PATH}/bin/glnxa64)

# Skip the control application when matlab is not installed.
if(EXISTS ${MATLAB_PATH})
  ## Find all files for the position package
  file(GLOB_RECURSE POSITION_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/position/*.cpp)
  list(REMOVE_ITEM POSITION_SOURCES src/position/DummyPositionReceiver.cpp)
  list(REMOVE_ITEM POSITION_SOURCES src/position/tmpmain.cpp)


  ## Declare a cpp executable
  add_executable(control_application_node
  ${POSITION_SOURCES}
  
  src/main.cpp
  src/KitrokopterMessages.cpp
  src/matlab/TrackingMath.cpp 
  src/matlab/Matrix2x2.cpp
  src/matlab/Vector.cpp 
  src/matlab/Line.cpp
  src/matlab/Matrix.cpp
  src/matlab/profiling.cpp
  src/matlab/TrackingArea.cpp
  src/matlab/PositionCalculator.cpp
  src/matlab/AmccCalibration.cpp
  src/controller/Controller.cpp
  src/controller/Formation.cpp
  src/controller/MovementHelper.cpp
  src/controller/MovementQuadruple.cpp
  src/controller/Mutex.cpp
  src/controller/PControl.cpp
  src/controller/PDIControl.cpp
  src/controller/Position6DOF.cpp
  src/controller/QuadcopterControl.cpp
  src/controller/QuadcopterInfo.cpp
  src/controller/QuadcopterThrust.cpp
)

add_dependencies(control_application_node
  control_application_generate_messages_cpp
  api_application_generate_messages_cpp
  camera_application_generate_messages_cpp
  quadcopter_application_generate_messages_cpp
)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(control_application_node Control-Application_generate_messages_cpp)

# Use full path names to go around strange errors since matlab has some strange versions of some libs that are already included in ubuntu.
target_link_libraries(control_application_node
  ${catkin_LIBRARIES}
  ${MATLAB_PATH}/bin/glnxa64/libeng.so
  ${MATLAB_PATH}/bin/glnxa64/libmx.so
  ${MATLAB_PATH}/bin/glnxa64/libmat.so
  libopencv_core.so
  libopencv_highgui.so
  libopencv_calib3d.so
  libopencv_imgproc.so
)

#add_executable(control_application_test_calibration_node
#  ${POSITION_SOURCES}
#  
#  test/testMatlab.cpp
#  src/KitrokopterMessages.cpp
#  src/matlab/Matlab.cpp 
#  src/matlab/Vector.cpp 
#  src/matlab/Line.cpp
#  src/matlab/Matrix.cpp
#  src/matlab/Matrix2x2.cpp
#  src/matlab/profiling.cpp
#  src/matlab/TrackingArea.cpp
#  src/matlab/Position.cpp
#  src/matlab/AmccCalibration.cpp
#  src/matlab/profiling.cpp
#  src/controller/Controller.cpp
#  src/controller/Formation.cpp
#  src/controller/MovementHelper.cpp
#  src/controller/MovementQuadruple.cpp
#  src/controller/Mutex.cpp
#  src/controller/PControl.cpp
#  src/controller/PDIControl.cpp
#  src/controller/Position6DOF.cpp
#  src/controller/QuadcopterControl.cpp
#  src/controller/QuadcopterInfo.cpp
#  src/controller/QuadcopterThrust.cpp
#)

#add_dependencies(control_application_test_calibration_node
#  control_application_generate_messages_cpp
#  api_application_generate_messages_cpp
#  camera_application_generate_messages_cpp
#  quadcopter_application_generate_messages_cpp
#)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(control_application_node Control-Application_generate_messages_cpp)

# Use full path names to go around strange errors since matlab has some strange versions of some libs that are already included in ubuntu.
#target_link_libraries(control_application_test_calibration_node
#  ${catkin_LIBRARIES}
#  ${MATLAB_PATH}/bin/glnxa64/libeng.so
#  ${MATLAB_PATH}/bin/glnxa64/libmx.so
#  ${MATLAB_PATH}/bin/glnxa64/libmat.so
#  libopencv_core.so
#  libopencv_highgui.so
#  libopencv_calib3d.so
#  libopencv_imgproc.so
#)


#add_executable(profile_control
#  ${POSITION_SOURCES}
#  
#  test/ProfileControl.cpp
#  test/helpers/MovementGenerator.cpp
#  src/controller/Controller.cpp
#  src/controller/Formation.cpp
#  src/controller/MovementHelper.cpp
#  src/controller/MovementQuadruple.cpp
#  src/controller/Mutex.cpp
#  src/controller/PControl.cpp
#  src/controller/PDIControl.cpp
#  src/controller/Position6DOF.cpp
#  src/controller/QuadcopterControl.cpp
#  src/controller/QuadcopterInfo.cpp
#  src/controller/QuadcopterThrust.cpp
#  src/matlab/Matlab.cpp 
#  src/matlab/Vector.cpp 
#  src/matlab/Line.cpp
#  src/matlab/Matrix.cpp
#  src/matlab/Matrix2x2.cpp
#  src/matlab/profiling.cpp
#  src/matlab/TrackingArea.cpp
#  src/matlab/Position.cpp
#  src/matlab/AmccCalibration.cpp
#  src/KitrokopterMessages.cpp
#)

#add_dependencies(profile_control
#  control_application_generate_messages_cpp
#  api_application_generate_messages_cpp
#  camera_application_generate_messages_cpp
#  quadcopter_application_generate_messages_cpp
#)

#target_link_libraries(profile_control
#  ${catkin_LIBRARIES}
#  ${MATLAB_PATH}/bin/glnxa64/libeng.so
#  ${MATLAB_PATH}/bin/glnxa64/libmx.so
#  ${MATLAB_PATH}/bin/glnxa64/libmat.so
#  libopencv_core.so
#  libopencv_highgui.so
#  libopencv_calib3d.so
#  libopencv_imgproc.so
#)

add_executable(position_module
  ${POSITION_SOURCES}
  
  test/testPositionModule.cpp
  test/helpers/DummyPositionReceiver.cpp
  
  src/KitrokopterMessages.cpp
  src/matlab/Vector.cpp
  src/matlab/Line.cpp
  src/matlab/Matrix.cpp
  src/matlab/Matrix2x2.cpp
  src/matlab/TrackingMath.cpp
  src/matlab/PositionCalculator.cpp
  src/matlab/TrackingArea.cpp
  src/matlab/AmccCalibration.cpp
  src/matlab/profiling.cpp
  src/controller/Mutex.cpp
)

add_dependencies(position_module
  control_application_generate_messages_cpp
  api_application_generate_messages_cpp
  camera_application_generate_messages_cpp
  quadcopter_application_generate_messages_cpp
)

target_link_libraries(position_module
  ${catkin_LIBRARIES}
  ${MATLAB_PATH}/bin/glnxa64/libeng.so
  ${MATLAB_PATH}/bin/glnxa64/libmx.so
  ${MATLAB_PATH}/bin/glnxa64/libmat.so
  libopencv_core.so
  libopencv_highgui.so
  libopencv_calib3d.so
  libopencv_imgproc.so
)

#add_executable(test_camera_queues
#  test/testCameraQueues.cpp
#  test/helpers/profiling.cpp
#  
#  src/position/tracking/SynchronousCameraQueue.cpp
#  src/position/tracking/SynchronousCameraQueue_Group.cpp
#  src/position/tracking/AbstractCameraQueue.cpp
#  
#  src/matlab/Vector.cpp
#  src/matlab/Matrix.cpp
#)

#target_link_libraries(test_camera_queues
#  ${catkin_LIBRARIES}
#  ${MATLAB_PATH}/bin/glnxa64/libeng.so
#  ${MATLAB_PATH}/bin/glnxa64/libmx.so
#  ${MATLAB_PATH}/bin/glnxa64/libmat.so
#)

else()
  message(WARNING
    "Not building control_application_node and profile_control because MATLAB_PATH is invalid.")
endif()

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
# install(TARGETS control_application Control-Application_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_control_application.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
