cmake_minimum_required(VERSION 2.8.3)
project(rosoclingo)

find_package(catkin REQUIRED COMPONENTS rospy actionlib_msgs message_generation)

######################################
# Test for the gringo python module This works for checking if
# gringo.so is present. However it does not work for checking for a
# particular version of the module. This could be added but would
# require the gringo python module to have a .__version__ attribute.

function(find_python_module module)
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for binary modules
    # it's a .so file.
    execute_process(COMMAND "python" "-c" 
      "import re, ${module}; print re.compile('/__init__.py.*').sub('',${module}.__file__)"
      RESULT_VARIABLE _${module}_status 
      OUTPUT_VARIABLE _${module}_location
      ERROR_QUIET 
      OUTPUT_STRIP_TRAILING_WHITESPACE)
   
    # Note: if the variable _${module}_status is 0 then succeeded, hence we test
    # if NOT true below to test if it succeeded.
    if(NOT _${module}_status)
      set(PY_${module_upper} ${_${module}_location} CACHE STRING 
	"Location of Python module ${module}")
      find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
    else(NOT _${module}_status)
      if(${module}_FIND_REQUIRED)
	message(FATAL_ERROR "Failed to find python module ${module}")
      endif()
    endif(NOT _${module}_status)
  endif(NOT PY_${module_upper})
endfunction(find_python_module)

find_python_module(gringo REQUIRED)

######################################
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(FILES
  ROSoClingoIn.msg
  ROSoClingoOut.msg
)

## Generate the action lib messages
add_action_files(
  DIRECTORY action
  FILES ROSoClingo.action
)

## Declare the service files to be built
add_service_files(FILES
    GetGoalInformation.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    actionlib_msgs
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
   CATKIN_DEPENDS actionlib_msgs message_runtime
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

install(PROGRAMS
        nodes/run-1.0.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
