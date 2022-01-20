cmake_minimum_required(VERSION 3.0.2)
project(bmb_utilities)

# Compile as C++20, supported in ROS Kinetic and newer
add_compile_options(-std=c++20)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  COMPONENTS
    actionlib
    control_msgs
    controller_manager_msgs
    roscpp
    roslib
    std_msgs
    std_srvs
    trajectory_msgs
    trexo_msgs
    trexo_utilities
)

find_package(Boost REQUIRED)

catkin_python_setup()

# Declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    trajectory_planner
    trexo_leg
    gait_generator
  CATKIN_DEPENDS
    actionlib
    control_msgs
    controller_manager_msgs
    roscpp
    roslib
    std_msgs
    std_srvs
    trajectory_msgs
    trexo_msgs
    trexo_utilities
  DEPENDS
    Boost
)


###########
## Build ##
###########

# Specify header include paths
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# Trajectory Planner Interface
add_library(trajectory_planner src/trajectory_planner_interface.cpp)
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})
add_dependencies(trajectory_planner ${catkin_EXPORTED_TARGETS})

# Trexo leg
add_library(trexo_leg src/trexo_leg.cpp)
target_link_libraries(trexo_leg ${catkin_LIBRARIES})
add_dependencies(trexo_leg ${catkin_EXPORTED_TARGETS})

# Gait generator
add_library(gait_generator src/gait_generator.cpp)
target_link_libraries(gait_generator trexo_leg ${catkin_LIBRARIES})
add_dependencies(gait_generator ${catkin_EXPORTED_TARGETS})

# Planner node
add_executable(basic_trajectory_planner src/basic_trajectory_planner.cpp)
target_link_libraries(basic_trajectory_planner
  trajectory_planner
  gait_generator
  trexo_leg
  ${catkin_LIBRARIES}
)
add_dependencies(basic_trajectory_planner ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install libraries
install(TARGETS gait_generator
                trajectory_planner
                trexo_leg
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install executables
install(TARGETS basic_trajectory_planner
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install python scripts
# Note: These scripts are for debugging only, and thus their dependencies are not installed through ROS/catkin
catkin_install_python(PROGRAMS scripts/gait_splinefit
                               scripts/send_traj
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install extras
install(DIRECTORY config
                  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
