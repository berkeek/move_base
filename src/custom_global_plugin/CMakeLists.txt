cmake_minimum_required(VERSION 3.0.2)
project(custom_global_plugin)

# 1) We usually need C++11 or newer
add_compile_options(-std=c++11)

# 2) Find the required packages for a global planner plugin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

# 3) Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 4) Create the library target from your planner source file(s)
#    For example, if your global planner is in src/simple_global_planner.cpp
add_library(simple_global_planner
  src/simple_global_planner.cpp
)

# 5) Link the library against catkin libraries
target_link_libraries(simple_global_planner
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# 6) Install the library into lib/ so pluginlib can find it
install(TARGETS simple_global_planner
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# 7) Install your plugin XML file so pluginlib can find it
#    Adjust the filename if yours differs. For example:
install(FILES simple_global_planner_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
