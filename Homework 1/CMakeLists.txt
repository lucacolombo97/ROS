cmake_minimum_required(VERSION 2.8.3)
project(project1)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs tf message_generation dynamic_reconfigure)

add_service_files(
  FILES
  ComputeDistance.srv
)

add_message_files(
  FILES
  Flag.msg
)

generate_dynamic_reconfigure_options(
  cfg/thresholds_param.cfg
)

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub_sub src/pub_sub.cpp)
target_link_libraries(pub_sub ${catkin_LIBRARIES})

add_executable(compute_distance src/compute_distance.cpp)
target_link_libraries(compute_distance ${catkin_LIBRARIES})
add_dependencies(compute_distance ${catkin_EXPORTED_TARGETS})

add_executable(custom_msg src/custom_msg.cpp)
target_link_libraries(custom_msg ${catkin_LIBRARIES})
add_dependencies(custom_msg ${catkin_EXPORTED_TARGETS})
