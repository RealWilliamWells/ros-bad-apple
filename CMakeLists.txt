cmake_minimum_required(VERSION 3.5)
project(bad_apple_publisher)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bad_apple_publisher
  src/main.cpp
)

target_link_libraries(bad_apple_publisher
  ${catkin_LIBRARIES}
)
