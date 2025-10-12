cmake_minimum_required(VERSION 3.16)
project(gz_throttle_tool)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-transport13 REQUIRED CONFIG)
find_package(gz-msgs10     REQUIRED CONFIG)

add_executable(gz_throttle src/throttle_relay.cc)
target_link_libraries(gz_throttle
  gz-transport13::core
  gz-msgs10::core)


