cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(rs_contact_sensor)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(contact SHARED ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_LIBRARIES})

