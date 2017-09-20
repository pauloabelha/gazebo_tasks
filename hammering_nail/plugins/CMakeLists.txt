cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
find_package(gazebo REQUIRED)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )

link_directories(${GAZEBO_LIBRARY_DIRS} 
    ${SDF_INCLUDE_DIRS} 
    ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(nail_collision SHARED nail_collision.cc)
target_link_libraries(nail_collision nail_sensor_msgs ${Boost_LIBRARIES} 
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(nail_collision nail_sensor_msgs)

add_library(hammer SHARED hammer.cc)
target_link_libraries(hammer nail_sensor_msgs nail_distance_msgs ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(hammer nail_sensor_msgs nail_distance_msgs)

add_library(measure_nail_distance SHARED measure_nail_distance.cc)
target_link_libraries(measure_nail_distance nail_distance_msgs ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(measure_nail_distance nail_distance_msgs)
