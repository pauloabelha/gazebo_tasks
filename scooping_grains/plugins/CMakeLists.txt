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

link_directories(${GAZEBO_LIBRARY_DIRS} ${SDF_INCLUDE_DIRS} )

add_library(grain_collision SHARED grain_collision.cc)
target_link_libraries(grain_collision grain_sensor_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(grain_collision grain_sensor_msgs)

add_library(scoop SHARED scoop.cc)
target_link_libraries(scoop grain_sensor_msgs measure_scoop_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(scoop grain_sensor_msgs measure_scoop_msgs)

add_library(measurer_scooping_grains SHARED measurer_scooping_grains.cc)
target_link_libraries(measurer_scooping_grains measure_scoop_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(measurer_scooping_grains measure_scoop_msgs measure_scoop_msgs)

add_library(factory_grains SHARED factory_grains.cc)
target_link_libraries(factory_grains ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


