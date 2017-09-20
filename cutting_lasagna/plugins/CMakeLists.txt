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


add_library(cut SHARED cut.cc)
target_link_libraries(cut lasagna_sensor_msgs measure_cut_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(cut lasagna_sensor_msgs measure_cut_msgs)

add_library(lasagna_collision SHARED lasagna_collision.cc)
target_link_libraries(lasagna_collision lasagna_sensor_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(lasagna_collision lasagna_sensor_msgs)

add_library(measurer SHARED measurer.cc)
target_link_libraries(measurer measure_cut_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(measurer measure_pancake_msgs measure_cut_msgs)

add_library(factory_lasagna SHARED factory_lasagna.cc)


