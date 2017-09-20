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

add_library(float_math SHARED float_math.cpp)
add_library(bestfit SHARED bestfit.cpp)
add_dependencies(bestfit float_math)

add_library(factory_pancake SHARED factory_pancake.cc)
target_link_libraries(factory_pancake ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(pancake_collision SHARED pancake_collision.cc)
target_link_libraries(pancake_collision pancake_sensor_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES})
add_dependencies(pancake_collision pancake_sensor_msgs)

add_library(lift SHARED lift.cc)
target_link_libraries(lift pancake_sensor_msgs measure_pancake_msgs pancake_stuck_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(lift pancake_sensor_msgs measure_pancake_msgs pancake_stuck_msgs)

add_library(measure_pancake_particles SHARED measure_pancake_particles.cc)
target_link_libraries(measure_pancake_particles measure_pancake_msgs pancake_stuck_msgs ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} bestfit)
add_dependencies(measure_pancake_particles measure_pancake_msgs pancake_stuck_msgs bestfit)
