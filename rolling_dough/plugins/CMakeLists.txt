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

add_library(factory_dough SHARED factory_dough.cc)
target_link_libraries(factory_dough ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(dough_collision SHARED dough_collision.cc)
target_link_libraries(dough_collision dough_sensor_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(dough_collision dough_sensor_msgs)

add_library(roll SHARED roll.cc)
target_link_libraries(roll dough_variance_msgs dough_sensor_msgs dough_stuck_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(roll dough_variance_msgs dough_sensor_msgs dough_stuck_msgs )

add_library(measure_dough_variance SHARED measure_dough_variance.cc)
target_link_libraries(measure_dough_variance dough_variance_msgs dough_stuck_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} bestfit)
add_dependencies(measure_dough_variance dough_variance_msgs dough_stuck_msgs bestfit)

add_library(end_collision SHARED end_collision.cc)
target_link_libraries(end_collision end_sensor_msgs ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(end_collision end_sensor_msgs)
