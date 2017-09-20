find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  pancake_sensor.proto
  measure_pancake.proto
  pancake_stuck.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(pancake_sensor_msgs SHARED ${PROTO_SRCS})
target_link_libraries(pancake_sensor_msgs ${PROTOBUF_LIBRARY})

add_library(measure_pancake_msgs SHARED ${PROTO_SRCS})
target_link_libraries(measure_pancake_msgs ${PROTOBUF_LIBRARY})

add_library(pancake_stuck_msgs SHARED ${PROTO_SRCS})
target_link_libraries(pancake_stuck_msgs ${PROTOBUF_LIBRARY})