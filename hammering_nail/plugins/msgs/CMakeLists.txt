find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs
  nail_sensor.proto
  nail_distance.proto
  check_time.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(nail_sensor_msgs SHARED ${PROTO_SRCS})
target_link_libraries(nail_sensor_msgs ${PROTOBUF_LIBRARY})

add_library(nail_distance_msgs SHARED ${PROTO_SRCS})
target_link_libraries(nail_distance_msgs ${PROTOBUF_LIBRARY})

add_library(check_time_msgs SHARED ${PROTO_SRCS})
target_link_libraries(check_time_msgs ${PROTOBUF_LIBRARY})
