find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
	if(ITR MATCHES ".*gazebo-[0-9.]+$")
		set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
	endif()
endforeach()

set (msgs
  grain_sensor.proto
  measure_scoop.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})

add_library(grain_sensor_msgs SHARED ${PROTO_SRCS})
target_link_libraries(grain_sensor_msgs ${PROTOBUF_LIBRARY})

add_library(measure_scoop_msgs SHARED ${PROTO_SRCS})
target_link_libraries(measure_scoop_msgs ${PROTOBUF_LIBRARY})
