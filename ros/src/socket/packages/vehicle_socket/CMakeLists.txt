cmake_minimum_required(VERSION 2.8.3)
project(vehicle_socket)


find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(tablet_socket_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  autoware_msgs
  autoware_can_msgs
  autoware_config_msgs
  tablet_socket_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  autoware_can_msgs
  autoware_msgs
  tablet_socket_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tablet_socket_msgs_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
)

add_executable(vehicle_receiver nodes/vehicle_receiver/vehicle_receiver.cpp)
target_link_libraries(vehicle_receiver ${catkin_LIBRARIES})
add_dependencies(vehicle_receiver
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(vehicle_sender nodes/vehicle_sender/vehicle_sender.cpp)
target_link_libraries(vehicle_sender ${catkin_LIBRARIES})
add_dependencies(vehicle_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(bus_socket
	nodes/bus_socket/bus_socket.cpp
	nodes/bus_socket/canif.cpp
	nodes/bus_socket/estima.cpp)
target_link_libraries(bus_socket ${catkin_LIBRARIES})
add_dependencies(bus_socket
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_can_receiver
	nodes/kvaser_can_receiver/kvaser_can_receiver.cpp)
target_link_libraries(kvaser_can_receiver
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_can_receiver
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_can_sender
	nodes/kvaser_can_sender/kvaser_can_sender.cpp)
target_link_libraries(kvaser_can_sender
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_sender
	nodes/kvaser_microbus_can_sender/kvaser_microbus_can_sender.cpp)
target_link_libraries(kvaser_microbus_can_sender
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_sender
  ${catkin_EXPORTED_TARGETS}
  )

add_executable(kvaser_microbus_can_receiver
	nodes/kvaser_microbus_can_receiver/kvaser_microbus_can_receiver.cpp)
target_link_libraries(kvaser_microbus_can_receiver
  ${catkin_LIBRARIES}
  canlib)
add_dependencies(kvaser_microbus_can_receiver
  ${catkin_EXPORTED_TARGETS}
  )

catkin_install_python(PROGRAMS
        nodes/microbus_can_topic_sender/microbus_can_topic_sender.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS vehicle_sender vehicle_receiver bus_socket kvaser_can_receiver kvaser_can_sender
                kvaser_microbus_can_sender kvaser_microbus_can_receiver
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
		scripts/bus_socket.launch
		scripts/kvaser_can_receiver.launch
		scripts/kvaser_can_sender.launch
        scripts/kvaser_microbus_can_sender.launch
        scripts/kvaser_microbus_can_receiver.launch
        scripts/kvaser_microbus.launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
		)
