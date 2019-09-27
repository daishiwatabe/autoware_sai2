cmake_minimum_required(VERSION 2.8.3)
project(microbus_interface)

# ビルドディレクトリを include ディレクトリに追加するオプションを有効にする。
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# MOC が必要な場合は自動的に実行するオプションを有効にする。
set(CMAKE_AUTOMOC ON)

# UI 定義ファイルの変換が必要な場合は自動的に実行するオプションを有効にする。
set(CMAKE_AUTOUIC ON)

# QtWidgets ライブラリを追加する。
find_package(Qt5Widgets REQUIRED)

find_package(autoware_build_flags REQUIRED)

find_package(autoware_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_msgs
  autoware_can_msgs
  autoware_config_msgs
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  autoware_can_msgs
  autoware_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tablet_socket_msgs_INCLUDE_DIRS}
  ${autoware_msgs_INCLUDE_DIRS}
  /usr/local/include/zmp/hev/
  src
)

add_executable(microbus_interface src/main.cpp src/mainwindow.cpp src/mainwindow.ui)
target_link_libraries(microbus_interface ${catkin_LIBRARIES} Qt5::Widgets)
add_dependencies(microbus_interface
  ${catkin_EXPORTED_TARGETS}
  )

install(TARGETS microbus_interface
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
		launch/microbus_interface.launch
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
		)
