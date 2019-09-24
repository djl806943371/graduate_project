cmake_minimum_required(VERSION 2.8.3)
project(trail)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #catkin_make
  roscpp
  rospy
  std_msgs
  geometry_msgs
)
find_package(Qt5  REQUIRED COMPONENTS  Widgets  )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/mainwindow.cpp
  src/qnode.cpp
  src/main.cpp
)

set(FORMS
  src/mainwindow.ui
)

catkin_package(
)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

 add_executable(app_node ${SOURCES} ${FORMS} )
 target_link_libraries(app_node  Qt5::Widgets ${catkin_LIBRARIES})

 add_executable(hello_node  src/hello.cpp)
 target_link_libraries(hello_node  ${catkin_LIBRARIES})



