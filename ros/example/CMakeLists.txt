cmake_minimum_required(VERSION 2.8.3)
project(trail)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #catkin_make
  roscpp
  geometry_msgs
  tf
)
# find_package(Qt5  REQUIRED COMPONENTS  Widgets  )

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES
  src/talker.cpp
  src/listener.cpp
  src/carrot_broadcast.cpp
)

catkin_package(
  CATKIN_DEPENDS roscpp
)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

#  add_executable(app_node ${SOURCES} ${FORMS} )
#  target_link_libraries(app_node  Qt5::Widgets ${catkin_LIBRARIES})

#  add_executable(hello_node  src/hello.cpp)
#  target_link_libraries(hello_node  ${catkin_LIBRARIES})

 add_executable(tf_broadcaster src/talker.cpp)
 add_executable(tf_listener src/listener.cpp)
 target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
 target_link_libraries(tf_listener ${catkin_LIBRARIES})
 add_executable(frame_tf_broadcaster src/carrot_broadcast.cpp)
 target_link_libraries(frame_tf_broadcaster ${catkin_LIBRARIES})

