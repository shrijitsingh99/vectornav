cmake_minimum_required(VERSION 2.8.3)
project(vectornav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vectornav
  CATKIN_DEPENDS roscpp tf2 sensor_msgs geometry_msgs
)

add_subdirectory(vnproglib-1.1.5.0/cpp)
include_directories(vnproglib-1.1.5.0/cpp/include include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
       src/vectornav_interface.cpp
       src/UTM.cpp include/vectornav/UTM.h)

add_dependencies(${PROJECT_NAME} libvncxx  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/vectornav_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
       ${PROJECT_NAME}
       libvncxx
 ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
