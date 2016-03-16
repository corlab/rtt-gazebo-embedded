cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rtt_gazebo_embedded CXX)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  cmake_modules
  )

find_package(gazebo REQUIRED)
find_package(OROCOS-RTT REQUIRED)
find_package(Eigen REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    )
    
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

orocos_component(${PROJECT_NAME} rtt_gazebo_embedded.cc)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} pthread ${USE_OROCOS_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS rtt_ros
)
