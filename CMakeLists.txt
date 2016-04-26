cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rtt_gazebo_embedded)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  )

find_package(gazebo REQUIRED)


if(${GAZEBO_VERSION} AND ${GAZEBO_VERSION} GREATER 6)
    add_definitions(-DGAZEBO_CREATER_6)
endif()


include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    )

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${USE_OROCOS_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

add_subdirectory(example)

orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS rtt_ros
)
