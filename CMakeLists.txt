cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rtt_gazebo_embedded)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS
  rtt_ros
  )

find_package(gazebo REQUIRED)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    )

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${USE_OROCOS_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

orocos_install_headers(FILE include/${PROJECT_NAME}/${PROJECT_NAME}.hh)

add_subdirectory(example)

orocos_generate_package(
    INCLUDE_DIRS include
    DEPENDS rtt_ros
)

