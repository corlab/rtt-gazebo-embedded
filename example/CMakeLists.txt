
orocos_component(model_plugin_example model_plugin_example.cpp)
target_link_libraries(model_plugin_example 
    ${GAZEBO_LIBRARIES} 
    ${CATKIN_LIBRARIES} 
    ${USE_OROCOS_LIBRARIES}
)