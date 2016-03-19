# RTT Gazebo Component

Simple [Gazebo stand alone](https://bitbucket.org/osrf/gazebo/src/d3b06088be22a15a25025a952414bffb8ff6aa2b/examples/stand_alone/custom_main/?at=default) object wrapped in an orocos component.

It is different from [rtt_gazebo](https://github.com/jhu-lcsr/rtt_gazebo) in design and fonctionnalities : 

* `rtt_gazebo_embedded` is just a simple orocos components, which wrapps gazebo as a stand alone library. `rtt_gazebo` loads and orocos deployer as a world plugin. 
An orocos component is then loaded inside this gazebo plugin. 
This is very nice, but you can't have access directly to the deployer's console unless your launch a CORBA interface, which in the end creates a big overhead when dealing with [custom types](https://github.com/orocos/rtt_ros_integration/pull/51). 
The goal of this component is to avoid this overhead and facilitate `syncing` gazebo with other orocos components.

* `rtt_gazebo_embedded` only supports **one** robot, whereas `rtt_gazebo` might supports several.
The main adantage of this package over `rtt_gazebo` is that you don't need to bother with the CORBA interface and see your robot as a standard rtt component.

* The component loads the ROS api, then waits at the configure step for a model to be spawned. `roslaunch rtt_lwr_sim spawn_robot robot_name:=lwr_sim tip_link:=ati_link`

* Then gazebo runs forever in a `separate thread`, so we can stop and start the component (using conman for example).

* Reading/Writing the model's joints and orocos ports is done using [gazebo event interface](https://github.com/ahoarau/rtt_gazebo_embedded/blob/master/rtt_gazebo_embedded.cc#L105:L106).

