# RTT Gazebo Embedded Component

Simple [Gazebo stand alone](https://bitbucket.org/osrf/gazebo/src/d3b06088be22a15a25025a952414bffb8ff6aa2b/examples/stand_alone/custom_main/?at=default) object wrapped in an orocos component.

## Design

A gazebo instance is launch with the RTTGazeboEmbedded component, which provides
a method to get the pointer to a specific model.

Then you just have to create an orocos TaskContext and bind those two functions :

```
gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MyModel::WorldUpdateBegin,this));

gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&MyModel::WorldUpdateEnd,this));
```


## Tutorial

You can launch the demo in examples/ : ```cd examples; ./gz_test.ops```

```ruby
#!/usr/bin/env deployer

import("rtt_gazebo_embedded")

# Loading gazebo (this will start rosnode)
loadComponent("gazebo","RTTGazeboEmbedded")

# WorldUpdateBegin and End will be called by gazebo
setActivity("gazebo",0,10,ORO_SCHED_OTHER)

# This is optional
gazebo.argv = strings("--verbose","--record_encoding=zlib")

# Load the world file (ex pr2 model, but it can be "worlds/empty.world" also)
gazebo.world_path = "/usr/share/gazebo-7/worlds/pr2.world"

# Load the ROS plugins
gazebo.add_plugin("libgazebo_ros_paths_plugin.so")
gazebo.add_plugin("libgazebo_ros_api_plugin.so")


gazebo.configure()

gazebo.start()

# If your model is not already loaded, you can spawn it like this :
# rosrun gazebo_ros spawn_model -param robot_description -urdf -model my_model


# Loading the model interface

import("rtt_gazebo_embedded")

# The component is also aperiodic,
# WorldUpdateBegin and End will be called by gazebo

loadComponent("pr2","ModelPluginExample")
connectPeers("pr2","gazebo")

# Get the gazebo model, timeout 10s
# By default, the name of the orocos component
# should be the name of the loaded model

pr2.configure()

pr2.start()

```
