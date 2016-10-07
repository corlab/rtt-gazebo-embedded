# RTT Gazebo Embedded Component

Simple [Gazebo stand alone](https://bitbucket.org/osrf/gazebo/src/d3b06088be22a15a25025a952414bffb8ff6aa2b/examples/stand_alone/custom_main/?at=default) object wrapped in an orocos component.

## Design

A gazebo instance is launched with the ```RTTGazeboEmbedded``` component, which allows **any** components living in the same deployer to access the gazebo API (get world, model, time etc).

In your own TaskContext, you can (for example) bind those two functions :

```cpp
gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MyModel::WorldUpdateBegin,this));

gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&MyModel::WorldUpdateEnd,this));
```

And you'll get updates at each gazebo timestep. Take a look at the ```example/``` component for a demo.

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
gazebo.argv = strings("--verbose")
# Load System Plugin for Clock-Sync
gazebo.add_plugin("libRTTGazeboClockPlugin.so")

gazebo.configure()

gazebo.start()

gazebo.toggleDynamicsSimulation(false)

# Loading the model
gazebo.spawn_model("kuka-lwr", "model://kuka-lwr-4plus", 10)

import("rtt-gazebo-lwr4plus-sim")
loadComponent("lwr_gazebo","lwr::LWR4plusSim")
setActivity("lwr_gazebo",0,11,ORO_SCHED_OTHER)
lwr_gazebo.misc.setUrdfPath(".../model.urdf")

lwr_gazebo.getModel("","kuka-lwr",4)
lwr_gazebo.configure()
gazebo.toggleDynamicsSimulation(true)

```
