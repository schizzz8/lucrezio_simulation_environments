# lucrezio_simulation_environments

### Description

This package includes *config* and *source* files to manage `Gazebo` simulations, namely: robots, plugins, objects and worlds.

### Robots

Robot models, compliant to [this](http://wiki.ros.org/urdf/Tutorials) format, are specified in the `urdf` folder through the following files:

* **lucrezio.urdf.xacro:** a *diff-drive* mobile platform equipped with 2D laser scanner and RGB-D camera

* **lucrezio_with_logical.urdf.xacro:** a *diff-drive* mobile platform equipped with 2D laser scanner, RGB-D camera, and logical camera (see below).

#### Usage

To test the above mentioned robot models, run:

	roslaunch lucrezio_simulation_environments robot.launch
	
this should launch a `Gazebo` empty world and spawn the robot model in it:

![robot_launch](https://github.com/schizzz8/lucrezio_simulation_environments/blob/master/pics/robot_launch.png  "robot_launch")

To check that the mobile platform and the sensors are working properly, run:

	roslaunch lucrezio_simulation_environments display.launch
	
this launches `RViz`, with several displays:

![display_launch](https://github.com/schizzz8/lucrezio_simulation_environments/blob/master/pics/display_launch.png  "display_launch")

### Plugins

The robot is equipped with a logical camera, for a detailed description see:

* http://gazebosim.org/tutorials?tut=logical_camera_sensor&cat=sensors

The major advantage of this sensor is that it can be used as *ground truth* for object detection.

### Objects

Objects are 3D models, compliant to [this](http://gazebosim.org/tutorials?tut=model_structure&cat=) specification, are stored in the `models` folder.

They can be built from CAD models freely available on the [Google 3D Warehouse](https://3dwarehouse.sketchup.com/?hl=it), by following the procedure explained in this tutorial:

* https://www.youtube.com/watch?v=PDIaUWenu0Q&t=46s

### Worlds

Gazebo [world files](http://gazebosim.org/tutorials?tut=build_world) are config files where it's possible to store different aspects of a robotic simulation. 

To check that everything is working properly, run

	roslaunch lucrezio_simulation_environments empty_world_with_apartment_and_robot.launch
	
you should see the robot in an indoor environment (built for experiments):

![simulation](https://github.com/schizzz8/lucrezio_simulation_environments/blob/master/pics/simulation.png  "simulation")