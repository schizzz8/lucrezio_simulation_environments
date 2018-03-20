# lucrezio_simulation_environments

### Description

This package includes *config* and *source* files to manage `Gazebo` simulations, namely: robots, plugins, objects and worlds.

### Robots

Robot models, compliant with [this](http://wiki.ros.org/urdf/Tutorials) format, are specified in the `urdf` folder through the following files:

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
