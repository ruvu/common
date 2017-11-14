# RUVU Gazebo Plugins

This package holds several Gazebo plugins that make modification to the world
entities in the simulation environment or expose data to the outside world.
The plugins can be included by referring to the plugin in the Gazebo world file
or in the URDF description. Examples can be found in the
[test directory](/test).

    <plugin name="[NAME]" filename="lib[NAME].so">

## Plugins

The following plugins are present within this package:

### ruvu_disable_physics (world plugin)

This plugin disables the physics engine when it is on. Often we are not
interested in exact physics simulation, it is cheaper (computationally) to
ignore the physics and thus the collision engine. This gives us full control
over the state-evolution in simulation. To test the plugin, run the following command:

    roslaunch ruvu_gazebo_plugins disable_physics_world.launch

### ruvu_twist_teleport (model plugin)

This plugin teleports entity in the scene base on a [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) command and publishes an [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html).
This plugin assumes that the physics are disabled and assumes infinite
acceleration. To test the plugin with use of a gazebo world file, run the following command:

    roslaunch ruvu_gazebo_plugins twist_teleport_world.launch

To test the plugin with use of XACRO URDF models, run the following command

    roslaunch ruvu_gazebo_plugins twist_teleport_urdf_world.launch
