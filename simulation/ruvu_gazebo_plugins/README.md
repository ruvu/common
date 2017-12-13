# RUVU Gazebo Plugins

This package holds several Gazebo plugins that make modification to the world
entities in the simulation environment or expose data to the outside world.
The plugins can be included by referring to the plugin in the Gazebo world file
or in the URDF description. Examples can be found in the
[test directory](/test).

    <plugin name="[NAME]" filename="lib[NAME].so">
      <param1>value1</param1>
    </plugin>

## Plugins

The following plugins are present within this package:

### ruvu_disable_physics (world plugin)

This plugin disables the physics engine when it is on. Often we are not
interested in exact physics simulation, it is cheaper (computationally) to
ignore the physics and thus the collision engine. This gives us full control
over the state-evolution in simulation.

![Disable Physics](doc/disable_physics.png)

### ruvu_twist_teleport (model plugin)

This plugin teleports entity in the scene base on a [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
command and publishes an
[nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html).
This plugin assumes that the physics are disabled and assumes infinite
acceleration.

![Twist teleport](doc/twist_teleport.png)

#### Parameters

- `robotNamespace`: Namespace of the plugin, also used to extract the `tfPrefix`
(default=`''`)
- `commandTopic`: Topic name of the incoming [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
 messages (default=`'cmd_vel'`)
- `commandTimeout`: Time for an input command to be valid (default=`0.5`)
- `odometryTopic`: Topic name of the outgoing  [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
message (default=`'odom'`)
- `odometryFrame`: Odom frame id (fixed frame to the world) (default=`odom`)
- `odometryRate`: Frequency of the odometry that is being published
(default=`20`)
- `robotFrame`: Frame id attached to the robot (default=`base_link`)

### ruvu_twist_move (model plugin)

This plugin moves the entity in the scene base on a [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
command and publishes an
[nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html).
This plugin assumes physics and applies infinite acceleration.

![Twist move](doc/twist_move.png)

#### Parameters

- `robotNamespace`: Namespace of the plugin, also used to extract the `tfPrefix`
(default=`''`)
- `commandTopic`: Topic name of the incoming [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
 messages (default=`'cmd_vel'`)
- `commandTimeout`: Time for an input command to be valid (default=`0.5`)
- `odometryTopic`: Topic name of the outgoing  [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
message (default=`'odom'`)
- `odometryFrame`: Odom frame id (fixed frame to the world) (default=`odom`)
- `odometryRate`: Frequency of the odometry that is being published
(default=`20`)
- `robotFrame`: Frame id attached to the robot (default=`base_link`)
