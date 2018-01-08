# Pozyx ROS

ROS Wrapper for the pozyx localization system

## Nodes

### [pose_graph_node.py](scripts/pozyx_node.py)

Publishes the UWB Pose and the on-board sensor data. This nodes assumes a tag connected to the robot and anchors connected to the world.

#### Interfaces

##### Publishers

- odom ([nav_msgs/Odometry](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html)) - Visualization of the graph for RVIZ
- magnetic_field ([sensor_msgs/MagneticField](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)) - Planned path output, also primarily for RVIZ
- imu ([sensor_msgs/Imu](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)) - Planned path output, also primarily for RVIZ
- temperature ([sensor_msgs/Temperature](http://docs.ros.org/api/nav_msgs/html/msg/Path.html)) - Planned path output, also primarily for RVIZ

#### Parameters

- ~world_frame_id (`default="map"`) - Frame id of the frame attached to the world
- ~robot_frame_id (`default="pozyx"`) - Frame id of the sensor
- ~port (`default=/dev/ttyACM0` - Serial port where the pozyx device is connected to
- ~anchors (`default=[]`) - Anchor set-up: Per anchor an id (network_id) and a position (position (x, y, z))

Example configuration file:

```
world_frame_id: world # ID of the frame attached to the world
sensor_frame_id: sensor # ID of the sensor frame
port: /dev/ttyACM0 # Serial port where the pozyx device is connected to
anchors: # Anchors used for localization
    - network_id: 0x6958 # NetworkID of the anchor
      position: # Position of the anchor
          x: 0
          y: 1
          z: 0
    - {network_id: 0x6e52, position: {x: -0.5, y: -1, z: 0}}
    - {network_id: 0x691e, position: {x: 0, y: -1, z: 0}}
    - {network_id: 0x6979, position: {x: 1.0, y: 0, z: 0}}
```
