<!--
Copyright 2020 RUVU Robotics B.V.
-->

# Pozyx ROS

ROS Wrapper for the pozyx localization system

## Nodes

### [pozyx_positioning_node.py](scripts/pozyx_positioning_node.py)

Publishes the UWB Pose and the on-board sensor data. This nodes assumes a tag connected to the robot and anchors connected to the world.

#### Interfaces

##### Publishers

- pose ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) - Estimated UWB pose
- ranges (pozyx_ros/Ranges) - Range information of the anchors
- magnetic_field ([sensor_msgs/MagneticField](http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html))
- imu ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html))
- temperature ([sensor_msgs/Temperature](http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html))

#### Parameters

- ~world_frame_id (`default="map"`) - Frame id of the frame attached to the world
- ~robot_frame_id (`default="pozyx"`) - Frame id of the sensor
- ~port (`default=/dev/ttyACM0` - Serial port where the pozyx device is connected to
- ~minimum_fix_factor (`default=0.33`) - Determines how many positioning failures we allow (for diagnostics)
- ~frequency (`default=15`) - How often per second the pozyx device is queried, we sleep in between
- ~anchors (`default=[]`) - Anchor set-up: Per anchor an id (network_id) and a position (position (x, y, z))

Example configuration file:

```
world_frame_id: world # ID of the frame attached to the wor ld
sensor_frame_id: sensor # ID of the sensor frame
port: /dev/ttyACM0 # Serial port where the pozyx device is connected to
minimum_fix_factor: 0.5
frequency: 15
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

### [range_info_node.py](scripts/pozyx_positioning_node.py)

Publishes the UWB ranges

### [two_tags_pozyx_positioning_node.py](scripts/two_tag_positioner_node)

Publish the estimated pose using two pozyx tags

### [two_tags_pozyx_socket_node.py](scripts/two_tag_pozyx_socket_node.py)

Publish the estimated pose using two pozyx tags, obtained via UDP socket