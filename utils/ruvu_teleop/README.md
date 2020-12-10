<!--
Copyright 2020 RUVU Robotics B.V.
-->

# RUVU Teleop

Teleop tools for robots

## Nodes

### teleop_single_joint_position_joy

Map `sensor_msgs/Joy` buttons to `control_msgs/SingleJointPosition` action.

#### Parameters

- `button_index_positions`: Required parameter that maps button indices to joint positions

Example config:

```
button_index_positions:
    - button_index: 1
      position: 0.025
    - button_index: 3
      position: 0.0
```

### teleop_publish_pose_joy

Map `sensor_msgs/Joy` buttons to `geometry_msgs/Pose` messages containing the pose of a child_frame in an other frame.

#### Parameters

- `button_index_pose_publishers`: Required parameter that maps button indices to transformations and topics

Example config:

```
button_index_pose_publishers:
    - button_index: 3
      topic: /graph_navigation/add_node
      frame_id: map
      child_frame_id: base_link
```

### teleop_cancel_actionlib_goal_joy

Cancels an actionlib goal when a joystick button has been pressed.

#### Parameters

- `button_index`: Button index used for canceling the actionlib goal. (default=`0`)
- `goal_id`: Which actionlib goal id to cancel. (default=`''`)

#### Topics

- joy ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))
- cancel ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))


### teleop_publish_float_joy

Map `sensor_msgs/Joy` axes to `std_msgs/Float64` topic.

#### Parameters

- `button_mapping`: Required parameter that maps button indices to joint positions
- `button_mapping[i]/button_index`: enable button for this mapping
- `button_mapping[i]/axis_index`: axis to map to output topic
- `button_mapping[i]/factor`: the axis value is multiplied by this factor before being published on the output topic
- `button_mapping[i]/topic`: output topic (`std_msgs/Float64`)

Example config:

```
button_mapping:
    - button_index: 2
      axis_index: 7
      factor: 1
      topic: front_lift_controller/command
    - button_index: 3
      axis_index: 7
      factor: 1
      topic: rear_lift_controller/command
```

#### Subscribed topics

- `joy` ([`sensor_msgs/Joy`](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))

#### Published topics

- depends on parameters ([`std_msgs/Float64`](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))
