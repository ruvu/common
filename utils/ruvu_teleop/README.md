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
