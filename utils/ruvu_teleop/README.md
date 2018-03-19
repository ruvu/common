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
