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
