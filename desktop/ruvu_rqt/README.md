# RUVU RQT

## Nodes

### rqt.bash

Starts an rqt instance and optionally copies an rviz config to the default location. This scripts is required because
rviz in rqt cannot load proper rviz configs.

```
Usage: ./rqt.bash [rqt_perspective] [rviz_config] (optional)
```

### publish_pose

Launches a widget with the Publish pose plugin.

![publish_pose](doc/publish_pose.png)

## Plugins

### publish_pose.PublishPose

Widget that publishes a pose based on a transformation available via TF. The user can specify the following:

- source_frame
- target_frame
- transform_timeout
- output topic (PoseStamped)

These parameters can be configured via the configuration button (gear-wheel).
