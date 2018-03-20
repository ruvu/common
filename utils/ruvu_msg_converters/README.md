# RUVU Message Converters package
This package contains nodes for conversions of message types for RUVU.

## Nodes

### odom_to_pose_with_covariance_stamped

Converts an incoming `nav_msgs/Odometry` to a `geometry_msgs/PoseWithCovarianceStamped`.

#### Topics

- `odom` (incoming `nav_msgs/Odometry`)
- `pose` (converted `geometry_msgs/PoseWithCovarianceStamped`)

#### Parameters

- `gaussian_noise` (optional parameter that overwrites the incoming covariance)
