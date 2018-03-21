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

### rewrite_odom_covariance

Rewrites the diagonal of the covariance of an incoming `nav_msgs/Odometry` with given values.

#### Topics

- `odom` (incoming `nav_msgs/Odometry`)
- `odom_covariance_rewritten` (outgoing `nav_msgs/Odometry` with rewritten covariance)

#### Parameters

- `pose_covariance_diagonal` (required): array of length 6 containing the desired diagonal elements of the pose covariance.
- `twist_covariance_diagonal` (required): array of length 6 containing the desired diagonal elements of the twist covariance.
