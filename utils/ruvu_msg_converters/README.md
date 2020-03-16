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

### rewrite_imu_covariance

Rewrites the diagonal of the covariances of an incoming `sensor_msgs/Imu` with given values.

#### Topics

- `imu/data` (incoming `sensor_msgs/Imu`)
- `imu/data_covariance_rewritten` (outgoing `sensor_msgs/Imu` with rewritten covariance)

#### Parameters

- `orientation_covariance_diagonal` (required): array of length 3 containing the desired diagonal elements of the orientation covariance.
- `angular_velocity_covariance_diagonal` (required): array of length 3 containing the desired diagonal elements of the angular velocity covariance.
- `linear_acceleration_covariance_diagonal` (required): array of length 3 containing the desired diagonal elements of the linear acceleration covariance.

### twist_to_twist_stamped

Adds a std_msgs/Header to a geometry_msgs/Twist message. This can be used for plotting cmd_vel messages in rqt.

#### Topics

- `input` (incoming `geometry_msgs/Twist`)
- `output` (converted `geometry_msgs/TwistStamped`)

### odom_to_tf

Publishes an odometry message as `/tf` transform.

#### Parameters

- `~publish_as_child`: Publishes an inverse transform and swaps the header and child frame id (defaults to `false`)

#### Topics

- `odom` (incoming `nav_msgs/Odometry`)
- `/tf`

### pose_stamped_to_tf

Pose stamped as tf

#### Parameters

- `~child_frame_id`: The child frame id of the transform, set as header if `publish_as_child` is true (required)
- `~publish_as_child`: Publishes an inverse transform and swaps the header and child frame id (defaults to `false`)

#### Topics

- `pose` (incoming `geometry_msgs/PoseStamped`)
- `/tf`

### tf_to_pose_stamped
Publishes a tf transform as a PoseStamped.

#### Parameters
- `~target_frame`: The frame to which data should be transformed
- `~source_frame`: The frame where the data originated

#### Topics
- `pose` (outgoing `geometry_msgs/PoseStamped`)

### pose_with_covariance_stamped_to_pose_stamped
Refactor pose with covariance stamped to pose stamped

#### Topics
- `input` (incoming `geometry_msgs/PoseWithCovarianceStamped`)
- `output` (converted `geometry_msgs/PoseStamped`)
