# Republish pose on tf or with covariance

This packages contains two node that subscribes to a pose topic. Supported messages are:

- `geometry_msgs/Pose`
- `geometry_msgs/PoseStamped`
- `geometry_msgs/Transform`
- `geometry_msgs/TransformStamped`
- `nav_msgs/Odometry`
- `sensor_msgs/Imu` (assumes null translation)

## Common parameters

- `topic`: topic to subscribe to (defaults to `pose_gt`)
- `parent_frame`: parent frame to be used in published messages (defaults to `world`)

## `pose_to_tf` node

This node will simply republish the pose as a transform on `/tf`. If the parent frame is not defined then `world` will be used.

It can be used to simulate perfect localization (on `/tf`) from a ground truth topic published by a simulator.

This node takes an additional parameter:

- `child_frame`: child frame to be used in tf publisher (defaults to `base_link`)

The frame parameters are only to complement messages that do not include the information:

- `Pose` and `Transform` do not convey any frame, so both parameters are used;
- `PoseStamped` and `Imu` only convey `child_frame` in the header, the `parent_frame` parameter is thus used;
- `TransformStamped` and `Odometry` convey both `child_frame` explicitely and `parent_frame` in the header.

## `pose_with_covariance` node

This node will republish the received `Pose` while adding the covariance field as a `PoseWithCovarianceStamped`, which may be useful to feed e.g. a Kalman filter. Additional parameters are:

- `cov.xyz` (default .01): the position covariance
- `cov.rpy` (default .01): the orientation covariance

A simple `robot_localization` parameter file is given to handle such a `PoseWithCovarianceStamped`.
