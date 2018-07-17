class Point(object):
    """
    A position
    """

    def __init__(self, x, y, z):
        """

        :param x: in meters
        :param y: in meters
        :param z: in meters
        """
        self.x = x
        self.y = y
        self.z = z


class Pose2D(object):
    """
    A 2d pose (position + orientation)
    """

    def __init__(self, x, y, yaw):
        """

        :param x: in meters
        :param y: in meters
        :param yaw: in radians
        """
        self.x = x
        self.y = y
        self.yaw = yaw


class Pose2DWithCovariance(object):
    """
    A 2d pose (position + orientation) with covariance
    """

    def __init__(self, x, y, yaw, vx, vy, vyaw, covariance):
        """

        :param x: in meters
        :param y: in meters
        :param yaw: in rad
        :param vx: in m/s
        :param vy: in m/s
        :param vyaw: in rad/s
        :param covariance: List of length 6x6 with row major order
        :type covariance: list
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.vyaw = vyaw
        self.covariance = covariance


class DeviceLocation(object):
    def __init__(self, network_id, point):
        pass


class UWBRange(object):
    def __init__(self, network_id, remote_network_id, distance, timestamp):
        """

        :param network_id: Id of the tag
        :type network_id: str
        :param remote_network_id: Id of the anchor
        :type remote_network_id: str
        :param distance: in meters
        :param timestamp: Time in seconds since the Epoch.
        """
        pass


class MultiTagPositionerError(RuntimeError):
    pass


class MultiTagPositioner(object):
    def __init__(self, anchor_locations, tag_locations, height_2_5d):
        """

        :param anchor_locations: Device locations with respect to the world frame
        :param tag_locations: Device locations with respect to the robot frame
        :param height_2_5d: Height constraint of the positioner in world frame
        :type height_2_5d: float
        """
        pass

    def get_position(self, uwb_ranges, odom_pose):
        """
        Update the positioner with new ranges and the latest odom pose. Returns the new estimated pose.

        :param uwb_ranges:
        :type uwb_ranges: [UWBRange]
        :param odom_pose: Pose of the robot with respect to its starting position
        :type odom_pose: Pose2D
        :return: Estimated pose
        :rtype: Pose2DWithCovariance

        :except: When the positioner is unable to determine a position, an exception will be thrown
        :exception: MultiTagPositionerError
        """
        pass
