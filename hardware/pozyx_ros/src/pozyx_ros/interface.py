import logging
from json import dumps

import numpy as np
from positioning.tag import Tag

logger = logging.getLogger(__name__)


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

    def __init__(self, timestamp, x, y, yaw):
        """

        :param x: in meters
        :param y: in meters
        :param yaw: in radians
        """
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.yaw = yaw


class Pose2DWithCovariance(object):
    """
    A 2d pose (position + orientation) with covariance
    """

    def __init__(self, timestamp, x, y, yaw, vx, vy, vyaw, covariance):
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
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.vyaw = vyaw
        self.covariance = covariance


class DeviceLocation(object):
    def __init__(self, network_id, point):
        """

        :param network_id: Id of the tag
        :param point: point in meters
        :type point: Point
        """
        self.network_id = network_id
        self.point = point


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
        self.network_id = network_id
        self.remote_network_id = remote_network_id
        self.distance = distance
        self.timestamp = timestamp


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
        self.tag = Tag(tag_type='EUROTEC',
                       anchor_positions = {str(al.network_id): [int(1000*al.point.x), int(1000*al.point.y), int(1000*al.point.z)] for al in anchor_locations},
                       multitag_positions = {str(tl.network_id): [int(1000*tl.point.x), int(1000*tl.point.y), int(1000*tl.point.z)] for tl in tag_locations},
                       height_2_5d = int(1000*height_2_5d))
        self.old_odom_pose = None

    def get_position(self, uwb_ranges, odom_pose=None):
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
        timestamp = 0  # process uwb_ranges
        positioning_input = {}
        for uwb_range in uwb_ranges:
            positioning_input[(str(uwb_range.network_id), str(uwb_range.remote_network_id))] = int(1000*uwb_range.distance)
            timestamp = max(timestamp, uwb_range.timestamp)
        if timestamp == 0:
            raise MultiTagPositionerError('No input timestamp provided')
        odom_pose_change = None  # process odom_pose
        if odom_pose is not None:
            if self.old_odom_pose is not None:
                lopyc = np.cos(self.old_odom_pose.yaw)
                lopys = np.sin(self.old_odom_pose.yaw)
                odom_pose_change = {'position': [1000*(lopyc * (odom_pose.x - self.old_odom_pose.x) +
                                                       lopys * (odom_pose.y - self.old_odom_pose.y)),
                                                 1000*(-lopys * (odom_pose.x - self.old_odom_pose.x) +
                                                       lopyc * (odom_pose.y - self.old_odom_pose.y)),
                                                 0],
                                    'orientation': odom_pose.yaw - self.old_odom_pose.yaw}
            self.old_odom_pose = odom_pose
        extra_input = {'prediction': {'pose_change': odom_pose_change}} if odom_pose_change is not None else {}

        logger.debug(dumps({'func': 'get_position',
                            'timestamp': timestamp,
                            'positioning_input': {str(k): v for (k, v) in positioning_input.items()},
                            'extra_input': extra_input}))
        positioning_output = self.tag.get_tag_loc(timestamp, positioning_input, **extra_input)  # calculate new pose
        logger.debug(dumps({'func': 'get_position_output', 'output': positioning_output}))
        if not positioning_output['success']:  # raise error if no success
            raise MultiTagPositionerError('Positioning update not successful')
        if (not 'position' in positioning_output['state'] or  # raise error if fallback positioner was used
            not 'orientation' in positioning_output['state'] or 
            not 'velocity' in positioning_output['state'] or 
            not 'orientation_velocity' in positioning_output['state'] or
            not 'covariance' in positioning_output['diagnostics']):
            raise MultiTagPositionerError('Non-standard positioning output')
        covariance = []  # covert covariance format
        for i in range(6):
            i_factor = 1e-3 if i < 4 else 1e0
            for j in range(6):
                j_factor = 1e-3 if j < 4 else 1e0
                covariance.append(positioning_output['diagnostics']['covariance'][i][j]*i_factor*j_factor)
        return Pose2DWithCovariance(timestamp,
                                    positioning_output['state']['position'][0]/1000.,  # format output
                                    positioning_output['state']['position'][1]/1000.,
                                    positioning_output['state']['orientation'],
                                    positioning_output['state']['velocity'][0]/1000.,
                                    positioning_output['state']['velocity'][1]/1000.,
                                    positioning_output['state']['orientation_velocity'],
                                    covariance)