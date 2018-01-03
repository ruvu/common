#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS node that publishes the transform to the pozyx tag
"""

import pypozyx
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion


class ROSPozyx:
    def __init__(self, port, anchors, frame_id):
        self._pozyx = pypozyx.PozyxSerial(port)
        self._anchors = anchors
        self._frame_id = frame_id

        self._setup_anchors()

        self._odometry_publisher = rospy.Publisher("odom", Odometry, queue_size=1)

    def _setup_anchors(self):
        self._pozyx.clearDevices()

        # Check for duplicate network_ids
        network_ids = [a.network_id for a in self._anchors]
        if len(network_ids) != len(set(network_ids)):
            raise RuntimeError("Duplicate network ids specified: {}".format(network_ids))

        status = self._pozyx.doDiscovery(pypozyx.POZYX_DISCOVERY_ANCHORS_ONLY)

        list_size = pypozyx.SingleRegister()
        status &= self._pozyx.getDeviceListSize(list_size)

        device_list = pypozyx.DeviceList(list_size=list_size[0])
        status &= self._pozyx.getDeviceIds(device_list)

        # Now verify if the specified anchors are present
        for anchor in self._anchors:
            if anchor.network_id not in device_list:
                raise RuntimeError("Anchor {} not present in device list: {}".format(anchor, device_list))
            status &= self._pozyx.addDevice(anchor)

        if len(self._anchors) < 4:
            raise RuntimeError("Please specify at least 4 anchors, available anchors: {}".format(device_list))
        elif len(self._anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO, len(self._anchors))

        if status != pypozyx.POZYX_SUCCESS:
            raise RuntimeError("Failed to set-up anchors")

    def spin(self):
        while not rospy.is_shutdown():
            position = pypozyx.Coordinates()
            orientation = pypozyx.Quaternion()
            covariance = pypozyx.PositionError()

            status = self._pozyx.doPositioning(position, pypozyx.POZYX_2D)
            status &= self._pozyx.getQuaternion(orientation)
            status &= self._pozyx.getPositionError(covariance)

            if status == pypozyx.POZYX_SUCCESS:
                self._odometry_publisher.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._frame_id
                    ),
                    pose=PoseWithCovariance(
                        pose=Pose(position=Point(position.x / 1e3, position.y / 1e3, position.z / 1e3),
                                  orientation=Quaternion(orientation.x, orientation.y, orientation.z, orientation.w)),
                        covariance=[covariance.x,  covariance.xy, covariance.xz, 0, 0, 0,
                                    covariance.xy, covariance.y,  covariance.yz, 0, 0, 0,
                                    covariance.xz, covariance.yz, covariance.z,  0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0]
                    )
                ))
            else:
                rospy.logerr("Failed to obtain position or orientation!")


if __name__ == '__main__':
    rospy.init_node('pozyx_ros_node')

    port = rospy.get_param("~port", "/dev/ttyACM0")

    try:
        anchors = [pypozyx.DeviceCoordinates(d['network_id'], d['flag'], pypozyx.Coordinates(d['pos']['x'] * 1e3,
                                                                                             d['pos']['y'] * 1e3,
                                                                                             d['pos']['z'] * 1e3))
                   for d in rospy.get_param('~anchors', [])]
    except KeyError as e:
        rospy.logerr("Missing key {} in anchor specification".format(e))
    else:
        try:
            ros_pozyx = ROSPozyx(port, anchors, rospy.get_param("frame_id", "map"))
            ros_pozyx.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
