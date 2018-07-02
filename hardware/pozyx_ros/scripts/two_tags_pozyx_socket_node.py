#!/usr/bin/env python
import os
import socket
import json

import diagnostic_updater
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


class TwoTagPozyxSocketNode:
    def __init__(self, ip, port, world_frame_id, sensor_frame_id, expected_frequency):
        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        self._odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)

        # # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("{}:{}".format(ip, port))

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)

        # Bind to socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((ip, port))

    def spin(self):
        while not rospy.is_shutdown():
            raw_data, _ = self._socket.recvfrom(2048)
            data = json.loads(raw_data.decode())

            if data and data["success"]:
                position = data["coordinates"]
                orientation = data["orientation"]
                position["z"] = 0
                c = data["covariance"]
                self._odom_publisher.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._world_frame_id
                    ),
                    child_frame_id=self._sensor_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(position=Point(position["x"] / 1e3, position["y"] / 1e3, position["z"] / 1e3),
                                  orientation=Quaternion(*quaternion_from_euler(orientation["roll"],
                                                                                orientation["pitch"],
                                                                                orientation["yaw"]))),
                        # Row major order, x [mm] y [mm] z [mm] yaw [rad] pitch [rad] roll [rad]
                        covariance=[c[0][0]/1e6, c[0][1]/1e6, c[0][2]/1e6, c[0][5]/1e3, c[0][4]/1e3, c[0][3]/1e3,
                                    c[1][0]/1e6, c[1][1]/1e6, c[1][2]/1e6, c[1][5]/1e3, c[1][4]/1e3, c[1][3]/1e3,
                                    c[2][0]/1e6, c[2][1]/1e6, c[2][2]/1e6, c[2][5]/1e3, c[2][4]/1e3, c[2][3]/1e3,
                                    c[5][0]/1e3, c[5][1]/1e3, c[5][2]/1e3, c[5][5]/1e3, c[5][4]/1e3, c[5][3]/1e3,
                                    c[4][0]/1e3, c[4][1]/1e3, c[4][2]/1e3, c[4][5]/1e3, c[4][4]/1e3, c[4][3]/1e3,
                                    c[3][0]/1e3, c[3][1]/1e3, c[3][2]/1e3, c[3][5]/1e3, c[3][4]/1e3, c[3][3]/1e3]
                    )
                ))

                self._frequency_status.tick()

            self._diagnostic_updater.update()


if __name__ == '__main__':
    rospy.init_node('two_tag_pozyx_socket_node')

    try:
        ros_pozyx = TwoTagPozyxSocketNode(rospy.get_param("~ip", "localhost"),
                                          rospy.get_param("~port", 2000),
                                          rospy.get_param("~world_frame_id", "map"),
                                          rospy.get_param("~sensor_frame_id", "pozyx"),
                                          rospy.get_param("~expected_frequency", 15.0))
        ros_pozyx.spin()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)
    except RuntimeError as e:
        rospy.logerr(e)
