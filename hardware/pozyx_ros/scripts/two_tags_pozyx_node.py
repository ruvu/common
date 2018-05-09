#!/usr/bin/env python
import os

import diagnostic_updater
import pypozyx
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from pozyx_ros.algorithms.device import DevicePositioner
from pozyx_ros.algorithms.procrustesgrouppositioner import ProcrustesGroupPositioner
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


class TwoTagPozyxNode:
    def __init__(self, tag_position_map, tag_serial_ports, anchor_position_map, world_frame_id, sensor_frame_id,
                 expected_frequency):
        self.tag_ids = [tag_id for tag_id in tag_position_map]
        self.tag_locations = tag_position_map
        self.anchor_ids = [anchor_id for anchor_id in anchor_position_map]
        self.anchor_locations = anchor_position_map
        self.pozyx_serials = self.initialize_pozyx_serials(tag_serial_ports)
        self.positioner_functions = self.initialize_positioner_functions()

        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        self._odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)

        # # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("pozyx_{}".format(port))

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)

    def initialize_pozyx_serials(self, tag_serial_ports):
        pozyx_serials_dict = {}
        pozyx_serials = [pypozyx.PozyxSerial(pp) for pp in tag_serial_ports]
        for ps in pozyx_serials:
            tag_id_object = pypozyx.NetworkID()
            ps.getNetworkId(tag_id_object)
            tag_id = int(str(tag_id_object), 0)
            if tag_id in self.tag_ids:
                pozyx_serials_dict[tag_id] = ps
        return pozyx_serials_dict

    def initialize_positioner_functions(self):
        positioner_functions = []
        dp = DevicePositioner(self.pozyx_serials, self.anchor_locations)
        positioner_functions.append(dp.get_positions)
        pgp = ProcrustesGroupPositioner(self.tag_locations)
        positioner_functions.append(pgp.calculate_group_position)
        return positioner_functions

    def spin(self):
        while not rospy.is_shutdown():
            output = []
            new_position = {}

            for positioner_function in self.positioner_functions:
                new_position = positioner_function(new_position)
                output.append(new_position)

            last_output = output[-1]

            if last_output and last_output["success"]:
                position = last_output["coordinates"]
                orientation = last_output["orientation"]
                position["z"] = 0
                self._odom_publisher.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._world_frame_id
                    ),
                    child_frame_id=self._sensor_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(position=Point(float(position["x"]) / 1e3, float(position["y"]) / 1e3, float(position["z"]) / 1e3),
                                  orientation=Quaternion(*quaternion_from_euler(0, 0, orientation["yaw"])))
                    )
                ))

                self._frequency_status.tick()

            self._diagnostic_updater.update()


if __name__ == '__main__':
    rospy.init_node('two_tag_pozyx_node')

    port = os.path.realpath(rospy.get_param("~port", "/dev/ttyACM0"))

    try:
        anchors = {d['network_id']: [int(float(d['position']['x']) * 1e3), int(float(d['position']['y']) * 1e3),
                                     int(float(d['position']['z']) * 1e3)] for d
                   in rospy.get_param('~anchors', [])}
        tags = {d['network_id']: [int(float(d['position']['x']) * 1e3), int(float(d['position']['y']) * 1e3),
                                  int(float(d['position']['z']) * 1e3)] for d
                in rospy.get_param('~tags', [])}
        tag_serial_ports = rospy.get_param("~tag_ports", [])

        if len(tags) != 2:
            raise ValueError("~tags should be of size 2")
        if len(tag_serial_ports) != 2:
            raise ValueError("~tag_serial_ports should be of size 2")
    except KeyError as e:
        rospy.logerr("Missing key {} in anchor specification".format(e))
    except ValueError as e:
        rospy.logerr("{}".format(e))
    else:
        try:
            ros_pozyx = TwoTagPozyxNode(tags, tag_serial_ports, anchors,
                                        rospy.get_param("~world_frame_id", "map"),
                                        rospy.get_param("~sensor_frame_id", "pozyx"),
                                        rospy.get_param("~expected_frequency", 15.0))
            ros_pozyx.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
