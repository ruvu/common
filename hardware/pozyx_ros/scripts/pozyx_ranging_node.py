#!/usr/bin/env python
from collections import namedtuple

import diagnostic_updater
import rospy
import sys
from pozyx_msgs.msg import Ranges, Range
from pozyx_ros.device import DeviceRangerPolling
from pozyx_ros.tag_connection import get_tag_connection, UWBSettings
from std_msgs.msg import Header

Tag = namedtuple('Tag', 'serial_port frame_id')


class RangingNode:
    def __init__(self, tags, anchor_ids, uwb_settings, expected_frequency):
        tag_connections = [get_tag_connection(tag.serial_port, uwb_settings) for tag in tags]
        self._tag_id_to_frame_id = dict(zip([tc.network_id for tc in tag_connections], [tag.frame_id for tag in tags]))

        self._device_ranger_polling = DeviceRangerPolling(
            pozyx_serials={tc.network_id: tc.serial_connection for tc in tag_connections},
            anchor_ids=anchor_ids
        )

        # ROS publisher
        self._ranges_publisher = rospy.Publisher('uwb_ranges', Ranges, queue_size=1)

        # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("_".join([tag.serial_port for tag in tags]))

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)

    def spin(self):
        while not rospy.is_shutdown():
            timestamp, ranges = self._device_ranger_polling.get_ranges()
            self._ranges_publisher.publish(Ranges(
                header=Header(
                    stamp=rospy.Time.from_sec(timestamp)
                ),
                ranges=[Range(
                    header=Header(
                        frame_id=self._tag_id_to_frame_id[tag_id],
                        stamp=rospy.Time.from_sec(timestamp)
                    ),
                    network_id=tag_id,
                    remote_network_id=anchor_id,
                    distance=distance_mm / 1e3
                ) for (tag_id, anchor_id), distance_mm in ranges.iteritems()]
            ))

            self._diagnostic_updater.update()


if __name__ == '__main__':
    rospy.init_node('pozyx_ranging_node')

    try:
        uwb_settings = UWBSettings(**rospy.get_param('~uwb_settings', {
            'channel': 5,
            'bitrate': 2,
            'prf': 2,
            'plen': 0x04,
            'gain_db': 30.0
        }))  # 6810 kbit/s 64 plen
    except KeyError as e:
        rospy.logfatal("Missing key {} in ~uwb_settings".format(e))
        sys.exit(1)

    try:
        tags = [Tag(**tag) for tag in rospy.get_param('~tags', [])]
    except KeyError as e:
        rospy.logfatal("Missing key {} for item in ~tags".format(e))
        sys.exit(1)

    try:
        ranging_node = RangingNode(tags,
                                   rospy.get_param("~anchor_ids", []),
                                   uwb_settings,
                                   rospy.get_param("~expected_frequency", 5))
        ranging_node.spin()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(e)
    except RuntimeError as e:
        rospy.logfatal(e)
