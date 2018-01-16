#!/usr/bin/env python
"""ROS node that publishes the device range between Pozyx's."""

import pypozyx
import rospy
from std_msgs.msg import Header
from pozyx_ros.msg import DeviceRanges, DeviceRange


def pozyx_ranging_pub():
    rospy.init_node('ranges_publisher')

    port = rospy.get_param("~port", "/dev/ttyACM0")
    source_id = rospy.get_param("~source_device_id", None)
    remote_ids = rospy.get_param("~remote_device_ids")

    if source_id is None:
        frame_id = rospy.get_param("~frame_id")
    else:
        frame_id = source_id

    pub = rospy.Publisher('ranges', DeviceRanges, queue_size=1)

    try:
        rospy.loginfo("Connecting to serial Pozyx device on port %s", port)
        pozyx = pypozyx.PozyxSerial(port)
    except:
        rospy.loginfo("Pozyx not connected")
        return

    rospy.loginfo("Succesfully connected to serial pypozyx device on port {}.".format(port))

    while not rospy.is_shutdown():
        ranges = []
        for remote_id in remote_ids:
            device_range = pypozyx.DeviceRange()
            if pozyx.doRanging(remote_id, device_range, remote_id=source_id):
                device_range_msg = DeviceRange(
                    remote_device_id=hex(remote_id),
                    stamp=rospy.Time.from_seconds(float(device_range.timestamp) / 1e3),
                    RSS=device_range.RSS,
                    distance=float(device_range.distance) / 1e3
                )
                ranges.append(device_range_msg)
                rospy.loginfo("=> Ranging from %s to %s:\t Distance=%.3f\tDB:%i", source_id, remote_id,
                              device_range_msg.distance, device_range_msg.RSS)
            else:
                rospy.logerr('=> Ranging from %s to %s failed', source_id, remote_id)
        pub.publish(DeviceRanges(
            header=Header(
                frame_id=hex(frame_id),
                stamp=rospy.Time.now()
            ),
            ranges=ranges
        ))


if __name__ == '__main__':
    try:
        pozyx_ranging_pub()
    except rospy.ROSInterruptException:
        pass
