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

    gain = pypozyx.SingleRegister()
    pozyx.getUWBGain(gain, source_id)

    rospy.loginfo("Source info:")
    rospy.loginfo("- gain: %s", gain.value)

    while not rospy.is_shutdown():
        result = "Ranging from source {}".format(hex(source_id) if source_id else hex(frame_id))
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
                result += "\t{} => {:.3f}".format(hex(remote_id), device_range_msg.distance)
            else:
                result += "\t\033[91m'{} => NaN'\033[0m".format(hex(remote_id))

        if len(ranges) == len(remote_ids):
            result += "\t\033[92mOK!\033[0m"
        else:
            result += "\t\033[91mFAIL!\033[0m"

        rospy.loginfo(result)

        pub.publish(DeviceRanges(
            header=Header(
                frame_id=hex(frame_id),
                stamp=rospy.Time.now()
            ),
            ranges=ranges
        ))
        rospy.Rate(10).sleep()


if __name__ == '__main__':
    try:
        pozyx_ranging_pub()
    except rospy.ROSInterruptException:
        pass
