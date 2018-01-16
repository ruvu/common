#!/usr/bin/env python
"""ROS node that publishes the device range between Pozyx's."""

import pypozyx
import rospy


def set_gain():
    rospy.init_node('set_gain')

    port = rospy.get_param("~port", "/dev/ttyACM0")
    source_id = rospy.get_param("~source_device_id", None)
    set_gain = rospy.get_param("~set_gain", 33.0)

    source_id = source_id if source_id not in ["None", "none"] else None

    try:
        rospy.loginfo("Connecting to serial Pozyx device on port %s", port)
        pozyx = pypozyx.PozyxSerial(port)
    except:
        rospy.loginfo("Pozyx not connected")
        return

    rospy.loginfo("Succesfully connected to serial pypozyx device on port {}.".format(port))

    rospy.loginfo("Source id: %s", source_id)

    gain = pypozyx.SingleRegister()
    while not rospy.is_shutdown():
        rospy.loginfo("Getting gain ...")
        if pozyx.getUWBGain(gain, source_id) == pypozyx.POZYX_SUCCESS:
            break
        rospy.sleep(1.0)

    rospy.loginfo("Current Gain: %s", gain.value)

    # Now set the gain to the specified value
    while not rospy.is_shutdown():
        rospy.loginfo("Setting gain to %s ...", set_gain)
        if pozyx.setUWBGain(set_gain, source_id) == pypozyx.POZYX_SUCCESS:
            break
        rospy.sleep(1.0)

    gain = pypozyx.SingleRegister()
    while not rospy.is_shutdown():
        rospy.loginfo("Getting gain ...")
        if pozyx.getUWBGain(gain, source_id) == pypozyx.POZYX_SUCCESS:
            break
        rospy.sleep(1.0)

    rospy.loginfo("Current Gain: %s", gain.value)




if __name__ == '__main__':
    try:
        set_gain()
    except rospy.ROSInterruptException:
        pass
