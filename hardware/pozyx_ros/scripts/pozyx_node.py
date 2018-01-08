#!/usr/bin/env python

import pypozyx
import rospy
from sensor_msgs.msg import Imu, MagneticField, Temperature
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, Vector3


class ROSPozyx:
    def __init__(self, port, anchors, world_frame_id, sensor_frame_id):
        """
        ROS Pozyx wrapper that publishes the UWB and on-board sensor data
        :param port: The serial port of the pozyx shield
        :param anchors: List of pozyx.DeviceCoordinates() of all the anchors
        :param world_frame_id: The frame id of the world frame
        :param sensor_frame_id: The frame id of the sensor
        """
        self._pozyx = pypozyx.PozyxSerial(port)
        self._anchors = [anchor for anchor in anchors]
        self._anchor_ranges = {}
        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        self._setup_anchors()

        self._odometry_publisher = rospy.Publisher("odom", Odometry, queue_size=1)
        self._imu_publisher = rospy.Publisher("imu", Imu, queue_size=1)
        self._magnetic_field_publisher = rospy.Publisher("magnetic_field", MagneticField, queue_size=1)
        self._temperature_publisher = rospy.Publisher("temperature", Temperature, queue_size=1)

    @staticmethod
    def _discover_anchors_in_range(pozyx):
        """
        Discovers the anchors that are within range
        :param pozyx: Pozyx proxy
        :return: Status and device list
        """
        rospy.loginfo("Discovering anchors ..")
        status = pozyx.doDiscovery(pypozyx.POZYX_DISCOVERY_ANCHORS_ONLY)

        num_anchors = pypozyx.SingleRegister()
        status &= pozyx.getNumberOfAnchors(num_anchors)
        rospy.loginfo("Found {} anchors".format(num_anchors.value))

        device_list = pypozyx.DeviceList(list_size=num_anchors[0])
        status &= pozyx.getDeviceIds(device_list)

        return status, device_list

    def _update_anchor_range_information(self):
        """
        Updates the range information of all anchors to keep track of whether they have been reached
        :return: The active and inactive anchor list
        """
        active_anchors = []
        inactive_anchors = []

        # Retrieve Range Info for each anchor
        for anchor in self._anchors:
            anchor_range = pypozyx.DeviceRange()
            self._pozyx.getDeviceRangeInfo(device_id=anchor.network_id, device_range=anchor_range)

            # RSS should be between -130 dBm and -60 dBm
            if -130 < anchor_range.RSS < -60:
                if anchor in self._anchor_ranges:
                    # If the timestamp of the measurement hasn't changed
                    # the anchor hasn't been reached during last positioning
                    if anchor_range.timestamp == self._anchor_ranges[anchor].timestamp:
                        inactive_anchors.append(anchor)
                    else:
                        active_anchors.append(anchor)

            self._anchor_ranges[anchor] = anchor_range

        return active_anchors, inactive_anchors

    def _setup_anchors(self):
        """
        Sets up the anchors. It checks the input and verifies if we can find the specified anchor ids
        """
        self._pozyx.clearDevices()

        # Check for duplicate network_ids
        network_ids = [a.network_id for a in self._anchors]
        if len(network_ids) != len(set(network_ids)):
            raise RuntimeError("Duplicate network ids specified: {}".format(network_ids))

        while not rospy.is_shutdown():
            status, anchors_in_range = self._discover_anchors_in_range(self._pozyx)

            if len(self._anchors) < 4:
                raise RuntimeError("Please specify at least 4 anchors, available anchors: {}".format(anchors_in_range))
            elif len(self._anchors) > 4:
                status = self.pozyx.setSelectionOfAnchors(pypozyx.POZYX_ANCHOR_SEL_AUTO, len(self._anchors))

            # Now verify if the specified anchors are present
            for anchor in self._anchors:
                if anchor.network_id not in anchors_in_range:
                    rospy.logwarn("Anchor {} not present in device list: {}".format(anchor.network_id,
                                                                                    anchors_in_range))
                    status = pypozyx.POZYX_FAILURE
                else:
                    status &= self._pozyx.addDevice(anchor)

            if status == pypozyx.POZYX_SUCCESS:
                break

            rospy.sleep(1.0)

        rospy.loginfo("Succesfully initialized PozyxROS with {} anchors".format(len(self._anchors)))

    def spin(self):
        """
        Loop that publishes the UWB and On-board sensor data
        """
        while not rospy.is_shutdown():
            # UWB System
            position = pypozyx.Coordinates()
            status = self._pozyx.doPositioning(position, pypozyx.POZYX_2D)

            # On-board sensors
            sensor_data = pypozyx.SensorData()
            status &= self._pozyx.getAllSensorData(sensor_data)

            if status == pypozyx.POZYX_SUCCESS:
                # TODO: Check conversions of the various values, apparently the acceleration is in mg? convert to m/s2
                self._imu_publisher.publish(Imu(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._sensor_frame_id
                    ),
                    orientation=Quaternion(sensor_data.quaternion.x, sensor_data.quaternion.y,
                                           sensor_data.quaternion.z, sensor_data.quaternion.w),
                    angular_velocity=Vector3(sensor_data.angular_vel.x / 1e3,
                                             sensor_data.angular_vel.y / 1e3,
                                             sensor_data.angular_vel.z / 1e3),
                    linear_acceleration=Vector3((sensor_data.linear_acceleration.x + sensor_data.gravity_vector.x)/1e3,
                                                (sensor_data.linear_acceleration.y + sensor_data.gravity_vector.y)/1e3,
                                                (sensor_data.linear_acceleration.z + sensor_data.gravity_vector.z)/1e3)
                ))
                self._magnetic_field_publisher.publish(MagneticField(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._sensor_frame_id
                    ),
                    magnetic_field=Vector3(sensor_data.magnetic.x / 1e3,
                                           sensor_data.magnetic.y / 1e3,
                                           sensor_data.magnetic.z / 1e3)
                ))
                self._temperature_publisher.publish(Temperature(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._sensor_frame_id
                    ),
                    temperature=sensor_data.temperature.value
                ))

                # Keep track of active and inactive anchors, this affects the positioning outcome
                active_anchors, inactive_anchors = self._update_anchor_range_information()

                if inactive_anchors:
                    rospy.logwarn("Anchors {} could not be reached in last positioning"
                                  .format([a.network_id for a in inactive_anchors]))
                else:
                    rospy.loginfo("Update success")
                    self._odometry_publisher.publish(Odometry(
                        header=Header(
                            stamp=rospy.Time.now(),
                            frame_id=self._world_frame_id
                        ),
                        child_frame_id=self._sensor_frame_id,
                        pose=PoseWithCovariance(
                            pose=Pose(position=Point(position.x / 1e3, position.y / 1e3, position.z / 1e3),
                                      orientation=Quaternion(sensor_data.quaternion.x, sensor_data.quaternion.y,
                                                             sensor_data.quaternion.z, sensor_data.quaternion.w))
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
            ros_pozyx = ROSPozyx(port, anchors, rospy.get_param("world_frame_id", "map"),
                                 rospy.get_param("sensor_frame_id", "pozyx"))
            ros_pozyx.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
