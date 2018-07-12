#!/usr/bin/env python
import os

import pypozyx
import rospy
import diagnostic_updater
import collections
from diagnostic_msgs.msg import DiagnosticStatus
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovariance, PoseWithCovarianceStamped, Pose, Point
from pozyx_ros.msg import DeviceRanges, DeviceRange


class ROSPozyx:
    def __init__(self, port, anchors, world_frame_id, sensor_frame_id, frequency, minimum_fix_factor):
        """
        ROS Pozyx wrapper that publishes the UWB and on-board sensor data
        :param port: The serial port of the pozyx shield
        :param anchors: List of pozyx.DeviceCoordinates() of all the anchors
        :param world_frame_id: The frame id of the world frame
        :param sensor_frame_id: The frame id of the sensor
        :param frequency: Frequency of the pozyx, we will sleep in between before continuously querying the pozyx device
        :param minimum_fix_factor: The factor that determines the numbre of positioning failures, used in diagnostics
        """
        self._anchors = [anchor for anchor in anchors]
        self._anchor_ranges = {}
        self._active_anchors = []
        self._inactive_anchors = []
        self._positioning_result_last_second = collections.deque(maxlen=frequency)  # Average over 1 second
        self._minimum_fix_factor = minimum_fix_factor
        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        rospy.logwarn("Connecting via Serial to Pypozyx on port {}. If the connection fails, a syntax "
                      "error is promted due to a bug in the pypozyx python library.".format(port))
        self._pozyx = pypozyx.PozyxSerial(port)
        rospy.loginfo("Succesfully connected to serial pypozyx device on port {}.".format(port))

        self._setup_anchors()

        self._pose_publisher = rospy.Publisher("pose", PoseWithCovarianceStamped, queue_size=1)
        self._imu_publisher = rospy.Publisher("imu", Imu, queue_size=1)
        self._magnetic_field_publisher = rospy.Publisher("magnetic_field", MagneticField, queue_size=1)
        self._temperature_publisher = rospy.Publisher("temperature", Temperature, queue_size=1)
        self._ranges_publisher = rospy.Publisher("ranges", DeviceRanges, queue_size=1)

        # Initialize diagnostics publisher
        self._frequency = frequency
        self._rate = rospy.Rate(frequency)
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("pozyx_{}".format(port))

        # Add frequency monitoring
        self._frequency_status_onboard_sensors = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': frequency, 'max': frequency}))
        self._diagnostic_updater.add(self._frequency_status_onboard_sensors)

        # Add uwb positioning monitoring
        self._diagnostic_updater.add("UWB Positioning", self._uwb_positioning_diagnostics)

    def _uwb_positioning_diagnostics(self, stat):
        # Calculate the fix factor
        fix_factor = round(self._positioning_result_last_second.count(True) / float(self._frequency), 2)

        if fix_factor < self._minimum_fix_factor:
            stat.summary(DiagnosticStatus.ERROR, "Fix factor too low: {} < {}".format(fix_factor,
                                                                                      self._minimum_fix_factor))
        else:
            stat.summary(DiagnosticStatus.OK, "Fix factor ok: {}".format(fix_factor))

        stat.add("Fix factor", fix_factor)
        stat.add("Window size", self._frequency)

        stat.add("Last update number of active anchors", len(self._active_anchors))
        stat.add("Last update active anchor ids", [a.network_id for a in self._active_anchors])
        stat.add("Last update number of inactive anchors", len(self._inactive_anchors))
        stat.add("Last update inactive anchor ids", [a.network_id for a in self._inactive_anchors])

        return stat

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
        """

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
                        self._inactive_anchors.append(anchor)
                    else:
                        self._active_anchors.append(anchor)

            self._anchor_ranges[anchor] = anchor_range

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

            # Reset status of active and inactive anchors
            self._inactive_anchors = []
            self._active_anchors = []

            if status == pypozyx.POZYX_SUCCESS:
                self._frequency_status_onboard_sensors.tick()

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
                self._update_anchor_range_information()

                # Loop over the active anchors and publish ranges w.r.t. our tag
                ranges_msg = DeviceRanges(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._sensor_frame_id
                    )
                )
                for active_anchor in self._active_anchors:
                    device_range = self._anchor_ranges[active_anchor]
                    ranges_msg.ranges.append(DeviceRange(
                        remote_device_id=hex(active_anchor.network_id),
                        stamp=rospy.Time.from_seconds(float(device_range.timestamp) / 1e3),
                        RSS=device_range.RSS,
                        distance=float(device_range.distance) / 1e3
                    ))
                self._ranges_publisher.publish(ranges_msg)

                # Only publish if we do not have any inactive anchors
                if not self._inactive_anchors:
                    self._pose_publisher.publish(PoseWithCovarianceStamped(
                        header=Header(
                            stamp=rospy.Time.now(),
                            frame_id=self._world_frame_id
                        ),
                        pose=PoseWithCovariance(
                            pose=Pose(position=Point(position.x / 1e3, position.y / 1e3, position.z / 1e3),
                                      orientation=Quaternion(sensor_data.quaternion.x, sensor_data.quaternion.y,
                                                             sensor_data.quaternion.z, sensor_data.quaternion.w))
                        )
                    ))
            else:
                rospy.logerr("Failed to obtain position or orientation!")

            # Update positioning result
            self._positioning_result_last_second.append(self._inactive_anchors == [])

            self._diagnostic_updater.update()

            self._rate.sleep()


if __name__ == '__main__':
    rospy.init_node('pozyx_ros_node')

    port = os.path.realpath(rospy.get_param("~port", "/dev/ttyACM0"))

    try:
        anchors = [pypozyx.DeviceCoordinates(d['network_id'], pypozyx.POZYX_ANCHOR,
                                             pypozyx.Coordinates(d['position']['x'] * 1e3,
                                                                 d['position']['y'] * 1e3,
                                                                 d['position']['z'] * 1e3))
                   for d in rospy.get_param('~anchors', [])]
    except KeyError as e:
        rospy.logerr("Missing key {} in anchor specification".format(e))
    else:
        try:
            ros_pozyx = ROSPozyx(port, anchors, rospy.get_param("~world_frame_id", "map"),
                                 rospy.get_param("~sensor_frame_id", "pozyx"),
                                 rospy.get_param("~frequency", 15.0),
                                 rospy.get_param("~minimum_fix_factor", 0.33))
            ros_pozyx.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
