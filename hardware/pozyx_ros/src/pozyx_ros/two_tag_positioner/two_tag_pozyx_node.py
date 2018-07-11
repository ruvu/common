import diagnostic_updater
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster

from two_tag_positioner import Tag, Anchor, Position, UWBSettings, TwoTagPositioner, Input, Velocity2D


class TwoTagPositionerNode:
    def __init__(self, tags, anchors, uwb_settings, world_frame_id, sensor_frame_id, expected_frequency,
                 warning_success_rate, height_2_5d):
        self._warning_success_rate = warning_success_rate
        self._unsuccessful_updates = 0
        self._successful_updates = 0

        def _position_to_mm(tag_or_anchor):
            return tag_or_anchor._replace(position=Position(*[int(p * 1e3) for p in tag_or_anchor.position]))

        self._two_tag_positioner = TwoTagPositioner([_position_to_mm(tag) for tag in tags],
                                                    [_position_to_mm(anchor) for anchor in anchors],
                                                    uwb_settings, int(height_2_5d * 1e3))

        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        self._odom_subscriber = rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1)
        self._pose_publisher = rospy.Publisher("uwb_pose", Odometry, queue_size=1)

        # # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("_".join([tag.serial_port for tag in tags]))

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)
        self._diagnostic_updater.add("UWB Positioning", self._uwb_positioning_diagnostics)

        self._anchor_broadcaster = StaticTransformBroadcaster()
        self._anchor_broadcaster.sendTransform([
            TransformStamped(
                header=Header(frame_id=world_frame_id, stamp=rospy.Time.now()),
                child_frame_id="anchor_{}".format(anchor.network_id),
                transform=Transform(
                    translation=Vector3(*anchor.position),
                    rotation=Quaternion(w=1)  # Unit quaternion
                )
            )
            for anchor in anchors])

    def _uwb_positioning_diagnostics(self, stat):
        success_rate = float(self._successful_updates) / (self._unsuccessful_updates + self._successful_updates)
        if success_rate < self._warning_success_rate:
            stat.summary(DiagnosticStatus.WARN, "Success rate too low: {}".format(success_rate))
        else:
            stat.summary(DiagnosticStatus.OK, "Success rate: {}".format(success_rate))

        stat.add("Position update success rate", success_rate)

        self._unsuccessful_updates = 0
        self._successful_updates = 0

        return stat

    def _odom_callback(self, msg):
        twist = msg.twist.twist
        twist_stamp = msg.header.stamp.to_sec()
        twist_covariance = msg.twist.covariance

        try:
            estimate = self._two_tag_positioner.get_position(
                Input(
                    current_time=rospy.get_time(),
                    velocity_time=twist_stamp,
                    velocity=Velocity2D(x=int(twist.linear.x * 1e3), y=int(twist.linear.y * 1e3), yaw=twist.angular.z),
                    covariance=[
                        int(twist_covariance[0] * 1e6), int(twist_covariance[1] * 1e6), int(twist_covariance[5] * 1e3),
                        int(twist_covariance[6] * 1e6), int(twist_covariance[7] * 1e6), int(twist_covariance[11] * 1e3),
                        int(twist_covariance[30] * 1e3), int(twist_covariance[31] * 1e3), twist_covariance[35]
                    ]
                )
            )

            c = estimate.covariance
            self._pose_publisher.publish(Odometry(
                header=Header(
                    stamp=rospy.Time.now(),
                    frame_id=self._world_frame_id
                ),
                child_frame_id=self._sensor_frame_id,
                pose=PoseWithCovariance(
                    pose=Pose(position=Point(*[p / 1e3 for p in estimate.position]),
                              orientation=Quaternion(*quaternion_from_euler(0, 0, estimate.orientation.yaw))),
                    covariance=[
                        c[0] / 1e6, c[1] / 1e6, c[2] / 1e6, c[3] / 1e3, c[4] / 1e3, c[5] / 1e3,
                        c[6] / 1e6, c[7] / 1e6, c[8] / 1e6, c[9] / 1e3, c[10] / 1e3, c[11] / 1e3,
                        c[12] / 1e6, c[13] / 1e6, c[14] / 1e6, c[15] / 1e3, c[16] / 1e3, c[17] / 1e3,
                        c[18] / 1e3, c[19] / 1e3, c[20] / 1e3, c[21], c[22], c[23],
                        c[24] / 1e3, c[25] / 1e3, c[26] / 1e3, c[27], c[28], c[29],
                        c[30] / 1e3, c[31] / 1e3, c[32] / 1e3, c[33], c[34], c[35]
                    ]
                )
            ))

            self._frequency_status.tick()
        except RuntimeError as runtime_error:
            self._unsuccessful_updates += 1
            rospy.logerr(runtime_error)
        else:
            self._successful_updates += 1
            rospy.logdebug("Position update succesful: %s", estimate)

        self._diagnostic_updater.update()


if __name__ == '__main__':
    rospy.init_node('two_tag_pozyx_node')

    try:
        rospy.loginfo("Parsing anchors ..")
        anchors = [Anchor(d['network_id'], Position(**d['position'])) for d in rospy.get_param('~anchors', [])]
        rospy.loginfo("Parsing tags ..")
        tags = [Tag(d['serial_port'], Position(**d['position'])) for d in rospy.get_param('~tags', [])]
        rospy.loginfo("Parsing uwb_settings ..")
        uwb_settings = UWBSettings(**rospy.get_param('~uwb_settings', {
            'channel': 5,
            'bitrate': 2,
            'prf': 2,
            'plen': 0x04,
            'gain_db': 30.0
        }))  # 6810 kbit/s 64plen

        if len(anchors) < 3:
            raise ValueError("~There should be at least 3 anchors")

        if len(tags) != 2:
            raise ValueError("~tags should be of size 2")
    except KeyError as e:
        rospy.logerr("Missing key {} in specification".format(e))
    except ValueError as e:
        rospy.logerr("{}".format(e))
    else:
        try:
            ros_pozyx = TwoTagPositionerNode(tags, anchors, uwb_settings,
                                             rospy.get_param("~world_frame_id", "map"),
                                             rospy.get_param("~sensor_frame_id", "pozyx"),
                                             rospy.get_param("~expected_frequency", 5),
                                             rospy.get_param("~warning_success_rate", 0.8),
                                             rospy.get_param("~height_2_5d", 0.1))
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
