import diagnostic_updater
import rospy
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import StaticTransformBroadcaster

from two_tag_positioner import Tag, Anchor, Position, UWBSettings, TwoTagPositioner, Input, Velocity2D

from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler


class TwoTagPositionerNode:
    def __init__(self, tags, anchors, uwb_settings, world_frame_id, sensor_frame_id, expected_frequency):
        def _position_to_mm(tag_or_anchor):
            return tag_or_anchor._replace(position=Position(*[int(p * 1e3) for p in tag_or_anchor.position]))

        self._two_tag_positioner = TwoTagPositioner([_position_to_mm(tag) for tag in tags],
                                                    [_position_to_mm(anchor) for anchor in anchors], uwb_settings)

        self._world_frame_id = world_frame_id
        self._sensor_frame_id = sensor_frame_id

        self._odom_publisher = rospy.Publisher("odom", Odometry, queue_size=1)

        # # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("_".join([tag.serial_port for tag in tags]))

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)

        self._anchor_broadcaster = StaticTransformBroadcaster()
        for anchor in anchors:
            anchor_frame_id = "anchor_{}".format(anchor.network_id)
            rospy.loginfo("Broadcasting static transform from %s to %s", world_frame_id, anchor_frame_id)
            self._anchor_broadcaster.sendTransform(TransformStamped(
                header=Header(frame_id=world_frame_id, stamp=rospy.Time.now()),
                child_frame_id=anchor_frame_id,
                transform=Transform(
                    translation=Vector3(*anchor.position),
                    rotation=Quaternion(w=1)  # Unit quaternion
                )
            ))

    def spin(self):
        while not rospy.is_shutdown():
            try:
                estimate = self._two_tag_positioner.get_position(
                    Input(
                        velocity=Velocity2D(x=0, y=0, yaw=0),
                        covariance=[1e3] * 9
                    )
                )

                self._odom_publisher.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._world_frame_id
                    ),
                    child_frame_id=self._sensor_frame_id,
                    pose=PoseWithCovariance(
                        pose=Pose(position=Point(*[p / 1e3 for p in estimate.position]),
                                  orientation=Quaternion(*quaternion_from_euler(0, 0, estimate.orientation.yaw)))
                    )
                ))

                self._frequency_status.tick()
            except RuntimeError as runtime_error:
                rospy.logerr(runtime_error)
            else:
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
                                             rospy.get_param("~expected_frequency", 15.0))
            ros_pozyx.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
