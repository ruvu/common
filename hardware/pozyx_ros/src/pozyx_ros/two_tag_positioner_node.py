from collections import namedtuple

import diagnostic_updater
import rospy
from algorithms.multi_tag_positioner import MultiTagPositioner
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from nav_msgs.msg import Odometry
from pozyx_msgs.msg import Ranges
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener, LookupException

from two_tag_positioner_types import Position, Input, Output, Velocity2D, Orientation

Anchor = namedtuple('Anchor', 'network_id frame_id position')  # Position w.r.t. global coordinate frame


class TwoTagPositionerNode:
    def __init__(self, anchors, tag_frame_ids, robot_frame_id, world_frame_id, expected_frequency, warn_success_rate):
        height_2_5d_mm, tag_id_position_mm_map = self._get_height_2_5d_and_tag_id_position_map(tag_frame_ids,
                                                                                               robot_frame_id)
        anchor_locations = {anchor.network_id: [int(anchor.position.x * 1e3), int(anchor.position.y * 1e3),
                                                int(anchor.position.z * 1e3)] for anchor in anchors}

        self._multitag_positioner = MultiTagPositioner(
            anchor_locations=anchor_locations,
            tag_locations=tag_id_position_mm_map,
            height_2_5d=height_2_5d_mm
        )

        self._world_frame_id = world_frame_id
        self._robot_frame_id = robot_frame_id

        self._ranges_subscriber = rospy.Subscriber("uwb_ranges", Ranges, self._ranges_callback, queue_size=1)
        self._odom_msg = None
        self._odom_subscriber = rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1)
        self._pose_publisher = rospy.Publisher("uwb_pose", Odometry, queue_size=1)

        self._warn_success_rate = warn_success_rate
        self._successful_updates = 0
        self._unsuccessful_updates = 0

        # # Initialize diagnostics publisher
        self._diagnostic_updater = diagnostic_updater.Updater()
        self._diagnostic_updater.setHardwareID("")

        # Add frequency monitoring
        self._frequency_status = diagnostic_updater.FrequencyStatus(
            diagnostic_updater.FrequencyStatusParam({'min': expected_frequency, 'max': expected_frequency}))
        self._diagnostic_updater.add(self._frequency_status)
        self._diagnostic_updater.add("UWB Positioning", self._uwb_positioning_diagnostics)

        self._anchor_broadcaster = StaticTransformBroadcaster()
        self._anchor_broadcaster.sendTransform([
            TransformStamped(
                header=Header(frame_id=world_frame_id, stamp=rospy.Time.now()),
                child_frame_id=anchor.frame_id,
                transform=Transform(
                    translation=Vector3(*anchor.position),
                    rotation=Quaternion(w=1)  # Unit quaternion
                )
            )
            for anchor in anchors])

    @staticmethod
    def _get_height_2_5d_and_tag_id_position_map(tag_frame_ids, robot_frame_id, tf_error_sleep_time=2):
        rospy.loginfo("Obtaining tag ids and positions for %s ...", tag_frame_ids)
        # We assume a static transform, non-rotated transform between the robot frame and the tag frame
        # We also neglect the height since we are using 2.5D localization

        tf_buffer = Buffer()
        _ = TransformListener(tf_buffer)
        rospy.sleep(1)  # Give the tf buffer some time too fill

        height_2_5d_mm = None
        tag_id_position_mm_map = {}
        while not rospy.is_shutdown() and len(tag_id_position_mm_map) != len(tag_frame_ids):
            msg = rospy.wait_for_message("uwb_ranges", Ranges)
            for tag_frame_id in tag_frame_ids:
                for r in msg.ranges:
                    if r.network_id not in tag_id_position_mm_map and r.header.frame_id == tag_frame_id:
                        rospy.loginfo("Obtaining tf %s (tag_id=%d) to %s", robot_frame_id, r.network_id, tag_frame_id)
                        try:
                            transform = tf_buffer.lookup_transform(robot_frame_id, tag_frame_id, rospy.Time(0))
                        except LookupException as e:
                            rospy.logerr("%sSleeping for %d seconds", e, tf_error_sleep_time)
                            rospy.sleep(tf_error_sleep_time)
                        else:
                            t = transform.transform.translation
                            h = int(t.z * 1e3)
                            tag_id_position_mm_map[r.network_id] = [int(t.x * 1e3), int(t.y * 1e3), 0]
                            if height_2_5d_mm is None:
                                height_2_5d_mm = h
                            elif h != height_2_5d_mm:
                                raise RuntimeError("{} should be on the same height w.r.t {}", tag_frame_ids,
                                                   robot_frame_id)
        return height_2_5d_mm, tag_id_position_mm_map

    def _uwb_positioning_diagnostics(self, stat):
        success_rate = float(self._successful_updates) / (self._unsuccessful_updates + self._successful_updates)
        if success_rate < self._warn_success_rate:
            stat.summary(DiagnosticStatus.WARN, "Success rate too low: {}".format(success_rate))
        else:
            stat.summary(DiagnosticStatus.OK, "Success rate: {}".format(success_rate))

        stat.add("Position update success rate", success_rate)

        self._unsuccessful_updates = 0
        self._successful_updates = 0

        return stat

    def _odom_callback(self, msg):
        self._odom_msg = msg

    def _ranges_callback(self, msg):
        if self._odom_msg is None:
            rospy.logwarn_throttle(1.0, "No odom message received, skipping ranges message")
        elif not msg.ranges:
            rospy.logwarn("No ranges in message!")
        else:
            twist_time = self._odom_msg.header.stamp.to_sec()
            twist = self._odom_msg.twist.twist
            twist_covariance = self._odom_msg.twist.covariance
            current_time = rospy.get_time()

            ranges_mm = {(r.network_id, r.remote_network_id): int(r.distance * 1e3) for r in msg.ranges}
            positioner_input_mm = Input(
                current_time=current_time,
                velocity_time=twist_time,
                velocity=Velocity2D(x=int(twist.linear.x * 1e3), y=int(twist.linear.y * 1e3), yaw=twist.angular.z),
                covariance=[
                    int(twist_covariance[0] * 1e6), int(twist_covariance[1] * 1e6), int(twist_covariance[5] * 1e3),
                    int(twist_covariance[6] * 1e6), int(twist_covariance[7] * 1e6), int(twist_covariance[11] * 1e3),
                    int(twist_covariance[30] * 1e3), int(twist_covariance[31] * 1e3), twist_covariance[35]
                ]
            )

            try:
                rospy.logdebug("Calling positioning update ...")
                # TODO: why this first timestamp?
                position = self._multitag_positioner.get_position(msg.ranges[0].header.stamp.to_sec(),  # ranges time
                                                                  ranges_mm,
                                                                  positioner_input_mm)
                if not position["success"] or "fallback" in position:
                    raise RuntimeError("Multitag positioning unsuccessful!")

                rospy.logdebug("Positioning update took %.3f seconds", rospy.get_time() - current_time)

                def _position_to_output(p):
                    c = p["diagnostics"]["covariance"]
                    return Output(
                        position=Position(
                            x=p["state"]["position"][0],
                            y=p["state"]["position"][1],
                            z=p["state"]["position"][2]
                        ),
                        orientation=Orientation(
                            roll=p["state"]["orientation"][1],
                            pitch=p["state"]["orientation"][0],
                            yaw=p["state"]["orientation"][2]
                        ),
                        covariance=[
                            c[0][0], c[0][1], 0, 0, 0, c[0][4],
                            c[1][0], c[1][1], 0, 0, 0, c[1][4],
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0, 0,
                            c[4][0], c[4][1], 0, 0, 0, c[4][4]
                        ]
                    )

                estimate = _position_to_output(position)

                c = estimate.covariance
                self._pose_publisher.publish(Odometry(
                    header=Header(
                        stamp=rospy.Time.now(),
                        frame_id=self._world_frame_id
                    ),
                    child_frame_id=self._robot_frame_id,
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
    rospy.init_node('two_tag_positioner_node')

    try:
        anchors = [Anchor(d['network_id'], d['frame_id'], Position(**d['position'])) for d in
                   rospy.get_param('~anchors', [])]
        tag_frame_ids = rospy.get_param("~tag_frame_ids", ["uwb_tag_left", "uwb_tag_right"])

        if len(anchors) < 3:
            raise ValueError("There should be at least 3 ~anchors")

        if len(tag_frame_ids) != 2:
            raise ValueError("~tag_frame_ids should be of size 2")
    except KeyError as e:
        rospy.logerr("Missing key {} in anchor specification".format(e))
    except ValueError as e:
        rospy.logerr("{}".format(e))
    else:
        try:
            two_tag_positioner_node = TwoTagPositionerNode(anchors,
                                                           tag_frame_ids,
                                                           rospy.get_param("~robot_frame_id", "base_link"),
                                                           rospy.get_param("~world_frame_id", "map"),
                                                           rospy.get_param("~expected_frequency", 5),
                                                           rospy.get_param("~warning_success_rate", 0.8))
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
