#!/usr/bin/env python
import logging
import numpy as np

import PyKDL
import diagnostic_updater
import pozyx_ros.interface as interface
import rospy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Quaternion, TransformStamped, Transform, Vector3, Pose, Point, PoseWithCovariance, \
    TwistWithCovariance, Twist
from nav_msgs.msg import Odometry
from pozyx_msgs.msg import Ranges
from std_msgs.msg import Header, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener, LookupException, TransformBroadcaster


def point_to_vector(point):
    return Vector3(point.x, point.y, point.z)


def xyzw_to_numpy(orientation):
    return np.array([orientation.x, orientation.y, orientation.z, orientation.w])


def pose_to_pose2d(timestamp, pose):
    q = xyzw_to_numpy(pose.orientation)
    return interface.Pose2D(timestamp, pose.position.x, pose.position.y, euler_from_quaternion(q)[2])


def pose2d_with_covariance_to_odom(pose):
    position = Point(pose.x, pose.y, 0)
    orientation = Quaternion(*quaternion_from_euler(0, 0, pose.yaw))
    pose_with_covarience = PoseWithCovariance(pose=Pose(position=position, orientation=orientation),
                                              covariance=pose.covariance)
    twist = TwistWithCovariance(twist=Twist(linear=Vector3(pose.vx, pose.vy, 0), angular=Vector3(0, 0, pose.vyaw)))
    return Odometry(pose=pose_with_covarience, twist=twist)


def pose_to_kdl(msg):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(msg.orientation.x, msg.orientation.y,
                                                 msg.orientation.z, msg.orientation.w),
                       PyKDL.Vector(msg.position.x, msg.position.y, msg.position.z))


class TwoTagPositionerNode:
    def __init__(self, anchors, tag_frame_ids, robot_frame_id, world_frame_id, publish_tf, odom_timeout,
                 expected_frequency, warn_success_rate):

        self._world_frame_id = world_frame_id
        self._robot_frame_id = robot_frame_id

        self._tf_broadcaster = TransformBroadcaster() if publish_tf else None

        self._odom_msg = None
        self._odom_subscriber = rospy.Subscriber("odom", Odometry, self._odom_callback, queue_size=1)

        if rospy.get_param('/use_sim_time', False):
            rospy.logwarn('Simulation mode, only listening to uwb_pose and publishing tf')
            self._uwb_pose_sub = rospy.Subscriber("uwb_pose", Odometry, self._uwb_pose_callback, queue_size=10)
            return

        height_2_5d, tag_id_position_map = self._get_height_2_5d_and_tag_id_position_map(tag_frame_ids,
                                                                                         robot_frame_id)
        self._multitag_positioner = interface.MultiTagPositioner(
            anchor_locations=anchors,
            tag_locations=tag_id_position_map,
            height_2_5d=height_2_5d
        )

        self._ranges_subscriber = rospy.Subscriber("uwb_ranges", Ranges, self._ranges_callback, queue_size=1)
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
                child_frame_id=str(anchor.network_id),
                transform=Transform(
                    translation=point_to_vector(anchor.point),
                    rotation=Quaternion(w=1)  # Unit quaternion
                )
            )
            for anchor in anchors])

        self._odom_timeout = odom_timeout

    @staticmethod
    def _get_height_2_5d_and_tag_id_position_map(tag_frame_ids, robot_frame_id, tf_error_sleep_time=2):
        rospy.loginfo("Obtaining tag ids and positions for %s ...", tag_frame_ids)
        # We assume a static transform, non-rotated transform between the robot frame and the tag frame
        # We also neglect the height since we are using 2.5D localization

        tf_buffer = Buffer()
        _ = TransformListener(tf_buffer)
        rospy.sleep(1)  # Give the tf buffer some time too fill

        height_2_5d_mm = None
        tag_id_position_map = {}

        while not rospy.is_shutdown() and len(tag_id_position_map) != len(tag_frame_ids):
            msg = rospy.wait_for_message("uwb_ranges", Ranges)
            for tag_frame_id in tag_frame_ids:
                for r in msg.ranges:
                    if r.network_id not in tag_id_position_map and r.header.frame_id == tag_frame_id:
                        rospy.loginfo("Obtaining tf %s (tag_id=%d) to %s", robot_frame_id, r.network_id, tag_frame_id)
                        try:
                            transform = tf_buffer.lookup_transform(robot_frame_id, tag_frame_id, rospy.Time(0))
                        except LookupException as e:
                            rospy.logerr("%sSleeping for %d seconds", e, tf_error_sleep_time)
                            rospy.sleep(tf_error_sleep_time)
                        else:
                            t = transform.transform.translation
                            tag_id_position_map[r.network_id] = interface.Point(t.x, t.y, 0)
                            if height_2_5d_mm is None:
                                height_2_5d_mm = t.z
                            elif t.z != height_2_5d_mm:
                                raise RuntimeError("{} should be on the same height w.r.t {}", tag_frame_ids,
                                                   robot_frame_id)
        return height_2_5d_mm, [interface.DeviceLocation(network_id=i, point=p) for (i, p) in
                                tag_id_position_map.items()]

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
        elif rospy.get_time() - self._odom_msg.header.stamp.to_sec() > self._odom_timeout:
            rospy.logwarn("Odom message too old, last received at %.3f", self._odom_msg.header.stamp.to_sec())
        elif not msg.ranges:
            rospy.logwarn("No ranges in message!")
        else:
            current_time = rospy.get_time()

            ranges = [
                interface.UWBRange(network_id=r.network_id, remote_network_id=r.remote_network_id, distance=r.distance,
                                   timestamp=r.header.stamp.to_sec()) for r in msg.ranges]
            odom_pose = pose_to_pose2d(self._odom_msg.header.stamp.to_sec(), self._odom_msg.pose.pose)

            try:
                rospy.logdebug("Calling positioning update ...")
                position = self._multitag_positioner.get_position(ranges, odom_pose=odom_pose)
                rospy.logdebug("Positioning update took %.3f seconds", rospy.get_time() - current_time)

                localization_odom_msg = pose2d_with_covariance_to_odom(position)

                localization_odom_msg.header = Header(stamp=rospy.Time.from_sec(position.timestamp),
                                                      frame_id=self._world_frame_id)
                localization_odom_msg.child_frame_id = self._robot_frame_id

                self._pose_publisher.publish(localization_odom_msg)
                self._frequency_status.tick()
            except RuntimeError as runtime_error:
                self._unsuccessful_updates += 1
                rospy.logerr(runtime_error)
            else:
                self._successful_updates += 1
                rospy.logdebug("Position update successful")

                self.publish_tf(localization_odom_msg)

                self._diagnostic_updater.update()

    def publish_tf(self, localization_odom_msg):
        if not self._tf_broadcaster:
            return

        # NOTE: we neglect the difference in time between odom and range. We assume here that the odom
        # message are coming in with a much higher rate than the ranges.
        odom_to_base_link = pose_to_kdl(self._odom_msg.pose.pose)
        map_to_base_link = pose_to_kdl(localization_odom_msg.pose.pose)

        # map->base_link = map->odom * odom->base_link
        # map->odom = inv(odom->base_link) * map->base_link
        map_to_odom = map_to_base_link * odom_to_base_link.Inverse()

        self._tf_broadcaster.sendTransform(
            TransformStamped(
                header=localization_odom_msg.header,
                child_frame_id=self._odom_msg.header.frame_id,
                transform=Transform(
                    translation=Vector3(*map_to_odom.p),
                    rotation=Quaternion(*map_to_odom.M.GetQuaternion())
                )
            )
        )

    def _uwb_pose_callback(self, pose):
        if self._odom_msg is None:
            rospy.logwarn_throttle(1.0, "No odom message received, skipping uwb pose message")
            return

        self.publish_tf(pose)


class Handler(logging.Handler):
    def __init__(self, *args, **kwargs):
        super(Handler, self).__init__(*args, **kwargs)
        self.pub = rospy.Publisher('~pozyx_tracing', String, queue_size=10)

    def emit(self, record):
        self.pub.publish(record.getMessage())


if __name__ == '__main__':
    rospy.init_node('two_tag_positioner_node')

    logger = logging.getLogger('pozyx_ros.interface')
    logger.setLevel(logging.DEBUG)
    logger.addHandler(Handler())

    try:
        anchors = rospy.get_param('~anchors')
        anchors = [interface.DeviceLocation(network_id=d['network_id'], point=interface.Point(**d['position'])) for d in
                   anchors]

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
                                                           rospy.get_param("~publish_tf", False),
                                                           rospy.get_param("~odom_timeout", 0.1),
                                                           rospy.get_param("~expected_frequency", 5),
                                                           rospy.get_param("~warning_success_rate", 0.8))
            rospy.spin()
        except rospy.ROSInterruptException as e:
            rospy.logwarn(e)
        except RuntimeError as e:
            rospy.logerr(e)
