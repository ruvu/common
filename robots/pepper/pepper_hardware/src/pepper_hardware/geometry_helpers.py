import almath

from geometry_msgs.msg import Pose, Point, Quaternion


def get_ros_pose_from_pose_array_azimuth(pose_array, azimuth):
    rotation_z = almath.transformFromRotZ(azimuth)
    transform = almath.transformFromPosition6D(almath.Position6D(*pose_array)) * rotation_z

    t = almath.position3DFromTransform(transform)
    q = almath.quaternionFromTransform(transform)

    return Pose(
        position=Point(
            x=t.x,
            y=t.y,
            z=t.z
        ),
        orientation=Quaternion(
            x=q.x,
            y=q.y,
            z=q.z,
            w=q.w
        )
    )



