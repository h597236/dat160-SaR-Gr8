import math

import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid


def latched_qos(depth=1):
    """Reliable + transient local: seine abonnentar får siste melding (som /map)."""
    return QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                      durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                      depth=depth)


def yaw_from_quat(q):
    """Kvaternion -> vinkel rundt z-aksen (retninga roboten peikar)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw):
    """Vinkel rundt z -> kvaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def grid_to_msg(arr, spec, stamp, frame='map'):
    """numpy-gitter (rad 0 = botn av kartet) -> nav_msgs/OccupancyGrid."""
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.header.stamp = stamp
    msg.info.resolution = float(spec.res)
    msg.info.width = int(spec.w)
    msg.info.height = int(spec.h)
    msg.info.origin.position.x = float(spec.ox)
    msg.info.origin.position.y = float(spec.oy)
    msg.info.origin.orientation.w = 1.0
    msg.data = np.asarray(arr, dtype=np.int8).reshape(-1).tolist()
    return msg


def msg_to_array(msg):
    """nav_msgs/OccupancyGrid -> numpy-gitter (høgd x breidd)."""
    return np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)


def goal_id(stamp):
    """Unik ID for eit mål, laga frå tidsstempelet på ruta."""
    return f'{stamp.sec}.{stamp.nanosec}'