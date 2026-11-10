import math

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def coloured_fraction(hsv_strip, s_min, v_min):
    s = hsv_strip[:, :, 1]
    v = hsv_strip[:, :, 2]
    return float(np.mean((s > s_min) & (v > v_min)))


def wall_band(dist, cam_h, fy, cy, height, wall_height=2.5, margin=3):
    v_floor = cy + fy * cam_h / dist
    v_top = cy - fy * (wall_height - cam_h) / dist
    top = int(max(0, v_top) + margin)
    bottom = int(min(height, v_floor) - margin)
    return top, bottom


class WallColour(Node):

    def __init__(self):
        super().__init__("wall_colour",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info("Wall colour node created")

        self.declare_parameter('robots', ['tb3_0', 'tb3_1'])
        robots = list(self.get_parameter('robots').value)

        self.declare_parameter('s_min', 70)
        self.declare_parameter('v_min', 40)
        self.declare_parameter('min_fraction', 0.7)
        self.s_min = int(self.get_parameter('s_min').value)
        self.v_min = int(self.get_parameter('v_min').value)
        self.min_fraction = float(self.get_parameter('min_fraction').value)
        self.max_white_fraction = 0.3
        self.votes_needed = 2
        self.column_step = 8
        self.min_dist = 0.4
        self.max_dist = 5.0
        self.max_turn = 0.3
        self.min_rows = 20
        self.cell = 0.1

        self.data = {r: {'image': None, 'info': None, 'scan': None, 'odom': None} for r in robots}
        self.votes = {}
        self.reported = set()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for r in robots:
            self.create_subscription(Image, f'/{r}/camera/image_raw', lambda m, r=r: self.store(r, 'image', m), 2)
            self.create_subscription(CameraInfo, f'/{r}/camera/camera_info', lambda m, r=r: self.store(r, 'info', m), 2)
            self.create_subscription(LaserScan, f'/{r}/scan', lambda m, r=r: self.store(r, 'scan', m), 2)
            self.create_subscription(Odometry, f'/{r}/odom', lambda m, r=r: self.store(r, 'odom', m), 10)

        self.pub = self.create_publisher(Float32MultiArray, '/wall_colour', 10)
        self.marker_pub = self.create_publisher(Marker, '/wall_colour_markers', 10)
        self.all_reported = []
        self.timer = self.create_timer(0.5, self.timer_callback)

    def store(self, robot, key, msg):
        self.data[robot][key] = msg

    def timer_callback(self):
        new_points = []
        for robot, d in self.data.items():
            if any(v is None for v in d.values()):
                continue
            if abs(d['odom'].twist.twist.angular.z) > self.max_turn:
                continue
            new_points += self.process(robot, d)
        if new_points:
            flat = [float(v) for p in new_points for v in p]
            self.pub.publish(Float32MultiArray(data=flat))
            self.all_reported += new_points
            self.publish_markers()

    def publish_markers(self):
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'not_white'
        msg.type = Marker.POINTS
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = 0.08
        msg.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
        msg.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in self.all_reported]
        self.marker_pub.publish(msg)

    def process(self, robot, d):
        try:
            t = self.tf_buffer.lookup_transform('map', f'{robot}/camera_rgb_optical_frame', rclpy.time.Time())
        except TransformException:
            return []
        cam = t.transform.translation
        q = t.transform.rotation
        fx_ = 2 * (q.x * q.z + q.w * q.y)
        fy_ = 2 * (q.y * q.z - q.w * q.x)
        cam_yaw = math.atan2(fy_, fx_)

        info = d['info']
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]

        img = d['image']
        if img.encoding not in ('rgb8', 'bgr8'):
            return []
        frame = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.step // 3, 3)[:, :img.width]
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV if img.encoding == 'rgb8' else cv2.COLOR_BGR2HSV)

        o = d['odom'].pose.pose
        yaw = math.atan2(2 * (o.orientation.w * o.orientation.z + o.orientation.x * o.orientation.y),
                         1 - 2 * (o.orientation.y**2 + o.orientation.z**2))
        s = d['scan']
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + yaw
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        px = o.position.x + r[ok] * np.cos(a[ok])
        py = o.position.y + r[ok] * np.sin(a[ok])
        bearing = np.arctan2(py - cam.y, px - cam.x)
        dist = np.hypot(px - cam.x, py - cam.y)

        found = []
        for u in range(0, img.width - self.column_step, self.column_step):
            ray = cam_yaw + math.atan((cx - (u + self.column_step / 2)) / fx)
            diff = np.abs(np.arctan2(np.sin(bearing - ray), np.cos(bearing - ray)))
            if len(diff) == 0:
                break
            i = int(np.argmin(diff))
            if diff[i] > math.radians(1.5) or not self.min_dist < dist[i] < self.max_dist:
                continue
            top, bottom = wall_band(dist[i], cam.z, fy, cy, img.height)
            if bottom - top < self.min_rows:
                continue
            frac = coloured_fraction(hsv[top:bottom, u:u + self.column_step], self.s_min, self.v_min)

            key = (round(px[i] / self.cell), round(py[i] / self.cell))
            v = self.votes.setdefault(key, [0, 0, 0.0, 0.0])
            if frac >= self.min_fraction:
                v[0] += 1
                v[2] += px[i]
                v[3] += py[i]
            elif frac <= self.max_white_fraction:
                v[1] += 1
            if v[0] >= self.votes_needed and v[1] == 0 and key not in self.reported:
                self.reported.add(key)
                found.append((v[2] / v[0], v[3] / v[0]))
        return found


def main(args=None):
    rclpy.init(args=args)

    node = WallColour()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
