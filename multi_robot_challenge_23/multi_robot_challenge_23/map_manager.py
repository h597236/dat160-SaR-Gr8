"""map_manager – occupancy grid (blueprint + lidar) og kamera-dekningskart.

Abonnerer:
  /map                      nav_msgs/OccupancyGrid   blueprint frå map_server (0.01 m)
  /<ns>/odom, /<ns>/scan    for kvar robot
  /<ns>/camera/image_raw    sensor_msgs/Image        fargefilter: farga veggar blir registrerte som skanna
  /coverage_reset           std_msgs/Float32         ny seen_range – nullstiller dekningskartet
Publiserer:
  /occupancy_map            nav_msgs/OccupancyGrid   0.05 m, 100 = okkupert, 0 = fritt (latched)
  /coverage_map             nav_msgs/OccupancyGrid   -1 = ikkje veggflate, 0 = usett vegg, 100 = sett vegg
"""
import bisect
import math
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32

from multi_robot_challenge_23 import rescue_core as rc
from multi_robot_challenge_23.ros_utils import latched_qos, yaw_from_quat, grid_to_msg


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('seen_range', 2.5)
        self.declare_parameter('camera_fov', 0.90)
        self.declare_parameter('lidar_x_offset', -0.064)   # base_scan i høve til base_footprint (waffle_pi)
        self.declare_parameter('camera_x_offset', 0.073)
        self.declare_parameter('use_lidar_mapping', True)
        self.names = list(self.get_parameter('namespaces').value)
        self.res = float(self.get_parameter('resolution').value)
        self.lidar_dx = float(self.get_parameter('lidar_x_offset').value)
        self.cam_dx = float(self.get_parameter('camera_x_offset').value)
        self.use_lidar = bool(self.get_parameter('use_lidar_mapping').value)
        # fargefilter: alle markørane heng på kvite/grå veggar
        self.declare_parameter('use_wall_colors', True)
        self.declare_parameter('color_period', 0.5)          # s mellom kvart bilete per robot
        self.declare_parameter('color_max_range', 4.0)       # m
        self.declare_parameter('color_max_turn_rate', 0.35)  # rad/s – hopp over bilete når roboten snur fort
        gp = lambda k: self.get_parameter(k).value  # noqa: E731
        self.use_colors = bool(gp('use_wall_colors'))
        self.color_period = float(gp('color_period'))
        self.color_range = float(gp('color_max_range'))
        self.color_max_w = float(gp('color_max_turn_rate'))
        self.wcc = rc.WallColorClassifier()
        self.odom_hist = {}      # namn -> siste odom-posar, for å finne posen då biletet vart teke
        self.last_img_t = {}
        self.img_count = 0

        self.mapper = None
        self.cov = None
        self.interior_ready = False
        self.poses = {}
        self.start_poses = {}
        self.published_version = -1

        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, latched_qos())
        for n in self.names:
            self.create_subscription(Odometry, f'/{n}/odom', lambda m, n=n: self.clbk_odom(m, n), 10)
            self.create_subscription(LaserScan, f'/{n}/scan', lambda m, n=n: self.clbk_scan(m, n),
                                     qos_profile_sensor_data)
            if self.use_colors:
                self.create_subscription(Image, f'/{n}/camera/image_raw', lambda m, n=n: self.clbk_image(m, n),
                                         qos_profile_sensor_data)
        self.create_subscription(Float32, '/coverage_reset', self.clbk_reset, 10)

        self.occ_pub = self.create_publisher(OccupancyGrid, '/occupancy_map', latched_qos())
        self.cov_pub = self.create_publisher(OccupancyGrid, '/coverage_map', latched_qos())

        self.create_timer(0.1, self.camera_timer)
        self.create_timer(1.0, self.publish_timer)
        self.get_logger().info('map_manager startar – ventar på /map og odom')

    # ------------------------------------------------------------------ callbacks
    def clbk_map(self, msg):
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        o = msg.info.origin.position
        self.mapper = rc.OccupancyMapper(data, msg.info.resolution, o.x, o.y, res=self.res)
        self.cov = rc.Coverage(self.mapper.spec,
                               seen_range=float(self.get_parameter('seen_range').value),
                               fov=float(self.get_parameter('camera_fov').value))
        self.interior_ready = False
        self.get_logger().info(f'Blueprint motteke: {msg.info.width}x{msg.info.height} -> '
                               f'{self.mapper.spec.w}x{self.mapper.spec.h} @ {self.res} m')

    def clbk_odom(self, msg, n):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.poses[n] = pose
        self.start_poses.setdefault(n, pose)
        st = msg.header.stamp
        h = self.odom_hist.setdefault(n, deque(maxlen=60))
        h.append((st.sec + st.nanosec * 1e-9, pose[0], pose[1], pose[2], msg.twist.twist.angular.z))

    def pose_at(self, n, t):
        """Odom-posen nærast tidspunktet t (då biletet vart teke). None om ingen innan 0.2 s."""
        h = self.odom_hist.get(n)
        if not h:
            return None
        ts = [e[0] for e in h]
        k = bisect.bisect_left(ts, t)
        cand = [h[i] for i in (k - 1, k) if 0 <= i < len(h)]
        best = min(cand, key=lambda e: abs(e[0] - t))
        return best if abs(best[0] - t) <= 0.2 else None

    def clbk_image(self, msg, n):
        """Klassifiser veggfargen i biletet og stem på veggane (sjå rescue_core.WallColorClassifier)."""
        if self.cov is None or not self.interior_ready:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last_img_t.get(n, -1e9) < self.color_period:
            return
        pose = self.pose_at(n, t)
        if pose is None or abs(pose[4]) > self.color_max_w:
            return                                   # ingen pose for bilettida, eller roboten snur for fort
        img = rc.image_to_rgb(msg.data, msg.height, msg.width, msg.step, msg.encoding)
        if img is None:
            self.get_logger().warn(f'Ukjend biletkoding "{msg.encoding}"', throttle_duration_sec=30.0)
            return
        self.last_img_t[n] = t
        _, x, y, yaw, _ = pose
        cx, cy = x + self.cam_dx * math.cos(yaw), y + self.cam_dx * math.sin(yaw)
        ang, lab = self.wcc.classify(img, cx, cy, yaw, self.cov.occ, self.cov.spec, self.color_range)
        known = lab != 0
        if known.any():
            self.cov.mark_colors(cx, cy, ang[known], lab[known] > 0, self.color_range)
        self.img_count += 1

    def color_report(self):
        if self.cov is not None and self.use_colors:
            self.get_logger().info(f'Fargefilter: {self.img_count} bilete, {int(self.cov.excluded.sum())} av '
                                   f'{int(self.cov.targets.sum())} veggceller registrerte som farga (skanna)')

    def clbk_scan(self, msg, n):
        if self.mapper is None or n not in self.poses or not self.use_lidar:
            return
        x, y, yaw = self.poses[n]
        sx, sy = x + self.lidar_dx * math.cos(yaw), y + self.lidar_dx * math.sin(yaw)
        others = [self.poses[o][:2] for o in self.names if o != n and o in self.poses]
        self.mapper.update_scan(sx, sy, yaw, msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_max, others, range_min=max(0.12, msg.range_min))

    def clbk_reset(self, msg):
        if self.cov is not None:
            self.cov.reset(float(msg.data))
            self.get_logger().info(f'Dekningskart nullstilt, seen_range = {msg.data:.1f} m')

    # ------------------------------------------------------------------ timers
    def refresh_coverage_targets(self):
        occ = self.mapper.occ
        seeds = [tuple(int(v) for v in self.mapper.spec.to_cell(p[0], p[1])) for p in self.start_poses.values()]
        interior = rc.flood_fill(~occ, seeds)
        self.cov.set_map(occ, interior)
        self.interior_ready = True

    def camera_timer(self):
        if self.mapper is None or len(self.start_poses) < len(self.names):
            return
        if not self.interior_ready:
            self.refresh_coverage_targets()
        for n in self.names:
            x, y, yaw = self.poses[n]
            self.cov.mark_camera(x + self.cam_dx * math.cos(yaw), y + self.cam_dx * math.sin(yaw), yaw)

    def publish_timer(self):
        if self.mapper is None or not self.interior_ready:
            return
        stamp = self.get_clock().now().to_msg()
        if self.mapper.version != self.published_version:
            self.published_version = self.mapper.version
            self.refresh_coverage_targets()   # nye hindringar -> nye veggflater
            occ = np.where(self.mapper.occ, 100, 0)
            self.occ_pub.publish(grid_to_msg(occ, self.mapper.spec, stamp))
        cov = np.full(self.cov.seen.shape, -1, dtype=np.int8)
        cov[self.cov.targets] = 0
        cov[self.cov.targets & self.cov.seen] = 100
        self.cov_pub.publish(grid_to_msg(cov, self.mapper.spec, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()