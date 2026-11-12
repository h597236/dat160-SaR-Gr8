import heapq
import math
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Twist, Point, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Empty


class GridPlanner:

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 cell_size=0.05, robot_radius=0.22, wall_margin=0.5, shortcut_clearance=0.3):
        self.res = cell_size
        self.ox = origin_x
        self.oy = origin_y

        f = max(1, int(round(cell_size / resolution)))
        grid = np.array(data, dtype=np.int16).reshape(height, width)
        h, w = height // f, width // f
        grid = grid[:h * f, :w * f].reshape(h, f, w, f)
        self.walls = (grid >= 50).any(axis=(1, 3))
        self.h, self.w = h, w

        self.robot_cells = int(math.ceil(robot_radius / cell_size))
        self.margin_cells = int(math.ceil(wall_margin / cell_size))
        self.shortcut_cells = int(math.ceil(shortcut_clearance / cell_size))

        self.dynamic = np.zeros_like(self.walls)
        self.update_costs()
        self.known_wall = self.dist <= 3
        self.blocked_static = self.blocked.copy()

    def distance_to_obstacle(self, obstacles):
        dist = np.full(obstacles.shape, self.margin_cells + 1, dtype=np.int16)
        dist[obstacles] = 0
        current = obstacles.copy()
        for d in range(1, self.margin_cells + 1):
            grown = current.copy()
            grown[1:, :] |= current[:-1, :]
            grown[:-1, :] |= current[1:, :]
            grown[:, 1:] |= current[:, :-1]
            grown[:, :-1] |= current[:, 1:]
            if d % 2 == 0:
                grown[1:, 1:] |= current[:-1, :-1]
                grown[1:, :-1] |= current[:-1, 1:]
                grown[:-1, 1:] |= current[1:, :-1]
                grown[:-1, :-1] |= current[1:, 1:]
            dist[grown & ~current] = d
            current = grown
        return dist

    def update_costs(self):
        self.dist = self.distance_to_obstacle(self.walls | self.dynamic)
        self.blocked = self.dist <= self.robot_cells
        closeness = np.clip(self.margin_cells - self.dist, 0, None) / max(1, self.margin_cells - self.robot_cells)
        self.cost = 1.0 + 4.0 * closeness

    def set_dynamic(self, points):
        new = np.zeros_like(self.walls)
        for x, y in points:
            r, c = self.to_cell(x, y)
            if 0 <= r < self.h and 0 <= c < self.w and not self.known_wall[r, c]:
                new[r, c] = True
        if new.any() or self.dynamic.any():
            changed = (new != self.dynamic).any()
            self.dynamic = new
            if changed:
                self.update_costs()

    def to_cell(self, x, y):
        return int((y - self.oy) / self.res), int((x - self.ox) / self.res)

    def to_world(self, r, c):
        return self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res

    def nearest_free(self, cell, max_dist=1.0):
        if not self.blocked[cell]:
            return cell
        limit = int(max_dist / self.res)
        seen = {cell}
        queue = [cell]
        while queue:
            nxt = []
            for r, c in queue:
                for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    n = (r + dr, c + dc)
                    if n in seen or not (0 <= n[0] < self.h and 0 <= n[1] < self.w):
                        continue
                    if abs(n[0] - cell[0]) > limit or abs(n[1] - cell[1]) > limit:
                        continue
                    if not self.blocked[n]:
                        return n
                    seen.add(n)
                    nxt.append(n)
            queue = nxt
        return None

    def plan(self, start_xy, goal_xy):
        robot_cell = self.to_cell(*start_xy)
        if not (0 <= robot_cell[0] < self.h and 0 <= robot_cell[1] < self.w):
            return None
        start = self.nearest_free(robot_cell, max_dist=0.5)
        goal = self.nearest_free(self.to_cell(*goal_xy))
        if start is None or goal is None:
            return None

        neighbours = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                      (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        g = {start: 0.0}
        parent = {start: None}
        open_list = [(0.0, start)]
        found = False

        while open_list:
            _, cur = heapq.heappop(open_list)
            if cur == goal:
                found = True
                break
            for dr, dc, step in neighbours:
                n = (cur[0] + dr, cur[1] + dc)
                if not (0 <= n[0] < self.h and 0 <= n[1] < self.w) or self.blocked[n]:
                    continue
                new_g = g[cur] + step * self.cost[n]
                if new_g < g.get(n, math.inf):
                    g[n] = new_g
                    parent[n] = cur
                    h = math.hypot(goal[0] - n[0], goal[1] - n[1])
                    heapq.heappush(open_list, (new_g + h, n))

        if not found:
            return None

        cells = []
        cur = goal
        while cur is not None:
            cells.append(cur)
            cur = parent[cur]
        cells.reverse()

        path = [self.to_world(r, c) for r, c in self.shortcut(cells)]
        return [start_xy] + path[1:] if start == robot_cell else [start_xy] + path

    def line_clear(self, a, b):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        for i in range(n + 1):
            r = int(round(a[0] + (b[0] - a[0]) * i / max(n, 1)))
            c = int(round(a[1] + (b[1] - a[1]) * i / max(n, 1)))
            if self.dist[r, c] < self.shortcut_cells:
                return False
        return True

    def shortcut(self, cells):
        result = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self.line_clear(cells[i], cells[j]):
                j -= 1
            result.append(cells[j])
            i = j
        return result

    def path_blocked(self, waypoints):
        r0, c0 = self.to_cell(*waypoints[0])
        for a, b in zip(waypoints, waypoints[1:]):
            ca, cb = self.to_cell(*a), self.to_cell(*b)
            n = max(abs(cb[0] - ca[0]), abs(cb[1] - ca[1]), 1)
            for i in range(n + 1):
                r = int(round(ca[0] + (cb[0] - ca[0]) * i / n))
                c = int(round(ca[1] + (cb[1] - ca[1]) * i / n))
                near_robot = abs(r - r0) <= self.robot_cells and abs(c - c0) <= self.robot_cells
                if self.blocked[r, c] and not self.blocked_static[r, c] and not near_robot:
                    return True
        return False


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info("Navigator node created")

        self.declare_parameter('speed', 0.26)
        self.declare_parameter('turn_speed', 1.8)
        self.declare_parameter('face_turn_speed', 1.0)

        self.forward_speed = float(self.get_parameter('speed').value)
        self.max_turn = float(self.get_parameter('turn_speed').value)
        self.face_turn = float(self.get_parameter('face_turn_speed').value)
        self.kp_turn = 2.5
        self.yaw_precision = 0.1
        self.yaw_refix = 0.6
        self.waypoint_tol = 0.15
        self.goal_tol = 0.10
        self.front_stop = 0.25
        self.scan_range = 1.5
        self.back_speed = 0.1
        self.back_time = (0.5, 1.5)
        self.max_retries = 5

        self.state = "idle"
        self.goal_reached = False
        self.retries = 0
        self.back_off_until = 0.0
        self.rear_min = math.inf
        self.position = None
        self.yaw = 0.0
        self.planner = None
        self.goal = None
        self.waypoints = []
        self.face_target = None
        self.scan = None
        self.front_min = math.inf

        map_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)

        self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.clbk_scan, 10)
        self.create_subscription(Point, 'goal', self.clbk_goal, 10)
        self.create_subscription(Point, 'face', self.clbk_face, 10)
        self.create_subscription(Empty, 'stop', self.clbk_stop, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'nav_status', 10)
        self.path_pub = self.create_publisher(Path, 'plan', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.check_timer = self.create_timer(1.0, self.check_path)

    def clbk_map(self, msg):
        info = msg.info
        self.planner = GridPlanner(msg.data, info.width, info.height, info.resolution,
                                   info.origin.position.x, info.origin.position.y)
        self.get_logger().info(f"Map received, planning grid {self.planner.w}x{self.planner.h}")

    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def clbk_scan(self, msg):
        self.scan = msg
        front = []
        rear = []
        for i, r in enumerate(msg.ranges):
            a = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                           math.cos(msg.angle_min + i * msg.angle_increment))
            if not msg.range_min < r < msg.range_max:
                continue
            if abs(a) < math.radians(25):
                front.append(r)
            elif abs(a) > math.radians(155):
                rear.append(r)
        self.front_min = min(front) if front else math.inf
        self.rear_min = min(rear) if rear else math.inf

    def clbk_goal(self, msg):
        new_goal = (msg.x, msg.y)

        if self.goal is not None and math.hypot(new_goal[0] - self.goal[0], new_goal[1] - self.goal[1]) < 0.05:
            if self.goal_reached:
                self.publish_status("reached")
                return
            if self.state in ("waiting", "fix_yaw", "go_straight", "back_off"):
                return

        self.goal = new_goal
        self.goal_reached = False
        self.retries = 0
        self.face_target = None
        self.replan()

    def clbk_face(self, msg):
        self.face_target = (msg.x, msg.y)
        self.waypoints = []
        self.set_state("face")

    def clbk_stop(self, msg):
        self.goal = None
        self.waypoints = []
        self.face_target = None
        self.set_state("idle")
        self.publish_status("idle")

    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f"State: {self.state} -> {new_state}")
            self.state = new_state

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def scan_points(self):
        if self.scan is None or self.position is None:
            return []
        points = []
        s = self.scan
        for i, r in enumerate(s.ranges):
            if s.range_min < r < min(s.range_max, self.scan_range):
                a = self.yaw + s.angle_min + i * s.angle_increment
                points.append((self.position.x + r * math.cos(a), self.position.y + r * math.sin(a)))
        return points

    def replan(self):
        if self.goal is None:
            return
        if self.planner is None or self.position is None:
            if self.state != "waiting":
                self.get_logger().info("Goal received, waiting for map and odom before planning")
            self.set_state("waiting")
            return

        self.planner.set_dynamic(self.scan_points())
        path = self.planner.plan((self.position.x, self.position.y), self.goal)

        if path is None and self.retries < self.max_retries:
            self.retries += 1
            self.get_logger().info(f"No path right now, retry {self.retries}/{self.max_retries}")
            self.back_off_until = self.get_clock().now().nanoseconds / 1e9 + random.uniform(*self.back_time)
            self.set_state("back_off")
            return

        if path is None:
            self.get_logger().warn(f"No path to ({self.goal[0]:.2f}, {self.goal[1]:.2f})")
            self.waypoints = []
            self.set_state("idle")
            self.publish_status("failed")
            return

        self.waypoints = path[1:] if len(path) > 1 else path
        self.set_state("fix_yaw")
        self.publish_status("moving")
        self.publish_path(path)

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in path:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def check_path(self):
        if self.state not in ("fix_yaw", "go_straight") or self.planner is None:
            return
        self.planner.set_dynamic(self.scan_points())
        current = (self.position.x, self.position.y)
        if self.planner.path_blocked([current] + self.waypoints):
            self.get_logger().info("Path blocked by an obstacle, replanning")
            self.replan()

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_callback(self):
        if self.state == "waiting" and self.planner is not None and self.position is not None:
            self.replan()
        if self.position is None:
            return

        vel_msg = Twist()
        now = self.get_clock().now().nanoseconds / 1e9

        if self.state == "back_off":
            if now < self.back_off_until:
                if self.rear_min > self.front_stop:
                    vel_msg.linear.x = -self.back_speed
            else:
                self.replan()

        elif self.state == "face":
            dx = self.face_target[0] - self.position.x
            dy = self.face_target[1] - self.position.y
            yaw_error = self.normalize_angle(math.atan2(dy, dx) - self.yaw)
            limit = min(self.max_turn, self.face_turn)
            vel_msg.angular.z = max(-limit, min(limit, self.kp_turn * yaw_error))
            if abs(yaw_error) < self.yaw_precision / 2:
                vel_msg.angular.z = 0.0
                self.set_state("idle")
                self.publish_status("faced")

        elif self.state in ("fix_yaw", "go_straight"):
            tx, ty = self.waypoints[0]
            dx = tx - self.position.x
            dy = ty - self.position.y
            distance = math.hypot(dx, dy)
            yaw_error = self.normalize_angle(math.atan2(dy, dx) - self.yaw)
            last = len(self.waypoints) == 1

            if distance < (self.goal_tol if last else self.waypoint_tol):
                self.waypoints.pop(0)
                if not self.waypoints:
                    self.set_state("idle")
                    self.goal_reached = True
                    self.retries = 0
                    self.publish_status("reached")
                else:
                    self.set_state("fix_yaw")

            elif self.state == "fix_yaw":
                vel_msg.angular.z = max(-self.max_turn, min(self.max_turn, self.kp_turn * yaw_error))
                if abs(yaw_error) < self.yaw_precision:
                    self.set_state("go_straight")

            elif self.state == "go_straight":
                if self.front_min < self.front_stop:
                    self.get_logger().info("Obstacle in front, backing off")
                    self.back_off_until = now + random.uniform(*self.back_time)
                    self.set_state("back_off")
                    self.cmd_vel_pub.publish(Twist())
                    return
                speed = self.forward_speed if not last else min(self.forward_speed, 0.6 * distance + 0.05)
                vel_msg.linear.x = speed
                vel_msg.angular.z = max(-self.max_turn, min(self.max_turn, self.kp_turn * yaw_error))
                if abs(yaw_error) > self.yaw_refix:
                    self.set_state("fix_yaw")

        self.cmd_vel_pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)

    navigator = Navigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.cmd_vel_pub.publish(Twist())
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
