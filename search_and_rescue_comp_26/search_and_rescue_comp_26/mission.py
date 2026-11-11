import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Point
from std_msgs.msg import String, Empty, Int8MultiArray, ColorRGBA
from visualization_msgs.msg import Marker
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition

from search_and_rescue_comp_26.navigator import GridPlanner
from search_and_rescue_comp_26.coverage import Coverage


class Robot:

    def __init__(self, name):
        self.name = name
        self.state = "start"
        self.x = None
        self.y = None
        self.yaw = 0.0
        self.last_yaw = 0.0
        self.nav = "idle"
        self.vp = None
        self.headings = []
        self.goal = None
        self.goal_kind = None
        self.last_send = -math.inf
        self.dwell_until = 0.0
        self.inspect_id = None
        self.inspect_until = 0.0
        self.inspect_marker = None
        self.inspect_front = None
        self.approached = False
        self.cooldown = {}
        self.wall_type = "none"
        self.faced_at = 0.0


class Mission(Node):
    def __init__(self):
        super().__init__("mission",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('robots', ['tb3_0', 'tb3_1'])
        self.declare_parameter('wait_for_scoring', True)
        self.declare_parameter('use_wall_check', True)
        names = list(self.get_parameter('robots').value)
        self.wait_for_scoring = bool(self.get_parameter('wait_for_scoring').value)
        self.use_wall_check = bool(self.get_parameter('use_wall_check').value)

        self.cover_range = 2.3
        self.cover_angle = 55.0
        self.min_new_wall = 15
        self.crowd_radius = 3.0
        self.crowd_penalty = 4.0
        self.dwell_time = 1.2
        self.max_seen_turn = 0.7
        self.seen_needed = 2
        self.colour_settle = 0.2
        self.inspect_time = 6.0
        self.inspect_range = 3.0
        self.inspect_view = 65.0
        self.notice_range = 3.4
        self.approach_dist = 1.5
        self.approach_time = 25.0
        self.meet_radius = 1.7
        self.zone_radius = 1.5
        self.resend_period = 2.0

        self.robots = [Robot(n) for n in names]
        self.map_msg = None
        self.planner = None
        self.coverage = None
        self.vp_done = set()
        self.passes = 0
        self.reported = set()
        self.big_fire = None
        self.big_fire_done = False
        self.meet_since = None
        self.started = False
        self.finished = False
        self.last_seen_update = 0.0

        map_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.clbk_map, map_qos)
        self.create_subscription(Int8MultiArray, '/reported_markers', self.clbk_reported, 10)
        self.create_subscription(Point, '/big_fire_position', self.clbk_big_fire, 10)
        self.scoring_client = self.create_client(SetMarkerPosition, '/set_marker_position')

        self.pubs = {}
        for r in self.robots:
            self.create_subscription(Odometry, f'/{r.name}/odom', lambda m, r=r: self.clbk_odom(m, r), 10)
            self.create_subscription(String, f'/{r.name}/nav_status', lambda m, r=r: self.clbk_nav(m, r), 10)
            self.create_subscription(ArucoMarkers, f'/{r.name}/aruco_markers', lambda m, r=r: self.clbk_aruco(m, r), 10)
            if self.use_wall_check:
                self.create_subscription(String, f'/{r.name}/wall_type', lambda m, r=r: self.clbk_wall_type(m, r), 10)
            self.pubs[r.name] = {
                'goal': self.create_publisher(Point, f'/{r.name}/goal', 10),
                'face': self.create_publisher(Point, f'/{r.name}/face', 10),
                'stop': self.create_publisher(Empty, f'/{r.name}/stop', 10),
            }
        self.viewpoint_pub = self.create_publisher(Marker, '/viewpoints', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)

    def clbk_map(self, msg):
        self.map_msg = msg

    def clbk_odom(self, msg, r):
        r.x = msg.pose.pose.position.x
        r.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        r.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))

    def clbk_nav(self, msg, r):
        r.nav = msg.data

    def clbk_reported(self, msg):
        self.reported = set(int(i) for i in msg.data)

    def clbk_big_fire(self, msg):
        if self.big_fire is None:
            self.big_fire = (msg.x, msg.y)
            self.get_logger().info(f"Big fire at ({msg.x:.2f}, {msg.y:.2f}), calling for help")

    def clbk_wall_type(self, msg, r):
        r.wall_type = msg.data

    def clbk_aruco(self, msg, r):
        if r.x is None or r.state not in ("explore", "to_vp", "scan"):
            return
        now = self.now()
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)
            p = pose.position
            dist = math.sqrt(p.x**2 + p.y**2 + p.z**2)
            if marker_id in self.reported or not 0 <= marker_id <= 4 or dist > self.notice_range:
                continue
            if now < r.cooldown.get(marker_id, 0.0):
                continue

            fwd = (math.cos(r.yaw), math.sin(r.yaw))
            right = (math.sin(r.yaw), -math.cos(r.yaw))
            mx = r.x + p.z * fwd[0] + p.x * right[0]
            my = r.y + p.z * fwd[1] + p.x * right[1]
            q = pose.orientation
            nx = 2 * (q.x * q.z + q.w * q.y)
            ny = 2 * (q.y * q.z - q.w * q.x)
            nz = 1 - 2 * (q.x**2 + q.y**2)
            view = math.degrees(math.acos(min(1.0, abs(nx * p.x + ny * p.y + nz * p.z) / max(dist, 1e-6))))
            normal = (nz * fwd[0] + nx * right[0], nz * fwd[1] + nx * right[1])
            if math.hypot(*normal) < 1e-6:
                normal = (r.x - mx, r.y - my)
            if normal[0] * (r.x - mx) + normal[1] * (r.y - my) < 0:
                normal = (-normal[0], -normal[1])
            length = math.hypot(*normal)

            r.inspect_id = marker_id
            r.inspect_marker = (mx, my)
            r.inspect_front = (mx + normal[0] / length * self.approach_dist, my + normal[1] / length * self.approach_dist)
            r.approached = False
            if dist <= self.inspect_range and view <= self.inspect_view:
                self.start_inspect(r, now)
            else:
                self.start_approach(r, now)
            return

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def set_state(self, r, new_state):
        if new_state != r.state:
            self.get_logger().info(f"{r.name}: {r.state} -> {new_state}")
            r.state = new_state

    def send(self, r, kind, xy):
        self.pubs[r.name][kind].publish(Point(x=float(xy[0]), y=float(xy[1]), z=0.0))
        r.goal = xy
        r.goal_kind = kind
        r.nav = "sent"
        r.last_send = self.now()

    def stop(self, r):
        self.pubs[r.name]['stop'].publish(Empty())
        r.goal = None
        r.goal_kind = None

    def resend(self, r):
        expected = ("moving", "reached", "failed") if r.goal_kind == 'goal' else ("faced",)
        if r.goal is not None and r.nav not in expected and self.now() - r.last_send > self.resend_period:
            self.pubs[r.name][r.goal_kind].publish(Point(x=float(r.goal[0]), y=float(r.goal[1]), z=0.0))
            r.last_send = self.now()

    def distance_map(self, xy):
        p = self.planner
        start = p.nearest_free(p.to_cell(*xy), max_dist=0.5)
        dist = np.full(p.blocked.shape, np.inf)
        if start is None:
            return dist
        dist[start] = 0.0
        queue = deque([start])
        while queue:
            rr, cc = queue.popleft()
            d = dist[rr, cc] + p.res
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (rr + dr, cc + dc)
                if 0 <= n[0] < p.h and 0 <= n[1] < p.w and not p.blocked[n] and dist[n] == np.inf:
                    dist[n] = d
                    queue.append(n)
        return dist

    def lookup(self, dist, xy):
        r, c = self.planner.to_cell(*xy)
        if 0 <= r < self.planner.h and 0 <= c < self.planner.w:
            return dist[r, c]
        return np.inf

    def setup(self):
        m = self.map_msg
        self.planner = GridPlanner(m.data, m.info.width, m.info.height, m.info.resolution,
                                   m.info.origin.position.x, m.info.origin.position.y)
        self.coverage = Coverage(self.planner, (self.robots[0].x, self.robots[0].y),
                                 max_range=self.cover_range, max_view_angle=self.cover_angle)
        vps = self.coverage.plan_viewpoints()
        self.get_logger().info(f"Coverage plan: {len(vps)} viewpoints")

    def start_inspect(self, r, now):
        r.inspect_until = now + self.inspect_time
        self.set_state(r, "inspect")
        self.send(r, 'face', r.inspect_marker)

    def start_approach(self, r, now):
        r.approached = True
        r.inspect_until = now + self.approach_time
        self.set_state(r, "approach")
        self.send(r, 'goal', r.inspect_front)

    def end_inspect(self, r, now):
        if r.inspect_id not in self.reported:
            r.cooldown[r.inspect_id] = now + 10.0
        self.set_state(r, "explore")

    def step_inspect(self, r, now):
        if r.state == "inspect":
            if r.inspect_id in self.reported:
                self.end_inspect(r, now)
            elif now > r.inspect_until:
                if r.approached:
                    self.end_inspect(r, now)
                else:
                    self.start_approach(r, now)
        elif r.state == "approach":
            if r.inspect_id in self.reported:
                self.end_inspect(r, now)
            elif r.nav == "reached":
                self.start_inspect(r, now)
            elif r.nav == "failed" or now > r.inspect_until:
                self.end_inspect(r, now)

    def choose_viewpoint(self, r):
        cov = self.coverage
        taken = [o.vp for o in self.robots if o is not r and o.vp is not None]
        others = [cov.viewpoints[i] for i in taken]
        dist = self.distance_map((r.x, r.y))
        best, best_cost = None, math.inf
        for i, (vx, vy) in enumerate(cov.viewpoints):
            if i in self.vp_done or i in taken:
                continue
            if len(cov.remaining(i)) < self.min_new_wall:
                self.vp_done.add(i)
                continue
            cost = self.lookup(dist, (vx, vy))
            if any(math.hypot(vx - ox, vy - oy) < self.crowd_radius for ox, oy in others):
                cost += self.crowd_penalty
            if cost < best_cost:
                best, best_cost = i, cost
        return best

    def step_explorer(self, r, now):
        cov = self.coverage
        if r.state in ("start", "explore", "no_work"):
            if r.state == "no_work" and now < r.dwell_until:
                return
            if r.vp is None or r.vp in self.vp_done:
                r.vp = self.choose_viewpoint(r)
            if r.vp is None:
                self.out_of_viewpoints(r)
                return
            self.set_state(r, "to_vp")
            self.send(r, 'goal', cov.viewpoints[r.vp])

        elif r.state == "to_vp":
            if len(cov.remaining(r.vp)) < self.min_new_wall:
                self.finish_viewpoint(r)
            elif r.nav == "reached":
                vx, vy = cov.viewpoints[r.vp]
                r.headings = cov.headings_for(vx, vy, cov.scan_targets(r.vp, True))
                self.set_state(r, "scan")
                self.next_heading(r)
            elif r.nav == "failed":
                self.finish_viewpoint(r)

        elif r.state == "scan":
            if r.nav == "faced" and r.dwell_until == 0.0:
                r.dwell_until = now + self.dwell_time
                r.faced_at = now
            elif r.dwell_until and now >= r.dwell_until:
                self.next_heading(r)
            elif r.dwell_until and self.use_wall_check and now - r.faced_at >= self.colour_settle and r.wall_type == "other":
                self.next_heading(r)

    def next_heading(self, r):
        r.dwell_until = 0.0
        if not r.headings:
            self.finish_viewpoint(r)
            return
        h = r.headings.pop(0)
        self.send(r, 'face', (r.x + math.cos(h), r.y + math.sin(h)))

    def finish_viewpoint(self, r):
        self.vp_done.add(r.vp)
        r.vp = None
        self.set_state(r, "explore")

    def out_of_viewpoints(self, r):
        missing = set(range(5)) - self.reported
        busy = any(o.vp is not None for o in self.robots)
        if missing and not busy and self.passes < 2:
            self.passes += 1
            self.get_logger().info(f"Still missing {sorted(missing)}, looking at all walls again")
            self.coverage.seen[:] = self.coverage.not_white
            self.coverage.view_count[:] = 0
            self.coverage.plan_viewpoints(remaining_only=True, goal=1.0)
            self.vp_done = set()
            self.set_state(r, "explore")
        else:
            if r.state != "no_work":
                self.set_state(r, "no_work")
                self.stop(r)
            r.dwell_until = self.now() + 2.0

    def step_big_fire(self, now):
        if self.big_fire is None or self.big_fire_done:
            return
        team = [r for r in self.robots if r.state in ("go_big_fire", "wait_big_fire")]
        if len(team) < 2:
            self.assign_big_fire(team)
            return

        bx, by = self.big_fire
        inside = [r for r in team if math.hypot(r.x - bx, r.y - by) < self.meet_radius]
        if len(inside) >= 2:
            if self.meet_since is None:
                self.meet_since = now
            elif now - self.meet_since > 1.5:
                self.get_logger().info("Both robots at the big fire")
                self.big_fire_done = True
                for r in team:
                    self.set_state(r, "explore")
        else:
            self.meet_since = None

        for r in team:
            if r.state == "go_big_fire" and r.nav == "reached":
                self.set_state(r, "wait_big_fire")
            elif r.state == "go_big_fire" and r.nav == "failed":
                self.send(r, 'goal', r.goal)

    def assign_big_fire(self, team):
        free = [r for r in self.robots if r not in team and r.state != "done"]
        zone = self.big_fire_zone()
        if not free or not zone:
            return
        if not team:
            finder = next((r for r in free if r.inspect_id == 4), None)
            if finder is None:
                finder = min(free, key=lambda r: self.best_zone_spot(r, zone, None)[0])
            self.go_to_big_fire(finder, self.best_zone_spot(finder, zone, None)[1])
            free.remove(finder)
            team = [finder]
            if not free:
                return
        other = team[0].goal if team[0].goal is not None else (team[0].x, team[0].y)
        best = None
        for r in free:
            cost, spot = self.best_zone_spot(r, zone, other)
            if spot is not None and (best is None or cost < best[0]):
                best = (cost, r, spot)
        if best is not None:
            self.go_to_big_fire(best[1], best[2])

    def big_fire_zone(self):
        p = self.planner
        bx, by = self.big_fire
        rc, cc = p.to_cell(bx, by)
        span = int(self.zone_radius / p.res) + 1
        zone = []
        for r in range(max(0, rc - span), min(p.h, rc + span + 1)):
            for c in range(max(0, cc - span), min(p.w, cc + span + 1)):
                if p.blocked[r, c]:
                    continue
                x, y = p.to_world(r, c)
                if math.hypot(x - bx, y - by) <= self.zone_radius:
                    zone.append((r, c, x, y))
        return zone

    def best_zone_spot(self, r, zone, avoid):
        bx, by = self.big_fire
        if math.hypot(r.x - bx, r.y - by) <= self.zone_radius and (avoid is None or math.hypot(r.x - avoid[0], r.y - avoid[1]) >= 0.6):
            return 0.0, (r.x, r.y)
        dist = self.distance_map((r.x, r.y))
        best_c, best_xy = math.inf, None
        for rr, cc, x, y in zone:
            if avoid is not None and math.hypot(x - avoid[0], y - avoid[1]) < 0.6:
                continue
            if dist[rr, cc] < best_c:
                best_c, best_xy = dist[rr, cc], (x, y)
        return best_c, best_xy

    def go_to_big_fire(self, r, xy):
        r.vp = None
        self.set_state(r, "go_big_fire")
        self.send(r, 'goal', xy)

    def publish_viewpoints(self):
        msg = Marker()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = 'viewpoints'
        msg.type = Marker.SPHERE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x = msg.scale.y = msg.scale.z = 0.25
        for i, (x, y) in enumerate(self.coverage.viewpoints):
            msg.points.append(Point(x=float(x), y=float(y), z=0.1))
            msg.colors.append(ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6) if i in self.vp_done else ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0))
        self.viewpoint_pub.publish(msg)

    def timer_callback(self):
        if self.finished:
            return
        if self.coverage is None:
            if self.map_msg is not None and all(r.x is not None for r in self.robots):
                self.setup()
            return
        if not self.started:
            if self.wait_for_scoring and not self.scoring_client.service_is_ready():
                return
            self.get_logger().info("Go!")
            self.started = True

        now = self.now()
        if now - self.last_seen_update > 0.5:
            self.last_seen_update = now
            for r in self.robots:
                turn_rate = abs(math.atan2(math.sin(r.yaw - r.last_yaw), math.cos(r.yaw - r.last_yaw))) / 0.5
                r.last_yaw = r.yaw
                if turn_rate < self.max_seen_turn:
                    self.coverage.mark_seen(r.x, r.y, r.yaw, self.seen_needed)
            self.publish_viewpoints()

        if self.reported >= {0, 1, 2, 3, 4} and (self.big_fire_done or len(self.robots) < 2):
            self.get_logger().info("All markers reported and big fire done, stopping")
            for r in self.robots:
                self.set_state(r, "done")
                self.stop(r)
            self.finished = True
            return

        self.step_big_fire(now)
        for r in self.robots:
            if r.state in ("inspect", "approach"):
                self.step_inspect(r, now)
            elif r.state not in ("go_big_fire", "wait_big_fire", "done"):
                self.step_explorer(r, now)
            self.resend(r)


def main(args=None):
    rclpy.init(args=args)
    mission = Mission()
    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
