"""coordinator – sentral planleggjar for heile laget.

Abonnerer:
  /occupancy_map        nav_msgs/OccupancyGrid       frå map_manager
  /coverage_map         nav_msgs/OccupancyGrid       frå map_manager
  /<ns>/odom            nav_msgs/Odometry
  /<ns>/nav_status      std_msgs/String              frå robot_controller
  /marker_found         geometry_msgs/PointStamped   (point.z = marker-ID) frå marker_reporter
Publiserer:
  /<ns>/plan            nav_msgs/Path                rute til kvar robot (tom = stopp)
  /coverage_reset       std_msgs/Float32
  /exploration_goals    visualization_msgs/MarkerArray   (piler i RViz)
  /coordinator/state    std_msgs/String
"""
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32, String
from visualization_msgs.msg import Marker, MarkerArray

from multi_robot_challenge_23 import rescue_core as rc
from multi_robot_challenge_23.ros_utils import latched_qos, yaw_from_quat, quat_from_yaw, msg_to_array, goal_id


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')
        self.declare_parameter('namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('seen_range', 2.5)
        self.declare_parameter('camera_fov', 0.90)
        self.declare_parameter('map_update_period', 3.0)   # s – oppdater planleggingskartet maks så ofte
        self.declare_parameter('territory_weight', 1.0)
        self.declare_parameter('dist_weight', 1.2)
        self.names = list(self.get_parameter('namespaces').value)
        self.map_period = float(self.get_parameter('map_update_period').value)

        self.core = rc.Coordinator(self.names, log=lambda s: self.get_logger().info(s))
        self.core.territory_w = float(self.get_parameter('territory_weight').value)
        self.core.reset_cb = self.send_reset
        self.cov = None
        self.spec = None
        self.pending_map = None
        self.last_map_t = -1e9
        self.poses = {}
        self.start_poses = {}
        self.sent_goal = {n: None for n in self.names}      # siste mål-ID sendt til kvar robot

        self.create_subscription(OccupancyGrid, '/occupancy_map', self.clbk_occ, latched_qos())
        self.create_subscription(OccupancyGrid, '/coverage_map', self.clbk_cov, latched_qos())
        self.create_subscription(PointStamped, '/marker_found', self.clbk_marker, latched_qos(10))
        self.plan_pubs = {}
        for n in self.names:
            self.create_subscription(Odometry, f'/{n}/odom', lambda m, n=n: self.clbk_odom(m, n), 10)
            self.create_subscription(String, f'/{n}/nav_status', lambda m, n=n: self.clbk_status(m, n), 10)
            self.plan_pubs[n] = self.create_publisher(Path, f'/{n}/plan', 10)
        self.reset_pub = self.create_publisher(Float32, '/coverage_reset', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/exploration_goals', 10)
        self.state_pub = self.create_publisher(String, '/coordinator/state', 10)

        self.create_timer(0.5, self.tick)
        self.get_logger().info('coordinator startar – ventar på kart og odom')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def clbk_odom(self, msg, n):
        p = msg.pose.pose
        self.poses[n] = (p.position.x, p.position.y, yaw_from_quat(p.orientation))
        self.start_poses.setdefault(n, self.poses[n])

    def clbk_status(self, msg, n):
        parts = msg.data.split()
        if len(parts) < 2 or parts[1] != self.sent_goal.get(n):
            return                                         # status for eit gammalt mål – ignorer
        self.core.on_status(n, parts[0])

    def clbk_marker(self, msg):
        self.core.on_marker(int(round(msg.point.z)), msg.point.x, msg.point.y)

    def clbk_occ(self, msg):
        self.pending_map = msg                             # blir brukt i tick() (maks kvart 3. sekund)

    def clbk_cov(self, msg):
        if self.cov is None or msg.info.width != self.spec.w or msg.info.height != self.spec.h:
            return
        self.cov.seen[:] = msg_to_array(msg) == 100

    def send_reset(self, seen_range):
        self.reset_pub.publish(Float32(data=float(seen_range)))

    # ------------------------------------------------------------------ hovudløkke
    def apply_map(self):
        msg = self.pending_map
        self.pending_map = None
        a = msg_to_array(msg)
        o = msg.info.origin.position
        spec = rc.GridSpec(msg.info.resolution, o.x, o.y, msg.info.height, msg.info.width)
        occ = a >= 50
        if self.cov is None:
            self.cov = rc.Coverage(spec, seen_range=float(self.get_parameter('seen_range').value),
                                   fov=float(self.get_parameter('camera_fov').value))
            self.spec = spec
        t0 = time.time()
        self.core.set_map(occ, spec, [p[:2] for p in self.start_poses.values()], self.cov)
        self.core.explorer.dist_weight = float(self.get_parameter('dist_weight').value)
        self.get_logger().info(f'Kart oppdatert ({time.time() - t0:.2f} s)', throttle_duration_sec=30.0)

    def tick(self):
        if len(self.start_poses) < len(self.names):
            return
        now = self.now()
        if self.pending_map is not None and (self.core.pg is None or now - self.last_map_t > self.map_period):
            self.last_map_t = now
            self.apply_map()
        if self.core.pg is None:
            return
        t0 = time.time()
        cmds = self.core.update(dict(self.poses), now)
        dt = time.time() - t0
        for n, (wps, yaw) in cmds.items():
            self.send_path(n, wps, yaw)
        if cmds:
            self.publish_viz()
            if dt > 0.5:
                self.get_logger().info(f'Planlegging tok {dt:.2f} s')
        self.state_pub.publish(String(data=f'{self.core.mode} found={sorted(self.core.found)} '
                                           f'fire_done={self.core.fire_done} unseen={self.cov.remaining()}'))

    def send_path(self, n, wps, yaw):
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()   # tidsstempelet blir mål-ID-en
        for i, (x, y) in enumerate(wps):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            if i == len(wps) - 1 and yaw is not None:
                q = quat_from_yaw(yaw)
                ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
            elif i == len(wps) - 1:
                ps.pose.orientation.w = 0.0                       # ingen sluttretning
            else:
                ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.sent_goal[n] = goal_id(path.header.stamp)
        self.plan_pubs[n].publish(path)
        g = self.core.goals.get(n)
        if wps:
            self.get_logger().info(f'{n}: nytt mål ({wps[-1][0]:.1f}, {wps[-1][1]:.1f}) '
                                   f'{g.get("kind") if g else ""} gain={g.get("gain", "-") if g else "-"}')

    def publish_viz(self):
        """Ei pil per robot i RViz: kvar han skal og kva retning kameraet skal peike."""
        arr = MarkerArray()
        cols = [(0.0, 0.4, 1.0), (1.0, 0.55, 0.0), (0.2, 0.8, 0.2), (0.8, 0.2, 0.8), (0.5, 0.5, 0.5)]
        for k, n in enumerate(self.names):
            g = self.core.goals.get(n)
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type = 'goals', k, Marker.ARROW
            m.action = Marker.ADD if g is not None else Marker.DELETE
            if g is not None:
                m.pose.position.x, m.pose.position.y = g['x'], g['y']
                q = quat_from_yaw(g.get('yaw', 0.0))
                m.pose.orientation.z, m.pose.orientation.w = q[2], q[3]
            m.scale.x, m.scale.y, m.scale.z = 0.5, 0.1, 0.1
            c = cols[k % len(cols)]
            m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 1.0
            arr.markers.append(m)
        self.viz_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()