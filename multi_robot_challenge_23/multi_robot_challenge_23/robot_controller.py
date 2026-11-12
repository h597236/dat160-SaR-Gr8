"""robot_controller – éin per robot (startast i namespace, t.d. tb3_0).

Abonnerer (relativt til namespace):
  odom   nav_msgs/Odometry
  scan   sensor_msgs/LaserScan
  plan   nav_msgs/Path        rute frå coordinator. Tom path = stopp.
                              Orienteringa til siste pose = sluttretning.
Publiserer:
  cmd_vel     geometry_msgs/Twist
  nav_status  std_msgs/String  "<status> <goal_id>", status = idle|active|reached|blocked|failed

Ved 'blocked'/'failed' ryggar roboten litt (om det er fritt bak) før status blir sendt,
slik at coordinator kan planleggje ei ny rute frå ein betre stad.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from multi_robot_challenge_23 import rescue_core as rc
from multi_robot_challenge_23.ros_utils import yaw_from_quat, goal_id


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.declare_parameter('v_max', 0.4)
        self.declare_parameter('w_max', 0.6)
        self.declare_parameter('safety_dist', 0.28)
        self.declare_parameter('front_half_angle', 0.6)   # rad – sektoren som tel som "framfor" (±34°)
        gp = lambda n: float(self.get_parameter(n).value)  # noqa: E731
        self.follower = rc.PathFollower(v_max=gp('v_max'), w_max=gp('w_max'), safety_dist=gp('safety_dist'))
        self.front_half = gp('front_half_angle')

        self.pose = None
        self.front_min = float('inf')
        self.rear_min = float('inf')
        self.goal = 'none'
        self.recovery_until = None
        self.pending_status = None

        # relative topic-namn -> blir /tb3_0/odom osv. når noden køyrer i namespace tb3_0
        self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)
        self.create_subscription(LaserScan, 'scan', self.clbk_scan, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self.clbk_plan, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'nav_status', 10)

        self.create_timer(0.05, self.control_loop)     # 20 Hz styring
        self.create_timer(0.5, self.publish_status)    # status 2 gonger i sekundet
        self.get_logger().info(f'robot_controller klar i namespace "{self.get_namespace()}"')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ callbacks
    def clbk_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def clbk_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float32)
        ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ang = (ang + math.pi) % (2 * math.pi) - math.pi
        valid = np.isfinite(r) & (r > max(0.12, msg.range_min))
        front = valid & (np.abs(ang) < self.front_half)
        rear = valid & (np.abs(ang) > math.pi - 0.6)
        # lidaren sit ca. 6 cm bak senteret: trekk frå framover, legg til bakover
        self.front_min = float(r[front].min()) - 0.064 if front.any() else float('inf')
        self.rear_min = float(r[rear].min()) + 0.064 if rear.any() else float('inf')

    def clbk_plan(self, msg):
        self.goal = goal_id(msg.header.stamp)
        self.recovery_until = None
        if len(msg.poses) == 0:
            self.follower.set_path([], None, self.now())
            self.stop()
            self.publish_status()
            return
        wps = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        q = msg.poses[-1].pose.orientation
        final_yaw = yaw_from_quat(q) if (q.z != 0.0 or q.w != 0.0) else None
        self.follower.set_path(wps, final_yaw, self.now())
        self.publish_status()

    # ------------------------------------------------------------------ styring
    def stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if self.pose is None:
            return
        now = self.now()
        cmd = Twist()
        # rygging etter blokkering
        if self.recovery_until is not None:
            if now < self.recovery_until and self.rear_min > 0.25:
                cmd.linear.x = -0.1
                self.cmd_pub.publish(cmd)
                return
            self.recovery_until = None
            self.follower.status = self.pending_status
            self.stop()
            self.publish_status()
            return
        before = self.follower.status
        v, w, status = self.follower.step(*self.pose, self.front_min, now)
        if status in ('blocked', 'failed') and before == 'active':
            self.pending_status = status
            self.follower.status = 'recovering'
            self.recovery_until = now + 1.5
            self.get_logger().warn(f'{status} – ryggar litt og ber om ny rute')
            return
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        if status != before:
            self.publish_status()

    def publish_status(self):
        st = self.follower.status
        if st == 'recovering':
            st = 'active'
        self.status_pub.publish(String(data=f'{st} {self.goal}'))


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()