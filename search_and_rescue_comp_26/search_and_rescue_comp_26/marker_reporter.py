import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from ros2_aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Point
from scoring_interfaces.srv import SetMarkerPosition


class MarkerReporter(Node):
    def __init__(self):
        super().__init__("marker_reporter",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.get_logger().info("Marker reporter node created")

        self.declare_parameter('robots', ['tb3_0', 'tb3_1'])
        robots = self.get_parameter('robots').value

        self.samples_needed = 5
        self.max_spread = 0.3
        self.max_distance = 3.0
        self.max_view_angle = 65.0
        self.max_queue_age = 1.0
        self.max_rejections = 3

        self.samples = {}
        self.reported = set()
        self.pending = set()
        self.rejections = {}
        self.queue = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for r in robots:
            self.create_subscription(ArucoMarkers, f'/{r}/aruco_markers', lambda msg, r=r: self.clbk_aruco(msg, r), 10)

        self.client = self.create_client(SetMarkerPosition, '/set_marker_position')

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reported_pub = self.create_publisher(Int8MultiArray, '/reported_markers', latched)
        self.big_fire_pub = self.create_publisher(Point, '/big_fire_position', latched)

        self.big_fire = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.republish_timer = self.create_timer(1.0, self.publish_status)

    def clbk_aruco(self, msg, robot):
        self.queue.append((robot, msg))

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9
        remaining = []
        for robot, msg in self.queue:
            frame = f'{robot}/camera_rgb_optical_frame'
            try:
                t = self.tf_buffer.lookup_transform('map', frame, msg.header.stamp)
            except TransformException:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                if now - stamp < self.max_queue_age:
                    remaining.append((robot, msg))
                continue

            for marker_id, pose in zip(msg.marker_ids, msg.poses):
                self.add_sample(int(marker_id), pose, t.transform)
        self.queue = remaining

    def add_sample(self, marker_id, pose, tf):
        if marker_id < 0 or marker_id > 4 or marker_id in self.reported or marker_id in self.pending:
            return

        p = pose.position
        q = pose.orientation
        distance = math.sqrt(p.x**2 + p.y**2 + p.z**2)

        nx = 2 * (q.x * q.z + q.w * q.y)
        ny = 2 * (q.y * q.z - q.w * q.x)
        nz = 1 - 2 * (q.x**2 + q.y**2)
        view_angle = math.degrees(math.acos(min(1.0, abs(nx * p.x + ny * p.y + nz * p.z) / distance)))

        if distance > self.max_distance or view_angle > self.max_view_angle:
            return

        x, y = self.to_map(p, tf)
        samples = self.samples.setdefault(marker_id, [])

        if samples:
            mx, my = self.mean(samples)
            if math.hypot(x - mx, y - my) > self.max_spread:
                return

        samples.append((x, y))
        if len(samples) >= self.samples_needed:
            self.report(marker_id)

    def to_map(self, p, tf):
        q = tf.rotation
        tx = 2 * (q.y * p.z - q.z * p.y)
        ty = 2 * (q.z * p.x - q.x * p.z)
        tz = 2 * (q.x * p.y - q.y * p.x)
        x = p.x + q.w * tx + (q.y * tz - q.z * ty)
        y = p.y + q.w * ty + (q.z * tx - q.x * tz)
        return x + tf.translation.x, y + tf.translation.y

    def mean(self, samples):
        n = len(samples)
        return sum(s[0] for s in samples) / n, sum(s[1] for s in samples) / n

    def report(self, marker_id):
        if not self.client.service_is_ready():
            self.get_logger().warn("Scoring service not available, will retry on next sample")
            return

        x, y = self.mean(self.samples[marker_id])

        req = SetMarkerPosition.Request()
        req.marker_id = marker_id
        req.marker_position = Point(x=x, y=y, z=0.0)

        self.pending.add(marker_id)
        future = self.client.call_async(req)
        future.add_done_callback(lambda f: self.clbk_response(f, marker_id, x, y))
        self.get_logger().info(f"Reporting marker {marker_id} at ({x:.2f}, {y:.2f})")

    def clbk_response(self, future, marker_id, x, y):
        self.pending.discard(marker_id)

        if future.result() is not None and future.result().accepted:
            self.get_logger().info(f"Marker {marker_id} accepted")
            self.mark_reported(marker_id, x, y)
            return

        self.rejections[marker_id] = self.rejections.get(marker_id, 0) + 1
        if self.rejections[marker_id] >= self.max_rejections:
            self.get_logger().warn(f"Marker {marker_id} rejected {self.max_rejections} times, assuming it is already reported")
            self.mark_reported(marker_id, x, y)
        else:
            self.get_logger().warn(f"Marker {marker_id} rejected, collecting new samples")
            self.samples[marker_id] = []

    def mark_reported(self, marker_id, x, y):
        self.reported.add(marker_id)
        self.get_logger().info(f"Reported so far: {sorted(self.reported)}")

        if marker_id == 4:
            self.big_fire = (x, y)
        self.publish_status()

    def publish_status(self):
        if self.reported:
            self.reported_pub.publish(Int8MultiArray(data=sorted(self.reported)))
        if self.big_fire is not None:
            self.big_fire_pub.publish(Point(x=self.big_fire[0], y=self.big_fire[1], z=0.0))


def main(args=None):
    rclpy.init(args=args)

    reporter = MarkerReporter()

    try:
        rclpy.spin(reporter)
    except KeyboardInterrupt:
        pass
    finally:
        reporter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
