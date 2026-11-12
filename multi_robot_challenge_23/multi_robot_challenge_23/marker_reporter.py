import math
import statistics

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from ros2_aruco_interfaces.msg import ArucoMarkers
from scoring_interfaces.srv import SetMarkerPosition

MARKER_NAMES = {0: 'Brann', 1: 'Brann', 2: 'Menneske', 3: 'Brann', 4: 'STOR BRANN'}


def latched_qos(depth=1):
    """Seine abonnentar får siste melding (same som /map brukar)."""
    return QoSProfile(reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                      durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
                      history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                      depth=depth)


def rotate(q, v):
    """Roter vektor v=(x,y,z) med kvaternion q=(x,y,z,w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


def transform_point(tf, p):
    """Bruk ein TransformStamped på eit punkt: p_map = R * p_kamera + t."""
    r = tf.transform.rotation
    t = tf.transform.translation
    x, y, z = rotate((r.x, r.y, r.z, r.w), p)
    return (x + t.x, y + t.y, z + t.z)


class MarkerReporter(Node):
    def __init__(self):
        super().__init__('marker_reporter')

        self.declare_parameter('namespaces', ['tb3_0', 'tb3_1'])
        self.declare_parameter('max_range', 3.0)      # m – ignorer markørar lenger unna
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_bearing', 0.5)    # |x/z| i biletet (~±27°), unngå biletkantane
        self.declare_parameter('min_samples', 5)      # målingar før vi rapporterer
        self.declare_parameter('max_spread', 0.15)    # m – maks spreiing mellom målingane
        gp = lambda n: self.get_parameter(n).value  # noqa: E731
        self.namespaces = list(gp('namespaces'))
        self.max_range = float(gp('max_range'))
        self.min_range = float(gp('min_range'))
        self.max_bearing = float(gp('max_bearing'))
        self.min_samples = int(gp('min_samples'))
        self.max_spread = float(gp('max_spread'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples = {}        # id -> [(x, y), ...]  målingar i kart-frame
        self.range_limit = {}    # id -> max_range (krympar om scoring avviser)
        self.accepted = {}       # id -> (x, y)  godkjende av scoring
        self.in_flight = set()   # id-ar vi ventar på svar for
        self.pending = []        # deteksjonar der tf ikkje var klar enno

        for ns in self.namespaces:
            self.create_subscription(ArucoMarkers, f'/{ns}/aruco_markers',
                                     lambda msg, ns=ns: self.clbk_markers(msg, ns), 10)

        self.score_client = self.create_client(SetMarkerPosition, '/set_marker_position')

        self.viz_pub = self.create_publisher(MarkerArray, '/reported_markers', latched_qos())
        self.found_pub = self.create_publisher(PointStamped, '/marker_found', latched_qos(10))

        self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(f'marker_reporter lyttar på {self.namespaces}')

    def clbk_markers(self, msg, ns):
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)
            if marker_id not in MARKER_NAMES or marker_id in self.accepted:
                continue
            p = pose.position          # optisk kamera-frame: z = framover, x = høgre
            limit = self.range_limit.get(marker_id, self.max_range)
            if not (self.min_range < p.z < limit):
                continue
            if abs(p.x / p.z) > self.max_bearing:
                continue
            self.pending.append((msg.header, marker_id, (p.x, p.y, p.z), ns, 0))

    def process_pending(self):
        still_pending = []
        for header, marker_id, p_cam, ns, tries in self.pending:
            if marker_id in self.accepted:
                continue
            frame = header.frame_id or f'{ns}/camera_rgb_optical_frame'
            try:
                tf = self.tf_buffer.lookup_transform('map', frame, Time.from_msg(header.stamp))
            except TransformException:
                if tries < 5:      # tf kan kome litt etter bildet – prøv igjen i ~0.5 s
                    still_pending.append((header, marker_id, p_cam, ns, tries + 1))
                continue
            x, y, _ = transform_point(tf, p_cam)
            self.samples.setdefault(marker_id, []).append((x, y))
        self.pending = still_pending

    def try_report(self, marker_id):
        s = self.samples.get(marker_id, [])
        if len(s) < self.min_samples or marker_id in self.in_flight:
            return
        s = s[-30:]                                     # dei siste målingane
        xs = [a[0] for a in s]
        ys = [a[1] for a in s]
        mx, my = statistics.median(xs), statistics.median(ys)
        spread = statistics.median([math.hypot(x - mx, y - my) for x, y in zip(xs, ys)])
        if spread > self.max_spread:
            return                                      # for mykje støy – vent
        if not self.score_client.service_is_ready():
            self.get_logger().warn('/set_marker_position er ikkje tilgjengeleg (køyrer scoring?)',
                                   throttle_duration_sec=5.0)
            return

        req = SetMarkerPosition.Request()
        req.marker_id = marker_id
        req.marker_position = Point(x=mx, y=my, z=0.0)  # z = 0: scoring måler mot botnen av tavla
        self.in_flight.add(marker_id)
        fut = self.score_client.call_async(req)
        fut.add_done_callback(lambda f, i=marker_id, x=mx, y=my: self.on_response(f, i, x, y))
        self.get_logger().info(f'Rapporterer {marker_id} ({MARKER_NAMES[marker_id]}) '
                               f'@ ({mx:.2f}, {my:.2f}), {len(s)} målingar, spreiing {spread:.3f} m')

    def on_response(self, future, marker_id, x, y):
        self.in_flight.discard(marker_id)
        try:
            ok = future.result().accepted
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Tenestekall feila: {e}')
            return
        if ok:
            self.accepted[marker_id] = (x, y)
            self.get_logger().info(f'GODKJEND: markør {marker_id} ({MARKER_NAMES[marker_id]})  '
                                   f'[{len(self.accepted)}/5 funne]')
            pt = PointStamped()
            pt.header.frame_id = 'map'
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point = Point(x=x, y=y, z=float(marker_id))   # z = marker-ID for coordinator
            self.found_pub.publish(pt)
            self.publish_viz()
        else:
            new_lim = max(1.5, self.range_limit.get(marker_id, self.max_range) - 0.5)
            self.range_limit[marker_id] = new_lim
            self.samples[marker_id] = []
            self.get_logger().warn(f'Markør {marker_id} avvist – prøver igjen nærare (max {new_lim:.1f} m)')

    def publish_viz(self):
        arr = MarkerArray()
        for mid, (x, y) in self.accepted.items():
            m = Marker()
            m.header.frame_id = 'map'
            m.ns, m.id, m.type, m.action = 'aruco', mid, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.25
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.4
            m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = (0.1, 0.4, 1.0, 0.8) if mid == 2 else (1.0, 0.3, 0.0, 0.8)
            arr.markers.append(m)
        self.viz_pub.publish(arr)

    def timer_callback(self):
        self.process_pending()
        for marker_id in list(self.samples.keys()):
            if marker_id not in self.accepted:
                self.try_report(marker_id)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerReporter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()