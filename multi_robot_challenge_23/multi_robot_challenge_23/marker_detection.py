import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point

class MarkerDetection(Node):
    def __init__(self):
        super().__init__('RobotLeaderNode')

        # TODO 1  <-- NY
        self.create_subscription(Pose, '/tb3_0/marker_map_pose', self.clbk_marker_map_pose, 10)
        self.create_subscription(Int64, '/tb3_0/marker_id', self.clbk_marker_id, 10)

        # Default values for variables
        self.prev_marker_id = -1
        self.marker_id = -1
        self.marker_position = Point()

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def clbk_marker_map_pose(self, msg):
        self.marker_position = msg.position

    def clbk_marker_id(self, msg):
        self.marker_id = msg.data

    def timer_callback(self):
        # TODO 2  <-- NY
        if self.marker_id != self.prev_marker_id:
            p = self.marker_position
            self.get_logger().info(
                f'Ny markør: id={self.marker_id}  posisjon=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
            self.prev_marker_id = self.marker_id
        
def main(args=None):
    rclpy.init(args=args)
    marker_detection = MarkerDetection()
    rclpy.spin(marker_detection)
    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()