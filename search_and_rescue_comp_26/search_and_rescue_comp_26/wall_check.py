import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String


class WallCheck(Node):
    def __init__(self):
        super().__init__("wall_check",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.declare_parameter('robots', ['tb3_0', 'tb3_1'])
        names = list(self.get_parameter('robots').value)

        self.max_dist = 2.5
        self.min_fraction = 0.9

        self.bridge = CvBridge()
        self.front = {}
        self.pubs = {}
        for name in names:
            self.front[name] = math.inf
            self.pubs[name] = self.create_publisher(String, f'/{name}/wall_type', 10)
            self.create_subscription(Image, f'/{name}/camera/image_raw',
                                     lambda m, n=name: self.clbk_image(m, n), qos_profile_sensor_data)
            self.create_subscription(LaserScan, f'/{name}/scan',
                                     lambda m, n=name: self.clbk_scan(m, n), qos_profile_sensor_data)

    def clbk_scan(self, msg, name):
        front = [r for r in list(msg.ranges[-10:]) + list(msg.ranges[:11]) if msg.range_min < r < msg.range_max]
        self.front[name] = min(front) if front else math.inf

    def clbk_image(self, msg, name):
        if self.front[name] > self.max_dist:
            self.pubs[name].publish(String(data="none"))
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = image.shape[:2]
        centre = image[int(h * 0.3):int(h * 0.6), int(w * 0.3):int(w * 0.7)]
        hsv = cv2.cvtColor(centre, cv2.COLOR_BGR2HSV)

        coloured = cv2.inRange(hsv, np.array([0, 70, 40]), np.array([180, 255, 255]))
        fraction = cv2.countNonZero(coloured) / coloured.size

        self.pubs[name].publish(String(data="other" if fraction >= self.min_fraction else "white"))


def main(args=None):
    rclpy.init(args=args)
    node = WallCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
