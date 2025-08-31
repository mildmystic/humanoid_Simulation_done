import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.subscription_rgb = self.create_subscription(Image, '/camera/image', self.rgb_callback, 10)
        self.subscription_depth = self.create_subscription(Image, '/camera/depth_image', self.depth_callback, 10)
        self.subscription_info = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        self.bridge = CvBridge()
        self.latest_rgb = None
        self.latest_depth = None
        self.fx = self.fy = self.cx = self.cy = 0.0  # Intrinsics

    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, '32FC1')

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.latest_depth is not None and self.fx > 0:
            self.detect_object()

    def detect_object(self):
        hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            u, v = int(x + w/2), int(y + h/2)
            depth = self.latest_depth[v, u]
            if not np.isnan(depth) and depth > 0:
                X = (u - self.cx) * depth / self.fx
                Y = (v - self.cy) * depth / self.fy
                Z = depth
                self.get_logger().info(f'Object 3D pose (camera frame): X={X:.2f}, Y={Y:.2f}, Z={Z:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()