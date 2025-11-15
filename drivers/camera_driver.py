#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('Camera opened')
            self.timer = self.create_timer(1.0/30.0, self.capture)
        else:
            self.get_logger().error('Cannot open camera')
    
    def capture(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = CameraDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
