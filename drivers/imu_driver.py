#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        self.pub = self.create_publisher(Imu, '/imu/data', 10)
        
        try:
            self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info('IMU connected to /dev/ttyUSB1')
            self.timer = self.create_timer(0.01, self.read_imu)
        except:
            self.get_logger().error('Cannot open IMU port')
    
    def read_imu(self):
        try:
            line = self.serial.readline().decode('ascii', errors='replace')
            parts = line.split(',')
            
            if len(parts) >= 6:
                msg = Imu()
                msg.header = Header()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu'
                
                msg.linear_acceleration.x = float(parts[0])
                msg.linear_acceleration.y = float(parts[1])
                msg.linear_acceleration.z = float(parts[2])
                msg.angular_velocity.x = float(parts[3])
                msg.angular_velocity.y = float(parts[4])
                msg.angular_velocity.z = float(parts[5])
                
                msg.orientation.w = 1.0
                
                self.pub.publish(msg)
        except:
            pass

def main():
    rclpy.init()
    node = IMUDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
