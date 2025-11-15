#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import serial

class GPSDriver(Node):
    def __init__(self):
        super().__init__('gps_driver')
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        try:
            self.serial = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('GPS connected to /dev/ttyUSB0')
            self.timer = self.create_timer(0.1, self.read_gps)
        except:
            self.get_logger().error('Cannot open GPS port')
    
    def read_gps(self):
        try:
            line = self.serial.readline().decode('ascii', errors='replace')
            if '$GPGGA' in line or '$GNGGA' in line:
                parts = line.split(',')
                if len(parts) > 9:
                    lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
                    if parts[3] == 'S':
                        lat = -lat
                    lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
                    if parts[5] == 'W':
                        lon = -lon
                    alt = float(parts[9]) if parts[9] else 0.0
                    
                    msg = NavSatFix()
                    msg.header = Header()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = 'gps'
                    msg.latitude = lat
                    msg.longitude = lon
                    msg.altitude = alt
                    self.pub.publish(msg)
                    self.get_logger().info(f'GPS: {lat:.6f}, {lon:.6f}')
        except:
            pass

def main():
    rclpy.init()
    node = GPSDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
