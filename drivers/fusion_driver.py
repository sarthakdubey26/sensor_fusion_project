#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
import csv
from datetime import datetime

class FusionDriver(Node):
    def __init__(self):
        super().__init__('fusion_driver')
        
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f'../data/fusion_{timestamp}.csv'
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        self.writer.writerow([
            'timestamp',
            'gps_lat', 'gps_lon', 'gps_alt',
            'imu_ax', 'imu_ay', 'imu_az',
            'imu_gx', 'imu_gy', 'imu_gz'
        ])
        
        self.latest_gps = None
        self.latest_imu = None
        self.count = 0
        
        self.get_logger().info('GPS+IMU Fusion Driver Started')
        self.get_logger().info(f'Recording to: {self.filename}')
    
    def gps_callback(self, msg):
        self.latest_gps = msg
        self.save_data()
    
    def imu_callback(self, msg):
        self.latest_imu = msg
        self.save_data()
    
    def save_data(self):
        if self.latest_gps and self.latest_imu:
            t = self.get_clock().now().nanoseconds / 1e9
            
            self.writer.writerow([
                t,
                self.latest_gps.latitude,
                self.latest_gps.longitude,
                self.latest_gps.altitude,
                self.latest_imu.linear_acceleration.x,
                self.latest_imu.linear_acceleration.y,
                self.latest_imu.linear_acceleration.z,
                self.latest_imu.angular_velocity.x,
                self.latest_imu.angular_velocity.y,
                self.latest_imu.angular_velocity.z
            ])
            
            self.count += 1
            if self.count % 50 == 0:
                self.get_logger().info(f'Recorded {self.count} data points')
    
    def __del__(self):
        if hasattr(self, 'file'):
            self.file.close()
            self.get_logger().info(f'Saved {self.count} total data points')

def main():
    rclpy.init()
    node = FusionDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
