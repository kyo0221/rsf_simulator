#!/usr/bin/env python3
import math
import struct
from collections import deque

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Gpgga
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

EARTH_RADIUS_M = 6378137.0


def quat_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


LIDAR_TO_IMU = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])


class RsfDeviceEmulator(Node):
    def __init__(self):
        super().__init__('rsf_device_emulator')
        p = self.declare_parameters('', [
            ('odom_in', '/rsf/rsf_odom'), ('imu_in', '/rsf/sim/imu'), ('navsat_in', '/rsf/sim/nav_sat_fix'), ('cloud_in', '/rsf/hokuyo_cloud2'),
            ('lio_out', '/rsf/lio_lidar_rate_odom'), ('imu_out', '/rsf/imu'), ('gga_out', '/rsf/gpgga'), ('navsat_out', '/rsf/nav_sat_fix'),
            ('datum_lat', 36.083470500000004), ('datum_lon', 140.07630341666666),
            ('map_theta_rad', -2.3500858378103646), ('map_tx', 156.89610202453204), ('map_ty', 64.64302275894745),
            ('base_to_sensor', [0.3, 0.0, 0.35]), ('gnss_noise_m', 0.02), ('lio_jitter_m', 0.0), ('lio_scale', 1.0), ('gyro_bias_dps', 0.0), ('gnss_gap_zones', [0.0]), ('seed', 0)])
        self.p = {v.name: v.value for v in p}
        z = self.p['gnss_gap_zones']
        self.zones = [z[i:i + 5] for i in range(0, len(z) - 4, 5)]
        self.rng = np.random.default_rng(self.p['seed'])
        self.odoms = deque(maxlen=400)
        self.origin = None
        self.lio_pub = self.create_publisher(Odometry, self.p['lio_out'], 50)
        self.imu_pub = self.create_publisher(Imu, self.p['imu_out'], 500)
        self.gga_pub = self.create_publisher(Gpgga, self.p['gga_out'], 10)
        self.navsat_pub = self.create_publisher(NavSatFix, self.p['navsat_out'], 10)
        self.create_subscription(Odometry, self.p['odom_in'], self.on_odom, 200)
        self.create_subscription(Imu, self.p['imu_in'], self.on_imu, 500)
        self.create_subscription(NavSatFix, self.p['navsat_in'], self.on_navsat, 10)
        self.create_subscription(PointCloud2, self.p['cloud_in'], self.on_cloud, 10, raw=True)

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        sensor = np.array([p.x, p.y, p.z]) + quat_matrix(q.x, q.y, q.z, q.w) @ self.p['base_to_sensor']
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (float(v) for v in sensor)
        self.odoms.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, msg))

    def odom_at(self, sec, nanosec):
        if not self.odoms:
            return None
        t = sec + nanosec * 1e-9
        return min(self.odoms, key=lambda o: abs(o[0] - t))[1]

    def on_cloud(self, raw):
        sec, nanosec = struct.unpack_from('<iI', raw, 4)
        odom = self.odom_at(sec, nanosec)
        if odom is None:
            return
        out = Odometry()
        out.header.stamp.sec, out.header.stamp.nanosec = sec, nanosec
        out.header.frame_id = odom.header.frame_id
        out.child_frame_id = odom.child_frame_id
        out.pose = odom.pose
        out.twist = odom.twist
        p = odom.pose.pose.position
        xyz = np.array([p.x, p.y, p.z])
        if self.origin is None:
            self.origin = xyz
        xyz = self.origin + (xyz - self.origin) * self.p['lio_scale'] + self.rng.normal(0.0, self.p['lio_jitter_m'], 3)
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (float(v) for v in xyz)
        self.lio_pub.publish(out)

    def on_imu(self, msg):
        a = LIDAR_TO_IMU @ [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        w = LIDAR_TO_IMU @ [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        up = a / max(np.linalg.norm(a), 1e-6)
        w = w + math.radians(self.p['gyro_bias_dps']) * up
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in a)
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in w)
        self.imu_pub.publish(msg)

    def in_gap(self, odom):
        p = odom.pose.pose.position
        for x1, y1, x2, y2, r in self.zones:
            d = np.array([x2 - x1, y2 - y1])
            t = np.clip(np.dot([p.x - x1, p.y - y1], d) / max(d @ d, 1e-9), 0.0, 1.0)
            if math.hypot(p.x - x1 - t * d[0], p.y - y1 - t * d[1]) <= r:
                return True
        return False

    def latlon(self, odom):
        p = odom.pose.pose.position
        xy = np.array([p.x, p.y]) + self.rng.normal(0.0, self.p['gnss_noise_m'], 2)
        c, s = math.cos(self.p['map_theta_rad']), math.sin(self.p['map_theta_rad'])
        d = xy - [self.p['map_tx'], self.p['map_ty']]
        e, n = c * d[0] + s * d[1], -s * d[0] + c * d[1]
        lat = self.p['datum_lat'] + math.degrees(n / EARTH_RADIUS_M)
        lon = self.p['datum_lon'] + math.degrees(e / (EARTH_RADIUS_M * math.cos(math.radians(self.p['datum_lat']))))
        return lat, lon

    def on_navsat(self, msg):
        odom = self.odom_at(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if odom is None:
            return
        lat, lon = self.latlon(odom)
        fix = NavSatFix()
        fix.header = msg.header
        fix.status.status = 0
        fix.latitude, fix.longitude, fix.altitude = lat, lon, msg.altitude
        fix.position_covariance[0] = fix.position_covariance[4] = self.p['gnss_noise_m'] ** 2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.navsat_pub.publish(fix)
        g = Gpgga()
        g.header = msg.header
        g.lat, g.lat_dir = abs(lat), 'N' if lat >= 0 else 'S'
        g.lon, g.lon_dir = abs(lon), 'E' if lon >= 0 else 'W'
        g.alt = msg.altitude
        g.gps_qual = 5 if self.in_gap(odom) else 4
        g.num_sats = 20
        g.hdop = 0.8
        self.gga_pub.publish(g)


def main():
    rclpy.init()
    node = RsfDeviceEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
