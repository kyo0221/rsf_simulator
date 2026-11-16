#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nmea_msgs/msg/gpgga.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "gnss_geo.hpp"

using Vec3 = std::array<double, 3>;
using nav_msgs::msg::Odometry;
using nmea_msgs::msg::Gpgga;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;

class RsfDeviceEmulator : public rclcpp::Node
{
public:
  RsfDeviceEmulator()
  : Node("rsf_device_emulator")
  {
    const auto odom_in = declare_parameter<std::string>("odom_in", "/rsf/rsf_odom");
    const auto imu_in = declare_parameter<std::string>("imu_in", "/rsf/sim/imu");
    const auto navsat_in = declare_parameter<std::string>("navsat_in", "/rsf/sim/nav_sat_fix");
    const auto cloud_in = declare_parameter<std::string>("cloud_in", "/rsf/hokuyo_cloud2");
    const auto lio_out = declare_parameter<std::string>("lio_out", "/rsf/lio_lidar_rate_odom");
    const auto imu_out = declare_parameter<std::string>("imu_out", "/rsf/imu");
    const auto gga_out = declare_parameter<std::string>("gga_out", "/rsf/gpgga");
    const auto navsat_out = declare_parameter<std::string>("navsat_out", "/rsf/nav_sat_fix");
    const auto wheel_out = declare_parameter<std::string>("wheel_out", "/icart/odom");
    wheel_scale_ = declare_parameter<double>("wheel_scale", 1.0);
    wheel_noise_ = declare_parameter<double>("wheel_noise", 0.02);
    datum_lat_ = declare_parameter<double>("datum_lat", 36.08288044139598);
    datum_lon_ = declare_parameter<double>("datum_lon", 140.07654638977903);
    datum_alt_ = declare_parameter<double>("datum_alt", 41.1224);
    base_to_sensor_ = toVec3(
      declare_parameter<std::vector<double>>("base_to_sensor", {0.3, 0.0, 0.35}));
    gnss_lever_ = toVec3(
      declare_parameter<std::vector<double>>("gnss_lever", {-0.1061, 0.1254, 0.0}));
    gnss_noise_m_ = declare_parameter<double>("gnss_noise_m", 0.02);
    lio_jitter_m_ = declare_parameter<double>("lio_jitter_m", 0.0);
    lio_scale_ = declare_parameter<double>("lio_scale", 1.0);
    gyro_bias_rad_ = declare_parameter<double>("gyro_bias_dps", 0.0) * M_PI / 180.0;
    const auto zones = declare_parameter<std::vector<double>>("gnss_gap_zones", {0.0});
    for (size_t i = 0; i + 5 <= zones.size(); i += 5) {
      zones_.push_back({zones[i], zones[i + 1], zones[i + 2], zones[i + 3], zones[i + 4]});
    }
    rng_.seed(static_cast<uint64_t>(declare_parameter<int64_t>("seed", 0)));

    lio_pub_ = create_publisher<Odometry>(lio_out, 50);
    wheel_pub_ = create_publisher<Odometry>(wheel_out, 50);
    imu_pub_ = create_publisher<Imu>(imu_out, 500);
    gga_pub_ = create_publisher<Gpgga>(gga_out, 10);
    navsat_pub_ = create_publisher<NavSatFix>(navsat_out, 10);
    odom_sub_ = create_subscription<Odometry>(
      odom_in, 200, [this](Odometry::ConstSharedPtr msg) {onOdom(*msg);});
    imu_sub_ = create_subscription<Imu>(
      imu_in, 500, [this](Imu::ConstSharedPtr msg) {onImu(*msg);});
    navsat_sub_ = create_subscription<NavSatFix>(
      navsat_in, 10, [this](NavSatFix::ConstSharedPtr msg) {onNavsat(*msg);});
    cloud_sub_ = create_generic_subscription(
      cloud_in, "sensor_msgs/msg/PointCloud2", 10,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {onCloud(*msg);});
  }

private:
  struct Zone
  {
    double x1, y1, x2, y2, radius;
  };

  struct OdomPair
  {
    double time;
    Odometry sensor;
    Odometry base;
  };

  static Vec3 toVec3(const std::vector<double> & v)
  {
    if (v.size() != 3) {
      throw std::invalid_argument("expected 3 elements");
    }
    return {v[0], v[1], v[2]};
  }

  static Vec3 rotate(const geometry_msgs::msg::Quaternion & q_in, const Vec3 & v)
  {
    const double norm = std::sqrt(
      q_in.x * q_in.x + q_in.y * q_in.y + q_in.z * q_in.z + q_in.w * q_in.w);
    const double x = q_in.x / norm, y = q_in.y / norm, z = q_in.z / norm, w = q_in.w / norm;
    return {
      (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - z * w) * v[1] + 2 * (x * z + y * w) * v[2],
      2 * (x * y + z * w) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - x * w) * v[2],
      2 * (x * z - y * w) * v[0] + 2 * (y * z + x * w) * v[1] + (1 - 2 * (x * x + y * y)) * v[2]};
  }

  static Vec3 offsetPosition(const Odometry & odom, const Vec3 & offset)
  {
    const auto & p = odom.pose.pose.position;
    const Vec3 r = rotate(odom.pose.pose.orientation, offset);
    return {p.x + r[0], p.y + r[1], p.z + r[2]};
  }

  double noise(double sigma)
  {
    return sigma * normal_(rng_);
  }

  void onOdom(const Odometry & msg)
  {
    Odometry wheel;
    wheel.header.stamp = msg.header.stamp;
    wheel.header.frame_id = "odom";
    wheel.child_frame_id = "base_footprint";
    wheel.twist.twist.linear.x = wheel_scale_ * msg.twist.twist.linear.x + noise(wheel_noise_);
    wheel.twist.twist.angular.z = msg.twist.twist.angular.z;
    wheel_pub_->publish(wheel);

    OdomPair pair{rclcpp::Time(msg.header.stamp).seconds(), msg, msg};
    const Vec3 sensor = offsetPosition(msg, base_to_sensor_);
    pair.sensor.pose.pose.position.x = sensor[0];
    pair.sensor.pose.pose.position.y = sensor[1];
    pair.sensor.pose.pose.position.z = sensor[2];
    odoms_.push_back(std::move(pair));
    if (odoms_.size() > 400) {
      odoms_.pop_front();
    }
  }

  const OdomPair * odomAt(const builtin_interfaces::msg::Time & stamp) const
  {
    if (odoms_.empty()) {
      return nullptr;
    }
    const double t = rclcpp::Time(stamp).seconds();
    return &*std::min_element(
      odoms_.begin(), odoms_.end(), [t](const OdomPair & a, const OdomPair & b) {
        return std::abs(a.time - t) < std::abs(b.time - t);
      });
  }

  void onCloud(const rclcpp::SerializedMessage & raw)
  {
    const auto & buffer = raw.get_rcl_serialized_message();
    if (buffer.buffer_length < 12) {
      return;
    }
    builtin_interfaces::msg::Time stamp;
    std::memcpy(&stamp.sec, buffer.buffer + 4, 4);
    std::memcpy(&stamp.nanosec, buffer.buffer + 8, 4);
    const OdomPair * pair = odomAt(stamp);
    if (pair == nullptr) {
      return;
    }
    Odometry out = pair->sensor;
    out.header.stamp = stamp;
    const auto & p = pair->sensor.pose.pose.position;
    const Vec3 xyz{p.x, p.y, p.z};
    if (!origin_) {
      origin_ = xyz;
    }
    const Vec3 & o = *origin_;
    out.pose.pose.position.x = o[0] + (xyz[0] - o[0]) * lio_scale_ + noise(lio_jitter_m_);
    out.pose.pose.position.y = o[1] + (xyz[1] - o[1]) * lio_scale_ + noise(lio_jitter_m_);
    out.pose.pose.position.z = o[2] + (xyz[2] - o[2]) * lio_scale_ + noise(lio_jitter_m_);
    lio_pub_->publish(out);
  }

  void onImu(const Imu & msg)
  {
    Imu out = msg;
    const auto & a_in = msg.linear_acceleration;
    const auto & w_in = msg.angular_velocity;
    const Vec3 a{a_in.y, a_in.z, a_in.x};
    const double norm = std::max(std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), 1e-6);
    out.linear_acceleration.x = a[0];
    out.linear_acceleration.y = a[1];
    out.linear_acceleration.z = a[2];
    out.angular_velocity.x = w_in.y + gyro_bias_rad_ * a[0] / norm;
    out.angular_velocity.y = w_in.z + gyro_bias_rad_ * a[1] / norm;
    out.angular_velocity.z = w_in.x + gyro_bias_rad_ * a[2] / norm;
    imu_pub_->publish(out);
  }

  bool inGap(const Odometry & odom) const
  {
    const auto & p = odom.pose.pose.position;
    for (const auto & z : zones_) {
      const double dx = z.x2 - z.x1, dy = z.y2 - z.y1;
      const double t = std::clamp(
        ((p.x - z.x1) * dx + (p.y - z.y1) * dy) / std::max(dx * dx + dy * dy, 1e-9), 0.0, 1.0);
      if (std::hypot(p.x - z.x1 - t * dx, p.y - z.y1 - t * dy) <= z.radius) {
        return true;
      }
    }
    return false;
  }

  void onNavsat(const NavSatFix & msg)
  {
    const OdomPair * pair = odomAt(msg.header.stamp);
    if (pair == nullptr) {
      return;
    }
    const Odometry & odom = pair->base;
    const Vec3 antenna = offsetPosition(odom, gnss_lever_);
    const double x = antenna[0] + noise(gnss_noise_m_);
    const double y = antenna[1] + noise(gnss_noise_m_);
    const auto [lat, lon] = rsf_simulator::xyToLatLon(x, y, datum_lat_, datum_lon_);
    const double alt = rsf_simulator::altitudeFromZ(antenna[2], datum_alt_);
    const bool gap = inGap(odom);

    NavSatFix fix;
    fix.header = msg.header;
    fix.status.status = gap ? NavSatStatus::STATUS_SBAS_FIX : NavSatStatus::STATUS_GBAS_FIX;
    fix.latitude = lat;
    fix.longitude = lon;
    fix.altitude = alt;
    fix.position_covariance[0] = fix.position_covariance[4] = gnss_noise_m_ * gnss_noise_m_;
    fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
    navsat_pub_->publish(fix);

    Gpgga gga;
    gga.header = msg.header;
    gga.lat = std::abs(lat);
    gga.lat_dir = lat >= 0 ? "N" : "S";
    gga.lon = std::abs(lon);
    gga.lon_dir = lon >= 0 ? "E" : "W";
    gga.alt = alt;
    gga.gps_qual = gap ? 5 : 4;
    gga.num_sats = 20;
    gga.hdop = 0.8;
    gga_pub_->publish(gga);
  }

  double wheel_scale_, wheel_noise_;
  double datum_lat_, datum_lon_, datum_alt_;
  Vec3 base_to_sensor_, gnss_lever_;
  double gnss_noise_m_, lio_jitter_m_, lio_scale_, gyro_bias_rad_;
  std::vector<Zone> zones_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  std::deque<OdomPair> odoms_;
  std::optional<Vec3> origin_;
  rclcpp::Publisher<Odometry>::SharedPtr lio_pub_, wheel_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<Gpgga>::SharedPtr gga_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr navsat_pub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<NavSatFix>::SharedPtr navsat_sub_;
  rclcpp::GenericSubscription::SharedPtr cloud_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RsfDeviceEmulator>());
  rclcpp::shutdown();
  return 0;
}
