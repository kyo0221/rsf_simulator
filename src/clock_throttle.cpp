#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class ClockThrottle : public rclcpp::Node
{
public:
  ClockThrottle()
  : Node("clock_throttle"),
    period_ns_(static_cast<int64_t>(1e9 / declare_parameter<double>("rate_hz", 100.0)))
  {
    pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::QoS(1).reliable());
    sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      declare_parameter<std::string>("input_topic", "/rsf/sim/clock_raw"), rclcpp::QoS(10),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const int64_t t = rclcpp::Time(msg->clock).nanoseconds();
        if (t < last_ns_ || t - last_ns_ >= period_ns_) {
          last_ns_ = t;
          pub_->publish(*msg);
        }
      });
  }

private:
  int64_t period_ns_;
  int64_t last_ns_ = 0;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottle>());
  rclcpp::shutdown();
  return 0;
}
