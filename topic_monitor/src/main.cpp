#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

class TopicMonitor : public rclcpp::Node
{
public:
  TopicMonitor()
  : Node("topic_monitor")
  {
    const auto odom_topic = "/NR/odom";
    const auto odom_target_hz = 50.0f;

    const std::vector<std::string> laser_topics = {
      "/NR/laser"
    };

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this, odom_topic, odom_target_hz](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odomCallback(msg, odom_topic, odom_target_hz);
          });

    last_odom_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    auto qos =
      rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    for (const auto & topic_name : laser_topics) {
      auto sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
          topic_name, qos,
        [this, topic_name](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laserCallback(msg, topic_name);
          });
      laser_subs_.push_back(sub);
      last_laser_stamps_[topic_name] = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }
  }

private:
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg, const std::string & topic_name,
    const double target_hz)
  {
    const rclcpp::Time current_stamp(msg->header.stamp);

    if (last_odom_stamp_.seconds() > 0.0) {
      const auto dt = (current_stamp - last_odom_stamp_).seconds();
      const auto rate = (dt > 0.0) ? 1.0 / dt : 0.0;
      if (std::ceil(rate) < std::ceil(target_hz)) {
        RCLCPP_WARN(this->get_logger(),
          "[ODOM][%s] stamp: %.9f | Δt: %.3f s | rate(calc < target): %.2f < %.2f Hz, Under target",
                    topic_name.c_str(), current_stamp.seconds(), dt, rate, target_hz);
      } else {
        if (!suppress_log_print_for_success) {
          RCLCPP_INFO(this->get_logger(),
            "[ODOM][%s] stamp: %.9f | Δt: %.3f s | rate(calc/target): %.2f / %.2f Hz",
                      topic_name.c_str(), current_stamp.seconds(), dt, rate, target_hz);
        }
      }
    }
    last_odom_stamp_ = current_stamp;
  }

  void laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg,
    const std::string & topic_name)
  {
    const rclcpp::Time current_stamp(msg->header.stamp);
    auto & last_stamp = last_laser_stamps_[topic_name];

    if (current_stamp == last_stamp) {
      RCLCPP_WARN(this->get_logger(), "[LASER][%s] Duplicate timestamp detected: %.9f",
        topic_name.c_str(), current_stamp.seconds());
    } else {
      if (!suppress_log_print_for_success) {
        RCLCPP_INFO(this->get_logger(), "[LASER][%s] stamp: %.9f", topic_name.c_str(),
          current_stamp.seconds());
      }
    }
    last_stamp = current_stamp;
  }

private:
  bool suppress_log_print_for_success = true;

  rclcpp::Time last_odom_stamp_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> laser_subs_;
  std::unordered_map<std::string, rclcpp::Time> last_laser_stamps_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TopicMonitor>());
  rclcpp::shutdown();
  return 0;
}
