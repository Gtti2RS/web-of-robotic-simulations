#include <chrono>
#include <functional>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/world_stats.pb.h>

class StatsBridge : public rclcpp::Node {
public:
  explicit StatsBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("stats_bridge", options) {
    this->declare_parameter<std::string>("gz_input", "/throttled/stats");
    this->declare_parameter<std::string>("ros_output", "/throttled/stats_json");

    const auto gzInput = this->get_parameter("gz_input").as_string();
    const auto rosOutput = this->get_parameter("ros_output").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>(rosOutput, rclcpp::QoS(10));

    auto cb = [this](const gz::msgs::WorldStatistics &msg) {
      std::ostringstream os;
      os << '{'
         << "\"sim_time\":{\"sec\":" << msg.sim_time().sec()
         << ",\"nsec\":" << msg.sim_time().nsec() << "},"
         << "\"real_time\":{\"sec\":" << msg.real_time().sec()
         << ",\"nsec\":" << msg.real_time().nsec() << "},"
         << "\"paused\":" << (msg.paused() ? "true" : "false") << ','
         << "\"iterations\":" << msg.iterations() << ','
         << "\"real_time_factor\":" << msg.real_time_factor() << ','
         << "\"step_size\":{\"sec\":" << msg.step_size().sec()
         << ",\"nsec\":" << msg.step_size().nsec() << "}"
         << '}';

      std_msgs::msg::String out;
      out.data = os.str();
      publisher_->publish(out);
    };

    if (!node_.Subscribe(
            gzInput,
            std::function<void(const gz::msgs::WorldStatistics&)>(cb))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to %s", gzInput.c_str());
      throw std::runtime_error("gz subscribe failed");
    }

    RCLCPP_INFO(this->get_logger(), "Bridging GZ %s -> ROS %s (std_msgs/String JSON)",
                gzInput.c_str(), rosOutput.c_str());
  }

private:
  gz::transport::Node node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<StatsBridge>());
  } catch (const std::exception &e) {
    // Already logged
  }
  rclcpp::shutdown();
  return 0;
}


