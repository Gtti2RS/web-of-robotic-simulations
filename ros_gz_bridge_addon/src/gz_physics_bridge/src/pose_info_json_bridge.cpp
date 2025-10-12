#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>

#include <sstream>
#include <string>

class PoseInfoJsonBridge : public rclcpp::Node {
public:
  explicit PoseInfoJsonBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("pose_info_json_bridge", options) {
    this->declare_parameter<std::string>("gz_input", "/throttled/pose/info");
    this->declare_parameter<std::string>("ros_output", "/throttled/pose/info_json");

    const auto gz_input  = this->get_parameter("gz_input").as_string();
    const auto ros_output = this->get_parameter("ros_output").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>(ros_output, rclcpp::QoS(10));

    auto cb = [this](const gz::msgs::Pose_V &msg) {
      std::ostringstream os;
      os << '{';
      // header.stamp
      if (msg.has_header() && msg.header().has_stamp()) {
        os << "\"header\":{\"stamp\":{\"sec\":" << msg.header().stamp().sec()
           << ",\"nsec\":" << msg.header().stamp().nsec() << "}},";
      }

      os << "\"pose\":[";
      const int n = msg.pose_size();
      for (int i = 0; i < n; ++i) {
        const auto &p = msg.pose(i);
        if (i) os << ',';
        os << '{'
           << "\"name\":\"" << p.name() << "\","  // always include name (may be empty)
           << "\"id\":" << p.id() << ',';              // include id (0 if unset)
        os << "\"position\":{\"x\":" << (p.has_position() ? p.position().x() : 0.0)
           << ",\"y\":" << (p.has_position() ? p.position().y() : 0.0)
           << ",\"z\":" << (p.has_position() ? p.position().z() : 0.0) << "},"
           << "\"orientation\":{\"x\":" << (p.has_orientation() ? p.orientation().x() : 0.0)
           << ",\"y\":" << (p.has_orientation() ? p.orientation().y() : 0.0)
           << ",\"z\":" << (p.has_orientation() ? p.orientation().z() : 0.0)
           << ",\"w\":" << (p.has_orientation() ? p.orientation().w() : 1.0) << "}"
           << '}';
      }
      os << "]}";

      std_msgs::msg::String out;
      out.data = os.str();
      publisher_->publish(out);
    };

    if (!gz_node_.Subscribe(
            gz_input,
            std::function<void(const gz::msgs::Pose_V&)>(cb))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to subscribe to %s", gz_input.c_str());
      throw std::runtime_error("gz subscribe failed");
    }

    RCLCPP_INFO(this->get_logger(), "Pose info JSON: %s -> %s", gz_input.c_str(), ros_output.c_str());
  }

private:
  gz::transport::Node gz_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseInfoJsonBridge>());
  rclcpp::shutdown();
  return 0;
}


