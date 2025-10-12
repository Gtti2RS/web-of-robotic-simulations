#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <sstream>
#include <string>

class PoseArrayJsonBridge : public rclcpp::Node {
public:
  explicit PoseArrayJsonBridge(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("pose_array_json_bridge", options)
  {
    this->declare_parameter<std::string>("ros_input", "/throttled/pose/info");
    this->declare_parameter<std::string>("ros_output", "/throttled/pose/info_json");

    const auto in_topic  = this->get_parameter("ros_input").as_string();
    const auto out_topic = this->get_parameter("ros_output").as_string();

    publisher_ = this->create_publisher<std_msgs::msg::String>(out_topic, rclcpp::QoS(10));

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      in_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseArray &msg){
        std::ostringstream os;
        os << '{'
           << "\"header\":{\"stamp\":{\"sec\":" << msg.header.stamp.sec
           << ",\"nanosec\":" << msg.header.stamp.nanosec
           << "},\"frame_id\":\"" << msg.header.frame_id << "\"},"
           << "\"poses\":[";
        for (size_t i = 0; i < msg.poses.size(); ++i) {
          const auto &p = msg.poses[i];
          if (i) os << ',';
          os << '{'
             << "\"position\":{\"x\":" << p.position.x
             << ",\"y\":" << p.position.y
             << ",\"z\":" << p.position.z << "},"
             << "\"orientation\":{\"x\":" << p.orientation.x
             << ",\"y\":" << p.orientation.y
             << ",\"z\":" << p.orientation.z
             << ",\"w\":" << p.orientation.w << "}"
             << '}';
        }
        os << "]}";

        std_msgs::msg::String out;
        out.data = os.str();
        publisher_->publish(out);
      });

    RCLCPP_INFO(this->get_logger(), "PoseArray JSON bridge: %s -> %s",
                in_topic.c_str(), out_topic.c_str());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseArrayJsonBridge>());
  rclcpp::shutdown();
  return 0;
}


