#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "gz_physics_bridge/srv/generate_world_sdf.hpp"

// Gazebo Transport / Messages
#include <gz/transport/Node.hh>
#include <gz/msgs/sdf_generator_config.pb.h>
#include <gz/msgs/stringmsg.pb.h>

using namespace std::chrono_literals;

class GenerateWorldSdfServer : public rclcpp::Node {
public:
  GenerateWorldSdfServer() : Node("generate_world_sdf_server")
  {
    world_ = this->declare_parameter<std::string>("world", "empty");
    service_name_ = "/world/" + world_ + "/generate_world_sdf";

    srv_ = this->create_service<gz_physics_bridge::srv::GenerateWorldSdf>(
      service_name_,
      std::bind(&GenerateWorldSdfServer::handle, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "GenerateWorldSdf server ready on %s (world: %s)", service_name_.c_str(), world_.c_str());
  }

private:
  void handle(
    const std::shared_ptr<gz_physics_bridge::srv::GenerateWorldSdf::Request> /*req*/,
    std::shared_ptr<gz_physics_bridge::srv::GenerateWorldSdf::Response> resp)
  {
    // Call Gazebo Transport service with default SdfGeneratorConfig request; expect StringMsg response containing SDF
    gz::msgs::SdfGeneratorConfig gz_req;
    gz::msgs::StringMsg gz_rep;
    bool result = false;    // transport call success

    bool service_ok = node_.Request(service_name_, gz_req, 2000 /*ms*/, gz_rep, result);

    if (!service_ok) {
      resp->success = false;
      resp->sdf = "";
      resp->message = "Transport error or timeout calling " + service_name_;
      return;
    }

    resp->success = result;
    resp->sdf = gz_rep.data();
    resp->message = resp->success ? "ok" : "error";
  }

  std::string world_;
  std::string service_name_;
  rclcpp::Service<gz_physics_bridge::srv::GenerateWorldSdf>::SharedPtr srv_;
  gz::transport::Node node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenerateWorldSdfServer>());
  rclcpp::shutdown();
  return 0;
}


