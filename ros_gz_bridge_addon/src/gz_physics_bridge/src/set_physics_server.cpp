#include <chrono>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "gz_physics_bridge/srv/set_physics.hpp"

// Gazebo Transport / Messages
#include <gz/transport/Node.hh>
#include <gz/msgs/physics.pb.h>
#include <gz/msgs/boolean.pb.h>

using namespace std::chrono_literals;

class SetPhysicsServer : public rclcpp::Node {
public:
  SetPhysicsServer() : Node("set_physics_server")
  {
    world_ = this->declare_parameter<std::string>("world", "empty");
    service_name_ = "/world/" + world_ + "/set_physics";

    srv_ = this->create_service<gz_physics_bridge::srv::SetPhysics>(
      service_name_,
      std::bind(&SetPhysicsServer::handle, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "SetPhysics server ready on %s (world: %s)", service_name_.c_str(), world_.c_str());
  }

private:
  void handle(
    const std::shared_ptr<gz_physics_bridge::srv::SetPhysics::Request> req,
    std::shared_ptr<gz_physics_bridge::srv::SetPhysics::Response> resp)
  {
    gz::msgs::Physics physics;

    int fields_set = 0;

    if (req->max_step_size > 0.0) {
      physics.set_max_step_size(req->max_step_size);
      ++fields_set;
    }
    if (req->real_time_factor > 0.0) {
      physics.set_real_time_factor(req->real_time_factor);
      ++fields_set;
    }
    if (req->real_time_update_rate > 0.0) {
      physics.set_real_time_update_rate(req->real_time_update_rate);
      ++fields_set;
    }
    if (!req->solver_type.empty()) {
      physics.set_solver_type(req->solver_type);
      ++fields_set;
    }
    if (req->sor > 0.0) {
      physics.set_sor(req->sor);
      ++fields_set;
    }
    if (!req->engine.empty()) {
      static const std::unordered_map<std::string, int> type_map{
        {"ode", 1}, {"bullet", 2}, {"dart", 3}, {"simbody", 4}, {"tpe", 6}
      };
      auto it = type_map.find(req->engine);
      if (it != type_map.end()) {
        physics.set_type(static_cast<gz::msgs::Physics::Type>(it->second));
        ++fields_set;
      } else {
        RCLCPP_WARN(this->get_logger(),
          "Unknown engine '%s'; ignoring. Try one of: ode, bullet, dart, simbody, tpe",
          req->engine.c_str());
      }
    }

    if (fields_set == 0) {
      resp->success = false;
      resp->message = "No fields set; nothing to update.";
      return;
    }

    // Call Gazebo Transport service
    gz::msgs::Boolean gz_reply;
    bool result = false;    // transport call success
    bool service_ok = node_.Request(
      service_name_, physics, 1000 /*ms timeout*/, gz_reply, result);

    if (!service_ok) {
      resp->success = false;
      resp->message = "Transport error or timeout calling " + service_name_;
      return;
    }

    resp->success = result && gz_reply.data();
    resp->message = resp->success ? "true" : "false";
  }

  std::string world_;
  std::string service_name_;
  rclcpp::Service<gz_physics_bridge::srv::SetPhysics>::SharedPtr srv_;
  gz::transport::Node node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetPhysicsServer>());
  rclcpp::shutdown();
  return 0;
}
