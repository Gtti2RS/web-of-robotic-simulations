#include <iostream>
#include <string>
#include <chrono>
#include <functional>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/world_stats.pb.h>

using Clock = std::chrono::steady_clock;

template <typename MsgT>
int runRelay(const std::string &inTopic,
             const std::string &outTopic,
             double rateHz)
{
  if (rateHz <= 0.0) {
    std::cerr << "Rate must be > 0 (got " << rateHz << ")\n";
    return 1;
  }

  gz::transport::Node node;

  auto pub = node.Advertise<MsgT>(outTopic);
  if (!pub) {
    std::cerr << "Failed to advertise " << outTopic << "\n";
    return 1;
  }

  if (rateHz >= 1.0) {
    gz::transport::SubscribeOptions sopts;
    sopts.SetMsgsPerSec(static_cast<unsigned>(rateHz));

    auto cb = [&pub](const MsgT &msg) {
      pub.Publish(msg);
    };

    if (!node.Subscribe(inTopic,
                        std::function<void(const MsgT&)>(cb),
                        sopts)) {
      std::cerr << "Failed to subscribe " << inTopic << "\n";
      return 1;
    }

    std::cout << "[throttle] " << inTopic << " -> " << outTopic
              << " @ " << static_cast<unsigned>(rateHz)
              << " Hz (transport throttle)\n";
  } else {
    const auto minPeriod = std::chrono::duration<double>(1.0 / rateHz);
    auto last = Clock::now() - std::chrono::duration<double>(1.0);

    auto cb = [&pub, minPeriod, last](const MsgT &msg) mutable {
      const auto now = Clock::now();
      if (now - last >= minPeriod) {
        pub.Publish(msg);
        last = now;
      }
    };

    if (!node.Subscribe(inTopic,
                        std::function<void(const MsgT&)>(cb))) {
      std::cerr << "Failed to subscribe " << inTopic << "\n";
      return 1;
    }

    std::cout << "[throttle] " << inTopic << " -> " << outTopic
              << " @ " << rateHz << " Hz (time-based)\n";
  }

  gz::transport::waitForShutdown();
  return 0;
}

int main(int argc, char **argv) {
  if (argc < 6) {
    std::cerr <<
      "Usage: " << argv[0] << " --type {pose_v|world_stats} <in_topic> <out_topic> <rate_hz>\n"
      "Examples:\n"
      "  " << argv[0] << " --type pose_v /world/default/pose/info /throttled/pose/info 2\n"
      "  " << argv[0] << " --type world_stats /world/default/stats /throttled/stats 0.5\n";
    return 1;
  }

  const std::string type   = argv[2];
  const std::string inTop  = argv[3];
  const std::string outTop = argv[4];
  const double rateHz      = std::stod(argv[5]);

  if (type == "pose_v")
    return runRelay<gz::msgs::Pose_V>(inTop, outTop, rateHz);
  if (type == "world_stats")
    return runRelay<gz::msgs::WorldStatistics>(inTop, outTop, rateHz);

  std::cerr << "Unsupported --type: " << type << " (use pose_v | world_stats)\n";
  return 1;
}


