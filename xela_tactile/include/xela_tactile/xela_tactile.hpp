#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "xela_tactile/picojson.h"
#include "xela_tactile/websocket_client.hpp"

#include <xela_msgs/msg/xela_server_msg.hpp>

namespace xela_tactile
{
class XelaTactile : public rclcpp::Node
{
public:
  XelaTactile(const rclcpp::NodeOptions& options);
  ~XelaTactile();
private:
  void callbackTimer();
  std::vector<std::string> simpleSplit(const std::string& s, char delim);

  rclcpp::Publisher<xela_msgs::msg::XelaServerMsg>::SharedPtr xela_server_pub_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<WebSocketClient> ws_client_;
};
}

