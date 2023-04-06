#include "xela_tactile/xela_tactile.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("xela_tactile");

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace xela_tactile
{
XelaTactile::XelaTactile(const rclcpp::NodeOptions& options)
  : rclcpp::Node("xela_tactile", options)
{
  // declare parameter
  this->declare_parameter("ip", "127.0.0.1");
  this->declare_parameter("port", 5000);
  this->declare_parameter("debag", false);
  this->declare_parameter("frame_id", "xela_frame");
  this->declare_parameter("topic_name", "xela_topic");

  // get parameter
  this->get_parameter("ip", ip_);
  this->get_parameter("port", port_);
  this->get_parameter("debag", debag_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("topic_name", topic_name_);

  ws_client_ = std::make_shared<WebSocketClient>();
  connect_id_ = ws_client_->connect("ws://" + ip_ + ":" + std::to_string(port_));
  if (connect_id_ == -1)
  {
    RCLCPP_ERROR(LOGGER, "Error: did not connect");
    std::exit(1);
  }

  xela_server_pub_ = this->create_publisher<xela_msgs::msg::XelaServerMsg>(topic_name_, 10);

  timer_ = this->create_wall_timer(100ms, std::bind(&XelaTactile::callbackTimer, this));
}

XelaTactile::~XelaTactile()
{
}

void XelaTactile::callbackTimer()
{
  // get JSON string
  ConnectionMetadata::ptr metadata = ws_client_->get_metadata(connect_id_);

  if (metadata)
  {
    if (debag_)
    {
      RCLCPP_INFO(LOGGER, "received: %s", metadata->get_message().c_str());
    }

    // parse json
    picojson::value v;
    std::string err = picojson::parse(v, metadata->get_message());
    if (!err.empty())
    {
      RCLCPP_ERROR(LOGGER, "Error: parse json");
      return;
    }

    // get each data
    int msg = static_cast<int>(v.get("message").get<double>());
    int num_sensor = static_cast<int>(v.get("sensors").get<double>());
    for (int i = 0; i < num_sensor; ++i)
    {
      picojson::object& obj = v.get<picojson::object>();
      picojson::object& each_sensor = obj[std::to_string(i+1)].get<picojson::object>();

      // get each sensor's data
      std::string data = each_sensor["data"].get<std::string>();
      int sensor = static_cast<int>(each_sensor["sensor"].get<double>());
      int taxels = static_cast<int>(each_sensor["taxels"].get<double>());
      std::string model = each_sensor["model"].get<std::string>();

      // split string
      char delim = ',';
      std::vector<std::string> split_data = simpleSplit(data, delim);

      // set each sensor's data in message
      auto sensor_data = std::make_unique<xela_msgs::msg::XelaServerMsg>();
      sensor_data->header.stamp = this->now();
      sensor_data->header.frame_id = frame_id_;
      sensor_data->sensor = sensor;
      sensor_data->model  = model;
      sensor_data->points.resize(taxels);
      for (int i = 0; i < taxels; ++i)
      {
        // set each taxcel's data
        sensor_data->points[i].taxels = i+1;
        sensor_data->points[i].point.x = static_cast<double>(std::stoi(split_data.at(i*3),   nullptr, 16));
        sensor_data->points[i].point.y = static_cast<double>(std::stoi(split_data.at(i*3+1), nullptr, 16));
        sensor_data->points[i].point.z = static_cast<double>(std::stoi(split_data.at(i*3+2), nullptr, 16));
      }
      // publish message
      xela_server_pub_->publish(std::move(sensor_data));
    }
  }
}

std::vector<std::string> XelaTactile::simpleSplit(const std::string& s, char delim)
{
  std::vector<std::string> elems;
  std::string item;
  for (char ch: s)
  {
    if (ch == delim)
    {
      if (!item.empty())
      {
        elems.push_back(item);
      }
      item.clear();
    }
    else
    {
      item += ch;
    }
  }
  if (!item.empty())
  {
    elems.push_back(item);
  }
  return elems;
}
}  // namespace xela_tactile

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(xela_tactile::XelaTactile)
