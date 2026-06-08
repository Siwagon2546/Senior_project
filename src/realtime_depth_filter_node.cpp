#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace robot_bringup
{

class RealtimeDepthFilterNode : public rclcpp::Node
{
public:
  explicit RealtimeDepthFilterNode(const rclcpp::NodeOptions & options)
  : Node("realtime_depth_filter_node", options)
  {
    fill_depth_ = this->declare_parameter<double>("fill_depth", 5.0);

    auto qos = rclcpp::SensorDataQoS().keep_last(1);

    pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "depth/image_filtered",
      qos
    );

    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "depth/image_raw",
      qos,
      std::bind(&RealtimeDepthFilterNode::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Zero-only depth filter started | fill_depth=%.2f",
      fill_depth_
    );
  }

private:
  void callback(sensor_msgs::msg::Image::UniquePtr msg)
  {
    if (msg->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Unsupported depth encoding: %s. Expected 32FC1.",
        msg->encoding.c_str()
      );
      return;
    }

    const float fill_depth = static_cast<float>(fill_depth_);

    const size_t pixel_count =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

    float * data = reinterpret_cast<float *>(msg->data.data());

    for (size_t i = 0; i < pixel_count; ++i) {
      if (data[i] == 0.0f) {
        data[i] = fill_depth;
      }
    }

    pub_->publish(std::move(msg));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  double fill_depth_;
};

}  // namespace robot_bringup

RCLCPP_COMPONENTS_REGISTER_NODE(robot_bringup::RealtimeDepthFilterNode)