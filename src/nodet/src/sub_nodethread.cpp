#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/node_thread.hpp"
#include <chrono>

/*

[INFO] [1621494925.846222163] [minimal_subscriber]: main thread id 140603377504064
[INFO] [1621494925.846675400] [minimal_subscriber]: wait 140603377504064
[INFO] [1621494925.939821002] [minimal_subscriber]: I heard hello_world_284, 140603102836480

NodeThread will use rclcpp::executor, its node callback will executed in its unique thread callback

*/
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  };

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard %s, %ld", msg->data.c_str(), std::this_thread::get_id());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  RCLCPP_INFO(node->get_logger(), "main thread id %ld", std::this_thread::get_id());

  auto node_thread = std::make_shared<nav2_util::NodeThread>(node);

  while (node_thread)
  {
    RCLCPP_INFO(node->get_logger(), "wait %ld", std::this_thread::get_id());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();

  return 0;
}