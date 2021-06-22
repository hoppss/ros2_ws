#include <string>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : rclcpp::Node("minimal_publisher"), count_(0)
  {
    setvbuf(stdout, NULL, _IONBF, 1024);
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(100ms, std::bind(&MinimalPublisher::timer_callback, this));
    node_name_ = "my_publisher";
    if (!this->has_parameter("age"))
    {
      this->declare_parameter("age", rclcpp::ParameterValue(250));
    }

    // 声明就建立参数， 并带有参数

    if (this->has_parameter("age"))
    {
      int age = 0;

      bool ret = this->get_parameter("age", age);
      std::cout << "result " << (int)ret << " age " << age << std::endl;
    }
    // nav2 中先声明， 后get，可能是结合参数文件yaml的方式，加载yaml后覆盖参数值
    // declare_parameter_if_not_declared
    // node->get_parameter()
  }

private:
  void timer_callback()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "hello_world_" + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: %s", msg.data.c_str());
    publisher_->publish(msg);
    std::flush(std::cout);
  }

  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string node_name_;
};

int main(int argc, char** argv)
{
  // setvbuf(stdout, nullptr, _IONBF, 1024);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());

  rclcpp::shutdown();
  return 0;
}
