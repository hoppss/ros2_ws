#include <iostream>
#include <string>
#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class MyServer : public rclcpp::Node
{
public:
  MyServer() : Node("add_two_int_server")
  {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", std::bind(&MyServer::add, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MyServer> node = std::make_shared<MyServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}