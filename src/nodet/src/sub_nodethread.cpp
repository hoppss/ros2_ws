#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_util/node_thread.hpp"

#include "tf2_ros/buffer.h"
#include "tf2/convert.h"                    // NOLINT
#include "tf2/utils.h"                      // NOLINT
#include "tf2_ros/transform_listener.h"     // NOLINT
#include "tf2_ros/transform_broadcaster.h"  //NOLINT


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

    tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    auto t = std::make_shared<std::thread>(std::bind(&MinimalSubscriber::loop, this));
    t->detach();
  };

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard %s, %ld", msg->data.c_str(), std::this_thread::get_id());
  }

  void loop()
  {
    // RCLCPP_ERROR(get_logger(), "t_join start");
    // t->join();
    // RCLCPP_ERROR(get_logger(), "t_join stop");

    while(rclcpp::ok()) {
      auto now_stamp =now();
      std::string tf_error;
      rclcpp::Time tfExpiration = now_stamp + rclcpp::Duration(0.1*10e9);
      RCLCPP_ERROR(this->get_logger(), "now %f, exp %f", now_stamp.seconds(), tfExpiration.seconds());

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // //1
      // try {
      //   auto temp = now_stamp - rclcpp::Duration(0.5);
      //   if(!tf_->canTransform("map", "base_footprint", tf2::TimePointZero, &tf_error)) {
      //   //tf_->canTransform("map", "base_footprint", temp, tf2::durationFromSec(0.5), &tf_error))
      //      RCLCPP_ERROR(get_logger(), "CANTRANSFORM FAILED, %s", tf_error.c_str());
      //   } else {
      //      RCLCPP_ERROR(get_logger(), "CANTRANSFORM YES. %s", tf_error.c_str());
      //   }
      // }
      // catch(...) {
      //       RCLCPP_ERROR(get_logger(), "CANTRANSFORM exception， %s", tf_error.c_str());
      // }

      // //2
      // try{
      // auto transform = tf_->lookupTransform("map", "base_footprint", tf2::TimePointZero); //tf2::LookupException  exception
      //      if ( (now_stamp - rclcpp::Time(transform.header.stamp) ).seconds() > 0.2)
      // {
      //   RCLCPP_ERROR(this->get_logger(), "LOOKUP: now: %f, avaluable %f", now_stamp.seconds(), rclcpp::Time(transform.header.stamp).seconds());
      // } else {
      //   RCLCPP_ERROR(this->get_logger(), "LOOKUP: now: %f, avaluable %f", now_stamp.seconds(), rclcpp::Time(transform.header.stamp).seconds());
      // }

      // } catch(...) {
      // RCLCPP_ERROR(this->get_logger(), "??? cathc .....");
      // }

    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::thread * t{nullptr};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  RCLCPP_INFO(node->get_logger(), "main thread id %ld", std::this_thread::get_id());

  auto node_thread = std::make_shared<nav2_util::NodeThread>(node);

  while (node_thread && rclcpp::ok())
  {
    RCLCPP_INFO(node->get_logger(), "wait %ld", std::this_thread::get_id());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  rclcpp::shutdown();

  return 0;
}
