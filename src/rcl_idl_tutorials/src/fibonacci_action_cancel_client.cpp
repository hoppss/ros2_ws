#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <thread>

#include "rcl_idl_tutorials/action/fibonacci.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
{
public:
  //   using Fibonacci = rcl_idl_tutorials::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<rcl_idl_tutorials::action::Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("fibonacci_action_client", options)
  {
    RCLCPP_INFO(this->get_logger(), "Setup fibonacci client");
    this->client_ptr_ = rclcpp_action::create_client<rcl_idl_tutorials::action::Fibonacci>(this, "fibonacci");
    int cnt = 0;
    int i_ = 0;

    while (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(3)) && rclcpp::ok() && cnt < 5)
    {
      RCLCPP_INFO(this->get_logger(), "Check for sever %d", cnt);
      ++cnt;
      //   if (!rclcpp::sleep_for(1s))
      //   {
      //     rclcpp::shutdown();  // ctrl+c and return;
      //   };
    }

    if (cnt >= 5)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to find server, shutdown client node");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Setup spin thread");

    spin_thread_ = std::make_shared<std::thread>(std::bind(&FibonacciActionClient::spinThread, this));

    int end = i_ + 1000;
    for (; i_ < end && rclcpp::ok(); ++i_)
    {
      RCLCPP_INFO(this->get_logger(), "Call Servier %d", i_);
      send_goal();
      rclcpp::sleep_for(std::chrono::milliseconds(300));
      RCLCPP_INFO(this->get_logger(), "cancel %d", i_);
      cancel();
    }

    RCLCPP_INFO(this->get_logger(), "___FINISH");
  }

  ~FibonacciActionClient()
  {
    if (spin_thread_)
    {
      spin_thread_->join();
    }
  }

  void send_goal()
  {
    using namespace std::placeholders;

    auto goal_msg = rcl_idl_tutorials::action::Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<rcl_idl_tutorials::action::Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FibonacciActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel()
  {
    //
    // auto cancel_future = this->client_ptr_->async_cancel_all_goals();
    // rclcpp::spin_until_future_complete(this->client_ptr_, cancel_future);

    // for result callback processing
    // rclcpp::spin_some(this->client_ptr_);

    if (this->goal_handle_ptr_)
    {
      RCLCPP_INFO(this->get_logger(), "Sending cancel request");
      try
      {
        this->client_ptr_->async_cancel_goal(goal_handle_ptr_);
      }
      catch (...)
      {
        // This can happen if the goal has already terminated and expired
        RCLCPP_INFO(this->get_logger(), "Cancel, excepiton");
      }
    }
  }

  void spinThread()
  {
    // while (rclcpp::ok())
    // {
    rclcpp::spin(this->get_node_base_interface());
    // }
  }

private:
  rclcpp_action::Client<rcl_idl_tutorials::action::Fibonacci>::SharedPtr client_ptr_;
  rclcpp_action::ClientGoalHandle<rcl_idl_tutorials::action::Fibonacci>::SharedPtr goal_handle_ptr_;
  std::shared_ptr<std::thread> spin_thread_;
  int i_;

  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    this->goal_handle_ptr_ = future.get();
    if (!this->goal_handle_ptr_)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFibonacci::SharedPtr,
                         const std::shared_ptr<const rcl_idl_tutorials::action::Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // rclcpp::shutdown();
  }
};  // class FibonacciActionClient

}  // namespace action_tutorials_cpp

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<action_tutorials_cpp::FibonacciActionClient>();

  //   rclcpp::spin(node);
  return 0;
}