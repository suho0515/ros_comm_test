#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/test_message.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("test_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<test_msgs::msg::TestMessage>("test_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = test_msgs::msg::TestMessage();
    message.id = count_++;
    message.message = "테스트 메시지 #" + std::to_string(count_);
    message.data = static_cast<double>(count_) * 1.5;
    message.stamp = this->now();
    
    RCLCPP_INFO(this->get_logger(), "발행 중: '%s'", message.message.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<test_msgs::msg::TestMessage>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
} 