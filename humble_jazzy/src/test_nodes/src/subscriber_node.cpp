#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/test_message.hpp"
#include "builtin_interfaces/msg/time.hpp"

using std::placeholders::_1;

class TestSubscriber : public rclcpp::Node
{
public:
  TestSubscriber()
  : Node("test_subscriber")
  {
    subscription_ = this->create_subscription<test_msgs::msg::TestMessage>(
      "test_topic", 10, std::bind(&TestSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const test_msgs::msg::TestMessage::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "수신 중: ID=%d, 메시지='%s', 데이터=%.2f", 
                msg->id, msg->message.c_str(), msg->data);
                
    auto now = this->now();
    auto msg_time = rclcpp::Time(msg->stamp.sec, msg->stamp.nanosec);
    auto latency = now - msg_time;
    double latency_ms = latency.nanoseconds() / 1000000.0;
    
    RCLCPP_INFO(this->get_logger(), "지연 시간: %.2f ms", latency_ms);
  }

  rclcpp::Subscription<test_msgs::msg::TestMessage>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSubscriber>());
  rclcpp::shutdown();
  return 0;
} 