#include <chrono>
#include <format>
#include <memory>
#include <string>

#include <fmt/format.h>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rdelfin_msgs/msg/custom_message.hpp"
#include "std_msgs/msg/header.hpp"

class Talker : public rclcpp::Node {
public:
  Talker() : Node("talker"), count_(0) {
    publisher_ =
        this->create_publisher<rdelfin_msgs::msg::CustomMessage>("my_msg", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     std::bind(&Talker::timer_callback, this));
  }

  ~Talker() {}

private:
  void timer_callback() {
    auto time = clock_.now();

    auto message = rdelfin_msgs::msg::CustomMessage();
    message.header.stamp = now_time();
    message.header.frame_id = "talker_frame";
    message.data = fmt::format("Hello, world! {}", count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  builtin_interfaces::msg::Time now_time() {
    auto now = clock_.now();
    auto now_ns = now.nanoseconds();
    builtin_interfaces::msg::Time t;
    t.sec = now_ns / 1'000'000'000;
    t.nanosec = now_ns % 1'000'000'000;
    return t;
  }

  rclcpp::Clock clock_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rdelfin_msgs::msg::CustomMessage>::SharedPtr publisher_;
  uint32_t count_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
