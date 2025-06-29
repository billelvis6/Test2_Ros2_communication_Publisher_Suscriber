#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node
{
public:
  SensorPublisher() : Node("sensor_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("/sensor_data", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&SensorPublisher::publish_data, this));
    std::srand(std::time(nullptr));
  }

private:
  void publish_data()
  {
    float temperature = 15 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (35 - 15)));
    float humidity = 30 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (70 - 30)));
    float pressure = 950 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (1050 - 950)));

    std_msgs::msg::String msg;
    std::stringstream ss;
    ss << temperature << "," << humidity << "," << pressure;
    msg.data = ss.str();

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher>());
  rclcpp::shutdown();
  return 0;
}

