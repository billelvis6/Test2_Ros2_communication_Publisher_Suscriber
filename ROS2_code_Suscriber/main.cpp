#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>
#include <string>

class SensorSubscriber : public rclcpp::Node
{
public:
  SensorSubscriber() : Node("sensor_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/sensor_data", 10,
      std::bind(&SensorSubscriber::process_message, this, std::placeholders::_1)
    );
  }

private:
  void process_message(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Message brut reçu : '%s'", msg->data.c_str());

    std::stringstream ss(msg->data);
    float temp, hum, press;
    char comma1, comma2;

    if (!(ss >> temp >> comma1 >> hum >> comma2 >> press)) {
      RCLCPP_ERROR(this->get_logger(), "⚠️ Format de message invalide !");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Température : %.2f°C | Humidité : %.2f%% | Pression : %.2f hPa",
                temp, hum, press);

    // Optionnel : ajout de vérifications de plage
    if (temp < 15 || temp > 35)
      RCLCPP_WARN(this->get_logger(), "⚠️ Température hors plage !");
    if (hum < 30 || hum > 70)
      RCLCPP_WARN(this->get_logger(), "⚠️ Humidité hors plage !");
    if (press < 950 || press > 1050)
      RCLCPP_WARN(this->get_logger(), "⚠️ Pression hors plage !");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
