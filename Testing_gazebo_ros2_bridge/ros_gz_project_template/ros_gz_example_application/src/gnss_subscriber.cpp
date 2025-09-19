// Rocky Ren 9_18 update the subscriber node. 
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

// A simple class that inherits from rclcpp::Node
class GnssSubscriber : public rclcpp::Node
{
public:
  // Constructor for the node
  GnssSubscriber()
  : Node("gnss_subscriber")
  {
    // Create a subscription to the /gnss/fix topic.
    // The QoS profile is set to 10, which is a common depth for sensor data.
    // The callback function (topic_callback) is bound to this class instance.
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gnss/fix", 10, std::bind(&GnssSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  // The callback function that is executed whenever a message is received.
  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
  {
    // Log the received latitude and longitude to the ROS 2 logger.
    RCLCPP_INFO(this->get_logger(), "Received GNSS Fix: Latitude=%.6f, Longitude=%.6f",
      msg->latitude, msg->longitude);
  }

  // Declare the subscription member variable.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

// The main function, which is the entry point of the program.
int main(int argc, char * argv[])
{
  // Initialize the ROS 2 client library.
  rclcpp::init(argc, argv);
  // Create and spin the node, which keeps it alive to receive messages.
  rclcpp::spin(std::make_shared<GnssSubscriber>());
  // Shut down the ROS 2 client library.
  rclcpp::shutdown();
  return 0;
}