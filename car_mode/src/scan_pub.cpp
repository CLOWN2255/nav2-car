#include <math.h>

#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node_pub = rclcpp::Node::make_shared("scan_pub");
  auto node_sub = rclcpp::Node::make_shared("scan_sub");

 
  auto laser_pub = node_pub->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  

  auto topic_callback =
      [node_sub,laser_pub](sensor_msgs::msg::LaserScan msg) -> void {
        msg.header.frame_id = "base_link";
        // RCLCPP_INFO(node_sub->get_logger(), "frame_id: %s", msg.header.frame_id.c_str());
        laser_pub->publish(msg);
      };

  sensor_msgs::msg::LaserScan msg;
  auto laser_sub = node_sub->create_subscription<sensor_msgs::msg::LaserScan>("/model/joker/lidar", 10, topic_callback);
  rclcpp::spin(node_sub);
  rclcpp::shutdown();

  return 0;
}