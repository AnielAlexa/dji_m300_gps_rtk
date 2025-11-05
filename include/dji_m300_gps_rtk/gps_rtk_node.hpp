#ifndef DJI_M300_GPS_RTK_NODE_HPP
#define DJI_M300_GPS_RTK_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>

// DJI Payload SDK includes
#include "dji_fc_subscription.h"
#include "dji_core.h"
#include "dji_platform.h"

class GpsRtkNode : public rclcpp::Node {
public:
  GpsRtkNode();
  ~GpsRtkNode();

private:
  // Initialization methods
  void init_payload_sdk();
  void init_subscriptions();
  void cleanup();

  // Timer callback for data polling
  void data_poll_callback();

  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr rtk_fix_pub_;

  // Timer for polling data
  rclcpp::TimerBase::SharedPtr timer_;

  // Frame ID
  std::string frame_id_;

  // Payload SDK initialized flag
  bool psdk_initialized_;
};

#endif  // DJI_M300_GPS_RTK_NODE_HPP
