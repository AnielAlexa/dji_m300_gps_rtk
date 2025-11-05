#include "dji_m300_gps_rtk/gps_rtk_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <cstring>
#include <unistd.h>
#include <pthread.h>

// DJI SDK headers (from Payload SDK)
extern "C" {
#include "dji_platform.h"
#include "dji_fc_subscription.h"
}

using namespace std::chrono_literals;

GpsRtkNode::GpsRtkNode()
    : rclcpp::Node("gps_rtk_node"),
      frame_id_("gps_rtk_antenna"),
      psdk_initialized_(false) {

  // Create publishers
  gps_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/m300/gps/fix", 10);
  rtk_fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(
      "/m300/rtk/fix", 10);

  RCLCPP_INFO(this->get_logger(), "GPS/RTK Node created");

  // Initialize Payload SDK
  try {
    init_payload_sdk();
    init_subscriptions();
    psdk_initialized_ = true;

    // Create timer for polling data at 1 Hz
    timer_ = this->create_wall_timer(
        1000ms, std::bind(&GpsRtkNode::data_poll_callback, this));

    RCLCPP_INFO(this->get_logger(), "Payload SDK initialized successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize Payload SDK: %s", e.what());
    psdk_initialized_ = false;
  }
}

GpsRtkNode::~GpsRtkNode() {
  cleanup();
}

void GpsRtkNode::init_payload_sdk() {
  T_DjiReturnCode returnCode;

  // Initialize FC subscription module
  // Note: The Payload SDK platform, OSAL, and HAL handlers should already be
  // registered by the main DJI SDK application that started this node
  returnCode = DjiFcSubscription_Init();
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    throw std::runtime_error(
        "Failed to initialize FC subscription - "
        "Ensure DJI SDK is properly initialized in your main application");
  }

  RCLCPP_INFO(this->get_logger(), "FC Subscription initialized");
}

void GpsRtkNode::init_subscriptions() {
  T_DjiReturnCode returnCode;

  // Subscribe to GPS Position
  returnCode = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      nullptr);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    throw std::runtime_error("Failed to subscribe to GPS position");
  }

  // Subscribe to GPS Details (for accuracy info)
  returnCode = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      nullptr);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    throw std::runtime_error("Failed to subscribe to GPS details");
  }

  // Subscribe to RTK Position
  returnCode = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      nullptr);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    throw std::runtime_error("Failed to subscribe to RTK position");
  }

  // Subscribe to RTK Position Info (for fix status)
  returnCode = DjiFcSubscription_SubscribeTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
      DJI_DATA_SUBSCRIPTION_TOPIC_1_HZ,
      nullptr);
  if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    throw std::runtime_error("Failed to subscribe to RTK position info");
  }

  RCLCPP_INFO(this->get_logger(), "All subscriptions initialized");
}

void GpsRtkNode::data_poll_callback() {
  if (!psdk_initialized_) {
    return;
  }

  T_DjiReturnCode returnCode;
  T_DjiDataTimestamp timestamp;

  // Get GPS Position
  T_DjiFcSubscriptionGpsPosition gpsPosition = {0};
  returnCode = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION,
      (uint8_t*)&gpsPosition,
      sizeof(T_DjiFcSubscriptionGpsPosition),
      &timestamp);

  if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // Get GPS Details for accuracy
    T_DjiFcSubscriptionGpsDetails gpsDetails = {0};
    DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS,
        (uint8_t*)&gpsDetails,
        sizeof(T_DjiFcSubscriptionGpsDetails),
        &timestamp);

    // Create NavSatFix message
    auto gps_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    gps_msg->header.stamp = this->now();
    gps_msg->header.frame_id = frame_id_;

    // Convert from DJI format: deg*10^-7
    gps_msg->latitude = gpsPosition.y / 10000000.0;
    gps_msg->longitude = gpsPosition.x / 10000000.0;
    gps_msg->altitude = gpsPosition.z / 1000.0;  // mm to m

    // Set fix status
    gps_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps_msg->status.service =
        sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
        sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

    // Set covariance based on horizontal/vertical accuracy
    double h_acc = gpsDetails.hacc / 1000.0;  // mm to m
    double v_acc = gpsDetails.vacc / 1000.0;

    // ENU covariance (East, North, Up)
    gps_msg->position_covariance[0] = h_acc * h_acc;  // East
    gps_msg->position_covariance[4] = h_acc * h_acc;  // North
    gps_msg->position_covariance[8] = v_acc * v_acc;  // Up
    gps_msg->position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    gps_fix_pub_->publish(std::move(gps_msg));
  }

  // Get RTK Position
  T_DjiFcSubscriptionRtkPosition rtkPosition = {0};
  returnCode = DjiFcSubscription_GetLatestValueOfTopic(
      DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION,
      (uint8_t*)&rtkPosition,
      sizeof(T_DjiFcSubscriptionRtkPosition),
      &timestamp);

  if (returnCode == DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
    // Get RTK Position Info for fix status
    T_DjiFcSubscriptionRtkPositionInfo rtkInfo = 0;
    DjiFcSubscription_GetLatestValueOfTopic(
        DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO,
        (uint8_t*)&rtkInfo,
        sizeof(T_DjiFcSubscriptionRtkPositionInfo),
        &timestamp);

    // Create NavSatFix message
    auto rtk_msg = std::make_unique<sensor_msgs::msg::NavSatFix>();
    rtk_msg->header.stamp = this->now();
    rtk_msg->header.frame_id = frame_id_;

    // RTK position is already in degrees
    rtk_msg->latitude = rtkPosition.latitude;
    rtk_msg->longitude = rtkPosition.longitude;
    rtk_msg->altitude = rtkPosition.hfsl;

    // Set fix status and service info
    rtk_msg->status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    rtk_msg->status.service =
        sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
        sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS;

    // Set covariance based on RTK fix state
    double accuracy = 0.1;  // Default ~10cm for float

    if (rtkInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT) {
      // Fixed solution - ~2cm accuracy
      accuracy = 0.02;
    } else if (rtkInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION) {
      // Float solution - ~10cm accuracy
      accuracy = 0.10;
    } else {
      // No fix or single point
      accuracy = 3.0;  // GPS accuracy
    }

    // ENU covariance
    rtk_msg->position_covariance[0] = accuracy * accuracy;  // East
    rtk_msg->position_covariance[4] = accuracy * accuracy;  // North
    rtk_msg->position_covariance[8] = accuracy * 1.5 * accuracy * 1.5;  // Up (worse)
    rtk_msg->position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    rtk_fix_pub_->publish(std::move(rtk_msg));

    // Log RTK status
    std::string status_str = "UNKNOWN";
    if (rtkInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_NARROW_INT) {
      status_str = "FIXED";
    } else if (rtkInfo == DJI_FC_SUBSCRIPTION_POSITION_SOLUTION_PROPERTY_FLOAT_SOLUTION) {
      status_str = "FLOAT";
    } else {
      status_str = "NO_FIX";
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "RTK Status: %s, Lat: %.8f, Lon: %.8f, Alt: %.3f",
                 status_str.c_str(),
                 rtkPosition.latitude,
                 rtkPosition.longitude,
                 rtkPosition.hfsl);
  }
}

void GpsRtkNode::cleanup() {
  if (psdk_initialized_) {
    // Unsubscribe from all topics
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_GPS_DETAILS);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION);
    DjiFcSubscription_UnSubscribeTopic(DJI_FC_SUBSCRIPTION_TOPIC_RTK_POSITION_INFO);

    // Deinit
    DjiFcSubscription_DeInit();
    psdk_initialized_ = false;
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<GpsRtkNode>());
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
