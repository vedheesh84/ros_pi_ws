/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node - Humble-compatible patched for X2/X2L
 *
 *  Drop into src/ydlidar_ros2_driver/src/ydlidar_ros2_driver_node.cpp
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h" // keep relative path if repo expects it
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

#define DRIVER_VERSION "1.0.1-humble-patch"

using namespace ydlidar;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
  RCLCPP_INFO(node->get_logger(),
              "[YDLIDAR] Driver %s starting (Humble patched)", DRIVER_VERSION);

  CYdLidar laser;

  // -----------------------------
  // STRING PARAMETERS (typed)
  // -----------------------------
  std::string port;
  node->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  node->get_parameter("port", port);

  std::string ignore_array;
  node->declare_parameter<std::string>("ignore_array", "");
  node->get_parameter("ignore_array", ignore_array);

  std::string frame_id;
  node->declare_parameter<std::string>("frame_id", "laser_frame");
  node->get_parameter("frame_id", frame_id);

  // -----------------------------
  // INT PARAMETERS
  // -----------------------------
  int baudrate;
  node->declare_parameter<int>("baudrate", 115200);
  node->get_parameter("baudrate", baudrate);

  int lidar_type;
  // For X2/X2L we choose TYPE_TRIANGLE by default (SDK constants)
  node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  node->get_parameter("lidar_type", lidar_type);

  int device_type;
  node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  node->get_parameter("device_type", device_type);

  int sample_rate;
  node->declare_parameter<int>("sample_rate", 3);
  node->get_parameter("sample_rate", sample_rate);

  int abnormal_check_count;
  node->declare_parameter<int>("abnormal_check_count", 4);
  node->get_parameter("abnormal_check_count", abnormal_check_count);

  int intensity_bit;
  node->declare_parameter<int>("intensity_bit", 0);
  node->get_parameter("intensity_bit", intensity_bit);

  // -----------------------------
  // BOOL PARAMETERS
  // -----------------------------
  bool fixed_resolution;
  node->declare_parameter<bool>("fixed_resolution", true); // X2L uses fixed resolution
  node->get_parameter("fixed_resolution", fixed_resolution);

  bool reversion;
  node->declare_parameter<bool>("reversion", false);
  node->get_parameter("reversion", reversion);

  bool inverted;
  node->declare_parameter<bool>("inverted", false);
  node->get_parameter("inverted", inverted);

  bool auto_reconnect;
  node->declare_parameter<bool>("auto_reconnect", true);
  node->get_parameter("auto_reconnect", auto_reconnect);

  bool isSingleChannel;
  node->declare_parameter<bool>("isSingleChannel", false);
  node->get_parameter("isSingleChannel", isSingleChannel);

  bool intensity;
  node->declare_parameter<bool>("intensity", false); // enable intensity by default for X2L
  node->get_parameter("intensity", intensity);

  bool support_motor_dtr;
  node->declare_parameter<bool>("support_motor_dtr", true);
  node->get_parameter("support_motor_dtr", support_motor_dtr);

  bool debug;
  node->declare_parameter<bool>("debug", false);
  node->get_parameter("debug", debug);

  // -----------------------------
  // FLOAT PARAMETERS
  // -----------------------------
  float angle_max;
  node->declare_parameter<float>("angle_max", 180.0f);
  node->get_parameter("angle_max", angle_max);

  float angle_min;
  node->declare_parameter<float>("angle_min", -180.0f);
  node->get_parameter("angle_min", angle_min);

  float range_max;
  node->declare_parameter<float>("range_max", 16.0f); // X2L nominal range
  node->get_parameter("range_max", range_max);

  float range_min;
  node->declare_parameter<float>("range_min", 0.05f);
  node->get_parameter("range_min", range_min);

  float frequency;
  node->declare_parameter<float>("frequency", 10.0f);
  node->get_parameter("frequency", frequency);

  bool invalid_range_is_inf;
  node->declare_parameter<bool>("invalid_range_is_inf", false);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  bool skip_health_check;
  node->declare_parameter<bool>("skip_health_check", true); // X2L: allow skipping defaults
  node->get_parameter("skip_health_check", skip_health_check);

  // -----------------------------
  // Apply options to SDK
  // -----------------------------
  // Strings
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  // Ints
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count, sizeof(int));
  laser.setlidaropt(LidarPropIntenstiyBit, &intensity_bit, sizeof(int));

  // Bools
  laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));
  laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr, sizeof(bool));
  laser.setEnableDebug(debug);

  // Floats
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  // -----------------------------
  // Initialize LIDAR
  // -----------------------------
  RCLCPP_INFO(node->get_logger(), "Initializing YDLidar on %s @ %d", port.c_str(), baudrate);
  bool initialized = laser.initialize();

  if (!initialized) {
    RCLCPP_ERROR(node->get_logger(), "YDLidar initialize failed: %s", laser.DescribeError());
    // Do not exit immediately; return nonzero to notify failure
    rclcpp::shutdown();
    return 1;
  }

/*
  // Health/baseplate queries — skip for X2L (some firmwares do not expose)
  if (!skip_health_check) {
    int health_code = 0;
    if (laser.getHealth(health_code) && health_code == 0) {
      RCLCPP_INFO(node->get_logger(), "Lidar health OK (code=0)");
    } else {
      RCLCPP_WARN(node->get_logger(), "Lidar health not OK or unavailable (code=%d). Proceeding as configured.", health_code);
    }
    // getBasePlateInfo may not be available for X2L; tolerant behavior above.
  } else {
    RCLCPP_INFO(node->get_logger(), "Skipping lidar health/baseplate checks (skip_health_check=true)");
  }
  */
  
  RCLCPP_INFO(node->get_logger(), "Skipping lidar health/baseplate checks");
  
  // Workmode params (GS only, harmless for non-GS)
  int m1_mode;
  node->declare_parameter<int>("m1_mode", 0);
  node->get_parameter("m1_mode", m1_mode);
  laser.setWorkMode(m1_mode, 0x01);

  int m2_mode;
  node->declare_parameter<int>("m2_mode", 0);
  node->get_parameter("m2_mode", m2_mode);
  laser.setWorkMode(m2_mode, 0x02);

  int m3_mode;
  node->declare_parameter<int>("m3_mode", 1);
  node->get_parameter("m3_mode", m3_mode);
  laser.setWorkMode(m3_mode, 0x04);

  // Turn on device
  bool turned_on = laser.turnOn();
  if (!turned_on) {
    RCLCPP_ERROR(node->get_logger(), "Failed to turn on lidar: %s", laser.DescribeError());
    laser.disconnecting();
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(node->get_logger(), "YDLidar turned on, publishing on /scan (frame_id=%s)", frame_id.c_str());

  // Publisher
  auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  // Services
  auto stop_cb = [&laser](const std::shared_ptr<rmw_request_id_t>,
                          const std::shared_ptr<std_srvs::srv::Empty::Request>,
                          std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
    return laser.turnOff();
  };
  node->create_service<std_srvs::srv::Empty>("stop_scan", stop_cb);

  auto start_cb = [&laser](const std::shared_ptr<rmw_request_id_t>,
                           const std::shared_ptr<std_srvs::srv::Empty::Request>,
                           std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool {
    return laser.turnOn();
  };
  node->create_service<std_srvs::srv::Empty>("start_scan", start_cb);

  // Main loop
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      auto msg = std::make_shared<sensor_msgs::msg::LaserScan>();

      msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(msg->header.stamp.sec);
      msg->header.frame_id = frame_id;

      msg->angle_min = scan.config.min_angle;
      msg->angle_max = scan.config.max_angle;
      msg->angle_increment = scan.config.angle_increment;
      msg->scan_time = scan.config.scan_time;
      msg->time_increment = scan.config.time_increment;
      msg->range_min = scan.config.min_range;
      msg->range_max = scan.config.max_range;

      int size = std::lround((scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment) + 1;
      if (size <= 0) size = static_cast<int>(scan.points.size());
      msg->ranges.assign(size, std::numeric_limits<float>::infinity());
      msg->intensities.assign(size, 0.0f);

      for (size_t i = 0; i < scan.points.size(); ++i) {
        int idx = static_cast<int>(std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment));
        if (idx >= 0 && idx < size) {
          msg->ranges[idx] = scan.points[i].range;
          msg->intensities[idx] = scan.points[i].intensity;
        }
      }

      pub->publish(*msg);
    } else {
      RCLCPP_WARN(node->get_logger(), "No scan received this cycle from SDK (doProcessSimple returned false)");
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "Shutting down YDLidar node");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}

