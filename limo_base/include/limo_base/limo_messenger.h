/**
 * @Kit       : Qt-Creator: Desktop
 * @Author    : Wang Zhe
 * @Date      : 2021-07-28  18:2:18
 * @FileName  : limo_messenger.h
 * @Mail      : zhe.wang@agilex.ai
 * Copyright  : AgileX Robotics (2021)
 **/

#ifndef LIMON_MESSENGER_H
#define LIMON_MESSENGER_H

#include <limo_msgs/msg/limo_setting.hpp>
#include <limo_msgs/msg/limo_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include "limo_base/limo_params.h"

#include <ugv_sdk/limo_base.h>


using namespace westonrobot;
namespace agx {

class LimoROSMessenger {
 public:
  explicit LimoROSMessenger(std::shared_ptr<rclcpp::Node> nh);
  LimoROSMessenger(LimoBase *limo, std::shared_ptr<rclcpp::Node> nh);

  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;
  bool pub_odom_tf_{true};

  int sim_control_rate_ = 50;

  void GenerateImuMsg(const LimoState& state);
  void SetupSubscription();
  void PublishStateToROS();
  void PublishOdometryToROS(double linear, double angle_vel,
                            double x_linear_vel, double y_linear_vel,
                            double dt);

  double ConvertInnerAngleToCentral(double angle);
  double ConvertCentralAngleToInner(double angle);

 private:
  LimoBase *limo_{nullptr};
  std::shared_ptr<rclcpp::Node> nh_;
  std::mutex twist_mutex_;
  geometry_msgs::msg::Twist current_twist_;
  sensor_msgs::msg::Imu imu_data_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;      // robot odometry
  rclcpp::Publisher<limo_msgs::msg::LimoStatus>::SharedPtr status_publisher_;    // robot status
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;       // imu data
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_cmd_sub_;     // get motion control
  rclcpp::Subscription<limo_msgs::msg::LimoSetting>::SharedPtr limo_setting_sub_;  // system setting

  static constexpr double l = LimoParams::wheelbase;
  static constexpr double w = LimoParams::track;
  static constexpr double steer_angle_tolerance = 0.002;  // +- 0.287 degrees

  // speed variables
  double linear_speed_{0.0};
  double angular_speed_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double theta_{0.0};
  double x_linear_vel_ = 0.0;  // x direction linear velocity
  double y_linear_vel_ = 0.0;  // y direction linear velocity

  rclcpp::Time last_time_;
  rclcpp::Time current_time_;
  uint8_t motion_mode_;  // current motion type

  void TwistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void LimoSettingCbk(const limo_msgs::msg::LimoSetting::SharedPtr msg);
};
}  // namespace agx
#endif
