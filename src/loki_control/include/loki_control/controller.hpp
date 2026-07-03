/**
 * @file controller.hpp
 * @brief Cascaded PID controller for the loki auv.
 *
 * Control architecture:
 *
 *   /target/speed    → speed PID   → /cmd/thruster    (PWM)
 *   /target/heading  → yaw PID     → /cmd/rudder      (PWM)
 *   /target/depth    → depth PID   → desired_pitch_
 *                      pitch PID   → /cmd/elevator     (PWM)
 *   /target/moving_mass            → /cmd/moving_mass  (Float64)
 *
 * Standard topic interface:
 *   Sub: /odometry/filtered, /target/depth, /target/heading,
 *        /target/speed, /target/moving_mass
 *   Pub: /cmd/thruster, /cmd/elevator, /cmd/rudder,
 *        /cmd/moving_mass, /system/arm_state,
 *        /monitor/target/depth, /monitor/target/heading,
 *        /monitor/target/speed, /monitor/target/moving_mass,
 *        /monitor/desired_pitch
 *   Srv: /system/arm
 */

#ifndef LOKI_CONTROL__CONTROLLER_HPP_
#define LOKI_CONTROL__CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "loki_control/pid.hpp"

namespace loki
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode();

private:

  // ── Callbacks ─────────────────────────────────────────────
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_target_depth(const std_msgs::msg::Float64::SharedPtr msg);
  void on_target_heading(const std_msgs::msg::Float64::SharedPtr msg);
  void on_target_speed(const std_msgs::msg::Float64::SharedPtr msg);
  void on_target_moving_mass(const std_msgs::msg::Float64::SharedPtr msg);
  void on_arm(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    const std_srvs::srv::SetBool::Response::SharedPtr res);

  // ── Control loops ─────────────────────────────────────────
  void outer_loop();   // 20Hz 
  void inner_loop();   // 100Hz 

  // ── Helpers ───────────────────────────────────────────────
  static int    effort_to_pwm(double effort);  // [-400, +400] → PWM [1100, 1900], neutral 1500
  static double wrap_angle(double deg);         // wraps heading error to [-180, +180]
  PIDParams     load_pid(const std::string & ns, double alpha);
  void          publish_f64(
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double value);

  // ── State ─────────────────────────────────────────────────
  bool   is_armed_           = false;
  bool   speed_unlocked_     = false;  // true only after a non-zero speed command
  double current_depth_      = 0.0;
  double current_heading_    = 0.0;
  double current_pitch_      = 0.0;
  double current_pitch_rate_ = 0.0;
  double current_speed_      = 0.0;
  double desired_pitch_      = 0.0; 

  // ── Targets ───────────────────────────────────────────────
  double target_depth_       = 0.0;
  double target_heading_     = 0.0;
  double target_speed_       = 0.0;
  double target_moving_mass_ = 0.0;

  // ── Parameters ────────────────────────────────────────────
  double max_pitch_cmd_;
  bool   odom_watchdog_enabled_ = true;

  // ── PID controllers ───────────────────────────────────────
  PID speed_pid_;
  PID yaw_pid_;
  PID depth_pid_;
  PID pitch_pid_;

  // ── Timing ────────────────────────────────────────────────
  rclcpp::Time last_time_outer_;
  rclcpp::Time last_time_inner_;
  rclcpp::Time last_odom_time_;

  // ── Subscriptions ─────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      target_depth_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      target_heading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      target_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      target_moving_mass_sub_;

  // ── Publishers ────────────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   thruster_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   elevator_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   rudder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr moving_mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    arm_state_pub_;

  // ── Monitor publishers ────────────────────────────────────
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mon_target_depth_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mon_target_heading_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mon_target_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mon_target_mass_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mon_desired_pitch_pub_;

  // ── Service and Timers ────────────────────────────────────
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
  rclcpp::TimerBase::SharedPtr                       outer_timer_;
  rclcpp::TimerBase::SharedPtr                       inner_timer_;
  rclcpp::TimerBase::SharedPtr                       arm_state_timer_;
};

}  // namespace loki

#endif  // LOKI_CONTROL__CONTROLLER_HPP_