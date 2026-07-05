/*
 * @file controller.cpp
 * @brief PID controller for loki auv.
 */

#include "loki_control/controller.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace loki
{

// Timer periods
static constexpr int    OUTER_LOOP_MS    = 40;   // 25Hz — depth PID (outer cascade)
static constexpr int    INNER_LOOP_MS    = 10;   // 100Hz — pitch, speed, yaw PIDs (inner cascade)
static constexpr double MAX_MOVING_MASS  = 0.2;  // m — physical travel limit of moving mass stepper (20 cm)
static constexpr double ODOM_TIMEOUT_S   = 0.5;  // s — odometry older than this is considered stale

ControllerNode::ControllerNode()
: Node("loki_controller")
{
  max_pitch_cmd_          = declare_parameter("max_pitch_cmd", 20.0);
  double alpha            = declare_parameter("alpha",          0.7);
  odom_watchdog_enabled_  = declare_parameter("odom_watchdog_enabled", true);

  // load PID params
  speed_pid_.update_params(load_pid("speed", alpha));
  yaw_pid_.update_params(load_pid("yaw",     alpha));
  depth_pid_.update_params(load_pid("depth", alpha));
  pitch_pid_.update_params(load_pid("pitch", alpha));

  auto qos = rclcpp::QoS(10).reliable();

  // ── Subscriptions ──────────────────────────────────────────
  odom_sub_               = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered", qos, std::bind(&ControllerNode::on_odometry, this, std::placeholders::_1));
  target_depth_sub_       = create_subscription<std_msgs::msg::Float64>("/target/depth", qos, std::bind(&ControllerNode::on_target_depth, this, std::placeholders::_1));
  target_heading_sub_     = create_subscription<std_msgs::msg::Float64>("/target/heading", qos, std::bind(&ControllerNode::on_target_heading, this, std::placeholders::_1));
  target_speed_sub_       = create_subscription<std_msgs::msg::Float64>("/target/speed", qos, std::bind(&ControllerNode::on_target_speed, this, std::placeholders::_1));
  target_moving_mass_sub_ = create_subscription<std_msgs::msg::Float64>("/target/moving_mass", qos, std::bind(&ControllerNode::on_target_moving_mass, this, std::placeholders::_1));

  // ── Actuator publishers ────────────────────────────────────
  thruster_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/thruster",     qos);
  elevator_pub_    = create_publisher<std_msgs::msg::Int32>  ("/cmd/elevator",     qos);
  rudder_pub_      = create_publisher<std_msgs::msg::Int32>  ("/cmd/rudder",       qos);
  moving_mass_pub_ = create_publisher<std_msgs::msg::Float64>("/cmd/moving_mass",  qos);
  arm_state_pub_   = create_publisher<std_msgs::msg::Bool>   ("/system/arm_state", qos);

  // ── Monitor publishers ─────────────────────────────────────
  mon_target_depth_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/depth",       qos);
  mon_target_heading_pub_ = create_publisher<std_msgs::msg::Float64>("/monitor/target/heading",     qos);
  mon_target_speed_pub_   = create_publisher<std_msgs::msg::Float64>("/monitor/target/speed",       qos);
  mon_target_mass_pub_    = create_publisher<std_msgs::msg::Float64>("/monitor/target/moving_mass", qos);
  mon_desired_pitch_pub_  = create_publisher<std_msgs::msg::Float64>("/monitor/desired_pitch",      qos);

  // ── Arm service ────────────────────────────────────────────
  arm_srv_ = create_service<std_srvs::srv::SetBool>("/system/arm",
    std::bind(&ControllerNode::on_arm, this, std::placeholders::_1, std::placeholders::_2));

  // ── Timers ─────────────────────────────────────────────────
  outer_timer_ = create_wall_timer(
    std::chrono::milliseconds(OUTER_LOOP_MS),
    std::bind(&ControllerNode::outer_loop, this));

  inner_timer_ = create_wall_timer(
    std::chrono::milliseconds(INNER_LOOP_MS),
    std::bind(&ControllerNode::inner_loop, this));

  // Republish arm state at 1 Hz so monitor always has current value
  arm_state_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    [this]() {
      auto msg = std_msgs::msg::Bool();
      msg.data = is_armed_;
      arm_state_pub_->publish(msg);
    });

  last_time_outer_ = now();
  last_time_inner_ = now();
  last_odom_time_  = now();
  RCLCPP_INFO(get_logger(), "Loki controller ready!");
}

// subscriptions callbacks
void ControllerNode::on_target_depth(const std_msgs::msg::Float64::SharedPtr msg){
  target_depth_ = msg->data;
}

void ControllerNode::on_target_heading(const std_msgs::msg::Float64::SharedPtr msg){
  target_heading_ = msg->data;
}

void ControllerNode::on_target_speed(const std_msgs::msg::Float64::SharedPtr msg){
  target_speed_    = msg->data;
  speed_unlocked_  = (msg->data != 0.0);
  if (!speed_unlocked_) speed_pid_.reset_controller();
}

void ControllerNode::on_target_moving_mass(const std_msgs::msg::Float64::SharedPtr msg){
  // Clamp to [0, MAX_MOVING_MASS] (physical limits)
  target_moving_mass_ = std::clamp(msg->data, 0.0, MAX_MOVING_MASS);
}

// arm service callback
void ControllerNode::on_arm(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  const std_srvs::srv::SetBool::Response::SharedPtr res)
{
  is_armed_ = req->data;

  if (is_armed_) {
    target_heading_     = current_heading_;
    target_depth_       = current_depth_;
    target_speed_       = 0.0;
    target_moving_mass_ = 0.0;
    speed_unlocked_     = false;

    speed_pid_.reset_controller();
    yaw_pid_.reset_controller();
    depth_pid_.reset_controller();
    pitch_pid_.reset_controller();

    res->message = "Armed";
    RCLCPP_INFO(get_logger(), "ARMED");
  } else {
    auto pwm = std_msgs::msg::Int32();
    pwm.data  = 1500;
    thruster_pub_->publish(pwm);
    elevator_pub_->publish(pwm);
    rudder_pub_->publish(pwm);

    auto mm = std_msgs::msg::Float64();
    mm.data  = 0.0;
    moving_mass_pub_->publish(mm);

    speed_pid_.reset_controller();
    yaw_pid_.reset_controller();
    depth_pid_.reset_controller();
    pitch_pid_.reset_controller();

    res->message = "Disarmed";
    RCLCPP_INFO(get_logger(), "DISARMED");
  }

  res->success = true;
  auto msg  = std_msgs::msg::Bool();
  msg.data  = is_armed_;
  arm_state_pub_->publish(msg);
}

void ControllerNode::on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_time_ = now();

  current_depth_ = msg->pose.pose.position.z;
  current_speed_ = msg->twist.twist.linear.x;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  current_pitch_      = pitch * 180.0 / M_PI;
  current_heading_    = yaw   * 180.0 / M_PI;
  current_pitch_rate_ = msg->twist.twist.angular.y * 180.0 / M_PI;  // derivative damping in pitch loop

  // Normalise to [0, 360] — wrap_angle() handles the error wrapping in the yaw loop.
  if (current_heading_ < 0.0) current_heading_ += 360.0;
}

// outer loop runs at 25Hz, computes depth PID (outer cascade) and updates desired pitch
void ControllerNode::outer_loop()
{
  auto   t  = now();
  double dt = std::clamp((t - last_time_outer_).seconds(), 1e-4, 0.1);
  last_time_outer_ = t;

  // Monitoring
  publish_f64(mon_target_depth_pub_,   target_depth_);
  publish_f64(mon_target_heading_pub_, target_heading_);
  publish_f64(mon_target_speed_pub_,   target_speed_);
  publish_f64(mon_target_mass_pub_,    target_moving_mass_);

  // Odom watchdog
  double odom_age = (now() - last_odom_time_).seconds();
  if (odom_watchdog_enabled_ && odom_age > ODOM_TIMEOUT_S) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
      "No odometry for %.2fs — suppressing control output", odom_age);
    if (is_armed_) return;
  }

  if (!is_armed_) return;  // inner loop handles neutral outputs

  // ── 3. Depth loop (outer cascade) ──────────────────────────
  double depth_err = current_depth_ - target_depth_;
  desired_pitch_   = depth_pid_.compute_control(dt, depth_err);
  desired_pitch_   = std::clamp(desired_pitch_, -max_pitch_cmd_, max_pitch_cmd_);
  publish_f64(mon_desired_pitch_pub_, desired_pitch_);  // publish for cascade tuning
}

// inner loop runs at 100Hz, computes speed, yaw, and pitch PIDs (inner cascade) and updates actuator commands
void ControllerNode::inner_loop()
{
  auto   t  = now();
  double dt = std::clamp((t - last_time_inner_).seconds(), 1e-4, 0.05);
  last_time_inner_ = t;

  if (!is_armed_) return;

  // Odom watchdog: without fresh odometry, desired_pitch_/current_pitch_ are frozen
  double odom_age = (now() - last_odom_time_).seconds();
  if (odom_watchdog_enabled_ && odom_age > ODOM_TIMEOUT_S) {
    auto t_msg = std_msgs::msg::Int32(); t_msg.data = 1500;
    auto e_msg = std_msgs::msg::Int32(); e_msg.data = 1500;
    auto r_msg = std_msgs::msg::Int32(); r_msg.data = 1500;
    thruster_pub_->publish(t_msg);
    elevator_pub_->publish(e_msg);
    rudder_pub_->publish(r_msg);

    speed_pid_.reset_controller();
    yaw_pid_.reset_controller();
    pitch_pid_.reset_controller();
    return;
  }

  // Speed loop 
  // Thruster is locked at neutral until a non-zero speed is explicitly published.
  double speed_effort = 0.0;
  if (speed_unlocked_) {
    speed_effort = speed_pid_.compute_control(dt, target_speed_ - current_speed_);
  }

  //  Yaw loop 
  double yaw_effort = yaw_pid_.compute_control(dt, wrap_angle(target_heading_ - current_heading_));

  // Pitch loop (inner cascade) 
  double pitch_effort = pitch_pid_.compute_control(
      dt, desired_pitch_ - current_pitch_, -current_pitch_rate_);

  // Actuator outputs 
  auto mm_msg = std_msgs::msg::Float64();
  mm_msg.data = target_moving_mass_;
  moving_mass_pub_->publish(mm_msg);

  auto t_msg = std_msgs::msg::Int32(); t_msg.data = effort_to_pwm(speed_effort);  // thruster
  auto e_msg = std_msgs::msg::Int32(); e_msg.data = effort_to_pwm(pitch_effort);  // elevator
  auto r_msg = std_msgs::msg::Int32(); r_msg.data = effort_to_pwm(yaw_effort);    // rudder

  thruster_pub_->publish(t_msg);
  elevator_pub_->publish(e_msg);
  rudder_pub_->publish(r_msg);
}

PIDParams ControllerNode::load_pid(const std::string & ns, double alpha)
{
  PIDParams p;
  p.Kp_gains                = declare_parameter(ns + ".kp",         0.0);
  p.Ki_gains                = declare_parameter(ns + ".ki",         0.0);
  p.Kd_gains                = declare_parameter(ns + ".kd",         0.0);
  p.alpha                   = alpha;
  p.antiwindup_cte          = declare_parameter(ns + ".antiwindup", 0.0);
  p.upper_output_saturation = declare_parameter(ns + ".output_max", 400.0);
  p.lower_output_saturation = declare_parameter(ns + ".output_min", -400.0);
  return p;
}

void ControllerNode::publish_f64(rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double value)
{
  std_msgs::msg::Float64 msg;
  msg.data = value;
  pub->publish(msg);
}

int ControllerNode::effort_to_pwm(double effort)
{
  return std::clamp(1500 + static_cast<int>(effort), 1100, 1900);
}

double ControllerNode::wrap_angle(double deg)
{
  double wrapped = std::fmod(deg + 180.0, 360.0);
  if (wrapped < 0.0) wrapped += 360.0;
  return wrapped - 180.0;
}

}  // namespace loki