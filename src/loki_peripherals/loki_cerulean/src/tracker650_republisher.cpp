#include <memory>
#include <string>
#include "tracker650_ros_driver/tracker650_republisher.hpp"
#include "tracker650_ros_driver/tracker650_parser.hpp"

void Tracker650Republisher::rawDvlCallBack(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.rfind("$DVPDX", 0) != 0) return;

  Tracker650Parser::DvlVelocity velocity;

  try {
    if (!parser_.parseDVPDX(msg->data, velocity))
      return;  // confidence=0 or other normal discard — parser already logged
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "DVPDX parse exception: %s | packet: %s",
                e.what(), msg->data.c_str());
    return;
  }

  geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
  twist_msg.header.stamp    = this->now();
  twist_msg.header.frame_id = "dvl_link";
  twist_msg.twist.twist.linear.x = velocity.vx;
  twist_msg.twist.twist.linear.y = velocity.vy;
  twist_msg.twist.twist.linear.z = velocity.vz;

  twist_msg.twist.covariance[0]  = 0.01;
  twist_msg.twist.covariance[7]  = 0.01;
  twist_msg.twist.covariance[14] = 0.01;
  twist_msg.twist.covariance[21] = 999999.0;
  twist_msg.twist.covariance[28] = 999999.0;
  twist_msg.twist.covariance[35] = 999999.0;

  twist_pub_->publish(twist_msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tracker650Republisher>());
  rclcpp::shutdown();
  return 0;
}
