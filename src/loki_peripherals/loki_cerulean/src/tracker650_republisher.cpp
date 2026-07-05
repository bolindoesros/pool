#include <memory>
#include <string>
#include "tracker650_ros_driver/tracker650_republisher.hpp"
#include "tracker650_ros_driver/tracker650_parser.hpp"

void Tracker650Republisher::rawDvlCallBack(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data.rfind("$DVPDX", 0) != 0) return;

  Tracker650Parser::DvlVelocity velocity;
  const auto result = parser_.parseDVPDX(msg->data, velocity);

  if (result != Tracker650Parser::Result::OK) {
    if (result == Tracker650Parser::Result::ZERO_CONFIDENCE) {
      // Normal when out of water or too close to the bottom — not a code error.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "DVL not publishing: confidence=0");
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "DVL sentence rejected (%s): %s",
        Tracker650Parser::resultName(result), msg->data.c_str());
    }
    return;
  }

  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
    "DVL ok: vx=%.3f vy=%.3f vz=%.3f conf=%d",
    velocity.vx, velocity.vy, velocity.vz, velocity.confidence);

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
