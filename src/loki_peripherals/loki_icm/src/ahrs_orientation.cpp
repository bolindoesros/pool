#include "ahrs_orientation/ahrs_orientation.hpp"

void ahrs_orientation::rawSensorCallBack(const auv_interfaces::msg::EspRawSensor::SharedPtr msg)
{
  // experiment code BTW : gyro bias auto-cal 
  if (calib_samples_ < CALIB_SAMPLES) {
    gx_sum_ += msg->gx;
    gy_sum_ += msg->gy;
    gz_sum_ += msg->gz;
    if (++calib_samples_ == CALIB_SAMPLES) { // averaging
      gx_bias_ = gx_sum_ / CALIB_SAMPLES;
      gy_bias_ = gy_sum_ / CALIB_SAMPLES;
      gz_bias_ = gz_sum_ / CALIB_SAMPLES;
      RCLCPP_INFO(this->get_logger(),
        "Gyro bias calibrated — gx: %.4f  gy: %.4f  gz: %.4f", gx_bias_, gy_bias_, gz_bias_);
    }
    return;  // hold off publishing until bias is known
  }

  sensor_msgs::msg::Imu raw_imu;
  sensor_msgs::msg::MagneticField raw_mag;

  const auto stamp = this->now();

  raw_imu.header.stamp    = stamp;
  raw_mag.header.stamp    = stamp;
  raw_imu.header.frame_id = "imu_link";
  raw_mag.header.frame_id = "mag_link";

  raw_imu.linear_acceleration.x = msg->ax;
  raw_imu.linear_acceleration.y = msg->ay;
  raw_imu.linear_acceleration.z = msg->az;

  // EXPERIMENTAL: subtract gyro bias computed at startup
  raw_imu.angular_velocity.x = msg->gx - gx_bias_;
  raw_imu.angular_velocity.y = msg->gy - gy_bias_;
  raw_imu.angular_velocity.z = msg->gz - gz_bias_;

  raw_mag.magnetic_field.x = msg->mx;
  raw_mag.magnetic_field.y = msg->my;
  raw_mag.magnetic_field.z = msg->mz;

  raw_imu.orientation_covariance[0] = -1.0;

  raw_imu.angular_velocity_covariance[0] = 0.001;
  raw_imu.angular_velocity_covariance[4] = 0.001;
  raw_imu.angular_velocity_covariance[8] = 0.001;

  raw_imu.linear_acceleration_covariance[0] = 0.05;
  raw_imu.linear_acceleration_covariance[4] = 0.05;
  raw_imu.linear_acceleration_covariance[8] = 0.05;

  imu_pub_->publish(raw_imu);
  mag_pub_->publish(raw_mag);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ahrs_orientation>());
  rclcpp::shutdown();
  return 0;
}
