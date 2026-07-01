#include "ahrs_orientation/ahrs_orientation.hpp"

void ahrs_orientation::rawSensorCallBack(const auv_interfaces::msg::EspRawSensor::SharedPtr msg)
{
 sensor_msgs::msg::Imu raw_imu;
  sensor_msgs::msg::MagneticField raw_mag;
  sensor_msgs::msg::FluidPressure pressure_msg;

  const auto stamp = this->now();

  raw_imu.header.stamp = stamp;
  raw_mag.header.stamp = stamp;
  pressure_msg.header.stamp = stamp;

  raw_imu.header.frame_id = "imu_link";
  raw_mag.header.frame_id = "mag_link";
  pressure_msg.header.frame_id = "pressure_link";

  raw_imu.orientation_covariance[0] = -1.0;

  raw_imu.linear_acceleration.x = msg->ax;
  raw_imu.linear_acceleration.y = msg->ay;
  raw_imu.linear_acceleration.z = msg->az;

  raw_imu.angular_velocity.x = msg->gx;
  raw_imu.angular_velocity.y = msg->gy;
  raw_imu.angular_velocity.z = msg->gz;

  raw_mag.magnetic_field.x = msg->mx;
  raw_mag.magnetic_field.y = msg->my;
  raw_mag.magnetic_field.z = msg->mz;

  pressure_msg.fluid_pressure = msg->pressure_pa;
  pressure_msg.variance = 0.0;  // placeholder for now

  raw_imu.orientation_covariance[0] = -1.0;

  raw_imu.angular_velocity_covariance[0] = 0.001;
  raw_imu.angular_velocity_covariance[4] = 0.001;
  raw_imu.angular_velocity_covariance[8] = 0.001;

  raw_imu.linear_acceleration_covariance[0] = 0.05;
  raw_imu.linear_acceleration_covariance[4] = 0.05;
  raw_imu.linear_acceleration_covariance[8] = 0.05;

  imu_pub_->publish(raw_imu);
  mag_pub_->publish(raw_mag);
  pressure_pub_->publish(pressure_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ahrs_orientation>());

  rclcpp::shutdown();
  return 0;
}