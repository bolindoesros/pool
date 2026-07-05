#include <cmath>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include <auv_interfaces/msg/esp_raw_sensor.hpp>

class ahrs_orientation : public rclcpp::Node
{
public:
    ahrs_orientation() : Node("ahrs_orientation_node")
    {
        RCLCPP_INFO(this->get_logger(), "ahrs_orientation_node started!");

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
        mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

        raw_sensor_sub_ = this->create_subscription<auv_interfaces::msg::EspRawSensor>("esp/raw_sensor", 10,
                                                                    [this](const auv_interfaces::msg::EspRawSensor::SharedPtr msg)
                                                                    {
                                                                        this->rawSensorCallBack(msg);
                                                                    });
    }
private:
    void rawSensorCallBack(const auv_interfaces::msg::EspRawSensor::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

    rclcpp::Subscription<auv_interfaces::msg::EspRawSensor>::SharedPtr raw_sensor_sub_;
    
};