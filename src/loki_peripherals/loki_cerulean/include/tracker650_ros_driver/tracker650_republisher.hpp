#include "tracker650_ros_driver/tracker650_parser.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <iostream>
#include <sstream> 
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class Tracker650Republisher : public rclcpp::Node
{
public:
    Tracker650Republisher() : Node("tracker650_republisher")
    {
        RCLCPP_INFO(this->get_logger(), "Republisher Started!");
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("dvl/twist_stamped", 10);
        raw_sub_ = this->create_subscription<std_msgs::msg::String>(  "dvl/raw_data",
                                                                    10,
                                                                    // std::bind(&Tracker650Republisher::rawDvlCallback, this, std::placeholders::_1)
                                                                    // Lambda Function: [this] allows lambda to access class object
                                                                    [this](const std_msgs::msg::String::SharedPtr msg)
                                                                    {
                                                                        this->rawDvlCallBack(msg);
                                                                    });
    }

    void rawDvlCallBack(const std_msgs::msg::String::SharedPtr msg); 
    
private:
    Tracker650Parser parser_; 
    //Republisher will publish twist_msg_ and Subscribe to std::string given by receiver.cpp on /dvl/raw_data.
    //Create shared pointer to ROS2 Subscription and Publish Object.
    //Subscription Pointer listens for std_msgs::String messages
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr raw_sub_;
};