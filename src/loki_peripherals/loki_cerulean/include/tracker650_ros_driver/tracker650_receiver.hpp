#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

class Tracker650Receiver : public rclcpp::Node
{
public:
    Tracker650Receiver() : Node("tracker650_receiver")
    {
        RCLCPP_INFO(this->get_logger(), "Receiver started");
        raw_pub_ = this->create_publisher<std_msgs::msg::String>("dvl/raw_data", 10);
        recv_thread_ = std::thread(&Tracker650Receiver::receiveLoop, this);
    }

    ~Tracker650Receiver()
    {
        if (sockfd_ >= 0) close(sockfd_);
        if (recv_thread_.joinable()) recv_thread_.join();
    }

private:
    bool UDPInitialize();
    void receiveLoop();
    void publishLine(const std::string & line);

    static constexpr int UDP_PORT   = 27000;
    static constexpr int BUFFER_SIZE = 2048;
    static constexpr int TIMEOUT_MS  = 200;

    int         sockfd_ = -1;
    sockaddr_in server_addr_{};
    char        buffer_[BUFFER_SIZE];

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_pub_;
    std::thread recv_thread_;
};