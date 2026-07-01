#include <memory>
#include <string>
#include "tracker650_ros_driver/tracker650_receiver.hpp"

bool Tracker650Receiver::UDPInitialize()
{
    sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
        return false;
    }

    // 200 ms receive timeout so the loop can check rclcpp::ok() and exit cleanly
    struct timeval tv{};
    tv.tv_sec  = 0;
    tv.tv_usec = TIMEOUT_MS * 1000;
    setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    server_addr_.sin_family      = AF_INET;
    server_addr_.sin_addr.s_addr = INADDR_ANY;
    server_addr_.sin_port        = htons(UDP_PORT);

    if (bind(sockfd_, reinterpret_cast<sockaddr *>(&server_addr_), sizeof(server_addr_)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket on port %d", UDP_PORT);
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "Listening on UDP port %d", UDP_PORT);
    return true;
}

void Tracker650Receiver::receiveLoop()
{
    if (!UDPInitialize()) return;

    while (rclcpp::ok()) {
        std::memset(buffer_, 0, BUFFER_SIZE);
        ssize_t n = recvfrom(sockfd_, buffer_, BUFFER_SIZE - 1, 0, nullptr, nullptr);

        if (n < 0) {
            // EAGAIN / EWOULDBLOCK = timeout, loop back and check rclcpp::ok()
            continue;
        }

        std::string packet(buffer_, n);
        std::stringstream ss(packet);
        std::string line;

        while (std::getline(ss, line)) {
            // strip carriage return from CRLF line endings
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.empty()) continue;
            publishLine(line);
        }
    }
}

void Tracker650Receiver::publishLine(const std::string & line)
{
    std_msgs::msg::String msg;
    msg.data = line;
    raw_pub_->publish(msg);
    RCLCPP_DEBUG(this->get_logger(), "Published: %s", line.c_str());
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker650Receiver>());
    rclcpp::shutdown();
    return 0;
}
