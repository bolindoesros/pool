#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <loki_msgs/msg/esp_packet.hpp>

// PWM microseconds (1100–1900) → VESC duty cycle (-1.0 to +1.0).
// 1500 µs = neutral, ±400 µs = full throw.
static double pwm_to_duty(int pwm)
{
    return std::clamp((pwm - 1500) / 400.0, -1.0, 1.0);
}

// Clamp to the valid PWM pulse-width window (microseconds).
static uint16_t clamp_pwm(int pwm)
{
    return static_cast<uint16_t>(std::max(1100, std::min(1900, pwm)));
}

// Full 20 cm travel = 100 revolutions (2 mm/rev), so revs = position_m * 500.
static uint16_t mass_to_revs(double mass_m)
{
    double revs = mass_m * 500.0;  // mass_m / 0.2 * 100
    return static_cast<uint16_t>(std::max(0.0, std::min(100.0, revs + 0.5)));
}

class HwBridge : public rclcpp::Node
{
public:
    HwBridge() : Node("hw_bridge")
    {
        auto qos = rclcpp::QoS(10).reliable();

        esp_pub_      = create_publisher<loki_msgs::msg::EspPacket>("/pc_to_esp_cmd", qos);
        duty_pub_     = create_publisher<std_msgs::msg::Float64>("vesc/commands/duty_cycle", qos);

        thruster_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/thruster", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                std_msgs::msg::Float64 d;
                d.data = pwm_to_duty(msg->data);
                duty_pub_->publish(d);
            });

        elevator_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/elevator", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { elevator_ = msg->data; });

        rudder_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/rudder", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) { rudder_ = msg->data; });

        moving_mass_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/cmd/moving_mass", qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { moving_mass_ = msg->data; });

        // Publish at 20 Hz
        timer_ = create_wall_timer(std::chrono::milliseconds(50),
                                   [this]() { publish(); });

        RCLCPP_INFO(get_logger(), "hw_bridge ready to publish commands");
    }

private:
    void publish()
    {
        loki_msgs::msg::EspPacket pkt;
        pkt.stamp  = this->now();
        pkt.seq    = seq_++;
        pkt.pwm[0] = clamp_pwm(elevator_);  // GPIO 39 — elevator (both fins)
        pkt.pwm[1] = clamp_pwm(rudder_);    // GPIO 40 — rudder (both fins)
        pkt.pwm[2] = 1500;                   // GPIO 41 — unused
        pkt.pwm[3] = 1500;                   // GPIO 42 — unused
        pkt.mass_target_revs = mass_to_revs(moving_mass_);
        esp_pub_->publish(pkt);
    }

    rclcpp::Publisher<loki_msgs::msg::EspPacket>::SharedPtr esp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    duty_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   thruster_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   elevator_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   rudder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr moving_mass_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int    elevator_    = 1500;
    int    rudder_      = 1500;
    double moving_mass_ = 0.0;
    uint32_t seq_       = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HwBridge>());
    rclcpp::shutdown();
    return 0;
}
