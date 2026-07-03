/**
 * @file hw_bridge.cpp
 * @brief Bridge from the /cmd topics to the actuators.
 *
 * Sub: /cmd/thruster (PWM µs), /cmd/elevator, /cmd/rudder, /cmd/moving_mass,
 *      /system/arm_state
 * Pub: /vesc/commands/duty_cycle (thruster), /pc_to_esp_cmd (fins + mass, 20 Hz)
 *
 * Disarmed (the default): fins neutral, mass zero, thruster duty 0.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
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

        arm_state_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/system/arm_state", qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data != armed_) {
                    RCLCPP_INFO(get_logger(), "[hw_bridge debug] arm_state changed: %s -> %s",
                        armed_ ? "true" : "false", msg->data ? "true" : "false");
                }
                armed_ = msg->data;
            });

        thruster_sub_ = create_subscription<std_msgs::msg::Int32>(
            "/cmd/thruster", qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                std_msgs::msg::Float64 d;
                d.data = armed_ ? pwm_to_duty(msg->data) : 0.0;
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
        int elevator = armed_ ? elevator_ : 1500;
        int rudder   = armed_ ? rudder_   : 1500;
        pkt.pwm[0] = clamp_pwm(elevator);                 // GPIO 39 right elevator
        pkt.pwm[1] = clamp_pwm(elevator);                 // GPIO 40 left elevator
        pkt.pwm[2] = clamp_pwm(rudder);                   // GPIO 41 top rudder
        pkt.pwm[3] = clamp_pwm(rudder);                   // GPIO 42 bot rudder
        pkt.mass_target_revs = mass_to_revs(armed_ ? moving_mass_ : 0.0);
        esp_pub_->publish(pkt);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[hw_bridge debug] armed_=%s  elevator_(raw in)=%d  rudder_(raw in)=%d  -> pwm=[%d,%d,%d,%d]",
            armed_ ? "true" : "false", elevator_, rudder_,
            pkt.pwm[0], pkt.pwm[1], pkt.pwm[2], pkt.pwm[3]);
    }

    rclcpp::Publisher<loki_msgs::msg::EspPacket>::SharedPtr esp_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    duty_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    arm_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   thruster_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   elevator_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr   rudder_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr moving_mass_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool   armed_       = false;  // safe default until /system/arm_state is heard
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
