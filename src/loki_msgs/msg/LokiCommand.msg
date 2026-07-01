# LokiCommand.msg
# target settings

std_msgs/Header header
float64 target_depth        # positive number (m)
float64 target_speed        # 0.0 - 1.5 (m/s)
float64 target_moving_mass  # 0.0 - 0.4 (m), clamped by controller
