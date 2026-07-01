builtin_interfaces/Time stamp
uint32 seq
# PWM microseconds. Range [1100, 1900], neutral 1500.
# [0] GPIO 39 — elevator (both fins)  [1] GPIO 40 — rudder (both fins)
# [2] unused                           [3] unused
# Thruster (VESC) is commanded separately over USB — not in this packet.
uint16[4] pwm
# stepper target position (1 rev = 2 mm, max 100 revs = 200 mm travel)
uint16 mass_target_revs
