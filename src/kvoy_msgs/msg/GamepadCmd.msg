# Gamepad velocity command
std_msgs/Header header
float32 vx       # forward velocity [m/s]
float32 vy       # lateral velocity [m/s]
float32 yaw_rate # yaw angular velocity [rad/s]

# State transition buttons
bool btn_standup  # trigger stand-up sequence
bool btn_liedown  # trigger lie-down sequence
bool btn_estop    # emergency stop
bool btn_policy_prev # switch to previous RL policy
bool btn_policy_next # switch to next RL policy

# Raw GameSir 7 Pro inputs
float32 axis_lx
float32 axis_ly
float32 axis_rx
float32 axis_ry
float32 axis_lt
float32 axis_rt
float32 axis_dpad_x
float32 axis_dpad_y

bool btn_a
bool btn_b
bool btn_x
bool btn_y
bool btn_lb
bool btn_rb
bool btn_lt
bool btn_rt
bool btn_dpad_up
bool btn_dpad_down
bool btn_dpad_left
bool btn_dpad_right
