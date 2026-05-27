# 12-joint motor command sent to MCU
std_msgs/Header header
float32[12] position   # target joint positions [rad]
float32[12] velocity   # target joint velocities [rad/s]
float32[12] torque     # feedforward torques [N*m]
