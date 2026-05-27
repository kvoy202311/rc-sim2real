# 12-joint motor state received from MCU
std_msgs/Header header
float32[12] position   # current joint positions [rad]
float32[12] velocity   # current joint velocities [rad/s]
float32[12] torque     # current joint torques [N*m]
