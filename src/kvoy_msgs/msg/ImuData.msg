# IMU data from Wit-Motion sensor
std_msgs/Header header
geometry_msgs/Quaternion orientation      # orientation quaternion (x,y,z,w)
geometry_msgs/Vector3 angular_velocity    # angular velocity [rad/s]
geometry_msgs/Vector3 linear_acceleration # accelerometer output, includes gravity [m/s^2]
