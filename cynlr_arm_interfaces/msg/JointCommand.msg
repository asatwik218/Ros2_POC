std_msgs/Header header

# Stream mode — determines which fields are used
# 0 = JOINT_POSITION, 1 = JOINT_TORQUE
uint8 mode

float64[7] joint_position        # rad
float64[7] joint_velocity        # rad/s (feedforward)
float64[7] joint_acceleration    # rad/s^2 (feedforward)
float64[7] joint_torque          # Nm (for JOINT_TORQUE mode)
