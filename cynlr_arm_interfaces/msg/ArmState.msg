std_msgs/Header header

# Joint state (7-DOF)
float64[7] joint_positions       # rad
float64[7] joint_velocities      # rad/s
float64[7] joint_torques         # Nm
float64[7] joint_torques_external  # Nm estimated external

# Cartesian TCP
float64[7] tcp_pose              # [x, y, z, qw, qx, qy, qz] m / unit quaternion
float64[6] tcp_velocity          # [vx, vy, vz, wx, wy, wz] m/s, rad/s

# Force/torque sensor
float64[6] ft_sensor_raw         # [fx, fy, fz, tx, ty, tz] N / Nm (raw)
float64[6] ext_wrench_in_tcp     # estimated external wrench in TCP frame
float64[6] ext_wrench_in_world   # estimated external wrench in world frame

# Status flags
bool fault
bool operational
bool estopped
