std_msgs/Header header

# Stream mode — CARTESIAN_MOTION_FORCE
# Pose: [x, y, z, qw, qx, qy, qz] m / unit quaternion
float64[7] cartesian_pose

# Wrench override: [fx, fy, fz, tx, ty, tz] N / Nm
# Applied on force-controlled axes only
float64[6] wrench
