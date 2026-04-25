#pragma once
#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace cynlr::arm {

struct ArmConfig {
    std::string vendor;         // "flexiv", "sim"
    std::string serial_number;
    std::string ip_address;
    int num_joints{7};
    std::unordered_map<std::string, std::string> params;
};

struct ArmState {
    std::array<double, 7> joint_positions{};        // rad
    std::array<double, 7> joint_velocities{};       // rad/s
    std::array<double, 7> joint_torques{};          // Nm
    std::array<double, 7> joint_torques_external{}; // Nm
    std::array<double, 7> tcp_pose{};               // [x,y,z,qw,qx,qy,qz] m/quat
    std::array<double, 6> tcp_velocity{};           // [vx,vy,vz,wx,wy,wz] m/s, rad/s
    std::array<double, 6> ft_sensor_raw{};          // [fx,fy,fz,tx,ty,tz] N/Nm
    std::array<double, 6> ext_wrench_in_tcp{};      // N/Nm
    std::array<double, 6> ext_wrench_in_world{};    // N/Nm
    bool fault{false};
    bool operational{false};
    bool estopped{false};
};

struct CartesianTarget {
    std::array<double, 7> pose{};  // [x,y,z,qw,qx,qy,qz] m/quat
};

struct JointTarget {
    std::array<double, 7> positions{};  // rad
};

struct MotionParams {
    double max_linear_vel{0.1};   // m/s
    double max_angular_vel{0.5};  // rad/s
    double max_linear_acc{1.0};   // m/s^2
    double max_joint_vel{1.0};    // rad/s
    double max_joint_acc{2.0};    // rad/s^2
};

struct ToolInfo {
    double mass_kg{0.0};
    std::array<double, 3> com{};       // centre of mass [x,y,z] m
    std::array<double, 6> inertia{};   // kg*m^2
    std::array<double, 7> tcp_pose{};  // [x,y,z,qw,qx,qy,qz]
};

enum class StreamMode {
    JOINT_POSITION,
    JOINT_TORQUE,
    CARTESIAN_MOTION_FORCE,
};

struct StreamCommand {
    StreamMode mode{StreamMode::JOINT_POSITION};
    std::array<double, 7> joint_position{};
    std::array<double, 7> joint_velocity{};
    std::array<double, 7> joint_acceleration{};
    std::array<double, 7> joint_torque{};
    std::array<double, 7> cartesian_pose{};  // [x,y,z,qw,qx,qy,qz]
    std::array<double, 6> wrench{};          // [fx,fy,fz,tx,ty,tz] N/Nm
};

// --- Capability-specific types ---

struct ForceAxisConfig {
    std::array<bool, 6> enabled{};  // [fx,fy,fz,tx,ty,tz] — true = force control on that axis
};

struct CartesianPose {
    std::array<double, 7> pose{};  // [x,y,z,qw,qx,qy,qz]
};

struct ImpedanceParams {
    std::array<double, 6> stiffness{};  // [x,y,z,rx,ry,rz]
    std::array<double, 6> damping{};
};

struct JointImpedanceParams {
    std::array<double, 7> stiffness{};
    std::array<double, 7> damping{};
};

struct JointState {
    std::array<double, 7> positions{};  // rad
};

struct NullSpaceWeights {
    std::array<double, 7> weights{};
};

} // namespace cynlr::arm
