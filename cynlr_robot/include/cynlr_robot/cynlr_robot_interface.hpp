#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/capabilities/digital_io_controllable.hpp"
#include "cynlr_arm_core/types.hpp"
#include "cynlr_robot/cynlr_arm_handle.hpp"

namespace cynlr_robot {

enum class ActiveMode { NONE, POSITION, VELOCITY, EFFORT, CARTESIAN };

class CynlrRobotInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(CynlrRobotInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type prepare_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string>& start_interfaces,
        const std::vector<std::string>& stop_interfaces) override;

    hardware_interface::return_type read(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    hardware_interface::return_type write(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    static constexpr int kDOF      = 7;
    static constexpr int kIOPorts  = 18;
    static constexpr int kCartSize = 13; // 7 pose + 6 wrench

    // The vendor-agnostic arm object — created from URDF params, never ROS-aware
    std::unique_ptr<cynlr::arm::ArmInterface> arm_;
    cynlr::arm::ArmConfig arm_config_;

    // Handle registered in CynlrArmRegistry on activate so CynlrArmNode can access it
    std::shared_ptr<CynlrArmHandle> arm_handle_;

    // Tool info loaded from URDF hardware params; applied in on_activate() after enable
    cynlr::arm::ToolInfo tool_info_{};

    // --- Standard joint state mirrors (written by read(), read by controllers) ---
    std::array<double, kDOF> hw_pos_{};
    std::array<double, kDOF> hw_vel_{};
    std::array<double, kDOF> hw_eff_{};

    // --- Joint command mirrors (written by controllers, read by write()) ---
    std::array<double, kDOF> cmd_pos_{};
    std::array<double, kDOF> cmd_vel_{};
    std::array<double, kDOF> cmd_eff_{};

    // Full ArmState struct — kept up-to-date in read(); individual fields are
    // exposed as standard double state interfaces for JTC/direct/cartesian controllers
    cynlr::arm::ArmState arm_state_{};

    // --- Cartesian command mirrors (escape hatch #3) ---
    // Indices 0-6: pose [x,y,z,qw,qx,qy,qz], indices 7-12: wrench [fx,fy,fz,tx,ty,tz]
    std::array<double, kCartSize> cmd_cart_{};

    // --- GPIO mirrors ---
    std::array<double, kIOPorts> gpio_in_{};
    std::array<double, kIOPorts> gpio_out_{};
    std::array<double, kIOPorts> last_gpio_out_{}; // diff tracking to avoid redundant writes

    // --- Mode switching state ---
    ActiveMode active_mode_{ActiveMode::NONE};
    // Pending transitions set by prepare, applied by perform
    ActiveMode pending_start_{ActiveMode::NONE};
    bool       pending_stop_{false};
    // Tracks NRT active flag edge so cmd_pos_/cmd_cart_/cmd_eff_ can be reseeded
    bool       prev_nrt_active_{false};
    // The RT StreamMode corresponding to active_mode_ — passed to the arm so it
    // can restore the correct mode after NRT motions complete.
    cynlr::arm::StreamMode intended_rt_mode_{cynlr::arm::StreamMode::JOINT_POSITION};

    std::string prefix_; // e.g. "arm_left_"

    static rclcpp::Logger getLogger();
};

} // namespace cynlr_robot
