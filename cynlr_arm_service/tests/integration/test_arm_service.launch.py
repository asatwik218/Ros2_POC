"""Launch file for arm_service integration tests.

Starts arm_service_node with SimArm, configures/activates it, then runs the C++
test binary. Launch shuts down when the C++ tests exit. Post-shutdown test checks
the exit code. Invoked by: colcon test --packages-select cynlr_arm_service
"""
import os
import unittest

import launch
import launch.actions
import launch.event_handlers
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    pkg_share = get_package_share_directory("cynlr_arm_service")
    pkg_prefix = get_package_prefix("cynlr_arm_service")
    sim_config = os.path.join(pkg_share, "config", "sim.yaml")
    test_bin = os.path.join(pkg_prefix, "lib", "cynlr_arm_service",
                            "test_arm_service_integration")

    arm_node = launch_ros.actions.LifecycleNode(
        package="cynlr_arm_service",
        executable="arm_service_node",
        name="arm_service_node",
        namespace="test_arm",
        parameters=[sim_config],
        output="screen",
    )

    cpp_tests = launch.actions.ExecuteProcess(
        cmd=[test_bin],
        name="cpp_integration_tests",
        output="screen",
    )

    return launch.LaunchDescription([
        arm_node,
        # Allow node to start before lifecycle transitions
        launch.actions.TimerAction(period=2.0, actions=[
            launch.actions.ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/test_arm/arm_service_node", "configure"],
                output="screen",
            ),
        ]),
        launch.actions.TimerAction(period=4.0, actions=[
            launch.actions.ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/test_arm/arm_service_node", "activate"],
                output="screen",
            ),
        ]),
        # Run C++ tests after node is active
        launch.actions.TimerAction(period=7.0, actions=[cpp_tests]),
        launch_testing.actions.ReadyToTest(),
    ]), {"arm_node": arm_node, "cpp_tests": cpp_tests}


class TestArmServiceReady(unittest.TestCase):
    def test_integration_completes(self, proc_output, cpp_tests):
        """Block until the C++ test binary finishes (keeps launch alive via @keep_alive)."""
        # GTest always ends with this line — wait up to 90 seconds
        proc_output.assertWaitFor(
            expected_output="test suite ran",
            timeout=90,
            stream="stdout",
        )


@launch_testing.post_shutdown_test()
class TestIntegrationResults(unittest.TestCase):
    def test_cpp_tests_passed(self, proc_info, cpp_tests):
        """All 5 C++ integration tests must exit 0."""
        launch_testing.asserts.assertExitCodes(proc_info, process=cpp_tests)
