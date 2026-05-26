"""Bring up the RoverMind nodes: safety controller + emergency braking gate.

LIMO base drivers (`limo_bringup`) and the agent script (`scripts/run_agent.py`)
are launched separately -- see README. This launch only covers the two nodes
in this repo so iteration on them doesn't re-init the slow vendor stack.

Usage:
    ros2 launch safety_controller_layer rovermind.launch.py
    ros2 launch safety_controller_layer rovermind.launch.py use_aeb:=false
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "safety_controller_layer"


def generate_launch_description() -> LaunchDescription:
    use_aeb = LaunchConfiguration("use_aeb")

    controller_node = Node(
        package=PACKAGE,
        executable="safety_controller_node",
        name="safety_controller",
        output="screen",
        emulate_tty=True,
        respawn=False,
    )
    aeb_node = Node(
        package=PACKAGE,
        executable="aeb_node",
        name="emergency_brake",
        output="screen",
        emulate_tty=True,
        respawn=False,
        condition=IfCondition(use_aeb),
    )

    # Any node death takes the whole launch down. Without this, a crashed AEB
    # leaves the controller publishing /cmd_vel_raw with nothing republishing
    # to /cmd_vel (rover silently stops); a crashed controller leaves AEB
    # alive on its zero-Twist fail-safe with no top-level signal. We want a
    # loud failure either way. OnProcessExit only fires if the target action
    # actually ran, so gating aeb_node via IfCondition is enough -- the
    # symmetric handler is harmless when use_aeb:=false.
    #
    # Narrow gap: ros2 launch executes top-level entities sequentially, so if
    # a node exits in the window between its launch and the RegisterEventHandler
    # below being processed, the ProcessExited event fires before the handler
    # is registered and the Shutdown never triggers. The window is sub-second
    # and would imply a node that crashes instantly on start, which is loud
    # enough to notice without this handler.
    controller_died = RegisterEventHandler(OnProcessExit(
        target_action=controller_node,
        on_exit=[
            LogInfo(msg="safety_controller_node exited -- shutting down launch."),
            Shutdown(reason="safety_controller_node exited"),
        ],
    ))
    aeb_died = RegisterEventHandler(OnProcessExit(
        target_action=aeb_node,
        on_exit=[
            LogInfo(msg="aeb_node exited -- shutting down launch."),
            Shutdown(reason="aeb_node exited"),
        ],
    ))

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_aeb",
            default_value="true",
            description="Launch the emergency-braking gate (set false for bring-up debugging).",
        ),
        controller_node,
        aeb_node,
        controller_died,
        aeb_died,
    ])
