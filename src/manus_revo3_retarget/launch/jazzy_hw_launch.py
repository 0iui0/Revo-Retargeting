#!/usr/bin/env python3
"""Minimal Jazzy-compatible launch for Revo3 hardware + MIT controller.

Avoids load_on_configure/activate_on_configure conflict with spawners.
"""

import os
import tempfile

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _generate_robot_description(hand_side: str, protocol_config: str, if_sim: str,
                                initial_positions: str) -> str:
    share = get_package_share_directory("revo3_description")
    xacro_path = os.path.join(share, "urdf", "revo3.single.system.xacro")
    doc = xacro.process_file(xacro_path, mappings={
        "hand_side": hand_side,
        "protocol_config_file": protocol_config,
        "if_sim": if_sim,
        "initial_positions_file": initial_positions,
    })
    return doc.toprettyxml(indent="  ")


CONTROLLER_JOINTS = [
    "HAND_PREFIX_little_MPR_joint",
    "HAND_PREFIX_little_MCP_joint",
    "HAND_PREFIX_little_PIP_joint",
    "HAND_PREFIX_little_DIP_joint",
    "HAND_PREFIX_ring_MPR_joint",
    "HAND_PREFIX_ring_MCP_joint",
    "HAND_PREFIX_ring_PIP_joint",
    "HAND_PREFIX_ring_DIP_joint",
    "HAND_PREFIX_middle_MPR_joint",
    "HAND_PREFIX_middle_MCP_joint",
    "HAND_PREFIX_middle_PIP_joint",
    "HAND_PREFIX_middle_DIP_joint",
    "HAND_PREFIX_index_MPR_joint",
    "HAND_PREFIX_index_MCP_joint",
    "HAND_PREFIX_index_PIP_joint",
    "HAND_PREFIX_index_DIP_joint",
    "HAND_PREFIX_thumb_MCP_joint",
    "HAND_PREFIX_thumb_PIP_joint",
    "HAND_PREFIX_thumb_DIP_joint",
    "HAND_PREFIX_thumb_CMP_joint",
    "HAND_PREFIX_thumb_CMR_joint",
]


def _make_controller_yaml(hand_side: str, update_rate: str) -> str:
    joints = [j.replace("HAND_PREFIX", f"{hand_side}_") for j in CONTROLLER_JOINTS]
    yaml = f"""controller_manager:
  ros__parameters:
    update_rate: {update_rate}
    joint_forward_mit_controller:
      type: revo3_mit_controller/Revo3MITController
joint_forward_mit_controller:
  ros__parameters:
    default_kp: 0.4
    default_kd: 0.05
    joints: {joints}
"""
    tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    tmp.write(yaml)
    tmp.close()
    return tmp.name


def launch_setup(context: LaunchContext):
    hand_side = context.perform_substitution(LaunchConfiguration("hand_side")).lower()
    update_rate = context.perform_substitution(LaunchConfiguration("update_rate"))
    protocol_config = context.perform_substitution(LaunchConfiguration("protocol_config_file"))
    initial_positions = context.perform_substitution(LaunchConfiguration("initial_positions_file"))
    launch_rsp = context.perform_substitution(LaunchConfiguration("launch_rsp")).lower() == "true"
    if_sim = context.perform_substitution(LaunchConfiguration("if_sim"))
    use_namespace = context.perform_substitution(LaunchConfiguration("use_namespace")).lower() == "true"

    if not protocol_config:
        driver_share = get_package_share_directory("revo3_driver")
        protocol_config = os.path.join(driver_share, "config", f"protocol_modbus_{hand_side}.yaml")
    if not initial_positions:
        driver_share = get_package_share_directory("revo3_driver")
        initial_positions = os.path.join(driver_share, "config", f"initial_positions_{hand_side}.yaml")

    robot_description = _generate_robot_description(
        hand_side, protocol_config, if_sim, initial_positions)

    controller_yaml = _make_controller_yaml(hand_side, update_rate)

    # Use dict for robot_description (launch handles YAML serialization)
    robot_description_dict = {"robot_description": robot_description}

    namespace = f"revo3_{hand_side}" if use_namespace else ""

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description_dict, controller_yaml],
        output="both",
    )

    rsp_node = None
    if launch_rsp:
        joint_state_topic = (f"/{namespace}/revo3_joint_state/joint_states"
                             if namespace else "/revo3_joint_state/joint_states")
        rsp_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            output="both",
            parameters=[{"robot_description": robot_description}],
            remappings=[("joint_states", joint_state_topic)],
        )

    # Spawner — controller params come from controller_yaml (/**/ scope)
    cm_topic = f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    mit_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_forward_mit_controller", "-c", cm_topic, "--controller-params-file", controller_yaml],
        output="both",
    )

    actions = [control_node]
    if rsp_node:
        actions.append(rsp_node)
    actions.append(TimerAction(period=2.0, actions=[mit_spawner]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("hand_side", default_value="right", choices=["left", "right"]),
        DeclareLaunchArgument("update_rate", default_value="200"),
        DeclareLaunchArgument("protocol_config_file", default_value=""),
        DeclareLaunchArgument("initial_positions_file", default_value=""),
        DeclareLaunchArgument("launch_rsp", default_value="true"),
        DeclareLaunchArgument("if_sim", default_value="false"),
        DeclareLaunchArgument("use_namespace", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
