"""Launches one ak_v3_driver `motor_driver_node` instance per motor.

Each motor gets its own node instance and its own namespace (from
`joint_name`), so topics/services come out as e.g.:
    /shoulder/cmd, /shoulder/state, /shoulder/set_origin, ...
    /elbow/cmd,    /elbow/state,    /elbow/set_origin,    ...

Multiple motors on the same physical CAN bus just repeat `can_interface`;
each node filters to its own `can_id` in software and ignores every other
frame on the wire, so they don't interfere with each other.

NOTE: pole_pairs and gear_ratio are NOT launch parameters -- they live in
src/motor_limits.cpp, keyed by `motor_type`. Edit that file with your real
hardware values; it ships with 1.0/1.0 placeholders for every model.

Edit the `MOTORS` list below (or replace it with a YAML-driven loop) to
match your robot's joints.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


MOTORS = [
    {
        "joint_name": "wheel1",
        "can_interface": "can0",
        "can_id": 2,
        "motor_type": "AK60-6",
        "invert_direction": False,
    },
    {
        "joint_name": "wheel2",
        "can_interface": "can0",
        "can_id": 3,
        "motor_type": "AK60-6",
        "invert_direction": True,
    },
]


def generate_launch_description():
    nodes = []
    for motor in MOTORS:
        joint_name = motor["joint_name"]
        nodes.append(
            Node(
                package="ak_v3_driver",
                executable="motor_driver_node",
                name="ak_v3_driver_node",
                namespace=joint_name,
                output="screen",
                parameters=[
                    {
                        "can_interface": motor["can_interface"],
                        "can_id": motor["can_id"],
                        "joint_name": joint_name,
                        "motor_type": motor["motor_type"],
                        "invert_direction": motor["invert_direction"],
                    }
                ],
            )
        )
    return LaunchDescription(nodes)
