"""
Starts the high-level driver of the Unitree G1 (29 DoF) on the standard legged robot topics, under
sas_g1/g1_1 (the namespace and the name of the node).

Real robot (onboard computer):
    ros2 launch sas_robot_driver_unitree_g1 start_high_level_driver.py

Simulation (unitree_mujoco, DDS domain 1 on the loopback interface):
    ros2 launch sas_robot_driver_unitree_g1 start_high_level_driver.py domain_id:=1 network_interface:=lo

The control level is high by default (locomotion controller and rt/arm_sdk). control_level:=low selects the
low-level control (rt/lowstate and rt/lowcmd only), which is not implemented yet: the driver stops with an error.

Run it in a different terminal window or tab. Be ready to close it, as it activates the real robot if the
connection is successful.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'sigterm_timeout',
            default_value='30'
        ),
        DeclareLaunchArgument(
            'domain_id',
            default_value='0',
            description='DDS domain: 0 for the real robot, 1 for unitree_mujoco.'
        ),
        DeclareLaunchArgument(
            'network_interface',
            default_value='eth0',
            description='Network interface: "eth0" on the onboard computer, "lo" for unitree_mujoco.'
        ),
        DeclareLaunchArgument(
            'control_level',
            default_value='high',
            description='"high" (locomotion controller and rt/arm_sdk) or "low" (rt/lowstate and rt/lowcmd; not implemented yet).'
        ),
        Node(
            package='sas_robot_driver_unitree_g1',
            executable='sas_robot_driver_unitree_g1_node',
            name='g1_1',
            namespace='sas_g1',
            output='screen',
            parameters=[{
                'domain_id': ParameterValue(LaunchConfiguration('domain_id'), value_type=int),
                'network_interface': ParameterValue(LaunchConfiguration('network_interface'), value_type=str),
                'thread_sampling_time_sec': 0.002,
                'twist_timeout_sec': 0.2,
                'control_level': ParameterValue(LaunchConfiguration('control_level'), value_type=str),
            }]
        ),
    ])
