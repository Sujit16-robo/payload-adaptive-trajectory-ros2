import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('payload_arm')
    xacro_file = os.path.join(pkg, 'urdf', 'payload_arm.urdf.xacro')

    payload_mass_value = '0.001'
    for arg in sys.argv:
        if arg.startswith('payload_mass:='):
            payload_mass_value = arg.split(':=')[1]

    doc = xacro.process_file(xacro_file, mappings={'payload_mass': payload_mass_value, 'controllers_file': ''})
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument('payload_mass', default_value='0.001'),

        ExecuteProcess(cmd=['gz', 'sim', '-r', 'empty.sdf'], output='screen'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[robot_description, {'use_sim_time': True}]),

        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'payload_arm', '-z', '0.0'],
             output='screen'),

        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/joint1/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                 '/joint2/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                 '/joint3/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
             ],
             output='screen'),
    ])