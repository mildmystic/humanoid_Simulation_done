from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('humannew')
    world_file = os.path.join(pkg_share, 'worlds', 'robot.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')

    # Start Gazebo Sim 8 (Fortress)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', LaunchConfiguration('world')],
        output='screen'
    )

    # Bridge Ignition topics to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        # arguments=[
        #     '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
        #     '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
        #     '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        #     '/world/nao_world/model/Nao/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
        #     # Bridge joint commands for controllers
        #     '/HeadYaw/position@std_msgs/msg/Float64@gz.msgs.Double',
        #     '/HeadPitch/position@std_msgs/msg/Float64@gz.msgs.Double',
        #     '/LShoulderPitch/position@std_msgs/msg/Float64@gz.msgs.Double',
        #     # Add more joint command topics as needed (e.g., LShoulderRoll, LElbowYaw, etc.)
        # ],
        output='screen'
    )

    # Load controllers (for JointPositionController plugins)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': ''},
            PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
        ],  # Update with URDF or SDF if needed
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='SDF world file'
        ),
        gz_sim,
        bridge,
        controller_manager
    ])

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     pkg_share = get_package_share_directory('human')
#     world_file = os.path.join(pkg_share, 'worlds', 'robot.sdf')

#     # Start Gazebo Harmonic (gz sim)
#     gz_sim = ExecuteProcess(
#         cmd=['gz', 'sim', LaunchConfiguration('world')],
#         output='screen'
#     )

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'world',
#             default_value=world_file,
#             description='SDF world file'
#         ),
#         gz_sim
#     ])
