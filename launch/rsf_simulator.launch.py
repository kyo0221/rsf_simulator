import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    simulator_dir = get_package_share_directory('rsf_simulator')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tsudanuma2-3',
        choices=['tsudanuma2-3', 'tsudanuma', 'tsukuba'],
        description='World in rsf_simulator/worlds: tsudanuma2-3 (building editor), '
                    'tsudanuma (Tsudanuma campus, generated from a point cloud map by tools/pcd_to_world) or '
                    'tsukuba (Tsukuba city hall area with slopes)'
    )
    gz_args_arg = DeclareLaunchArgument('gz_args', default_value='-r -v 4')
    emulator_params_arg = DeclareLaunchArgument(
        'emulator_params',
        default_value='',
        description='Parameters of the node that emulates the RSF-X001 outputs missing in Gazebo '
                    '(LiDAR-rate LIO odometry, GPGGA fix quality, IMU in device axes). '
                    'If empty, config/rsf_device_emulator_<world>.yaml is used when it exists, '
                    'otherwise config/rsf_device_emulator.yaml'
    )

    world_file = PathJoinSubstitution([
        simulator_dir, 'worlds', [LaunchConfiguration('world'), '.sdf']
    ])

    model_path = os.path.join(simulator_dir, 'models')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', [LaunchConfiguration('gz_args'), ' ', world_file])]
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/rsf/hokuyo3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/rsf/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/rsf/nav_sat_fix@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        ],
        remappings=[
            ('/clock', '/rsf/sim/clock_raw'),
            ('/rsf/hokuyo3d/points', '/rsf/hokuyo_cloud2'),
            ('/odom', '/rsf/rsf_odom'),
            ('/rsf/imu', '/rsf/sim/imu'),
            ('/rsf/nav_sat_fix', '/rsf/sim/nav_sat_fix'),
        ],
        output='screen',
    )

    clock_node = Node(
        package='rsf_simulator',
        executable='clock_throttle',
        name='clock_throttle',
        output='screen',
    )

    def make_emulator(context):
        params = LaunchConfiguration('emulator_params').perform(context)
        if not params:
            world = LaunchConfiguration('world').perform(context)
            params = os.path.join(simulator_dir, 'config', f'rsf_device_emulator_{world}.yaml')
            if not os.path.exists(params):
                params = os.path.join(simulator_dir, 'config', 'rsf_device_emulator.yaml')
        return [Node(
            package='rsf_simulator',
            executable='rsf_device_emulator.py',
            name='rsf_device_emulator',
            parameters=[params],
            output='screen',
        )]

    emulator_node = OpaqueFunction(function=make_emulator)

    return LaunchDescription([
        world_arg,
        gz_args_arg,
        emulator_params_arg,
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', model_path),
        gazebo,
        bridge_node,
        clock_node,
        emulator_node,
    ])
