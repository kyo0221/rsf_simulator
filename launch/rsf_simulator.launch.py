import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


SPAWN_POSES = {
    'tsudanuma2-3': (0.0, 0.0, 0.0, 0.0),
    'tsudanuma': (0.0, 0.0, -0.344, 0.0),
    'tsukuba': (24.808, 3.303, 25.049, 3.065),
}


def generate_launch_description():
    simulator_dir = get_package_share_directory('rsf_simulator')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='tsukuba',
        choices=['tsudanuma2-3', 'tsudanuma', 'tsukuba'],
        description='World in rsf_simulator/worlds: tsudanuma2-3 (building editor), '
                    'tsudanuma (Tsudanuma campus, generated from a point cloud map) or '
                    'tsukuba (Tsukuba city hall area with slopes)'
    )
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='orne_boxF',
        choices=['orne_boxF', 'none'],
        description='Robot model in rsf_simulator/models to spawn. '
                    'none starts only Gazebo with the world (no robot, bridge, clock or emulator nodes)'
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

    def make_robot(context):
        robot = LaunchConfiguration('robot').perform(context)
        if robot == 'none':
            return []
        world = LaunchConfiguration('world').perform(context)
        x, y, z, yaw = SPAWN_POSES[world]

        spawn_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world,
                '-name', robot,
                '-file', os.path.join(model_path, robot, f'{robot}.sdf'),
                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            ],
            output='screen',
        )

        bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/rsf/hokuyo3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/rsf/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/rsf/nav_sat_fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
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

        params = LaunchConfiguration('emulator_params').perform(context)
        if not params:
            params = os.path.join(simulator_dir, 'config', f'rsf_device_emulator_{world}.yaml')
            if not os.path.exists(params):
                params = os.path.join(simulator_dir, 'config', 'rsf_device_emulator.yaml')
        emulator_node = Node(
            package='rsf_simulator',
            executable='rsf_device_emulator.py',
            name='rsf_device_emulator',
            parameters=[params],
            output='screen',
        )

        return [spawn_node, bridge_node, clock_node, emulator_node]

    return LaunchDescription([
        world_arg,
        robot_arg,
        gz_args_arg,
        emulator_params_arg,
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        gazebo,
        OpaqueFunction(function=make_robot),
    ])
