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
    config_dir = os.path.join(simulator_dir, 'config')
    model_path = os.path.join(simulator_dir, 'models')

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
        default_value='none',
        description='Robot to spawn: none (world only) or a model name that has '
                    'models/<robot>/model.sdf. config/<robot>_bridge.yaml is bridged when it exists'
    )
    gz_args_arg = DeclareLaunchArgument('gz_args', default_value='-r -v 4')
    emulator_params_arg = DeclareLaunchArgument(
        'emulator_params',
        default_value='',
        description='Parameters of the node that emulates the RSF-X001 outputs missing in Gazebo '
                    '(LiDAR-rate LIO odometry, GPGGA fix quality, IMU in device axes), used by robot:=boxf. '
                    'If empty, config/rsf_device_emulator_<world>.yaml is used when it exists, '
                    'otherwise config/rsf_device_emulator.yaml'
    )

    world_file = PathJoinSubstitution([
        simulator_dir, 'worlds', [LaunchConfiguration('world'), '.sdf']
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[('gz_args', [LaunchConfiguration('gz_args'), ' ', world_file])]
    )

    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[('/clock', '/rsf/sim/clock_raw')],
        output='screen',
    )

    clock_node = Node(
        package='rsf_simulator',
        executable='clock_throttle',
        name='clock_throttle',
        output='screen',
    )

    def make_robot(context):
        robot = LaunchConfiguration('robot').perform(context)
        if robot == 'none':
            return []
        model_file = os.path.join(model_path, robot, 'model.sdf')
        if not os.path.exists(model_file):
            robots = sorted(d for d in os.listdir(model_path) if d not in SPAWN_POSES)
            raise RuntimeError(f"robot '{robot}' has no models/{robot}/model.sdf. Available: none, {', '.join(robots)}")
        world = LaunchConfiguration('world').perform(context)
        x, y, z, yaw = SPAWN_POSES[world]

        nodes = [Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-world', world,
                '-name', robot,
                '-file', model_file,
                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            ],
            output='screen',
        )]

        bridge_config = os.path.join(config_dir, f'{robot}_bridge.yaml')
        if os.path.exists(bridge_config):
            nodes.append(Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_config}],
                output='screen',
            ))

        if robot == 'boxf':
            params = LaunchConfiguration('emulator_params').perform(context)
            if not params:
                params = os.path.join(config_dir, f'rsf_device_emulator_{world}.yaml')
                if not os.path.exists(params):
                    params = os.path.join(config_dir, 'rsf_device_emulator.yaml')
            nodes.append(Node(
                package='rsf_simulator',
                executable='rsf_device_emulator',
                name='rsf_device_emulator',
                parameters=[params],
                output='screen',
            ))

        return nodes

    return LaunchDescription([
        world_arg,
        robot_arg,
        gz_args_arg,
        emulator_params_arg,
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        gazebo,
        clock_bridge_node,
        clock_node,
        OpaqueFunction(function=make_robot),
    ])
