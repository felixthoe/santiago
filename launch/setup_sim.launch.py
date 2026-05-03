import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare the launch argument for the controller
    declare_controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='joint_trajectory_controller',   # geändert: jetzt sinnvoller Default
        description='Controller to be used'
    )

    # Access the launch configuration value
    controller = LaunchConfiguration('controller')

    # Specify the name of the package and path to urdf file within the package
    pkg_name = 'gazebo_simulation'
    file_subpath = 'urdf/robot.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # load an empty gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'gui': 'false',  # GUI crashes on WSL (run gzclient separately)
            'extra_gazebo_args': '-s libgazebo_ros_factory.so'
        }.items()
    )

    # load RViz with the robot model
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    # used to display the human joints in rviz
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # spawn the bed in gazebo to test the collision of the robot with the bed
    spawn_bed = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory(pkg_name), 'sdf', 'Bed.sdf'),
            '-entity', 'bed',
            '-x', '0.774', '-y', '0', '-z', '0.285',
        ],
        output='screen'
    )

    # Configure the node robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    # spawn our assist_robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )

    # ---------- NEU: ROS2 Control Node mit YAML-Konfiguration ----------
    controller_config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'controller_manager.yaml'   # hier liegt Ihre YAML-Datei (Name anpassen falls nötig)
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_raw},
                    controller_config],
        output='screen'
    )

    # ---------- Spawner (laden & aktivieren Controller automatisch) ----------
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[controller],   # der über den Launch-Argument gewählte Controller
        output='screen'
    )

    # Time matching
    Node(
        package='gazebo_simulation',
        executable='helloise_trajektorie',
        parameters=[{'use_sim_time': True}],
    )

    # Run the launch description
    return LaunchDescription([
        declare_controller_arg,
        gazebo,
        rviz,
        joint_state_publisher,
        node_robot_state_publisher,
        ros2_control_node,                # startet den Controller Manager mit YAML
        spawn_entity,
        spawn_bed,
        spawn_joint_state_broadcaster,    # wartet automatisch, bis Manager bereit ist
        spawn_controller,                 # wartet automatisch, bis Broadcaster da ist
    ])