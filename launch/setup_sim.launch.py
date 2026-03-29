import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Declare the launch argument for the controller
    declare_controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid_controller',
        description='Controller to be used'
    )

    # Access the launch configuration value
    controller = LaunchConfiguration('controller')

    # Specify the name of the package and path to urdf file within the package
    pkg_name = 'gazebo_simulation'
    file_subpath = 'urdf/robot.urdf'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # load an empty gazebo world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py'
        ]),
    )

    #spawn the bed in gazebo to test the collision of the robot with the bed 
    spawn_bed = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.join(get_package_share_directory(pkg_name),'sdf', 'Bed.sdf'),
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
        'use_sim_time': True}] # add other parameters here if required
    )

    # spawn our assist_robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )
    
    # load joint_state_broadcaster first after spawn of entity
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # load the specified controller after joint_state_broadcaster is running
    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
        output='screen'
    )

    # Run the node
    return LaunchDescription([
        declare_controller_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(   #added these event handlers for testing during debug sessions. They are not strictly necessary but left in because they ensure the correct order of loading the robot and the controllers)
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        spawn_bed,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[TimerAction(
                    period=2.0,  # wait for 2 seconds to ensure the joint_state_broadcaster is fully active
                    actions=[load_controller]
                )],
            )
        ),
    ]) 