import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    package_dir = get_package_share_directory('webots_ros2_husarion')

    rosbot_description_package =  get_package_share_directory('rosbot_description')
    rosbot_xl_description_package =  get_package_share_directory('rosbot_xl_description')

    rosbot_description = os.path.join(rosbot_description_package, 'urdf', 'rosbot.urdf.xacro')
    rosbot_xl_description = os.path.join(rosbot_xl_description_package, 'urdf', 'rosbot_xl.urdf.xacro')


    rosbot_description_urdf = Command(['xacro ', rosbot_description, ' use_sim:=true simulation_engine:=webots'])
    rosbot_xl_description_urdf = Command(['xacro ', rosbot_xl_description, ' use_sim:=true simulation_engine:=webots'])


    rosbot_ros2_control_params = os.path.join(
        package_dir, 'resource', 'rosbot_controllers.yaml')
    rosbot_xl_ros2_control_params = os.path.join(
        package_dir, 'resource', 'rosbot_xl_controllers.yaml')

    rosbot_world = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'rosbot.wbt'),
        condition=LaunchConfigurationEquals('robot_name', 'rosbot')
    )
    rosbot_xl_world = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'rosbot_xl.wbt'),
        condition=LaunchConfigurationEquals('robot_name', 'rosbot_xl')
    )

    ekf_config = os.path.join(
        package_dir, 'resource', 'ekf.yaml')

    ros2_supervisor = Ros2SupervisorLauncher()

    rosbot_webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': 'rosbot'},
        parameters=[
            {'robot_description': rosbot_description_urdf},
            {'set_robot_state_publisher': True},
            {'use_sim_time': use_sim_time},
            rosbot_ros2_control_params,
        ],
        remappings=[
            ("rosbot_base_controller/cmd_vel_unstamped", "cmd_vel"),
            ("odom", "rosbot_base_controller/odom"),
        ],
        condition=LaunchConfigurationEquals('robot_name', 'rosbot')
    )

    rosbot_xl_webots_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        additional_env={'WEBOTS_CONTROLLER_URL': 'rosbot_xl'},
        parameters=[
            {'robot_description': rosbot_xl_description_urdf},
            {'set_robot_state_publisher': True},
            {'use_sim_time': use_sim_time},
            rosbot_xl_ros2_control_params,
        ],
        remappings=[
            ("rosbot_base_controller/cmd_vel_unstamped", "cmd_vel"),
            ("odom", "rosbot_base_controller/odom"),
        ],
        condition=LaunchConfigurationEquals('robot_name', 'rosbot_xl')
    )

    rosbot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': rosbot_description_urdf},
            {'use_sim_time': use_sim_time},
        ],
        condition=LaunchConfigurationEquals('robot_name', 'rosbot')
    )

    rosbot_xl_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': rosbot_xl_description_urdf},
            {'use_sim_time': use_sim_time},
        ],
        condition=LaunchConfigurationEquals('robot_name', 'rosbot_xl')
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rosbot_base_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Standard ROS 2 launch description
    return launch.LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='robot_name', default_value='rosbot',
                                            description='Spawned robot name'),
        # Start the Webots node
        rosbot_world,
        rosbot_xl_world,

        # Start the Ros2Supervisor node
        ros2_supervisor,

        # Start the Webots robot driver
        rosbot_webots_robot_driver,
        rosbot_xl_webots_robot_driver,


        # Start the robot_state_publisher
        rosbot_state_publisher,
        rosbot_xl_state_publisher,


        # Start the ros2_controllers
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_localization_node,

        # This action will kill all nodes once the Webots simulation has exited
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=rosbot_world,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        ),
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=rosbot_xl_world,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )
        )
    ])
