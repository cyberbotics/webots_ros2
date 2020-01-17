# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os

from ament_index_python.packages import get_package_share_directory
import launch

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    hello_world = get_package_share_directory('hello_world_simulation')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    gazebo_client = launch.actions.IncludeLaunchDescription(
	launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
     )
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    turtlebot3_description_reduced_mesh = get_package_share_directory('turtlebot3_description_reduced_mesh')
    turtlebot3_description_reduced_mesh_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_description_reduced_mesh, 'launch', 'spawn_turtlebot.launch.py'))
    )

    turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_state_publisher_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(hello_world, 'worlds', 'empty.world'), ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=launch.substitutions.EnvironmentVariable(
                'TURTLEBOT3_MODEL'),
            description='model type [burger, waffle, waffle_pi]'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        gazebo_server,
        gazebo_client,
        turtlebot3_description_reduced_mesh_launch,
        turtlebot3_state_publisher_launch
    ])
