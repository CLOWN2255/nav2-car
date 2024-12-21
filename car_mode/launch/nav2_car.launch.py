# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition,UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import GroupAction
pkg_ros_gz_car_mode = get_package_share_directory('car_mode')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('map',
                           default_value=PathJoinSubstitution([pkg_ros_gz_car_mode, 'map', 'map.yaml']), 
                           description='navigation2 map'),
    DeclareLaunchArgument('params',
                           default_value=PathJoinSubstitution([pkg_ros_gz_car_mode, 'config', 'nav2.yaml']), 
                           description='navigation2 configuration file'),
    DeclareLaunchArgument('slam', default_value='False',
                          choices=['True', 'False'],
                          description='Use SLAM'),
    DeclareLaunchArgument('rviz', default_value='true',description='Open RViz.'),
    DeclareLaunchArgument('rvizconfig', default_value='nav2.rviz',description='RViz config file'),
    
]


def generate_launch_description():

    #turtlebot3_waffle.urdf
    sdf_file = os.path.join(pkg_ros_gz_car_mode, 'models', 'car_urdf.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_ros_gz_car_mode,
            'worlds',
            'maze.sdf'
        ])}.items(),
    )

    # Bridge to forward tf and joint states to ros2

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',

        ],
            
        output='screen'
    )


    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    


    #nav
    use_sim_time=LaunchConfiguration('use_sim_time')
    map_params = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('params')
    slam=LaunchConfiguration('slam')
    nav2=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py'),
        ),
        #注意  key值类型必须是 LaunchConfiguration
        launch_arguments={
            'map': map_params,
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'slam':slam
        }.items(),
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[['-d', pkg_ros_gz_car_mode,  '/rviz/', LaunchConfiguration('rvizconfig') ]],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )
    

 
    
    nodelist = ARGUMENTS +[
        gazebo,
        bridge,
        robot_state_publisher,
        rviz,
        nav2
    ]
    return LaunchDescription(nodelist)