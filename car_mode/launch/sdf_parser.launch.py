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

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('sync', default_value='true',
                          choices=['true', 'false'],
                          description='Use synchronous SLAM'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('autostart', default_value='true',
                          choices=['true', 'false'],
                          description='Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.'),  # noqa: E501
    DeclareLaunchArgument('use_lifecycle_manager', default_value='false',
                          choices=['true', 'false'],
                          description='Enable bond connection during node activation'),
    DeclareLaunchArgument('params',
                          default_value=PathJoinSubstitution([pkg_ros_gz_car_mode, 'config', 'slam.yaml']),  # noqa: E501
                          description='Path to the SLAM Toolbox configuration file'),
    DeclareLaunchArgument('rviz', default_value='true',description='Open RViz.'),
    
]


def generate_launch_description():

  
    sdf_file = os.path.join(pkg_ros_gz_car_mode, 'models', 'model.sdf')

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
    gz_topic = '/model/car_mode'
    joint_state_gz_topic = '/world/demo' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint states (Gazebo -> ROS2)
            joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Link poses (Gazebo -> ROS2)
            link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Velocity and odometry (Gazebo -> ROS2)
            gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            gz_topic + '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            gz_topic + '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            (joint_state_gz_topic, 'joint_states'),
            (link_pose_gz_topic, '/tf'),
            (link_pose_gz_topic + '_static', '/tf_static'),
        ],
        parameters=[{'qos_overrides./tf_static.publisher.durability': 'transient_local'},
                  
                  ],
        output='screen'
    )

    # Since sdformat_urdf can't parse joints with sensors, 
    # I added a node to change frame_id to TF that can be resolved, 
    # although this is not the best way.
    scan_pub = Node(
        package='car_mode',
        executable='scan_pub',
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
    
    #slam
    sync = LaunchConfiguration('sync')
    slam_params = LaunchConfiguration('params')
    slam = GroupAction([
      IncludeLaunchDescription
      (
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_sync_launch.py'),
        ),
        launch_arguments=[('slam_params_file', slam_params)],
        condition=IfCondition(sync)
      ),
      IncludeLaunchDescription
      (
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments=[('slam_params_file', slam_params)],
        condition=UnlessCondition(sync)
      )
    ])


    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_ros_gz_car_mode, 'rviz', 'car_mode.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )
    

    #Control the direction of travel of the trolley
    robot_steering = Node(
        package='robot_steering',
        executable='robot_steering',
    )
    
    nodelist = ARGUMENTS +[
        slam,
        gazebo,
        bridge,
        scan_pub,
        robot_state_publisher,
        rviz,
        robot_steering,
    ]
    return LaunchDescription(nodelist)