# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    no_rviz2 = LaunchConfiguration('no_rviz2', default='false')
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.123.99')
    robot_interface = LaunchConfiguration('robot_interface', default='eno3')
    rviz_config = "robot_conf.rviz"
    
    path = get_package_prefix('go2_robot_sdk')
    # /home/bill/ros2_ws/install/go2_robot_sdk

    pathSrc = path.replace("install", "src")
    mvc = '_config:=' + os.path.join(pathSrc, 'config/map.mvc')
    print(mvc)

    urdf_launch_nodes = []

    joy_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )

    mux_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'twist_mux.yaml')

    foxglove_launch = os.path.join(
        get_package_share_directory('foxglove_bridge'),
        'launch',
        'foxglove_bridge_launch.xml',
    )

    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml'
    )

    ekf_config_internal = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'ekf_params_internal.yaml'
    )
    ekf_config_global = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'ekf_params_global.yaml'
    )
    ekf_config_gps = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'ekf_params_gps.yaml'
    )

    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(get_package_share_directory('go2_robot_sdk'),"urdf", urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        # print("robot_description\n", robot_desc)

    # this listens to joint_states
    # this provides PART of the /tf, namely the state of all the limbs and the sensors 
    urdf_launch_nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time'     : use_sim_time, 
                'robot_description': robot_desc
            }],
            arguments=[urdf]
        ),
    )

    # this is giving us a synthetic laserscan channel
    # takes /utlidar/cloud_deskewed and generates /scan messages
    urdf_launch_nodes.append(
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', 'point_cloud2'),
                ('scan',     'scan')],
            parameters=[{
                'target_frame'    : 'base_link', 
                'angle_increment' :  0.01,
                'min_height'      :  0.10,
                'max_height'      :  2.00,
                'range_min'       :   0.5,
                'range_max'       :  20.0,
            }],
            output='screen',
        ),
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     "log_level",
        #     default_value=["debug"],
        #     description="Logging level",
        # ),
        *urdf_launch_nodes,
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{
                'robot_ip': robot_ip, 
                'robot_interface': robot_interface,
            }],
        ),
        Node(
            package='go2_robot_sdk',
            executable='camera_to_image',
            parameters=[{
                'robot_interface': robot_interface,
            }],
        ),
        Node(
            package='go2_robot_sdk',
            executable='coco_detector_node',
            parameters=[{
                'publish_annotated_image': True,
                'device': 'cuda',
                'detection_threshold': 0.7
            }],
        ),
        Node(
            package='go2_robot_sdk',
            executable='imu_mag_node',
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            condition=UnlessCondition(no_rviz2),
            name='rviz2',
            arguments=['-d' + os.path.join(pathSrc, 'config', rviz_config)]
        ),
        Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
            arguments=['_config:=' + os.path.join(pathSrc, 'config/map.mvc')]
        ),
        Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame"  : "map"},
                {"local_xy_origin" : "house"},
                {"local_xy_origins": [39.000000000,-120.00000000,20.00,0.0]},
                #{"local_xy_navsatfix_topic" : "/gps/filtered"}, # this is coming from the navsat node 
                {"use_sim_time": use_sim_time},
            ]
        ),
        
        # That tool will publish a static transform from the parent frame foo to 
        # the child frame bar with (X, Y, Z) translation (1, 2, 3) and (yaw, pitch, roll) 
        # body-fixed axis rotation sequence (0.5, 0.1, -1.0).

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        ),

        # Connects to a joystick and produces /joy messages
        # publishes /joy (sensor_msgs/msg/Joy)
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_config]
        ),

        # converts joy messages to velocity commands
        # takes     /joy     (sensor_msgs/msg/Joy)
        # publishes /cmd_vel (geometry_msgs/msg/Twist or /TwistStamped)
        # reampped to cmd_vel_joy for sanity
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            remappings=[('cmd_vel', 'cmd_vel_joy')],
            parameters=[mux_config],
        ),

        # Take /cmd_vel_nav and /cmd_vel_joy, multiplex them, and send the result to 
        # /cmd_vel_out?
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }, mux_config],
        ),
        Node(
            package='gpsx',
            executable='gps_node',
            output='screen',
        ),
        # IncludeLaunchDescription(
        #     FrontendLaunchDescriptionSource(foxglove_launch)
        # ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            # arguments=['--ros-args', '--log-level', 'debug'],
            remappings=[
                ('/odometry/filtered', '/odom_mag')],
            parameters=[{
                'use_sim_time': use_sim_time,
            }, ekf_config_internal],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            # arguments=['--ros-args', '--log-level', 'debug'],
            parameters=[{
                'use_sim_time': use_sim_time,
            }, ekf_config_gps],
            remappings=[
                ("imu",               "/imu_wit"),       # imu with quarterion with mag
                ("gps/fix",           "/gpsx"),          # gps position
                ("odometry/filtered", "/odom_mag"),      # odom with mag, coming from the EKF
                ("gps/filtered",      "/gps/filtered"),  
                # output - very good - basically just 
                # odom offset by origin, in different frame
                ("odometry/gps",      "/odom_navsat")],  # output - crappy and noisy - contains GPS data
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[
                ('/odometry/filtered', '/odom_global')
            ],
            parameters=[{
                'use_sim_time': use_sim_time,
            }, ekf_config_global],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'params_file' : slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        # GroupAction(
        #     actions=[
        #         SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
        #         IncludeLaunchDescription(
        #             PythonLaunchDescriptionSource([
        #                 os.path.join(get_package_share_directory(
        #                     'nav2_bringup'), 'launch', 'navigation_launch.py')
        #             ]),
        #             launch_arguments={
        #                 'params_file' : nav2_config,
        #                 'use_sim_time': use_sim_time,
        #             }.items(),
        #         )
        #     ]
        # )        
    ])
