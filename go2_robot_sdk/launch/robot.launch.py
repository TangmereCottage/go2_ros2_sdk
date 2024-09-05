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
from ament_index_python.packages import get_package_share_directory
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
    
    urdf_launch_nodes = []

    joy_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )

    mux_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'twist_mux.yaml')

    # foxglove_launch = os.path.join(
    #     get_package_share_directory('foxglove_bridge'),
    #     'launch',
    #     'foxglove_bridge_launch.xml',
    # )

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

    ekf_config_local = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'ekf_params_local.yaml'
    )

    ekf_config_global = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'ekf_params_global.yaml'
    )

    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(get_package_share_directory('go2_robot_sdk'),"urdf", urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        # print("robot_description\n", robot_desc)

    # this listens to joint_states
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
    # can we get those more directly, too?
    urdf_launch_nodes.append(
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', 'point_cloud2')],
            parameters=[{
                'target_frame': 'base_link', 
                #'angle_min'       : -1.04,
                #'angle_max'       :  1.0,
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
            arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', rviz_config)]
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

        # this produces /gpsx
        # Start GPS
        Node(
            package='gpsx',
            executable='gps_node',
            #name='ekf_filter_node',
            output='screen',
            #remappings=[('gpsx', 'gpsx_raw')],
            #remappings=[('tf', 'tf_ekf')],
            #parameters=[{
            #    'use_sim_time': use_sim_time,
            #}, ekf_config],
        ),

        # # Start robot localization using an Extended Kalman filter
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     remappings=[('tf', 'tf_ekf_local'),('/odometry/filtered', '/odometry/filtered_local')],
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #     }, ekf_config_local],
        # ),

        # # Start robot localization using an Extended Kalman filter
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     remappings=[('tf', 'tf_ekf_global'),('/odometry/filtered', '/odometry/filtered_global')],
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #     }, ekf_config_global],
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         os.path.join(get_package_share_directory(
        #             'slam_toolbox'), 'launch', 'online_async_launch.py')
        #     ]),
        #     launch_arguments={
        #         'params_file' : slam_toolbox_config,
        #         'use_sim_time': use_sim_time,
        #     }.items(),
        # ),

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
