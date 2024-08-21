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
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    no_rviz2 = LaunchConfiguration('no_rviz2', default='false')

    robot_token = os.getenv('ROBOT_TOKEN', '')
    robot_ip = os.getenv('ROBOT_IP', '')
    print("IP:", robot_ip)

    conn_mode = "single"

    # these are debug only
    map_name = os.getenv('MAP_NAME', '3d_map')
    save_map = os.getenv('MAP_SAVE', 'true')

    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    rviz_config = "single_robot_conf.rviz"
    
    # else:
    #     rviz_config = "multi_robot_conf.rviz"

    # if conn_type == 'cyclonedds':
    #     rviz_config = "single_robot_conf.rviz"

    # urdf_file_name = 'multi_go2.urdf'
    # urdf = os.path.join(
    #     get_package_share_directory('go2_robot_sdk'),
    #     "urdf",
    #     urdf_file_name)
    # with open(urdf, 'r') as infp:
    #     robot_desc = infp.read()

    # robot_desc_modified_lst = []

    # for i in range(len(robot_ip_lst)):
    #     robot_desc_modified_lst.append(robot_desc.format(robot_num=f"robot{i}"))

    urdf_launch_nodes = []

    joy_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )

    default_config_topics = os.path.join(
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

    # if conn_mode == 'single':

    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(get_package_share_directory('go2_robot_sdk'),"urdf", urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        print("robot_description\n", robot_desc)

    urdf_launch_nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': robot_desc
            }],
            arguments=[urdf]
        ),
    )        # urdf_launch_nodes.append(
        #     Node(
        #         package='ros2_go2_video',
        #         executable='ros2_go2_video',
        #         parameters=[{'robot_ip': robot_ip,
        #                      'robot_token': robot_token}],
        #     ),
        # )

    # this is giving us a fake laserscan channel?
    # we are also wiring raw laser data anyway, or could at least
    # takes /utlidar/cloud_deskewed in, and generates /scan messages
    urdf_launch_nodes.append(
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[('cloud_in', '/utlidar/cloud_deskewed'),('scan', 'scan')],
            parameters=[{
                'target_frame': 'base_link', 
                'min_height': 0.05,
                'max_height': 0.50,
                'range_min': 0.0,
                'range_max': 4.0,
            }],
            output='screen',
        ),
    )

    # min_height (double, default: 2.2e-308) - The minimum height to sample in the point cloud in meters.
    # max_height (double, default: 1.8e+308) - The maximum height to sample in the point cloud in meters.
    # angle_min (double, default: -π) - The minimum scan angle in radians.
    # angle_max (double, default: π) - The maximum scan angle in radians.
    # angle_increment (double, default: π/180) - Resolution of laser scan in radians per ray.
    # queue_size (double, default: detected number of cores) - Input point cloud queue size.
    # scan_time (double, default: 1.0/30.0) - The scan rate in seconds. Only used to populate the scan_time field of the output laser scan message.
    # range_min (double, default: 0.0) - The minimum ranges to return in meters.
    # range_max (double, default: 1.8e+308) - The maximum ranges to return in meters.
    # target_frame (str, default: none) - If provided, transform the pointcloud into this frame before converting to a laser scan. Otherwise, laser scan will be generated in the same frame as the input point cloud.
    # transform_tolerance (double, default: 0.01) - Time tolerance for transform lookups. Only used if a target_frame is provided.
    # use_inf (boolean, default: true) - If disabled, report infinite range (no obstacle) as range_max + 1. Otherwise report infinite range as +inf.

    # else:

    #     for i in range(len(robot_ip_lst)):
    #         urdf_launch_nodes.append(
    #             Node(
    #                 package='robot_state_publisher',
    #                 executable='robot_state_publisher',
    #                 name='robot_state_publisher',
    #                 output='screen',
    #                 namespace=f"robot{i}",
    #                 parameters=[{'use_sim_time': use_sim_time,
    #                              'robot_description': robot_desc_modified_lst[i]}],
    #                 arguments=[urdf]
    #             ),
    #         )
    #         urdf_launch_nodes.append(
    #             Node(
    #                 package='ros2_go2_video',
    #                 executable='ros2_go2_video',
    #                 parameters=[{'robot_ip': robot_ip_lst[i],
    #                              'robot_token': robot_token}],
    #             ),
    #         )
            # urdf_launch_nodes.append(
            #     Node(
            #         package='pointcloud_to_laserscan',
            #         executable='pointcloud_to_laserscan_node',
            #         name='pointcloud_to_laserscan',
            #         remappings=[
            #             ('cloud_in', f'robot{i}/point_cloud2'),
            #             ('scan', f'robot{i}/scan'),
            #         ],
            #         parameters=[{
            #             'target_frame': f'robot{i}/base_link',
            #             'max_height': 0.1
            #         }],
            #         output='screen',
            #     ),
            # )

    return LaunchDescription([

        *urdf_launch_nodes,
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{
                'robot_ip': robot_ip, 
                'token': robot_token, 
                'conn_type': conn_type
            }],
        ),
        Node(
            package='go2_robot_sdk',
            executable='camera_to_image',
            parameters=[{'robot_interface': 'eno3'}],
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            condition=UnlessCondition(no_rviz2),
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', rviz_config)]
        ),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[default_config_topics],
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, default_config_topics],
        ),

        # IncludeLaunchDescription(
        #     FrontendLaunchDescriptionSource(foxglove_launch)
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            launch_arguments={
                'params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'nav2_bringup'), 'launch', 'navigation_launch.py')
            ]),
            launch_arguments={
                'params_file': nav2_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
