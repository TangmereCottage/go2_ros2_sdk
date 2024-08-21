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

import json
import logging
import os
import threading
import asyncio

from scripts.go2_constants import ROBOT_CMD, RTC_TOPIC
from scripts.go2_func import gen_command, gen_mov_command
from scripts.go2_lidar_decoder import update_meshes_for_cloud2
from scripts.go2_math import get_robot_joints
from scripts.webrtc_driver import Go2Connection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from go2_interfaces.msg import Go2State, IMU
from unitree_go.msg import LowState
from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        self.declare_parameter('robot_ip', os.getenv('ROBOT_IP', os.getenv('GO2_IP')))
        self.declare_parameter('token', os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', '')))
        self.declare_parameter('conn_type', os.getenv('CONN_TYPE', os.getenv('CONN_TYPE', '')))

        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.token = self.get_parameter('token').get_parameter_value().string_value
        self.conn_type = self.get_parameter('conn_type').get_parameter_value().string_value

        self.conn_mode = "single"

        self.get_logger().info(f"Received ip: {self.robot_ip}")
        self.get_logger().info(f"Connection type is {self.conn_type}")
        self.get_logger().info(f"Connection mode is {self.conn_mode}")

        self.conn = {}
        qos_profile = QoSProfile(depth=10)

        self.joint_pub = []
        #self.go2_state_pub = []
        self.go2_lidar_pub = []
        #self.go2_odometry_pub = []
        #self.imu_pub = []

        self.joint_pub.append(self.create_publisher(JointState, 'joint_states', qos_profile))
        #self.go2_state_pub.append(self.create_publisher(Go2State, f'SDK/robot0/go2_states', qos_profile))
        self.go2_lidar_pub.append(self.create_publisher(PointCloud2, 'point_cloud2', qos_profile))
        #self.go2_odometry_pub.append(self.create_publisher(Odometry, f'SDK/robot0/odom', qos_profile))
        #self.imu_pub.append(self.create_publisher(IMU, f'SDK/robot0/imu', qos_profile))

        # this is shorthand for the TransformBroadcaster
        # which will broadcast odom type information?
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.robot_cmd_vel = {}
        self.robot_odom = {}
        self.robot_low_cmd = {}
        self.robot_sport_state = {}
        self.robot_lidar = {}

        self.joy_state = Joy()

        self.create_subscription(Twist, 'cmd_vel_out', lambda msg: self.cmd_vel_cb(msg), qos_profile)
        self.create_subscription(Joy, 'joy', self.joy_cb, qos_profile)

        # Support for CycloneDDS (EDU version via ethernet)
        if self.conn_type == 'cyclonedds':
            
            # power, temp, IMU state
            # this feeds into the joint_states, which are needed for the TF
            self.create_subscription(
                LowState,
                'lowstate',
                self.publish_joint_state_cyclonedds,
                qos_profile)

            # frame_id: odom
            # postion/orientation
            self.create_subscription(
                PoseStamped,
                '/utlidar/robot_pose',
                self.publish_body_poss_cyclonedds,
                qos_profile)

            # subscribe to the raw LIDAR data
            # frame_id: odom already
            self.create_subscription(
                PointCloud2,
                '/utlidar/cloud_deskewed',
                self.publish_lidar_cyclonedds,
                qos_profile)

    #     self.timer = self.create_timer(0.1, self.timer_callback)
    #     self.timer_lidar = self.create_timer(0.5, self.timer_callback_lidar)

    # def timer_callback(self):

    # def timer_callback_lidar(self):

    def cmd_vel_cb(self, msg):

        x = msg.linear.x
        y = msg.linear.y
        z = msg.angular.z

        if x > 0.0 or y > 0.0 or z != 0.0:
            self.robot_cmd_vel = gen_mov_command(round(x, 2), round(y, 2), round(z, 2))

    def joy_cb(self, msg):
        self.joy_state = msg

    def publish_body_poss_cyclonedds(self, msg):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = msg.pose.position.z
        odom_trans.transform.rotation.x = msg.pose.orientation.x
        odom_trans.transform.rotation.y = msg.pose.orientation.y
        odom_trans.transform.rotation.z = msg.pose.orientation.z
        odom_trans.transform.rotation.w = msg.pose.orientation.w
        self.broadcaster.sendTransform(odom_trans)

    # joint states are angles and therefore do not have/need a frame_id 
    def publish_joint_state_cyclonedds(self, msg):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
        ]
        joint_state.position = [
            msg.motor_state[3].q, msg.motor_state[ 4].q, msg.motor_state[ 5].q,
            msg.motor_state[0].q, msg.motor_state[ 1].q, msg.motor_state[ 2].q,
            msg.motor_state[9].q, msg.motor_state[10].q, msg.motor_state[11].q,
            msg.motor_state[6].q, msg.motor_state[ 7].q, msg.motor_state[ 8].q,
        ]
        # self.get_logger().info(f"Joint state FL_hip: {msg.motor_state[3].q}")
        self.joint_pub[0].publish(joint_state)

    def publish_lidar_cyclonedds(self, msg):
    #     # manually set the frame_id
    #     # basically, intercept the /utlidar/cloud
    #     # add a frame_id: lidar_frame
    #     # publish as /SDK/robot0/point_cloud2
    #     # this is based on listening to the raw LIDAR data from '/utlidar/cloud'
    #     # we could also listen to /utlidar/cloud_deskewed - that uses odom
    #     # msg.header = Header(frame_id="lidar_frame") # no need because /utlidar/cloud_deskewed directly uses odom
    #     # msg.header.stamp = self.get_clock().now().to_msg()
        self.go2_lidar_pub[0].publish(msg)

    def joy_cmd(self):
        if self.robot_cmd_vel != None:
            self.get_logger().info(f"Move {self.robot_cmd_vel}")
            #self.conn.data_channel.send(self.robot_cmd_vel)
            self.robot_cmd_vel = None

        if self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Stand down")
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            #self.conn.data_channel.send(stand_down_cmd)

        if self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            #self.conn.data_channel.send(stand_up_cmd)
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            #self.conn.data_channel.send(move_cmd)

    def on_validated(self):
        for topic in RTC_TOPIC.values():
            self.get_logger().info(f"topic: {topic}")
            #self.conn.data_channel.send(json.dumps({"type": "subscribe", "topic": topic}))

    # add messages to lidar/odom etc
    def on_data_channel_message(self, _, msg):
        
        self.get_logger().info(f"message: {msg}")
        
        if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
            self.robot_lidar = msg

        if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
            self.robot_odom = msg

        if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
            self.robot_sport_state = msg

        if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
            self.robot_low_cmd = msg

    async def run(self, conn, robot_num):
        self.conn[robot_num] = conn
        while True:
            self.joy_cmd()
            await asyncio.sleep(0.1)

async def spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)
    def _spin(node: Node,
              future: asyncio.Future,
              event_loop: asyncio.AbstractEventLoop):
        while not future.cancelled():
            rclpy.spin_once(node)
        if not future.cancelled():
            event_loop.call_soon_threadsafe(future.set_result, None)
    event_loop = asyncio.get_event_loop()
    spin_task = event_loop.create_future()
    spin_thread = threading.Thread(target=_spin, args=(node, spin_task, event_loop))
    spin_thread.start()
    try:
        await spin_task
    except asyncio.CancelledError:
        cancel.trigger()
    spin_thread.join()
    node.destroy_guard_condition(cancel)

async def start_node():
    base_node = RobotBaseNode()
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task_lst = []

    conn = Go2Connection(
        robot_ip=base_node.robot_ip,
        robot_num='0',
        token=base_node.token,
        on_validated=base_node.on_validated,
        on_message=base_node.on_data_channel_message,
    )
    
    sleep_task_lst.append(asyncio.get_event_loop().create_task(base_node.run(conn, '0')))
    #sleep_task_lst.append(asyncio.get_event_loop().create_task(base_node.run(conn)))
    await asyncio.wait([spin_task, *sleep_task_lst], return_when=asyncio.FIRST_COMPLETED)

def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
