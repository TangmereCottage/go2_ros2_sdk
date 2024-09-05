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
import time
import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster, TransformStamped

from tf_transformations import euler_from_quaternion
#from transforms3d import euler_from_quaternion

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, Quaternion, Vector3

from sensor_msgs.msg import PointCloud2, PointField, JointState, Joy, Imu
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

#from go2_interfaces.msg import Go2State, IMU

from unitree_go.msg import LowState
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import (
    SportClient,
    PathPoint,
    SPORT_PATH_POINT_SIZE,
)

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class RobotBaseNode(Node):

    def __init__(self):
        super().__init__('go2_driver_node')

        self.declare_parameter('robot_ip', os.getenv('ROBOT_IP', os.getenv('GO2_IP')))
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.get_logger().info(f"Received ip: {self.robot_ip}")

        # self.conn = {}
        qos_profile = QoSProfile(depth=10)

        # movement
        # Time count
        self.t = 0
        self.dt = 0.01

        # Initial position and yaw
        self.px0 = 0
        self.py0 = 0
        self.yaw0 = 0

        self.client = SportClient()  # Create a sport client
        self.client.SetTimeout(10.0)
        self.client.Init()

        self.joint_pub = []
        #self.go2_state_pub = []
        self.lidar_pub = []
        self.odometry_pub = []
        self.imu_pub = []

        self.joint_pub.append(self.create_publisher(JointState, 'joint_states', qos_profile))
        #self.go2_state_pub.append(self.create_publisher(Go2State, f'SDK/robot0/go2_states', qos_profile))
        self.lidar_pub.append(self.create_publisher(PointCloud2, '/bits/point_cloud2', qos_profile))
        self.odometry_pub.append(self.create_publisher(Odometry, '/bits/odom_int', qos_profile))
        self.imu_pub.append(self.create_publisher(Imu,           '/bits/imu_int', qos_profile))

        # Broadcast TF information
        # this does all the TF, probably
        # needs the profile and the pose data  
        self.TF_broadcaster = TransformBroadcaster(self, qos=qos_profile)

        self.robot_cmd_vel = {}
        self.robot_odom = {}
        self.robot_low_cmd = {}
        self.robot_sport_state = {}
        self.robot_lidar = {}

        # control overview
        # /joy            # from joystick
        # /cmd_vel_out    # from the mux, flowing into the move command
        # /cmd_vel_joy    # from joystick
        # /cmd_vel_nav    # from nv2
        self.create_subscription(Twist, 'cmd_vel_out', lambda msg: self.cmd_vel_cb(msg), qos_profile)

        self.joy_state = Joy()
        self.create_subscription(Joy, 'joy', self.joy_cb, qos_profile)

        # power, temp, IMU state
        # this feeds into the joint_states, which are needed for the TF
        self.create_subscription(
            LowState,
            'lowstate',
            self.publish_joint_state_imu_cyclonedds,
            qos_profile)

        # frame_id: odom
        # postion/orientation
        self.create_subscription(
            PoseStamped,
            '/utlidar/robot_pose',
            self.publish_body_poss_cyclonedds,
            qos_profile)

        # frame_id: odom
        # straight passthrough 
        self.create_subscription(
            Odometry,
            '/utlidar/robot_odom',
            self.publish_odom_cyclonedds,
            qos_profile)

        # subscribe to the raw LIDAR data
        # frame_id: odom already
        # but we do not know how good that is?
        self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_deskewed',
            self.publish_lidar_cyclonedds,
            qos_profile)

    # subscribe to /joy and send those data to self.joy_state
    def joy_cb(self, msg):
        self.joy_state = msg

    def GetInitState(self, robot_state: SportModeState_):
        self.px0 = robot_state.position[0]
        self.py0 = robot_state.position[1]
        self.yaw0 = robot_state.imu_state.rpy[2]

    # subscribe to Twist /cmd_vel_out and if there are non-zero values, move the dog
    # movement is based on /cmd_vel_out and not directly on /joy
    # so the movement data must flow from /joy to someone else and then to /cmd_vel_out 
    def cmd_vel_cb(self, msg):
        x = round(msg.linear.x, 2)
        y = round(msg.linear.y, 2)
        z = round(msg.angular.z, 2)
        deadband = 0.02
        if abs(x) > deadband or abs(y) > deadband or abs(z) > deadband:
            self.get_logger().info(f"Move {x, y, z}")
            self.client.Move(x, y, z)  # vx, vy vyaw
            time.sleep(0.1)
            self.client.StopMove()

    # this is body pose and is being sent to TF
    # coming from '/utlidar/robot_pose'
    def publish_body_poss_cyclonedds(self, msg):
        odom_trans = TransformStamped()
        odom_trans.header.stamp = msg.header.stamp #self.get_clock().now().to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = msg.pose.position.x
        odom_trans.transform.translation.y = msg.pose.position.y
        odom_trans.transform.translation.z = msg.pose.position.z
        odom_trans.transform.rotation.x    = msg.pose.orientation.x
        odom_trans.transform.rotation.y    = msg.pose.orientation.y
        odom_trans.transform.rotation.z    = msg.pose.orientation.z
        odom_trans.transform.rotation.w    = msg.pose.orientation.w
        self.TF_broadcaster.sendTransform(odom_trans)

    # joint states are angles and therefore do not have/need a frame_id 
    def publish_joint_state_imu_cyclonedds(self, msg):
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
        self.joint_pub[0].publish(joint_state)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'IMU'
        
        gyro = Vector3()
        gyro.x = float(msg.imu_state.gyroscope[0])
        gyro.y = float(msg.imu_state.gyroscope[1])
        gyro.z = float(msg.imu_state.gyroscope[2])
        imu_msg.angular_velocity = gyro
        imu_msg.angular_velocity_covariance[0] = 0.00001
        imu_msg.angular_velocity_covariance[4] = 0.00001
        imu_msg.angular_velocity_covariance[8] = 0.00001

        accel = Vector3()
        accel.x = float(msg.imu_state.accelerometer[0])
        accel.y = float(msg.imu_state.accelerometer[1])
        accel.z = float(msg.imu_state.accelerometer[2])
        imu_msg.linear_acceleration = accel
        imu_msg.linear_acceleration_covariance[0] = 0.00001
        imu_msg.linear_acceleration_covariance[4] = 0.00001
        imu_msg.linear_acceleration_covariance[8] = 0.00001

        quat_w, quat_x, quat_y, quat_z = msg.imu_state.quaternion
        imu_msg.orientation.x = float(quat_x)
        imu_msg.orientation.y = float(quat_y)
        imu_msg.orientation.z = float(quat_z)
        imu_msg.orientation.w = float(quat_w)
        imu_msg.orientation_covariance[0] = 0.00001
        imu_msg.orientation_covariance[4] = 0.00001
        imu_msg.orientation_covariance[8] = 0.00001

        orientation_list = [float(quat_x), float(quat_y), float(quat_z), float(quat_w)]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # self.get_logger().info(f"Internal (RAD) Roll:{roll} Pitch:{pitch} Yaw:{yaw}")

        # these are the same thing!
        rolli  = msg.imu_state.rpy[0]
        pitchi = msg.imu_state.rpy[1]
        yawi   = msg.imu_state.rpy[2]
        # self.get_logger().info(f"InternalQ (DEG) Roll:{roll*57.2958} Pitch:{pitch*57.2958} Yaw:{yaw*57.2958}")
        # self.get_logger().info(f"InternalD (DEG) Roll:{rolli*57.2958} Pitch:{pitchi*57.2958} Yaw:{yawi*57.2958}")

        self.imu_pub[0].publish(imu_msg)

    def publish_lidar_cyclonedds(self, msg):
        # NOTE - we are listening to /utlidar/cloud_deskewed - that uses odom already
        # msg.header = Header(frame_id="odom") 
        # no need because /utlidar/cloud_deskewed directly uses odom
        # msg.header.stamp = self.get_clock().now().to_msg()
        # no need because the time should be good
        self.lidar_pub[0].publish(msg)

    def publish_odom_cyclonedds(self, msg):
        self.odometry_pub[0].publish(msg)

    def joy_cmd(self):
        # Sit Down
        # Button B
        if self.joy_state.buttons and self.joy_state.buttons[1]:
            self.get_logger().info("Sit down")
            self.client.StandDown()

        # Stand Up
        # Button A
        if self.joy_state.buttons and self.joy_state.buttons[0]:
            self.get_logger().info("Stand up")
            self.client.StandUp()
            time.sleep(0.3)        
            self.client.Euler(0.0, 0.0, 0.2)  # roll, pitch, yaw
            time.sleep(0.3)  
            self.client.BalanceStand()

    # def on_validated(self):
    #     for topic in RTC_TOPIC.values():
    #         self.get_logger().info(f"topic: {topic}")

    # # add messages to lidar/odom etc
    # def on_data_channel_message(self, _, msg):
        
    #     self.get_logger().info(f"message: {msg}")
        
    #     if msg.get('topic') == RTC_TOPIC["ULIDAR_ARRAY"]:
    #         self.robot_lidar = msg

    #     if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
    #         self.robot_odom = msg

    #     if msg.get('topic') == RTC_TOPIC['LF_SPORT_MOD_STATE']:
    #         self.robot_sport_state = msg

    #     if msg.get('topic') == RTC_TOPIC['LOW_STATE']:
    #         self.robot_low_cmd = msg

    # async def run(self, conn, robot_num):
    #     self.conn[robot_num] = conn
    #     while True:
    #         self.joy_cmd()
    #         await asyncio.sleep(0.1)

    async def run(self):
        while True:
            # deals with standing up and sitting down only 
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

# Robot state
robot_state = unitree_go_msg_dds__SportModeState_()
def HighStateHandler(msg: SportModeState_):
    global robot_state
    robot_state = msg

async def start_node():
    
    ChannelFactoryInitialize(0, 'eno3')
    
    sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    sub.Init(HighStateHandler, 10)
    time.sleep(1)

    base_node = RobotBaseNode()
    spin_task = asyncio.get_event_loop().create_task(spin(base_node))
    sleep_task_lst = []    
    sleep_task_lst.append(asyncio.get_event_loop().create_task(base_node.run()))
    await asyncio.wait([spin_task, *sleep_task_lst], return_when=asyncio.FIRST_COMPLETED)

def main():
    rclpy.init()
    asyncio.get_event_loop().run_until_complete(start_node())
    asyncio.get_event_loop().close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()