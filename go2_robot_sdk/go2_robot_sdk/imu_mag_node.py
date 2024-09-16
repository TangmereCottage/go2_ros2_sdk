#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct

import rclpy # Python Client Library for ROS 2
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node # Handles the creation of nodes
from rclpy.qos import QoSProfile

import time
import math
import sys
import platform
import threading

import serial.tools.list_ports

import logging

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

from std_msgs.msg import String

from tf_transformations import quaternion_from_euler, euler_from_quaternion

def hex_to_short(raw_data):
    return list(struct.unpack("hhh", bytearray(raw_data)))

def hex_to_angle(raw_data):
    return list(struct.unpack("i", bytearray(raw_data)))
    
logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class ImuMagNode(Node):

    # modbus ID
    ADDR = 0x50
    # Start register
    statReg = None
    
    # region Calculate CRC
    auchCRCHi = [
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
        0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
        0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
        0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
        0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
        0x40]

    auchCRCLo = [
        0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
        0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
        0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
        0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
        0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
        0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
        0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
        0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
        0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
        0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
        0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
        0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
        0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
        0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
        0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
        0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
        0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
        0x40]

    def __init__(self):

        super().__init__('imu_mag_node')
        
        self.declare_parameter('wit_imu_port', '/dev/ttyUSB0')
        self.wit_imu_port = self.get_parameter('wit_imu_port').get_parameter_value().string_value
        self.get_logger().info(f"WitMotion port: {self.wit_imu_port}")

        self.declare_parameter('wit_imu_baud', 115200)
        self.wit_imu_baud = self.get_parameter('wit_imu_baud').get_parameter_value().integer_value
        self.get_logger().info(f"WitMotion baud: {self.wit_imu_baud}")
    
        self.iapflag=0

        # Temporary array
        self.TempBytes = []
        self.deviceData = {}

        try:
            self.wt_imu = serial.Serial(port=self.wit_imu_port, baudrate=self.wit_imu_baud, timeout=10)
            if self.wt_imu.isOpen():
                self.wt_imu.close()
                self.get_logger().info("Serial port was already open - restarting")
                self.wt_imu.open()
            else:
                self.wt_imu.open()
                self.get_logger().info("Serial port enabled successfully...")
        except Exception as e:
            print(e)
            self.get_logger().info("Serial port failed to open...")
            exit(0)

        self.imu_msg = Imu()
        self.mag_msg = MagneticField()

        qos_profile = QoSProfile(depth=10)

        self.publisher_imu = self.create_publisher(Imu,           'imu_wit', qos_profile)
        self.publisher_mag = self.create_publisher(MagneticField, 'mag_wit', qos_profile)
          
        # Check the sensor
        timer_period = 0.01
          
        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Send data over serial port
    def sendData(self, data):
        try:
            self.wt_imu.write(data)
        except Exception as ex:
            print(ex)

    # read register
    def readReg(self, regAddr, regCount):
        # Get start register from instruction
        self.statReg = regAddr
        # Encapsulate read instructions and send data to the serial port
        self.sendData(self.get_readBytes(self.ADDR, regAddr, regCount))

    # Send read instruction encapsulation
    def get_readBytes(self, devid, regAddr, regCount):
        tempBytes = [None] * 8
        tempBytes[0] = devid
        tempBytes[1] = 0x03
        tempBytes[2] = regAddr >> 8
        tempBytes[3] = regAddr & 0xff
        tempBytes[4] = regCount >> 8
        tempBytes[5] = regCount & 0xff
        tempCrc = self.get_crc(tempBytes, len(tempBytes) - 2)
        tempBytes[6] = tempCrc >> 8
        tempBytes[7] = tempCrc & 0xff
        return tempBytes
    
    # Obtain CRC verification
    def get_crc(self, datas, dlen):
        tempH = 0xff  # High CRC byte initialization
        tempL = 0xff  # Low CRC byte initialization
        for i in range(0, dlen):
            tempIndex = (tempH ^ datas[i]) & 0xff
            tempH = (tempL ^ self.auchCRCHi[tempIndex]) & 0xff
            tempL = self.auchCRCLo[tempIndex]
        return (tempH << 8) | tempL
        pass

    def set(self, key, value):
        # Saving device data to key values
        self.deviceData[key] = value

    def onDataReceived(self, data):
        tempdata = bytes.fromhex(data.hex())
        for val in tempdata:
            self.TempBytes.append(val)
            # Determine if the ID is correct
            if self.TempBytes[0] != self.ADDR:
                del self.TempBytes[0]
                continue
            # Determine whether it is 03 to read the function code
            if len(self.TempBytes) > 2:
                if not (self.TempBytes[1] == 0x03):
                    del self.TempBytes[0]
                    continue
                tLen = len(self.TempBytes)
                # Get a complete package of protocol data
                if tLen == self.TempBytes[2] + 5:
                    # CRC
                    tempCrc = self.get_crc(self.TempBytes, tLen - 2)
                    if (tempCrc >> 8) == self.TempBytes[tLen - 2] and (tempCrc & 0xff) == self.TempBytes[tLen - 1]:
                        self.processData(self.TempBytes[2])
                    else:
                        del self.TempBytes[0]

    def processData(self, length):
        if length == 30:
            stamp = self.get_clock().now().to_msg()
            
            # What type of sensor is this?

            # Push forwards - x acceleration INCREASES
            # Push right    - y acceleration INCREASES
            # Gravity is NEGATIVE 9.81 
            # So, this is a NED sensor

            # ENU means that the reference frame with respect to which the IMU's orientation is reported is oriented X=East, Y=North, Z=Up. 
            # When the linear acceleration value reads (0, 0, +9.8), i.e. "z is up", 
            # and the magnetometer reads (0, +, *) (i.e., "north is east"), 
            # the orientation reported by the IMU should be neutral 
            # (roll/pitch/yaw all 0, quaternion = (0, 0, 0, 1)).

            # Convention:
            # X is forward
            # Y is left
            # Z is up

            # REP 103 right hand rule
            # Rotation about the y axis (pitch) should be positive in the nose down direction
            # Rotation about the x axis (roll)  should be positive for right side down motion 
            # Rotation about the z axis (yaw)   should be positive for CCW rotation

            # First, we ensure a "standard" IMU
            # X axis is aligned front to back, and X velocity (and acceleration) is POSITIVE when you move forwards
            # When you ROLL RIGHT SIDE DOWN, you get a POSITIVE roll  
            
            # Y axis is aligned left to right, and Y velocity (and acceleration) is POSITIVE when you move __________
            # When you PITCH DOWN, you get a POSITIVE pitch 

            # Z is up/down and gravity is NEGATIVE 9.81
            # When you YAW CCW (left), you get a POSITIVE yaw.
            # Since there is a magnetometer, zero yaw faces NORTH (at least for NED) convention

            self.imu_msg.header.stamp = stamp
            self.imu_msg.header.frame_id = "IMU_wit"

            self.mag_msg.header.stamp = stamp
            self.mag_msg.header.frame_id = "IMU_wit"
            
            # witmotion has flipped x and y axes
            AccY = self.getSignInt16(self.TempBytes[3] << 8 | self.TempBytes[4]) / 32768 * 16
            AccX = self.getSignInt16(self.TempBytes[5] << 8 | self.TempBytes[6]) / 32768 * 16
            AccZ = self.getSignInt16(self.TempBytes[7] << 8 | self.TempBytes[8]) / 32768 * 16
            #self.get_logger().info(f"WIT XYZ AACEL:{AccX} {AccY} {AccZ}")
            
            # also sign of sideways acceleration is flipped
            self.imu_msg.linear_acceleration.x = AccX *  9.80665
            self.imu_msg.linear_acceleration.y = AccY *  9.80665 * -1.0
            self.imu_msg.linear_acceleration.z = AccZ *  9.80665
            self.imu_msg.linear_acceleration_covariance[0] = 0.0002
            self.imu_msg.linear_acceleration_covariance[4] = 0.0002
            self.imu_msg.linear_acceleration_covariance[8] = 0.0002

            # witmotion has flipped x and y axes
            # used to be 2000, reduced to 40 to reconcile with Unitree
            AsY = self.getSignInt16(self.TempBytes[ 9] << 8 | self.TempBytes[10]) / 32768 * 40
            AsX = self.getSignInt16(self.TempBytes[11] << 8 | self.TempBytes[12]) / 32768 * 40
            AsZ = self.getSignInt16(self.TempBytes[13] << 8 | self.TempBytes[14]) / 32768 * 40
            
            # also sign of sideways accel is flipped
            self.imu_msg.angular_velocity.x = AsX 
            self.imu_msg.angular_velocity.y = AsY * -1.0
            self.imu_msg.angular_velocity.z = AsZ
            self.imu_msg.angular_velocity_covariance[0] = 0.0002
            self.imu_msg.angular_velocity_covariance[4] = 0.0002
            self.imu_msg.angular_velocity_covariance[8] = 0.0002            

            # Euler angles
            # witmotion has flipped x and y axes
            AngY = self.getSignInt32(self.TempBytes[23] << 24 | self.TempBytes[24] << 16 | self.TempBytes[21] << 8 | self.TempBytes[22]) / 1000
            AngX = self.getSignInt32(self.TempBytes[27] << 24 | self.TempBytes[28] << 16 | self.TempBytes[25] << 8 | self.TempBytes[26]) / 1000
            AngZ = self.getSignInt32(self.TempBytes[31] << 24 | self.TempBytes[32] << 16 | self.TempBytes[29] << 8 | self.TempBytes[30]) / 1000

            # witmotion roll direction is correct (roll right = positive)
            # witmotion pitch direction is flipped
            AngY = AngY * -1.0
            # witmotion yaw direction is correct (CCW yaw = positive)
            
            self.get_logger().info(f"WIT RPY:{AngX} {AngY} {AngZ}")

            angle_radian = [AngX * math.pi / 180, AngY * math.pi / 180, AngZ * math.pi / 180]
            quaNED = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            # self.get_logger().info(f"WIT QUA xyzw:{quaNED[0]} {quaNED[1]} {quaNED[2]} {quaNED[3]}")
            # def euler_from_quaternion(quaternion):
            #     """
            #     Converts quaternion (w in last place) to euler roll, pitch, yaw
            #     quaternion = [x, y, z, w]
            #     """
            # orientation_list = [float(qua[0]), float(qua[1]), float(qua[2]), float(qua[3])]
            # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            # self.get_logger().info(f"rpy cor:{roll*57} {pitch*57} {yaw*57}")
            
            # for ease of debug WRT Unitree, * -1.0
            # this is does not change the direction, since we are multiplying all values by -1 
            self.imu_msg.orientation.x = quaNED[0] * -1.0
            self.imu_msg.orientation.y = quaNED[1] * -1.0
            self.imu_msg.orientation.z = quaNED[2] * -1.0
            self.imu_msg.orientation.w = quaNED[3] * -1.0
            self.imu_msg.orientation_covariance[0] = 0.0002
            self.imu_msg.orientation_covariance[4] = 0.0002
            self.imu_msg.orientation_covariance[8] = 0.0002
            self.publisher_imu.publish(self.imu_msg)

            # orientation_ENU = [float(quaNED[1]), float(quaNED[0]), -1.0 * float(quaNED[2]), float(quaNED[3])]
            # (roll, pitch, yaw) = euler_from_quaternion(orientation_ENU)
            # self.get_logger().info(f"WIT XYZ RPY ENU:{roll*57} {pitch*57} {yaw*57}")
            
            # The angle runs from +180 to -180 degrees
            # that's flipped compared what you expect
            # First, map MAG to CW 0 to 360 Deg
            AngZ = -1.0 * AngZ
            if (AngZ < 0.0):
                AngZ = 360.0 + AngZ
            
            # Then, correct for variation
            AngT = AngZ - 12.8

            # Finally, smooth TRUE CW 0 to 360 Deg
            if (AngT < 0.0):
                AngT = 360.0 + AngT

            self.get_logger().info(f"Heading (MAG):{AngZ} (TRUE):{AngT}")
            
            # What are the units? Looks like milliT 
            # mT(millit), Gs(Gauss), 1mT=10Gs
            # x: -22.425
            # y: -11.765
            # z: -46.982
            # does not actually matter, as long as the offsets are in the same units
            # this may or may not be internally flipped
            # we do not have because we do not use these data and the quarterion above is correct 
            HX = self.getSignInt16(self.TempBytes[15] << 8 | self.TempBytes[16]) * 13 / 1000
            HY = self.getSignInt16(self.TempBytes[17] << 8 | self.TempBytes[18]) * 13 / 1000
            HZ = self.getSignInt16(self.TempBytes[19] << 8 | self.TempBytes[20]) * 13 / 1000
            self.mag_msg.magnetic_field.x = HX
            self.mag_msg.magnetic_field.y = HY
            self.mag_msg.magnetic_field.z = HZ
            # NOTE - the magnetomer needs to have a correct internal hard iron calibration
            # this is not a ROS2 issue, but needs to be taken care of seperately 
            # depending on the magnetomer and its internal offset registers
            self.publisher_mag.publish(self.mag_msg)

            # convert from NED to ENU
            # swap x and y and flip z

        self.TempBytes.clear()

    @staticmethod
    def getSignInt16(num):
        if num >= pow(2, 15):
            num -= pow(2, 16)
        return num

    @staticmethod
    def getSignInt32(num):
        if num >= pow(2, 31):
            num -= pow(2, 32)
        return num

    def timer_callback(self):
        self.readReg(0x34, 15)
        time.sleep(0.01)
        #print("Sending data request 0x34")
        try:
            buff_count = self.wt_imu.inWaiting()
            #self.get_logger().info(f"Waiting {buff_count}")
            if buff_count > 0:
                buff_data = self.wt_imu.read(buff_count)
                #self.get_logger().info(f"Data {buff_data}")
                self.onDataReceived(buff_data)
        except Exception as e:
            print("exception:" + str(e))
            print("imu loss of connection, poor contact, or broken wire")
            exit(0)

def main(args=None):
  
    global wt_imu
    wt_imu = serial.Serial()

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    imu_mag_node = ImuMagNode()
  
    # Spin the node so the callback function is called.
    rclpy.spin(imu_mag_node)
  
    # Destroy the node explicitly
    imu_mag_node.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
