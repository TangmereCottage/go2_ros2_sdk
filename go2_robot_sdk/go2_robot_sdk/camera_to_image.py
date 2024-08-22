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

import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import Header

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient

import cv2
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import numpy as np
import sys
 
class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):

    super().__init__('image_publisher')
     
    # Publisher will publish an Image to the images topic. 
    # The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'image', 10)
      
    # Publish a message every 0.1 seconds
    timer_period = 0.1
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
      
    # Create a video client   
    self.client = VideoClient()  
    self.client.SetTimeout(3.0)
    self.client.Init()
         
    # Used later on to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):

    # Capture an image
    code, data = self.client.GetImageSample()

    if code != 0:
        self.get_logger().info(f'Video error: {code}')
    else:
        image_data = np.frombuffer(bytes(data), dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        # Here we have the raw image for us to work with 
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS 2 image message
        image_ROS2 = self.br.cv2_to_imgmsg(image, encoding='bgr8')
        image_ROS2.header = Header(frame_id="video_frame") 
        image_ROS2.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(image_ROS2)
        # self.get_logger().info('Publishing video frame')

def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)

  ChannelFactoryInitialize(0, 'eno3')
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()