
```shell
source install/setup.bash
export ROBOT_IP="robot_ip" #for multiple robots, just split by ,
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py



```colcon build```
```

The `lowstate` imu does not have a magnetometer, and reports data in the `ENU` convention 

```shell
ros2 topic echo /lowstate

imu_state:
  quaternion:             // yaw pitch roll
  - -0.9998785257339478
  - -0.0007873051799833775
  - 0.0032413762528449297
  - 0.015227883122861385
  gyroscope:               // rotation (angular velocity - rad/s)
  - 0.0074568502604961395  
  - -0.012783171609044075
  - -0.015978964045643806
  accelerometer:           // linear acceleration (m/s^2)
  - -0.05746084079146385
  - -0.05147533491253853
  - 9.581595420837402
  rpy:                     // roll pitch yaw
  - 0.0016731732757762074
  - -0.0064580319449305534
  - -0.030462516471743584
  temperature: 80
```


### Internal IMU

Convention / Direction

Angular Velocity
Rotate around yaw (z) 
CCW gives POSITIVE angular velocity
CW  gives NEGATIVE angular velocity

Linear Acceleration
Move robot side to side (y axis)
Move LEFT - NEGATIVE 
Move RIGHT - POSITIVE 

Move robot forwards/backwards (x axis)
Move FWD - POSITIVE 
Move BACK - NEGATIVE 


```shell
ros2 topic echo /bits/odometry/gps

header:
  stamp:
    sec: 1725571868
    nanosec: 37879040
  frame_id: odom
child_frame_id: ''
pose:
  pose:
    position:
      x: -1.2766470423274532
      y: 5.24541326190924
      z: 0.0
...
```

```shell
ros2 topic echo /bits/gps/filtered

Absolute coordiantes

header:
  stamp:
    sec: 1725571982
    nanosec: 221003160
  frame_id: base_link
status:
  status: 2
  service: 0
latitude: 1.234567892
longitude: 12.3456789
altitude: 23.477809265276257
position_covariance:
- 0.0
```








![Ros2 SDK](https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/49edebbe-11b6-49c6-b82d-bc46257674bd)

# Welcome to the Unitree Go2 ROS2 SDK Project!

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
![ROS2 Build](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
[![License](https://img.shields.io/badge/license-BSD--2-yellow.svg)](https://opensource.org/licenses/BSD-2-Clause)

We are happy to present you our integration of the Unitree Go2 with ROS2 over Wi-Fi, that was designed by the talented [@tfoldi](https://github.com/tfoldi). You can explore his groundbreaking work at [go2-webrtc](https://github.com/tfoldi/go2-webrtc).

This repo will empower your Unitree GO2 AIR/PRO/EDU robots with ROS2 capabilities, using both WebRTC (Wi-Fi) and CycloneDDS (Ethernet) protocols.

If you are using WebRTC (Wi-Fi) protocol, close the connection with a mobile app before connecting to the robot.

## Project RoadMap:

1. URDF :white_check_mark: 
2. Joint states sync in real time :white_check_mark: 
3. IMU sync in real time :white_check_mark: 
4. Joystick control in real time :white_check_mark: 
6. Go2 topics info in real time :white_check_mark: 
7. Foot force sensors info in real time :white_check_mark: 
8. Lidar stream (added pointCloud2) :white_check_mark: 
9. Camera stream :white_check_mark:
10. Foxglove bridge :white_check_mark:
11. Laser Scan :white_check_mark:
12. Multi robot support :white_check_mark:
13. WebRTC and CycloneDDS support :white_check_mark:
14. Creating a PointCloud map and store it :white_check_mark:
15. SLAM (slam_toolbox) :white_check_mark:
16. Navigation (nav2) :white_check_mark:
17. Object detection
18. AutoPilot

## Your feedback and support mean the world to us. 

If you're as enthusiastic about this project as we are, please consider giving it a :star: star!!!

Your encouragement fuels our passion and helps us develop our RoadMap further. We welcome any help or suggestions you can offer!

Together, let's push the boundaries of what's possible with the Unitree Go2 and ROS2!

## Exciting Features:

:sparkles: Full ROS2 SDK support for your Unitree GO2

:robot: Compatible with AIR, PRO, and EDU variants

:footprints: Access to foot force sensor feedback (available on some Go2 PRO models or EDU)

## Real time Go2 Air/PRO/EDU joints sync:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/bf3f5a83-f02b-4c78-a7a1-b379ce057492" alt='Go2 joints sync'>
</p>

## Go2 Air/PRO/EDU lidar point cloud:

<p align="center">
<img width="1280" height="640" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/9c1c3826-f875-4da1-a650-747044e748e1" alt='Go2 point cloud'>
</p>

## System requirements

Tested systems and ROS2 distro
|systems|ROS2 distro|Build status
|--|--|--|
|Ubuntu 22.04|iron|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|humble|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
|Ubuntu 22.04|rolling|![ROS2 CI](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)

## Installation

```shell
mkdir -p ros2_ws
cd ros2_ws
git clone --recurse-submodules https://github.com/abizovnuralem/go2_ros2_sdk.git src
sudo apt install ros-$ROS_DISTRO-image-tools
sudo apt install ros-$ROS_DISTRO-vision-msgs
sudo apt install python3-pip clang
pip install -r requirements.txt
cd ..
```
Pay attention to any error messages. If `pip install` does not complete cleanly, various features will not work. For example, `open3d` does not yet support `python3.12` and therefore you will need to set up a 3.11 `venv` first etc.

Install `rust` language support following these [instructions](https://www.rust-lang.org/tools/install). Then, install version 1.79 of `cargo`, the `rust` package manager.
```shell
rustup install 1.79.0
rustup default 1.79.0
```

`cargo` should now be available in the terminal:
```shell
cargo --version
```

Build `go2_ros_sdk`. You need to have `ros2` and `rosdep` installed. If you do not, follow these [instructions](https://docs.ros.org/en/humble/Installation.html). Then:
```shell
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

## Usage

Don't forget to set up your Go2 robot in Wifi-mode and obtain the IP. You can use the mobile app to get it. Go to Device -> Data -> Automatic Machine Inspection and look for STA Network: wlan0.

```shell
source install/setup.bash
export ROBOT_IP="robot_ip" #for multiple robots, just split by ,
export CONN_TYPE="webrtc"
ros2 launch go2_robot_sdk robot.launch.py
```

The `robot.launch.py` code starts many services/nodes simultaneously, including 

* robot_state_publisher
* ros2_go2_video (front color camera)
* pointcloud_to_laserscan_node
* go2_robot_sdk/go2_driver_node
* go2_robot_sdk/lidar_to_pointcloud
* rviz2
* `joy` (ROS2 Driver for Generic Joysticks and Game Controllers)
* `teleop_twist_joy` (facility for tele-operating Twist-based ROS2 robots with a standard joystick. Converts joy messages to velocity commands)       
* `twist_mux` (twist_multiplexer with source prioritization)        
* foxglove_launch (launches the foxglove bridge)
* slam_toolbox/online_async_launch.py
* av2_bringup/navigation_launch.py

When you run `robot.launch.py`, `rviz` will fire up, lidar data will begin to accumulate, the front color camera data will be displayed too (typically after 4 seconds), and your dog will be waiting for commands from your joystick (e.g. a X-box controller). You can then steer the dog through your house, e.g., and collect LIDAR mapping data. 

### SLAM and Nav2

![Initial Rviz Display](doc_images/slam_nav_map.png)

The goal of SLAM overall, and the `slam_toolbox` in particular, is to create a map. The `slam_toolbox` is a grid mapper - it thinks about the world in terms of a fixed grid that the dog operates in. When the dog initially moves through a new space, data accumulate and the developing map is and published it to the `/map` topic. The goal of `Nav2` is to navigate and perform other tasks in this map.

The `rviz` settings that are used upon initial launch (triggered by `ros2 launch go2_robot_sdk robot.launch.py`) showcase various datastreams.  

* `RobotModel` is the dimensionally correct model of the G02 
* `PointCloud2` are the raw LIDAR data transformed into 3D objects/constraints 
* `LaserScan` are lower level scan data before translation into an x,y,z frame
* `Image` are the data from the front-facing color camera 
* `Map` is the map being created by the `slam_toolbox`
* `Odometry` is the history of directions/movements of the dog

If there is too much going on in the initial screen, deselect the `map` topic to allow you to see more.

![Simplified Rviz Display](doc_images/slam_nav.png)

### Mapping - creating your first map

Use painter's tape to mark a 'dock' rectangle (or use a real dock) to create a defined starting point for your dog on your floor. In the `rviz` `SlamToolboxPlugin`, on the left side of the your `rviz` screen, select "Start At Dock". Then, use your controller to manually explore a space, such as a series of rooms. You will see the map data accumulating in `rviz`. In this map, white, black and grey pixels represent the free, occupied, and unknown space, respectively. When you are done mapping, enter a file name into the "Save Map" field and click "Save Map". Then enter a file name into "Serialize Map" field and click "Serialize Map". Now, you should have 2 new files in `/ros2_ws`:

```shell
map_1.yaml: the metadata for the map as well as the path to the .pgm image file.
map_1.pgm: the image file with white, black and grey pixels representing the free, occupied, and unknown space.
map_1.data: 
map_1.posegraph: 
```

The next time you start the system, the map can be loaded and is ready for you to complete/extend by mapping more spaces. Upon restart and loading a map, the dog does not know where it is relative to the map you created earlier. Assuming you rebooted the dog in its marked rectangle, or in an actual dock, it will have a high quality initial position and angle.  

### Autonomous Navigation - navigating in your new map

As shown in the `rviz` `Navigation 2` plugin, the system will come up in:

```shell
Navigation: active
Localization: inactive
Feedback: unknown
```

Then, load your map via the `SlamToolboxPlugin` (enter your map's filename (without any extension) in the 'Deserialize Map' field and then click 'Deserialize Map'). 

**WARNING**: please make sure that (1) the dog is correctly oriented WRT to the map and (2) the map itself is sane and corresponds to your house. Especially if you have long corridors, the overall map can be distorted relative to reality, and this means that the route planner will try to route your dog through walls, leaving long scratches in your walls. 

You can now give the dog its first target, via 'Nav2 Goal' in the `rviz` menu. Use the mouse cursor to provide a target to navigate to.

**NOTE**: the `Nav2 Goal` cursor sets both the target position and the final angle of the dog, that you wish the dog to adopt upon reaching the target (need to double check). The long green arrow that is revealed when you click an point and keep moving your mouse cursor is the angle setter. 

Until you have some experience, we suggest following your dog and picking it up when it is about to do something silly.

**NOTE**: Virtually all fault behaviors - spinning in circles, running into walls, trying to walk through walls, etc reflect (1) a map that is incorrect, (2) incorrect initial position/angle of the dog relative to that map, or (3) inability to compute solutions/paths based on overloaded control loops. To prevent #3, which results in no motion or continuous spinning, the key loop rates (`controller_frequency`: 3.0 and `expected_planner_frequency`: 1.0 have been set to very conservative rates). 

## Real time image detection and tracking

This capability is directly based on [J. Francis's work](https://github.com/jfrancis71/ros2_coco_detector). Launch the `go2_ro2_sdk`. After a few seconds, the color image data will be available at `go2_camera/color/image`. On another terminal enter:

```bash
source install/setup.bash
ros2 run coco_detector coco_detector_node
```

There will be a short delay the first time the node is run for PyTorch TorchVision to download the neural network. You should see a download progress bar. TorchVision cached for subsequent runs.

On another terminal, to view the detection messages:
```shell
source install/setup.bash
ros2 topic echo /detected_objects
```
The detection messages contain the detected object (`class_id`) and the `score`, a number from 0 to 1. For example: `detections:results:hypothesis:class_id: giraffe` and `detections:results:hypothesis:score: 0.9989`. The `bbox:center:x` and `bbox:center:y` contain the centroid of the object in pixels. These data can be used to implement real-time object following for animals and people. People are detected as `detections:results:hypothesis:class_id: person`.

To view the image stream annotated with the labels and bounding boxes:
```shell
source install/setup.bash
ros2 run image_tools showimage --ros-args -r /image:=/annotated_image
```

Example Use:
```shell
ros2 run coco_detector coco_detector_node --ros-args -p publish_annotated_image:=False -p device:=cuda -p detection_threshold:=0.7
```

This will run the coco detector without publishing the annotated image (it is True by default) using the default CUDA device (device=cpu by default). It sets the detection_threshold to 0.7 (it is 0.9 by default). The detection_threshold should be between 0.0 and 1.0; the higher this number the more detections will be rejected. If you have too many false detections try increasing this number. Thus only Detection2DArray messages are published on topic /detected_objects.

## 3D raw pointcloud dump

To save raw LIDAR data, `export` the following:

```shell
export MAP_SAVE=True
export MAP_NAME="3d_map"
```

Every 10 seconds, pointcloud data (in `.ply` format) will be saved to the root folder of the repo. **NOTE**: This is _not_ a Nav2 map but a raw data dump of LIDAR data useful for low-level debugging. 

## Multi robot support 
If you want to connect several robots for collaboration:

```shell
export ROBOT_IP="robot_ip_1, robot_ip_2, robot_ip_N"
```

## Switching between webrtc connection (Wi-Fi) to CycloneDDS (Ethernet)

```shell
export CONN_TYPE="webrtc"
```
or
```
export CONN_TYPE="cyclonedds"
```

## Foxglove

<p align="center">
<img width="1200" height="630" src="https://github.com/abizovnuralem/go2_ros2_sdk/assets/33475993/f0920d6c-5b7a-4718-b781-8cfa03a88095" alt='Foxglove bridge'>
</p>

To use Foxglove, you need to install Foxglove Studio:
```
sudo snap install foxglove-studio
```

1. Open Foxglove Studio and press "Open Connection".
2. In the "Open Connection" settings, choose "Foxglove WebSocket" and use the default configuration ws://localhost:8765, then press "Open".

## WSL 2

If you are running ROS2 under WSL2 - you may need to configure Joystick\Gamepad to navigate the robot.

1. Step 1 - share device with WSL2

    Follow steps here https://learn.microsoft.com/en-us/windows/wsl/connect-usb to share your console device with WSL2

2. Step 2 - Enable WSL2 joystick drivers

    WSL2 does not come by default with the modules for joysticks. Build WSL2 Kernel with the joystick drivers. Follow the instructions here: https://github.com/dorssel/usbipd-win/wiki/WSL-support#building-your-own-wsl-2-kernel-with-additional-drivers  If you're comfortable with WSl2, skip the export steps and start at `Install prerequisites.`

    Before buiding, edit `.config` file and update the CONFIG_ values listed in this GitHub issue: https://github.com/microsoft/WSL/issues/7747#issuecomment-1328217406

2. Step 3 - Give permissions to /dev/input devices

    Once you've finished the guides under Step 3 - you should be able to see your joystick device under /dev/input

    ```bash
    ls /dev/input
    by-id  by-path  event0  js0
    ```

    By default /dev/input/event* will only have root permissions, so joy node won't have access to the joystick

    Create a file `/etc/udev/rules.d/99-userdev-input.rules` with the following content:
    `KERNEL=="event*", SUBSYSTEM=="input", RUN+="/usr/bin/setfacl -m u:YOURUSERNAME:rw $env{DEVNAME}"`

    Run as root: `udevadm control --reload-rules && udevadm trigger`

    https://askubuntu.com/a/609678

3. Step 3 - verify that joy node is able to see the device properly. 

    Run `ros2 run joy joy_enumerate_devices`

    ```
    ID : GUID                             : GamePad : Mapped : Joystick Device Name
    -------------------------------------------------------------------------------
    0 : 030000005e040000120b000007050000 :    true :  false : Xbox Series X Controller
    ```

## Thanks

Special thanks to @tfoldi, @legion1581, @budavariam, @alex.lin and TheRoboVerse community!

## License

This project is licensed under the BSD 2-clause License - see the [LICENSE](https://github.com/abizovnuralem/go2_ros2_sdk/blob/master/LICENSE) file for details.


export 192.168.123.99

export ROBOT_IP=192.168.123.99

```shell
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## The setup and transforms 

### GPS

```shell
source install/setup.bash
colcon build
ros2 run gpsx gps_node
ros2 topic echo gpsx 
```
```
longitude: -122.11874700000001
latitude: 37.40014516666667
altitude: 29.1
ground_speed: 0.04899999871850014
satellites: 12
mode_indicator: 1
separation: -30.0
true_course: .nan
true_course_magnetic: .nan
dilution: 0.7699999809265137
utc_time: 191946
```

We three sort of different position/orientation estiamtes 

### /odometry/filtered

This is coming from the EKF, and that uses the imu and the /odom
So really, this should duplicate the /odom

### /utlidar/robot_odom AKA /odom

this is the same as /pose, except that is also has twist data (linear and angular velocities)


header:
  stamp:
    sec: 1725132789
    nanosec: 87844026
  frame_id: odom
child_frame_id: base_link
pose:
  pose:
    position:
      x: -0.030529789626598358
      y: 1.1890190839767456
      z: 0.30895406007766724
    orientation:
      x: -0.008451441302895546
      y: 0.0017630413640290499
      z: 0.37821388244628906
      w: -0.9256780743598938
  covariance:
  - 0.0
twist:   // these are linear and angular velocities
  twist:
    linear:
      x: -0.008428853936493397
      y: 0.0024671705905348063
      z: -0.016539515927433968
    angular:
      x: -0.019174758344888687
      y: -0.0074568502604961395
      z: 0.0031957929022610188
  covariance:
  - 0.0

### /utlidar/robot_pose - this goes to the transform broadcaster and then results in TF

header:
  stamp:
    sec: 1725132698
    nanosec: 117847230
  frame_id: odom
pose:
  position:
    x: -0.0911208987236023  // absolute postion in the x odom frame in m
    y: 1.0456092357635498   //
    z: 0.30894628167152405  // 
  orientation:
    x: -0.008235638029873371 //
    y: 0.0011225291527807713 // 
    z: 0.295285701751709     // left/right in odom frame
    w: -0.9553729295730591   // changes too

### /utlidar/imu

header:
  stamp:
    sec: 1725132650
    nanosec: 926952838
  frame_id: utlidar_imu
orientation:
  x: -0.9149233102798462
  y: -0.3784295916557312
  z: -0.02151946909725666
  w: 0.12598009407520294
orientation_covariance:
- 0.0
angular_velocity:
  x: -0.0027744758408516645
  y: 0.014536685310304165
  z: 0.006824033334851265
angular_velocity_covariance:
- 0.0
linear_acceleration:
  x: 1.7145205736160278
  y: -1.8578453063964844
  z: -9.176068305969238
linear_acceleration_covariance:
- 0.0


ros2 topic echo /lowstate

imu_state:
  quaternion:             // yaw pitch roll
  - -0.9998785257339478
  - -0.0007873051799833775
  - 0.0032413762528449297
  - 0.015227883122861385
  gyroscope:
  - 0.0074568502604961395  // rotations
  - -0.012783171609044075
  - -0.015978964045643806
  accelerometer:           // linear acceleration
  - -0.05746084079146385
  - -0.05147533491253853
  - 9.581595420837402
  rpy:                     // roll pitch yaw
  - 0.0016731732757762074
  - -0.0064580319449305534
  - -0.030462516471743584
  temperature: 80



header:
  stamp:
    sec: 1725213719
    nanosec: 413800001
  frame_id: utlidar_imu
orientation:
  x: -0.980896532535553
  y: -0.1354389786720276
  z: -0.05633513256907463
  w: 0.1138082891702652

imu_state:
  quaternion:
  - -0.9978463649749756
  - -0.0005274121649563313
  - 0.0032289731316268444
  - 0.06551428139209747
  gyroscope:
  - 0.0
  - 0.008522114716470242
  - -0.021305285394191742
  accelerometer:
  - -0.12210428714752197
  - -0.043095629662275314
  - 9.586383819580078
  rpy:
  - 0.0014756706077605486
  - -0.0063749756664037704
  - -0.13112790882587433
  temperature: 79



https://github.com/ElettraSciComp/witmotion_IMU_ros.git


cd ros2_ws/src
git clone -b ros2 --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git witmotion_ros
colcon build --packages-select witmotion_ros
source install/setup.bash




**A ROS2 node for NMEA GPS**

This node is implementing basic telegrams for NMEA GPS communicating via serial lines (NMEA-0183). It publishes a message providing more information than the standard gps message (speed etc.). It also provides a service to read the observed satellites including signal strengthes.
It is currently developed and tested with ROS2 Foxy Fitzroy and Humble Hawksbill on Linux. Used devices were several Garmin and a u-blox based device.

**Build the node with:**

  ```colcon build```

**Run the node with:**

  ```ros2 run gpsx gps_node```

Additional parameters can be added for connection speed and connection port e.g.

```ros2 run gpsx gps_node --ros-args -p "comm_port:=/dev/ttyUSB1" -p "comm_speed:=9600"```



utlidar/robot_pose 

position
orientation

utlidar/robot_odom

position
orientation
twist-linear
twist-angualar

utlidar/imu is different!

