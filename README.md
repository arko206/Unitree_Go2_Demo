# Unitree_Go2_Demo

A ROS 2 and Unitree Go2 demo repository for AprilTag-based perception, TF estimation, visualization, and robot execution.

This repository contains demo-related packages and scripts used for:
- AprilTag-based robot and obstacle detection
- TF estimation and transformation logging
- RViz-based verification
- Planning and visualization
- Executing the final plan on the Unitree Go2 robot

---
This project implements an AprilTag-based perception and SE(2) planning pipeline for the Unitree Go2 robot. A webcam detects tags placed on the robot's head, obstacles, goal, and floor to estimate robot and object poses in RViz. These poses are used by an RRT planner to generate a collision-free path in an SE(2) environment, and the resulting waypoints are converted into velocity commands for real robot execution.


## Repository contents

This repository currently includes components such as:

- `my_tf_logger/` — TF logging and transformation-related package
- `unitree_sdk2/example/go2/` — Go2 demo scripts, planning files, and execution utilities

You can expand this section later with package-specific details.

---

## Requirements

Before running the demo, make sure your environment is set up with the required dependencies, for example:

- ROS 2
- AprilTag detection pipeline
- RViz
- Unitree Go2 SDK / communication setup
- Docker environment for robot execution
- Any required camera drivers and TF packages

---

## Demo terminal navigation

The live demo uses a 10-terminal workflow. The navigation shortcuts are:

- `t10` — open 10 terminals
- `Ctrl + PageUp` — move to the previous terminal
- `Ctrl + PageDown` — move to the next terminal
---

## Demo startup


## Launch sequence by terminal

- `s1` - Open camera parameters.
- alias s1='ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_1 --params-file /home/arka/Desktop/ros2_ws/src/usb_cam/config/params_3.yaml'
Then press `Ctrl + PageDown`.

- `s2` - Open rectified image parameters.
- alias s2='ros2 run image_proc image_proc --ros-args -r __ns:=/usb_cam_1 -r image:=image_raw'
Then press `Ctrl + PageDown`.

- `s3` - Start AprilTag detection. 
-alias s3='ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/usb_cam_1/image_rect -r camera_info:=/usb_cam_1/camera_info -r detections:=/webcam/detections --params-file /home/arka/Desktop/ros2_ws/src/apriltag_ros/cfg/Demo_tags_36h11.yaml'
Then press `Ctrl + PageDown`.

- `s4` - Launch robot description / robot launcher.
-alias s4='ros2 launch go2_description robot.launch.py'
Then press `Ctrl + PageDown`.

- `s5` - Start mapping between ROS 2 joint names and the launch-file naming.
-alias s5='python3 ~/go2_sim_ws/lowstate_to_jointstate.py' 
Then press `Ctrl + PageDown`.

- `s6` - Run TF estimation.  
-alias s6='ros2 run my_tf_logger demo_test_log --ros-args -p parent_frame:=camera -p child_frame:=object_1 -p intermediate_frame:=object_2 -p goal_frame:=object_3 -p rate_hz:=10.0 -p floor_frame:=floor -p deque_maxlen:=1000'
Then press `Ctrl + PageDown`.

- `s7` - Launch RViz.  
- alias s7='ros2 launch go2_rviz rviz.launch.py'
  Then press `Ctrl + PageDown`.

- `s8` - This step is used after RViz verification and after the `query.cfg` step. 
- alias s8='ros2 run my_tf_logger getting_transformations'
  Then press `Ctrl + PageDown`.


---

## Inside the Docker Container

-`dock` - Launch RViz. 
- alias dock='cd /home/arka/ && docker start -ai MountedGo2_arka_u20'


-`getp` -  Getting Waypoints and Visulaizing
- alias getp = './pla2exec && python3 visualize.py'

-`runp` -  Executing the Waypoints on the Robot
- alias runp = './go2_simple_left_movement enp132s0'
