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

- 'S1'
Open camera parameters.
- alias s1='ros2 run usb_cam usb_cam_node_exe --ros-args --remap __ns:=/usb_cam_1 --params-file /home/arka/Desktop/ros2_ws/src/usb_cam/config/params_3.yaml'
Then press `Ctrl + PageUp`.

### S2
Open rectified image parameters.  
Then press `Ctrl + PageUp`.

### S3
Start AprilTag detection.  
Then press `Ctrl + PageUp`.

### S4
Launch robot description / robot launcher.  
Then press `Ctrl + PageUp`.

### S5
Start mapping between ROS 2 joint names and the launch-file naming.  
Then press `Ctrl + PageUp`.

### S6
Run TF estimation.  
Then press `Ctrl + PageUp`.

### S7
Launch RViz.  
Verify the visualization after launch.

### S8
Get transformation values.  
This step is used after RViz verification and after the `query.cfg` step.

---

## Verification and execution workflow

After **S7**, confirm that RViz can find and display the expected objects, especially the AprilTag.

Then return to the terminal window, move to the next terminal, and run the `query.cfg` step before **S8**.

After **S8**, enter the Docker container using:

```bash
dock

getp

runp
