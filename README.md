# Unitree_Go2_Demo

A ROS 2 and Unitree Go2 demo repository for AprilTag-based perception, TF estimation, visualization, and robot execution.

This repository contains demo-related packages and scripts used for:
- AprilTag-based robot and obstacle detection
- TF estimation and transformation logging
- RViz-based verification
- Planning and visualization
- Executing the final plan on the Unitree Go2 robot

---

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
- `Ctrl + PageUp` — move to the next terminal
- `Ctrl + PageDown` — move to the previous terminal

In the original notes, **CP** means `Ctrl + PageUp`. :contentReference[oaicite:1]{index=1}

---

## Demo startup

1. Open a terminal.
2. Run:

```bash
t10
