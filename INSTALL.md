# Installation Guide

This guide covers the parallel installation for both the **AGX Orin (Client)** and the **Server**.

## 1\. Initial Workspace Setup (Both Machines)

First, clone the repository and import dependencies on *both* the Orin and the Server. This step is identical for both machines.

```bash
git clone https://github.com/CTHMIT/realsense_streaming
cd realsense_streaming
vcs import src < workspace.repos
```

-----

## 2\. Parallel Installation

Follow the steps in the table below, executing the commands in the appropriate column for your machine.

| Step | AGX Orin (Client) Installation | Server (SLAM) Installation |
| :--- | :--- | :--- |
| **1. Install Drivers** | **Install JetPack:**<br>(Includes drivers & CUDA for Jetson)<br>` bash sudo apt update sudo apt install nvidia-jetpack  ` | **Install CUDA & cuDNN:**<br>(Requires standard NVIDIA driver)<br>Refer to the official NVIDIA documentation to install CUDA Toolkit 12.x+ and the corresponding cuDNN. |
| **2. Install ROS 2** | **Install ROS 2 Humble:**<br>(Identical for both)<br>` bash sudo apt install software-properties-common sudo add-apt-repository universe sudo apt update && sudo apt install curl -y sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \| sudo apt-key add - sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' sudo apt update sudo apt install ros-humble-desktop  ` | **Install ROS 2 Humble:**<br>(Identical for both)<br>` bash # ... (run the same ROS 2 Humble installation commands as the Orin) ... sudo apt update sudo apt install ros-humble-desktop  ` |
| **3. Install Isaac ROS** | **Run Orin Install Script:**<br>` bash chmod +x src/scripts/setup/install_isaac_ros.sh sudo ./src/scripts/setup/install_isaac_ros.sh  ` | **Install x86\_64 Debian Packages:**<br>` bash sudo apt-get install -y \ ros-humble-isaac-ros-h264-decoder \ ros-humble-isaac-ros-visual-slam \ ros-humble-nvblox-ros  ` |
| **4. System Config** | **Run Setup Script:**<br>` bash chmod +x src/scripts/setup/initial_setup_fixed.sh ./src/scripts/setup/initial_setup_fixed.sh  `<br>Select "**orin**" as the machine type. | **Run Setup Script:**<br>` bash chmod +x src/scripts/setup/initial_setup_fixed.sh ./src/scripts/setup/initial_setup_fixed.sh  `<br>Select "**server**" as the machine type. |
| **5. Build Workspace** | **Build (Jetson Optimized):**<br>` bash source /opt/ros/humble/setup.bash colcon build --symlink-install --parallel-workers 4 source install/setup.bash  ` | **Build (Standard):**<br>` bash source /opt/ros/humble/setup.bash colcon build --symlink-install source install/setup.bash  ` |