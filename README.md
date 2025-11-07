# AGX Orin ROS2 Vision System

A complete robotics vision system using an NVIDIA AGX Orin to process RealSense D4S35i camera data, with SLAM and 3D reconstruction performed on a server.

<img src="https://img.shields.io/badge/Python-3.10+-blue.svg" alt="Python 3.10+"> 
<img src="https://img.shields.io/badge/Ubuntu-22.04-orange.svg" alt="Ubuntu 22.04"> 
<img src="https://img.shields.io/badge/ROS%202-Humble-blueviolet.svg" alt="ROS 2 Humble"> 
<img src="https://img.shields.io/badge/NVIDIA-JetPack%206-brightgreen.svg" alt="JetPack 6"> 
<img src="https://img.shields.io/badge/NVIDIA-CUDA%2012.x-brightgreen.svg" alt="CUDA 12.x"> 
<img src="https://img.shields.io/badge/Build-Passing-success.svg" alt="Build Passing"> 
<img src="https://img.shields.io/badge/License-MIT-yellow.svg" alt="License MIT">

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    AGX Orin (Client)                        │
│  ┌──────────────┐    ┌──────────────┐     ┌──────────────┐  │
│  │  RealSense   │───▶│ H264 Encoder │────▶│ DDS Network  │  │
│  │   D435i      │    │   (NITROS)   │     │  (CycloneDDS)│  │
│  └──────────────┘    └──────────────┘     └──────┬───────┘  │
└──────────────────────────────────────────────────┼──────────┘
                                                   │
           ┌──────────────Network (UDP)────────────┘
           │                                        
┌──────────┼──────────────────────────────────────────────────┐
│          │          Server (SLAM)                           │
│  ┌───────▼──────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ H264 Decoder │───▶│ Visual SLAM  │───▶│    Nvblox     │  │
│  │   (NITROS)   │    │  (cuVSLAM)   │    │  (3D Mapping) │  │
│  └──────────────┘    └──────────────┘    └───────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

## System Requirements & Installation

This table outlines the differences between the AGX Orin (Client) and the Server setup.

| Feature | AGX Orin (Client) | Server (SLAM & Reconstruction) |
| :--- | :--- | :--- |
| **Role** | Camera processing, Encoding, Publishing | Decoding, SLAM, 3D Mapping |
| **Hardware** | NVIDIA Jetson AGX Orin | NVIDIA dGPU (e.g., RTX 30/40 series) |
| **OS** | Ubuntu 22.04 (JetPack 6.x) | Ubuntu 22.04 |
| **GPU Driver** | JetPack Drivers | NVIDIA Driver (CUDA 12.x) |
| **Core Software** | `ros-humble-isaac-ros-h264-encoder` | `ros-humble-isaac-ros-h264-decoder` |
| **ROS** | `realsense-ros` | `ros-humble-nvblox-ros`, `ros-humble-isaac-ros-visual-slam` |
| **DDS** | `cyclonedds` | `cyclonedds` |

### **Detailed Installation Guide**

For a step-by-step installation guide for both Orin and Server, please see the dedicated installation document:

**[Click here for INSTALL.md](https://github.com/CTHMIT/realsense_streaming/INSTALL.md)**

-----

## Configuration

### Network Configuration (CycloneDDS)

| AGX Orin (Client) | Server |
| :--- | :--- |
| `src/client/bringup/config/dds/cyclonedds_orin.xml`: | `src/server/bringup/config/dds/cyclonedds_server.xml`: |
| ` xml <NetworkInterface> <Name>eth0</Name> </NetworkInterface> <Peers> <Peer address="192.168.1.100"/> </Peers>  ` | ` xml <NetworkInterface> <Name>eth0</Name> </NetworkInterface> <Peers> <Peer address="192.168.1.50"/> </Peers>  ` |

### Parameter Configuration

| Component | Configuration File |
| :--- | :--- |
| **RealSense Camera** | `src/client/sensors/config/realsense_params.yaml` |
| **Visual SLAM** | `src/server/perception/config/vslam/vslam_params.yaml` |

-----

## Usage

| AGX Orin (Client) | Server |
| :--- | :--- |
| **Manual Start:** <br> `ros2 launch client orin.launch.py` | **Start SLAM:** <br> `ros2 launch server server.launch.py` |
| **Systemd Service (Recommended):** <br> `sudo systemctl start ros2-camera.service` | **Start Visualization:** <br> `ros2 launch server camera_visualization.launch.py` |
| **Enable on Boot:** <br> `sudo systemctl enable ros2-camera.service` | |

### System Verification & Debugging

| Command | Purpose |
| :--- | :--- |
| `ros2 topic list` | Check all active topics. |
| `ros2 topic hz /camera/color/image_compressed/h264` | Verify camera H264 stream frequency. |
| `ros2 topic echo /visual_slam/tracking/odometry` | Check if SLAM is running and publishing odometry. |
| `rs-enumerate-devices` | Verify RealSense camera is detected by the Orin. |
| `ping <target_ip>` | Test basic network connectivity. |
| `ros2 run tf2_tools view_frames` | Generate a PDF (`frames.pdf`) of the TF tree. |
| `journalctl -u ros2-camera -f` | (Orin) View logs for the `systemd` service. |

-----

## Published Topics

### AGX Orin (Client) Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/camera/color/image_compressed/h264` | `sensor_msgs/CompressedImage` | H264 Compressed Color Image |
| `/camera/infra1/image_compressed/h264`| `sensor_msgs/CompressedImage` | H264 Compressed Infrared 1 Image |
| `/camera/depth/compressed` | `sensor_msgs/CompressedImage` | Compressed Depth Image |
| `/camera/imu` | `sensor_msgs/Imu` | IMU Data |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` | Camera Calibration Info |

### Server Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | SLAM Odometry |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | SLAM Trajectory |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | 3D Mesh |
| `/nvblox_node/map_slice` | `nvblox_msgs/DistanceMapSlice`| Distance Map Slice |

-----

## File Structure

```
realsense_streaming/
├── config/
│   ├── chrony/           # Time synchronization config
│   └── sysctl/           # Kernel network parameters
├── src/
│   ├── client/           # AGX Orin Client
│   │   ├── bringup/      # Launch files and config
│   │   └── sensors/      # RealSense config
│   ├── server/           # Server Side
│   │   ├── bringup/      # Launch files
│   │   └── perception/   # SLAM and Nvblox config
│   ├── robot/            # Robot URDF/XRDF
│   ├── robot_msgs/       # Custom messages
│   └── scripts/          # Install and config scripts
├── INSTALL.md            
├── README.md             
├── requirements.apt      # APT dependencies
├── requirements.txt      # Python dependencies
└── workspace.repos       # VCS dependencies
```

## License

MIT License

## Contact

Maintainer: cthsu [chuntsehsu@gmail.com](mailto:chuntsehsu@gmail.com)


