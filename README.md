# AGX Orin ROS2 Vision System

完整的機器人視覺系統，使用 NVIDIA AGX Orin 處理 RealSense D435i 相機數據，並在服務器上執行 SLAM 和 3D 重建。

## 系統架構

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

## 硬件需求

### AGX Orin (Client)
- NVIDIA Jetson AGX Orin (32GB推薦)
- Intel RealSense D435i
- 網絡連接(有線或WiFi)
- Ubuntu 20.04/22.04 (JetPack 5.x)

### Server
- NVIDIA GPU (RTX 30系列或更高)
- Ubuntu 22.04
- CUDA 11.8+
- 16GB+ RAM

## 軟件依賴

### 核心組件
- ROS 2 Humble
- Isaac ROS (H264, Visual SLAM, Nvblox)
- RealSense SDK 2.0
- CycloneDDS
- CUDA Toolkit

## 安裝步驟

### 1. 準備工作

```bash
# Clone repository
git clone <your-repo-url>
cd robot_vision_system_ws

# Import external dependencies
vcs import src < workspace.repos
```

### 2. AGX Orin 安裝

#### Step 2.1: 安裝 JetPack (如未安裝)

```bash
sudo apt update
sudo apt install nvidia-jetpack
```

#### Step 2.2: 安裝 ROS 2 Humble

```bash
# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | \
  sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
  /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update
sudo apt install ros-humble-desktop
```

#### Step 2.3: 安裝 Isaac ROS

```bash
# Run Isaac ROS installation script
chmod +x src/scripts/setup/install_isaac_ros.sh
sudo ./src/scripts/setup/install_isaac_ros.sh
```

#### Step 2.4: 系統配置

```bash
# Run main setup script
chmod +x src/scripts/setup/initial_setup_fixed.sh
./src/scripts/setup/initial_setup_fixed.sh
```

選擇 "orin" 作為機器類型，並按照提示輸入配置。

#### Step 2.5: 編譯工作空間

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Build
colcon build --symlink-install --parallel-workers 4

# Source workspace
source install/setup.bash
```

### 3. Server 安裝

#### Step 3.1: 安裝 CUDA 和 cuDNN

請參考 NVIDIA 官方文檔安裝 CUDA Toolkit 11.8+ 和 cuDNN。

#### Step 3.2: 安裝 ROS 2 Humble

```bash
# Same as Orin setup
sudo apt install ros-humble-desktop
```

#### Step 3.3: 安裝 Isaac ROS

```bash
# For x86_64 systems, follow Isaac ROS documentation
# https://nvidia-isaac-ros.github.io/getting_started/index.html

# Install from Debian packages
sudo apt-get install -y \
  ros-humble-isaac-ros-h264-decoder \
  ros-humble-isaac-ros-visual-slam \
  ros-humble-nvblox-ros
```

#### Step 3.4: 系統配置

```bash
# Run setup script
./src/scripts/setup/initial_setup_fixed.sh
```

選擇 "server" 作為機器類型。

#### Step 3.5: 編譯工作空間

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 配置

### 網絡配置

#### AGX Orin (Client)
編輯 `src/client/bringup/config/dds/cyclonedds_orin.xml`:
```xml
<NetworkInterface>
    <Name>eth0</Name>  <!-- 更改為你的網絡接口 -->
</NetworkInterface>
<Peers>
    <Peer address="192.168.1.100"/>  <!-- Server IP -->
</Peers>
```

#### Server
編輯 `src/server/bringup/config/dds/cyclonedds_server.xml`:
```xml
<NetworkInterface>
    <Name>eth0</Name>  <!-- 更改為你的網絡接口 -->
</NetworkInterface>
<Peers>
    <Peer address="192.168.1.50"/>  <!-- Orin IP -->
</Peers>
```

### RealSense 配置

編輯 `src/client/sensors/config/realsense_params.yaml` 來調整相機參數。

### SLAM 參數

編輯 `src/server/perception/config/vslam/vslam_params.yaml` 來調整 SLAM 參數。

## 使用方法

### 啟動系統

#### 在 AGX Orin 上:

```bash
# 手動啟動
ros2 launch client orin.launch.py

# 或使用 systemd 服務
sudo systemctl start ros2-camera.service

# 開機自啟
sudo systemctl enable ros2-camera.service
```

#### 在 Server 上:

```bash
ros2 launch server server.launch.py
```

### 驗證運行

```bash
# 檢查話題
ros2 topic list

# 檢查相機數據
ros2 topic hz /camera/color/image_compressed/h264
ros2 topic hz /camera/depth/compressed

# 檢查 SLAM
ros2 topic echo /visual_slam/tracking/odometry

# 檢查 TF
ros2 run tf2_tools view_frames
```

### 可視化

```bash
# 在 Server 上啟動 RViz
ros2 launch server camera_visualization.launch.py

# 或單獨啟動
rviz2
```

## 故障排除

### 相機無法啟動

```bash
# 檢查 RealSense 設備
rs-enumerate-devices

# 檢查 USB 連接
lsusb | grep Intel

# 重置相機
rs-reset
```

### 網絡連接問題

```bash
# 測試 multicast
ros2 multicast receive
# 在另一終端
ros2 multicast send

# 檢查 DDS 配置
echo $CYCLONEDDS_URI

# 檢查防火牆
sudo ufw status

# 測試連接
ping <server_ip>
```

### H264 編解碼問題

```bash
# 檢查 NVIDIA 驅動
nvidia-smi

# 檢查 NVENC/NVDEC
/usr/bin/nvpmodel -q

# 查看日誌
journalctl -u ros2-camera -f
```

### SLAM 不工作

```bash
# 檢查 IMU 數據
ros2 topic echo /camera/imu

# 檢查圖像
ros2 topic echo /camera/infra1/image_raw --no-arr

# 檢查 TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### 性能問題

```bash
# AGX Orin: 檢查性能模式
sudo nvpmodel -q
sudo jetson_clocks --show

# 檢查 CPU/GPU 使用率
htop
tegrastats

# 檢查網絡帶寬
iftop -i eth0
```

## 性能優化

### AGX Orin

```bash
# 設置最大性能
sudo nvpmodel -m 0
sudo jetson_clocks

# 設置風扇最大轉速
echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm
```

### 網絡優化

已在 `config/sysctl/99-ros2-dds.conf` 中配置:
- 增加接收緩衝區大小
- 優化 UDP 分片處理

### DDS 優化

CycloneDDS 配置已優化:
- 禁用 multicast (使用 unicast peers)
- 增加接收緩衝區到 10MB
- 設置最大消息大小為 65500B

## 主要話題

### AGX Orin 發布

| 話題 | 類型 | 描述 |
|------|------|------|
| `/camera/color/image_compressed/h264` | `sensor_msgs/CompressedImage` | H264 壓縮彩色圖像 |
| `/camera/infra1/image_compressed/h264` | `sensor_msgs/CompressedImage` | H264 壓縮紅外圖像1 |
| `/camera/infra2/image_compressed/h264` | `sensor_msgs/CompressedImage` | H264 壓縮紅外圖像2 |
| `/camera/depth/compressed` | `sensor_msgs/CompressedImage` | 壓縮深度圖 |
| `/camera/imu` | `sensor_msgs/Imu` | IMU 數據 |
| `/camera/*/camera_info` | `sensor_msgs/CameraInfo` | 相機標定信息 |

### Server 發布

| 話題 | 類型 | 描述 |
|------|------|------|
| `/visual_slam/tracking/odometry` | `nav_msgs/Odometry` | SLAM 里程計 |
| `/visual_slam/tracking/slam_path` | `nav_msgs/Path` | SLAM 軌跡 |
| `/nvblox_node/mesh` | `nvblox_msgs/Mesh` | 3D 網格 |
| `/nvblox_node/map_slice` | `nvblox_msgs/DistanceMapSlice` | 距離地圖切片 |

## 文件結構

```
robot_vision_system_ws/
├── config/
│   ├── chrony/           # 時間同步配置
│   └── sysctl/           # 內核網絡參數
├── src/
│   ├── client/           # AGX Orin 客戶端
│   │   ├── bringup/      # Launch 文件和配置
│   │   └── sensors/      # RealSense 配置
│   ├── server/           # 服務器端
│   │   ├── bringup/      # Launch 文件
│   │   └── perception/   # SLAM 和 Nvblox 配置
│   ├── robot/            # 機器人 URDF/XRDF
│   ├── robot_msgs/       # 自定義消息
│   └── scripts/          # 安裝和配置腳本
├── requirements.apt      # APT 依賴
├── requirements.txt      # Python 依賴
└── workspace.repos       # VCS 依賴
```

## 開發

### 添加新的感測器

1. 在 `src/client/bringup/launch/orin.launch.py` 添加節點
2. 更新 DDS 配置如需要
3. 重新編譯

### 修改 SLAM 參數

編輯 `src/server/perception/config/vslam/vslam_params.yaml`

### 調試技巧

```bash
# 啟用 ROS 2 日誌
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_USE_STDOUT=1

# 啟用 DDS 調試
export CYCLONEDDS_URI=file:///path/to/config.xml,verbosity=finest
```

## 許可證

MIT License

## 貢獻

歡迎提交 Issues 和 Pull Requests!

## 參考

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [RealSense ROS](https://github.com/IntelRealSense/realsense-ros)
- [CycloneDDS Configuration](https://github.com/eclipse-cyclonedds/cyclonedds)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)

## 聯繫

維護者: cthsu <chuntsehsu@gmail.com>

## 更新日誌

### Version 0.1.0 (2024-11-07)
- 初始版本
- 支持 RealSense D435i
- H264 硬件編解碼
- Visual SLAM
- Nvblox 3D 重建