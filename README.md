RealSense 串流與 3D 重建專案 (realsense_streaming)

本專案旨在使用 NVIDIA AGX Orin 作為客戶端 (Client) 擷取 Intel RealSense D435i 感測器數據，並透過 WiFi 將壓縮後的數據流傳輸到伺KING 伺服器 (Server) 進行即時的 VSLAM 和 3D 重建 (Nvblox)。

專案架構

src/client: Orin 端的 ROS 2 套件。

bringup: 啟動檔 (orin.launch.py)、DDS 設定、systemd 服務。

sensors: Realsense D435i 的參數設定 (realsense_params.yaml)。

src/server: Server 端的 ROS 2 套件。

bringup: 啟動檔 (server.launch.py)、DDS 設定、RViz 設定。

perception: VSLAM 和 Nvblox 的參數設定。

src/robot: 機器人共用套件。

description: 包含 mir_ur.urdf 的機器人模型。

diagnostics: 包含機器人狀態診斷工具。

src/robot_msgs: 自定義的 ROS 2 訊息 (.msg) 和服務 (.srv)。

config: 全域設定檔 (Chrony, Sysctl)。

系統需求

Client: NVIDIA AGX Orin (或相容的 Jetson 設備)

Ubuntu 22.04 (L4T)

NVIDIA JetPack 5.x

Server: 帶有 NVIDIA GPU 的 x86_64 主機

Ubuntu 22.04

NVIDIA 驅動程式 (515+ Cuda 11.7+)

通用:

ROS 2 Humble

NVIDIA Isaac ROS 2.0 (DP3) (將由 rosdep 自動安裝)

1. 安裝與設定 (Installation & Setup)

您只需在 Client (Orin) 和 Server 兩台機器上執行相同的安裝步驟。腳本將自動根據您的選擇來設定對應的角色。

1.1. 複製專案

git clone <您的_REPO_URL> realsense_streaming
cd realsense_streaming


1.2. 執行自動設定腳本

此腳本將會：

詢問您這台機器是 orin 還是 server。

安裝 requirements.apt 和 requirements.txt 中的依賴。

根據您的選擇設定時間同步 (Chrony)。

設定高效能網路核心參數 (Sysctl) 與防火牆。

使用 rosdep 自動安裝所有 ROS 2 依賴 (包含 Isaac ROS)。

在您的 .bashrc 中設定必要的環境變數。

./src/scripts/setup/initial_setup.sh


重要提示： 執行完腳本後，您必須關閉並重新開啟您的終端機，或執行 source ~/.bashrc 來載入新的環境變數。

2. (可選) Python 虛擬環境 (PDM)

本專案使用 colcon 作為主要的建置系統。colcon 依賴於系統的 ROS 2 環境 (/opt/ros/humble/setup.bash)。

不建議在 colcon 工作區中使用 Python 虛擬環境 (如 venv, pdm, poetry) 來 執行 ROS 節點，因為這會導致環境隔離問題。

然而，您可以使用 pdm 來管理開發工具 (例如 linter, formatter)。initial_setup.sh 已經為您安裝了 pdm。

# (可選) 初始化 PDM 專案
pdm init
pdm install

# (可選) 啟用虛擬環境 (僅用於開發，不要用它來 build 或 run)
pdm shell


再次強調： 所有的 colcon build 和 ros2 launch 指令都應在沒有啟用 PDM shell 的一般終端機中執行。

3. 建置工作區 (Building the Workspace)

在執行過 initial_setup.sh 並且重新開啟終端機後，您現在可以建置工作區。

# 確定您位於工作區根目錄 (realsense_streaming)
cd /path/to/realsense_streaming

# 執行 Colcon Build
colcon build --symlink-install


--symlink-install 允許您在修改 Python 檔案 (如 launch 檔) 後無需重新建置即可生效。

4. 執行系統 (Running the System)

執行時，請先啟動 Server 端，再啟動 Client 端。

Terminal 1: 啟動 Server (在伺服器上)

# 載入您的工作區環境 (如果 .bashrc 沒有自動載入)
# source install/setup.bash

# 啟動 Server 端的感知管線
ros2 launch server server.launch.py


您應該會看到 RViz 啟動，並等待來自 Client 的數據。

Terminal 2: 啟動 Client (在 Orin 上)

# 載入您的工作區環境 (如果 .bashrc 沒有自動載入)
# source install/setup.bash

# 啟動 Client 端的感測器串流
ros2 launch client orin.launch.py


啟動後，您應該會在 Server 端的 RViz 中看到影像、點雲和 VSLAM 的姿態。

(可選) 在 Client 上使用 Systemd 服務 (背景執行)

initial_setup.sh 已經為 Orin (Client) 安裝了一個 systemd 服務，使其可以開機自動啟動。

# 啟用服務 (開機自啟)
sudo systemctl enable sensors.service

# 手動啟動服務
sudo systemctl start sensors.service

# 檢查服務狀態與日誌
sudo systemctl status sensors.service
journalctl -u sensors.service -f
