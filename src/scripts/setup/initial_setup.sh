#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ROS_DOMAIN_ID=161

read -p "Select [orin/server]: " MACHINE_TYPE

if [[ "$MACHINE_TYPE" != "orin" && "$MACHINE_TYPE" != "server" ]]; then
    echo "Error: Invalid type"
    exit 1
fi

echo "Machine Type: $MACHINE_TYPE"

echo ""
echo "[1/7] Intsall Basic ..."
sudo apt update
sudo apt install -y \
    chrony \
    net-tools \
    iftop \
    htop \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

echo ""
echo "[2/7] Configure time synchronization..."
if [[ "$MACHINE_TYPE" == "server" ]]; then
    sudo cp "$WS_ROOT/config/chrony/server.conf" /etc/chrony/chrony.conf
else
    read -p "Please enter the server IP address: " SERVER_IP
    sudo cp "$WS_ROOT/config/chrony/client.conf" /etc/chrony/chrony.conf
fi
sudo systemctl restart chronyd
sudo systemctl enable chronyd

echo ""
echo "[3/7] Configure kernel network parameters..."
sudo cp "$WS_ROOT/config/sysctl/99-ros2-dds.conf" /etc/sysctl.d/
sudo sysctl -p /etc/sysctl.d/99-ros2-dds.conf

echo ""
echo "[4/7] Configure firewall rules..."
sudo ufw allow in proto udp to 224.0.0.0/4
sudo ufw allow in proto udp from 224.0.0.0/4
sudo ufw allow 7400:7500/udp

echo ""
echo "[5/7] Initialize rosdep..."
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init
fi
rosdep update

echo ""
echo "[6/7] Install ROS2..."
cd "$WS_ROOT"
rosdep install --from-paths src --ignore-src -r -y

echo ""
echo "[7/7] build Workspace..."
cd "$WS_ROOT"
colcon build --symlink-install

BASHRC_ADDITION="
source /opt/ros/humble/setup.bash
source $WS_ROOT/install/setup.bash
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ \"\$MACHINE_TYPE\" == \"orin\" ]]; then
    export CYCLONEDDS_URI=file://$WS_ROOT/src/orin_bringup/config/dds/cyclonedds_orin.xml
else
    export CYCLONEDDS_URI=file://$WS_ROOT/src/server_bringup/config/dds/cyclonedds_server.xml
fi
"

if ! grep -q "ROS2 multi-machine distributed system environment" ~/.bashrc; then
    echo "$BASHRC_ADDITION" >> ~/.bashrc
    echo "Added environment variables to ~/.bashrc"
fi

if [[ "$MACHINE_TYPE" == "orin" ]]; then
    echo ""
    echo "Jetson Orin setup ..."
    if command -v jetson_clocks &> /dev/null; then
        sudo jetson_clocks
        sudo nvpmodel -m 0
        echo "Enable Jetson max performance"
    fi
fi

echo ""
echo "======================================"
echo "Done for Init!"
echo "======================================"
echo ""
echo "下一步:"
echo "1. source ~/.bashrc"
echo "2. chronyc tracking"
echo "3. ros2 multicast send/receive"
if [[ "$MACHINE_TYPE" == "orin" ]]; then
    echo "4. ros2 launch orin_bringup orin.launch.py"
else
    echo "4. ros2 launch server_bringup server.launch.py"
fi
echo ""