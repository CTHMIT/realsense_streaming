#!/bin/bash
# Isaac ROS Installation Script for AGX Orin

set -e

info()   { echo -e "\033[1;34m[INFO]\033[0m $1"; }
warn()   { echo -e "\033[1;33m[WARN]\033[0m $1"; }
fail()   { echo -e "\033[1;31m[ERROR]\033[0m $1" >&2; exit 1; }

if [ ! -f /etc/nv_tegra_release ]; then
    fail "This script is designed for NVIDIA Jetson devices (AGX Orin)"
fi

info "Starting Isaac ROS installation for AGX Orin..."

info "[1/6] Checking CUDA installation..."
if ! command -v nvcc &> /dev/null; then
    warn "CUDA not found. Installing NVIDIA JetPack..."
    sudo apt-get update
    sudo apt-get install -y nvidia-jetpack
fi

CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | cut -c2-)
info "CUDA Version: $CUDA_VERSION"

info "[2/6] Adding Isaac ROS APT repository..."

sudo apt-get update
sudo apt-get install -y software-properties-common

wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/isaac-ros.list

sudo apt-get update

info "[3/6] Installing Isaac ROS packages..."

sudo apt-get install -y \
    ros-humble-isaac-ros-h264-encoder \
    ros-humble-isaac-ros-common \
    ros-humble-isaac-ros-image-pipeline

info "[4/6] Installing additional dependencies..."

sudo apt-get install -y \
    ros-humble-vision-msgs \
    ros-humble-isaac-ros-nitros \
    ros-humble-isaac-ros-nitros-bridge-interfaces \
    ros-humble-isaac-ros-nitros-image-type \
    ros-humble-isaac-ros-nitros-camera-info-type

info "[5/6] Checking VPI installation..."
if ! dpkg -l | grep -q vpi2; then
    warn "VPI not found. It should be included in JetPack."
    warn "Please install JetPack manually: sudo apt install nvidia-jetpack"
fi

info "[6/6] Verifying installation..."

REQUIRED_PACKAGES=(
    "ros-humble-isaac-ros-h264-encoder"
    "ros-humble-isaac-ros-h264-decoder"
    "ros-humble-isaac-ros-visual-slam"
    "ros-humble-nvblox-ros"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if dpkg -l | grep -q "$pkg"; then
        info "✓ $pkg installed"
    else
        warn "✗ $pkg NOT installed"
    fi
done

info "Isaac ROS installation complete!"