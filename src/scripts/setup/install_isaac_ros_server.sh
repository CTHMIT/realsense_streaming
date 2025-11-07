#!/bin/bash
# Isaac ROS Installation Script for x86_64 Server
# For use with NVIDIA GPUs (RTX series)

set -e

info()   { echo -e "\033[1;34m[INFO]\033[0m $1"; }
warn()   { echo -e "\033[1;33m[WARN]\033[0m $1"; }
fail()   { echo -e "\033[1;31m[ERROR]\033[0m $1" >&2; exit 1; }
success() { echo -e "\033[1;32m[SUCCESS]\033[0m $1"; }

# Check if running on x86_64
ARCH=$(uname -m)
if [ "$ARCH" != "x86_64" ]; then
    fail "This script is for x86_64 systems. Detected: $ARCH"
fi

info "Starting Isaac ROS installation for x86_64 Server..."

# 1. Check CUDA installation
info "[1/7] Checking CUDA installation..."
if command -v nvcc &> /dev/null; then
    CUDA_VERSION=$(nvcc --version | grep "release" | awk '{print $6}' | cut -c2-)
    success "CUDA available (version $CUDA_VERSION)"
else
    warn "CUDA not found. Please install CUDA 11.8 or later."
    warn "Download from: https://developer.nvidia.com/cuda-downloads"
    read -p "Continue anyway? [y/N]: " response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

# Check GPU
if command -v nvidia-smi &> /dev/null; then
    success "NVIDIA driver available"
    nvidia-smi --query-gpu=name --format=csv,noheader | while read gpu; do
        info "  GPU: $gpu"
    done
else
    warn "nvidia-smi not available. Please install NVIDIA driver."
fi

# 2. Check OS version
info "[2/7] Checking OS version..."
OS_VERSION=$(lsb_release -rs)
OS_CODENAME=$(lsb_release -cs)

if [ "$OS_VERSION" != "22.04" ]; then
    warn "This script is tested on Ubuntu 22.04. You have: $OS_VERSION"
    read -p "Continue anyway? [y/N]: " response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 0
    fi
fi

# 3. Install dependencies
info "[3/7] Installing dependencies..."
sudo apt-get update
sudo apt-get install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    git \
    git-lfs \
    build-essential \
    cmake

# 4. Setup Isaac ROS APT repository
info "[4/7] Setting up Isaac ROS APT repository..."

# Add NVIDIA Isaac ROS repository key
if [ ! -f /usr/share/keyrings/nvidia-isaac-ros-archive-keyring.gpg ]; then
    curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | \
        sudo gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros-archive-keyring.gpg
    success "Added Isaac ROS repository key"
fi

# Add repository
if [ ! -f /etc/apt/sources.list.d/isaac-ros.list ]; then
    echo "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros-archive-keyring.gpg] https://isaac.download.nvidia.com/isaac-ros/release-3 $OS_CODENAME main" | \
        sudo tee /etc/apt/sources.list.d/isaac-ros.list
    success "Added Isaac ROS repository"
fi

sudo apt-get update

# 5. Install Isaac ROS packages
info "[5/7] Installing Isaac ROS packages..."

# Core packages
sudo apt-get install -y \
    ros-humble-isaac-ros-common || warn "Failed to install isaac-ros-common"

# H264 decoder (server needs decoder)
sudo apt-get install -y \
    ros-humble-isaac-ros-h264-decoder || warn "Failed to install H264 decoder"

# Visual SLAM
sudo apt-get install -y \
    ros-humble-isaac-ros-visual-slam || warn "Failed to install Visual SLAM"

# Nvblox
sudo apt-get install -y \
    ros-humble-nvblox-ros \
    ros-humble-nvblox-rviz-plugin || warn "Failed to install Nvblox"

# Additional useful packages
sudo apt-get install -y \
    ros-humble-isaac-ros-image-pipeline \
    ros-humble-isaac-ros-nitros \
    ros-humble-vision-msgs || warn "Failed to install additional packages"

# 6. Install cuDNN (if not already installed)
info "[6/7] Checking cuDNN..."
if ! dpkg -l | grep -q libcudnn; then
    warn "cuDNN not found. This is required for optimal performance."
    info "Download from: https://developer.nvidia.com/cudnn"
    info "Install: sudo dpkg -i cudnn-local-repo-*.deb"
else
    success "cuDNN installed"
fi

# 7. Verify installation
info "[7/7] Verifying installation..."

REQUIRED_PACKAGES=(
    "ros-humble-isaac-ros-h264-decoder"
    "ros-humble-isaac-ros-visual-slam"
    "ros-humble-nvblox-ros"
)

ALL_INSTALLED=true
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if dpkg -l | grep -q "$pkg"; then
        success "✓ $pkg installed"
    else
        warn "✗ $pkg NOT installed"
        ALL_INSTALLED=false
    fi
done

echo ""
info "======================================"
if [ "$ALL_INSTALLED" = true ]; then
    success "Isaac ROS installation complete!"
else
    warn "Some packages failed to install."
    warn "You may need to install CUDA/cuDNN first."
fi
info "======================================"
