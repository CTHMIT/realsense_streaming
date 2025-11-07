#!/bin/bash
# Complete Setup Script for AGX Orin ROS2 Vision System
# Run this AFTER installing Isaac ROS (install_isaac_ros.sh)

set -e

# --- Variables ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_DOMAIN_ID=161
ROS_DISTRO="humble"
CURRENT_USER=$(whoami)

# --- Helpers ---
info()   { echo -e "\033[1;34m[INFO]\033[0m $1"; }
warn()   { echo -e "\033[1;33m[WARN]\033[0m $1"; }
fail()   { echo -e "\033[1;31m[ERROR]\033[0m $1" >&2; exit 1; }
success() { echo -e "\033[1;32m[SUCCESS]\033[0m $1"; }

# --- Main steps ---

print_banner() {
    echo "========================================"
    echo "  AGX Orin ROS2 Vision System Setup"
    echo "========================================"
    echo ""
}

check_prerequisites() {
    info "Checking prerequisites..."
    
    # Check if ROS 2 is installed
    if [[ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
        fail "ROS 2 $ROS_DISTRO not found. Please install it first."
    fi
    
    # Check if Isaac ROS is installed
    if ! dpkg -l | grep -q "ros-humble-isaac-ros"; then
        warn "Isaac ROS packages not found."
        warn "Please run install_isaac_ros.sh first!"
        read -p "Continue anyway? [y/N]: " response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            exit 0
        fi
    fi
    
    # Check if running on Jetson
    if [[ ! -f /etc/nv_tegra_release ]]; then
        warn "Not running on NVIDIA Jetson. Some features may not work."
    fi
    
    success "Prerequisites check passed"
}

select_machine_type() {
    echo ""
    info "Select machine role:"
    echo "  1) orin   - AGX Orin (RealSense camera client)"
    echo "  2) server - Processing server (SLAM/Nvblox)"
    read -p "Enter choice [1/2]: " choice
    
    case "$choice" in
        1) MACHINE_TYPE="orin" ;;
        2) MACHINE_TYPE="server" ;;
        *) fail "Invalid choice. Enter 1 or 2." ;;
    esac
    
    info "Machine role: $MACHINE_TYPE"
}

detect_network_interface() {
    info "Detecting network interfaces..."
    
    # List available interfaces
    echo "Available network interfaces:"
    ip -o link show | awk -F': ' '{print "  - " $2}' | grep -v "lo"
    
    if [[ "$MACHINE_TYPE" == "orin" ]]; then
        read -p "Enter network interface for Orin (e.g., eth0, wlan0): " NET_IFACE
    else
        read -p "Enter network interface for Server (e.g., eth0, enp3s0): " NET_IFACE
    fi
    
    # Validate interface
    if ! ip link show "$NET_IFACE" &> /dev/null; then
        fail "Interface $NET_IFACE not found!"
    fi
    
    # Get IP address
    LOCAL_IP=$(ip -4 addr show "$NET_IFACE" | grep -oP '(?<=inet\s)\d+(\.\d+){3}')
    info "Local IP: $LOCAL_IP on $NET_IFACE"
}

install_apt_packages() {
    info "[1/9] Installing system packages (apt)..."
    local apt_req_file="$WS_ROOT/requirements.apt"
    [[ -f "$apt_req_file" ]] || fail "Missing $apt_req_file"

    sudo apt-get update
    sudo xargs -a "$apt_req_file" apt-get install -y
    
    success "APT packages installed"
}

install_pip_packages() {
    info "[2/9] Installing Python packages (pip)..."
    local pip_req_file="$WS_ROOT/requirements.txt"
    if [[ ! -f "$pip_req_file" ]]; then
        warn "Missing $pip_req_file. Skipping pip."
        return
    fi
    python3 -m pip install --upgrade pip
    python3 -m pip install -r "$pip_req_file" --break-system-packages
    
    success "Python packages installed"
}

configure_chrony() {
    info "[3/9] Configuring time sync (Chrony)..."

    local svc="chronyd"
    if systemctl list-unit-files | grep -q '^chrony\.service'; then
        svc="chrony"
    fi

    if [[ "$MACHINE_TYPE" == "server" ]]; then
        sudo cp "$WS_ROOT/config/chrony/server.conf" /etc/chrony/chrony.conf
        info "Configured as Chrony server."
    else
        read -p "Enter Server IP for Chrony client: " SERVER_IP
        [[ -n "$SERVER_IP" ]] || fail "Server IP not provided."
        sudo cp "$WS_ROOT/config/chrony/client.conf" /etc/chrony/chrony.conf
        sudo sed -i "s/10.28.121.28/$SERVER_IP/g" /etc/chrony/chrony.conf
        info "Configured as Chrony client -> $SERVER_IP"
    fi

    sudo systemctl restart "$svc"
    sudo systemctl enable "$svc"
    
    success "Chrony configured"
}

configure_sysctl() {
    info "[4/9] Applying sysctl settings for DDS..."
    sudo cp "$WS_ROOT/config/sysctl/99-ros2-dds.conf" /etc/sysctl.d/
    sudo sysctl -p /etc/sysctl.d/99-ros2-dds.conf
    
    success "Sysctl configured"
}

configure_firewall() {
    info "[5/9] Configuring UFW..."
    sudo ufw --force enable
    sudo ufw allow in proto udp to 224.0.0.0/4
    sudo ufw allow in proto udp from 224.0.0.0/4
    sudo ufw allow 7400:7500/udp
    sudo ufw allow 7400:7500/tcp
    
    success "Firewall configured"
}

update_dds_config() {
    info "[6/9] Updating CycloneDDS configuration..."
    
    if [[ "$MACHINE_TYPE" == "orin" ]]; then
        DDS_CONFIG="$WS_ROOT/src/client/bringup/config/dds/cyclonedds_orin.xml"
        # Update network interface
        sed -i "s|<Name>.*</Name>|<Name>$NET_IFACE</Name>|" "$DDS_CONFIG"
        # Update peer address if needed
        if [[ -n "$SERVER_IP" ]]; then
            sed -i "s|<Peer address=\"[^\"]*\"/>|<Peer address=\"$SERVER_IP\"/>|" "$DDS_CONFIG"
        fi
    else
        DDS_CONFIG="$WS_ROOT/src/server/bringup/config/dds/cyclonedds_server.xml"
        # Update network interface
        sed -i "s|<Name>.*</Name>|<Name>$NET_IFACE</Name>|" "$DDS_CONFIG"
    fi
    
    success "DDS config updated"
}

configure_ros() {
    info "[7/9] Initializing rosdep and installing ROS deps..."

    if [[ ! -d "/etc/ros/rosdep/sources.list.d" ]]; then
        info "rosdep init..."
        sudo rosdep init
    fi
    rosdep update

    info "Installing workspace ROS dependencies..."
    cd "$WS_ROOT"
    
    # Install with skip-keys for Isaac ROS packages
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys="isaac_ros_h264_encoder isaac_ros_h264_decoder isaac_ros_visual_slam nvblox_ros"
    
    success "ROS dependencies installed"
}

set_orin_performance() {
    if [[ "$MACHINE_TYPE" != "orin" ]]; then
        return
    fi
    
    info "[8/9] Configuring Jetson Orin performance..."
    
    # Set max performance mode
    if command -v nvpmodel >/dev/null 2>&1; then
        sudo nvpmodel -m 0 || warn "Failed to set nvpmodel"
    fi
    
    # Set max clocks
    if command -v jetson_clocks >/dev/null 2>&1; then
        sudo /usr/bin/jetson_clocks || warn "Failed to set jetson_clocks"
    fi
    
    # Set fan to max
    if [[ -w /sys/devices/pwm-fan/target_pwm ]]; then
        echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm >/dev/null || true
    elif compgen -G "/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1" > /dev/null; then
        for p in /sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1; do
            echo 255 | sudo tee "$p" >/dev/null || true
        done
    fi
    
    success "Orin performance configured"
}

configure_bashrc() {
    info "[9/9] Updating ~/.bashrc..."

    if [[ "$MACHINE_TYPE" == "orin" ]]; then
        DDS_CONFIG_PATH="$WS_ROOT/src/client/bringup/config/dds/cyclonedds_orin.xml"
        LAUNCH_CMD="ros2 launch client orin.launch.py"
    else
        DDS_CONFIG_PATH="$WS_ROOT/src/server/bringup/config/dds/cyclonedds_server.xml"
        LAUNCH_CMD="ros2 launch server server.launch.py"
    fi

    BASHRC_ADDITION=$(cat <<EOF

# --- ROS2 Vision System (auto-generated) ---
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$DDS_CONFIG_PATH
export ROS_LOCALHOST_ONLY=0

# Isaac ROS
export ISAAC_ROS_WS=$WS_ROOT

# Aliases
alias rs-camera='ros2 launch client orin.launch.py'
alias rs-server='ros2 launch server server.launch.py'
alias rs-rviz='ros2 launch server camera_visualization.launch.py'
# --- end auto-generated ---
EOF
)

    if ! grep -q "# --- ROS2 Vision System (auto-generated) ---" ~/.bashrc; then
        echo "$BASHRC_ADDITION" >> ~/.bashrc
        success "Bashrc updated"
    else
        info "~/.bashrc already configured. Skipped."
    fi
}

create_systemd_service() {
    if [[ "$MACHINE_TYPE" != "orin" ]]; then
        return
    fi
    
    info "Creating systemd service..."
    
    local service_file="/tmp/ros2-camera.service"
    cat > "$service_file" <<EOF
[Unit]
Description=ROS2 RealSense Camera Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$CURRENT_USER
Group=$CURRENT_USER
WorkingDirectory=$HOME

ExecStartPre=/bin/bash -c 'until ping -c1 ${SERVER_IP:-10.28.121.28}; do sleep 1; done'

Environment="ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
Environment="RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
Environment="CYCLONEDDS_URI=file://$WS_ROOT/src/client/bringup/config/dds/cyclonedds_orin.xml"

ExecStart=/bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash && source $WS_ROOT/install/setup.bash && ros2 launch client orin.launch.py'

Restart=on-failure
RestartSec=5
StartLimitBurst=3
StartLimitInterval=60

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    sudo cp "$service_file" /etc/systemd/system/ros2-camera.service
    sudo systemctl daemon-reload
    
    info "Systemd service created at /etc/systemd/system/ros2-camera.service"
    info "To enable: sudo systemctl enable ros2-camera.service"
    info "To start: sudo systemctl start ros2-camera.service"
}

print_summary() {
    echo ""
    echo "========================================"
    success "Setup Complete!"
    echo "========================================"
    echo ""
    info "Configuration Summary:"
    echo "  - Machine Type: $MACHINE_TYPE"
    echo "  - Network Interface: $NET_IFACE"
    echo "  - Local IP: $LOCAL_IP"
    echo "  - ROS Domain ID: $ROS_DOMAIN_ID"
    echo "  - Workspace: $WS_ROOT"
    echo ""
    info "Next Steps:"
    echo "  1. Restart shell: exec bash (or source ~/.bashrc)"
    echo "  2. Verify time sync: chronyc tracking"
    echo "  3. Test multicast: ros2 multicast"
    echo "  4. Build workspace:"
    echo "     cd $WS_ROOT"
    echo "     colcon build --symlink-install"
    echo "  5. Launch system:"
    echo "     $LAUNCH_CMD"
    echo ""
    if [[ "$MACHINE_TYPE" == "orin" ]]; then
        info "Optional: Enable autostart service"
        echo "  sudo systemctl enable ros2-camera.service"
        echo "  sudo systemctl start ros2-camera.service"
        echo ""
    fi
    info "For troubleshooting, check:"
    echo "  - Logs: journalctl -u ros2-camera -f"
    echo "  - Topics: ros2 topic list"
    echo "  - Nodes: ros2 node list"
    echo ""
    echo "========================================"
}

main() {
    print_banner
    check_prerequisites
    select_machine_type
    detect_network_interface
    install_apt_packages
    install_pip_packages
    configure_chrony
    configure_sysctl
    configure_firewall
    update_dds_config
    configure_ros
    set_orin_performance
    configure_bashrc
    create_systemd_service
    print_summary
}

main "$@"