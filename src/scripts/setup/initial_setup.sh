#!/bin/bash

set -e

# --- Variables ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_DOMAIN_ID=161
ROS_DISTRO="humble"

# --- Helpers ---
info()   { echo -e "\033[1;34m[INFO]\033[0m $1"; }
warn()   { echo -e "\033[1;33m[WARN]\033[0m $1"; }
fail()   { echo -e "\033[1;31m[ERROR]\033[0m $1" >&2; exit 1; }

# --- Main steps ---

# 1) Select machine role (orin/server)
select_machine_type() {
  read -p "Select machine role [orin/server]: " MACHINE_TYPE
  if [[ "$MACHINE_TYPE" != "orin" && "$MACHINE_TYPE" != "server" ]]; then
    fail "Invalid type. Enter 'orin' or 'server'."
  fi
  info "Machine role: $MACHINE_TYPE"
}

# 2) Install system packages from requirements.apt
install_apt_packages() {
  info "[1/8] Installing system packages (apt)..."
  local apt_req_file="$WS_ROOT/requirements.apt"
  [[ -f "$apt_req_file" ]] || fail "Missing $apt_req_file"

  sudo apt-get update
  sudo xargs -a "$apt_req_file" apt-get install -y
}

# 3) Install Python packages from requirements.txt
install_pip_packages() {
  info "[2/8] Installing Python packages (pip)..."
  local pip_req_file="$WS_ROOT/requirements.txt"
  if [[ ! -f "$pip_req_file" ]]; then
    warn "Missing $pip_req_file. Skipping pip."
    return
  fi
  python3 -m pip install --upgrade pip
  python3 -m pip install -r "$pip_req_file"
}

# 4) Configure Chrony time sync
configure_chrony() {
  info "[3/8] Configuring time sync (Chrony)..."

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
}

# 5) Kernel network sysctl
configure_sysctl() {
  info "[4/8] Applying sysctl settings..."
  sudo cp "$WS_ROOT/config/sysctl/99-ros2-dds.conf" /etc/sysctl.d/
  sudo sysctl -p /etc/sysctl.d/99-ros2-dds.conf
}

# 6) UFW firewall rules
configure_firewall() {
  info "[5/8] Configuring UFW..."
  sudo ufw allow in proto udp to 224.0.0.0/4
  sudo ufw allow in proto udp from 224.0.0.0/4
  sudo ufw allow 7400:7500/udp
}

# 7) rosdep init and install
configure_ros() {
  info "[6/8] Initializing rosdep and installing ROS deps..."

  [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]] || \
    fail "ROS 2 $ROS_DISTRO not found. Please install ROS 2 Humble first."

  if [[ ! -d "/etc/ros/rosdep/sources.list.d" ]]; then
    info "rosdep init..."
    sudo rosdep init
  fi
  rosdep update

  info "Installing project ROS deps (Isaac ROS, RealSense, URDF meshes)..."
  cd "$WS_ROOT"
  rosdep install --from-paths src --ignore-src -r -y
}

# Orin fan to max: prefer jetson_clocks --fan, else sysfs fallbacks
set_orin_fan_max() {
  info "Setting Orin fan to maximum..."
  if command -v jetson_clocks >/dev/null 2>&1; then
    if jetson_clocks --help 2>/dev/null | grep -q -- '--fan'; then
      sudo /usr/bin/jetson_clocks --fan   # sets max fan speed
      return
    else
      sudo /usr/bin/jetson_clocks         # old behavior may set high clocks
    fi
  fi

  # sysfs fallbacks (may vary by BSP)
  if [[ -w /sys/devices/pwm-fan/target_pwm ]]; then
    echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm >/dev/null || true
  elif compgen -G "/sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1" > /dev/null; then
    for p in /sys/devices/platform/pwm-fan/hwmon/hwmon*/pwm1; do
      echo 255 | sudo tee "$p" >/dev/null || true
    done
  else
    warn "Could not find a writable PWM fan sysfs node."
  fi
}

# 8) Append env to .bashrc and set Orin performance
configure_bashrc() {
  info "[7/8] Updating ~/.bashrc..."

  if [[ "$MACHINE_TYPE" == "orin" ]]; then
    DDS_CONFIG_PATH="$WS_ROOT/src/client/bringup/config/dds/cyclonedds_orin.xml"
    LAUNCH_CMD="ros2 launch client orin.launch.py"
  else
    DDS_CONFIG_PATH="$WS_ROOT/src/server/bringup/config/dds/cyclonedds_server.xml"
    LAUNCH_CMD="ros2 launch server server.launch.py"
  fi

  BASHRC_ADDITION=$(cat <<EOF
# --- realsense_streaming (auto) ---
source /opt/ros/$ROS_DISTRO/setup.bash
if [ -f "$WS_ROOT/install/setup.bash" ]; then
  source "$WS_ROOT/install/setup.bash"
fi
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$DDS_CONFIG_PATH
# --- end ---
EOF
)

  if ! grep -q "# --- realsense_streaming (auto) ---" ~/.bashrc; then
    echo "$BASHRC_ADDITION" >> ~/.bashrc
    info "Appended environment to ~/.bashrc"
  else
    info "~/.bashrc already configured. Skipped."
  fi

  if [[ "$MACHINE_TYPE" == "orin" ]]; then
    info "Applying Jetson Orin performance mode..."
    if command -v nvpmodel >/dev/null 2>&1; then
      sudo nvpmodel -m 0 || true
    fi
    if command -v jetson_clocks >/dev/null 2>&1; then
      # Try to set max clocks and fan with one call, then ensure fan max
      if jetson_clocks --help 2>/dev/null | grep -q -- '--fan'; then
        sudo /usr/bin/jetson_clocks --fan
      else
        sudo /usr/bin/jetson_clocks
      fi
    fi
    set_orin_fan_max
    info "Jetson Orin max performance and fan applied."
  fi

  echo ""
  info "======================================"
  info "Setup complete."
  info "======================================"
  echo ""
  info "Next:"
  info "1) Restart your shell or: source ~/.bashrc"
  info "2) Optional: chronyc tracking"
  info "3) Optional: ros2 multicast send/receive"
  info "4) Build: cd $WS_ROOT && colcon build --symlink-install"
  info "5) Launch: $LAUNCH_CMD"
  echo ""
}

main() {
  select_machine_type
  install_apt_packages
  install_pip_packages
  configure_chrony
  configure_sysctl
  configure_firewall
  configure_ros
  configure_bashrc
}

main
