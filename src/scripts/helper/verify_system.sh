#!/bin/bash
# System Verification Script
# Tests all components to ensure proper installation

# NOTE: Removed 'set -e' to allow all tests to run even if some fail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
TESTS_PASSED=0
TESTS_FAILED=0
TESTS_WARNING=0

info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[PASS]${NC} $1"; ((TESTS_PASSED++)); }
fail()    { echo -e "${RED}[FAIL]${NC} $1"; ((TESTS_FAILED++)); }
warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; ((TESTS_WARNING++)); }

print_header() {
    echo "========================================"
    echo "  System Verification Test"
    echo "========================================"
    echo ""
}

test_ros2() {
    info "Testing ROS 2 installation..."
    
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        success "ROS 2 Humble found"
    else
        fail "ROS 2 Humble not found"
        return
    fi
    
    # Source ROS 2 (silently)
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    
    if command -v ros2 &> /dev/null; then
        success "ros2 command available"
    else
        fail "ros2 command not available"
    fi
}

test_isaac_ros() {
    info "Testing Isaac ROS packages..."
    
    local packages=(
        "ros-humble-isaac-ros-h264-encoder"
        "ros-humble-isaac-ros-h264-decoder"
        "ros-humble-isaac-ros-visual-slam"
        "ros-humble-nvblox-ros"
    )
    
    local all_found=true
    for pkg in "${packages[@]}"; do
        if dpkg -l 2>/dev/null | grep -q "$pkg"; then
            success "$pkg installed"
        else
            fail "$pkg not installed"
            all_found=false
        fi
    done
    
    if [ "$all_found" = false ]; then
        warn "Some Isaac ROS packages missing. Run install_isaac_ros.sh"
    fi
}

test_realsense() {
    info "Testing RealSense SDK..."
    
    if command -v rs-enumerate-devices &> /dev/null; then
        success "RealSense tools available"
        
        # Try to enumerate devices
        if timeout 3 rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense"; then
            success "RealSense camera detected"
        else
            warn "No RealSense camera connected (or not accessible)"
        fi
    else
        fail "RealSense tools not found"
    fi
    
    # Check ROS package
    if dpkg -l 2>/dev/null | grep -q "ros-humble-realsense2-camera"; then
        success "RealSense ROS 2 package installed"
    else
        fail "RealSense ROS 2 package not installed"
    fi
}

test_cuda() {
    info "Testing CUDA installation..."
    
    if command -v nvcc &> /dev/null; then
        local cuda_version=$(nvcc --version 2>/dev/null | grep "release" | awk '{print $6}' | cut -c2-)
        success "CUDA available (version $cuda_version)"
        
        # Check GPU
        if command -v nvidia-smi &> /dev/null; then
            success "nvidia-smi available"
            local gpu_info=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
            if [ -n "$gpu_info" ]; then
                info "  GPU: $gpu_info"
            fi
        else
            warn "nvidia-smi not available"
        fi
    else
        if [ -f /etc/nv_tegra_release ]; then
            warn "Running on Jetson - CUDA expected but nvcc not in PATH"
        else
            fail "CUDA not found"
        fi
    fi
}

test_network() {
    info "Testing network configuration..."
    
    # Check multicast support
    if ip link show 2>/dev/null | grep -q "MULTICAST"; then
        success "Multicast supported"
    else
        fail "Multicast not supported"
    fi
    
    # Check firewall
    if command -v ufw &> /dev/null; then
        if sudo ufw status 2>/dev/null | grep -q "Status: active"; then
            success "UFW firewall active"
            
            # Check DDS ports
            if sudo ufw status 2>/dev/null | grep -q "7400:7500"; then
                success "DDS ports allowed"
            else
                warn "DDS ports not explicitly allowed in UFW"
            fi
        else
            warn "UFW firewall inactive"
        fi
    else
        warn "UFW not installed"
    fi
}

test_chrony() {
    info "Testing time synchronization..."
    
    if systemctl is-active --quiet chronyd 2>/dev/null || systemctl is-active --quiet chrony 2>/dev/null; then
        success "Chrony service running"
        
        if command -v chronyc &> /dev/null; then
            local sync_status=$(chronyc tracking 2>/dev/null | grep "Reference ID" || echo "")
            if [ -n "$sync_status" ]; then
                success "Time synchronized"
                info "  $sync_status"
            else
                warn "Time not yet synchronized"
            fi
        fi
    else
        warn "Chrony service not running"
    fi
}

test_dds() {
    info "Testing DDS configuration..."
    
    if [ -n "$ROS_DOMAIN_ID" ]; then
        success "ROS_DOMAIN_ID set: $ROS_DOMAIN_ID"
    else
        fail "ROS_DOMAIN_ID not set"
    fi
    
    if [ -n "$RMW_IMPLEMENTATION" ]; then
        success "RMW_IMPLEMENTATION set: $RMW_IMPLEMENTATION"
    else
        fail "RMW_IMPLEMENTATION not set"
    fi
    
    if [ -n "$CYCLONEDDS_URI" ]; then
        success "CYCLONEDDS_URI set"
        
        # Check if file exists
        local dds_file="${CYCLONEDDS_URI#file://}"
        if [ -f "$dds_file" ]; then
            success "CycloneDDS config file exists: $dds_file"
        else
            fail "CycloneDDS config file not found: $dds_file"
        fi
    else
        fail "CYCLONEDDS_URI not set"
    fi
}

test_workspace() {
    info "Testing workspace..."
    
    # Find workspace root
    local ws_root=""
    if [ -n "$ISAAC_ROS_WS" ]; then
        ws_root="$ISAAC_ROS_WS"
    elif [ -n "$COLCON_PREFIX_PATH" ]; then
        ws_root=$(echo "$COLCON_PREFIX_PATH" | cut -d: -f1 | sed 's|/install||')
    else
        # Try to find workspace from current directory
        ws_root=$(pwd)
        while [ "$ws_root" != "/" ]; do
            if [ -d "$ws_root/src" ] && [ -f "$ws_root/.colcon_workspace" -o -d "$ws_root/build" ]; then
                break
            fi
            ws_root=$(dirname "$ws_root")
        done
        if [ "$ws_root" = "/" ]; then
            ws_root=""
        fi
    fi
    
    if [ -n "$ws_root" ] && [ -d "$ws_root" ]; then
        success "Workspace found: $ws_root"
        
        # Check if built
        if [ -d "$ws_root/install" ]; then
            success "Workspace built"
            
            # Check packages
            local packages=(
                "client"
                "server"
                "robot"
                "robot_msgs"
            )
            
            for pkg in "${packages[@]}"; do
                if [ -d "$ws_root/install/$pkg" ]; then
                    success "Package $pkg built"
                else
                    warn "Package $pkg not built"
                fi
            done
        else
            warn "Workspace not built yet"
        fi
    else
        warn "Workspace not found or not sourced"
    fi
}

test_jetson() {
    if [ ! -f /etc/nv_tegra_release ]; then
        return 0
    fi
    
    info "Testing Jetson-specific features..."
    
    # Check nvpmodel
    if command -v nvpmodel &> /dev/null; then
        local power_mode=$(sudo nvpmodel -q 2>/dev/null | grep "Mode:" || echo "Unknown")
        success "nvpmodel available"
        info "  $power_mode"
    else
        warn "nvpmodel not available"
    fi
    
    # Check jetson_clocks
    if command -v jetson_clocks &> /dev/null; then
        success "jetson_clocks available"
    else
        warn "jetson_clocks not available"
    fi
    
    # Check fan
    if [ -w /sys/devices/pwm-fan/target_pwm 2>/dev/null ]; then
        local fan_speed=$(cat /sys/devices/pwm-fan/target_pwm 2>/dev/null || echo "unknown")
        success "Fan control available (speed: $fan_speed)"
    else
        warn "Fan control not available or not writable"
    fi
}

test_ros2_tools() {
    info "Testing ROS 2 tools..."
    
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    
    # Test multicast (with timeout)
    if command -v ros2 &> /dev/null; then
        success "ros2 command works"
        
        # Try multicast test briefly
        timeout 2 ros2 multicast receive &> /dev/null &
        local mc_pid=$!
        sleep 1
        
        if kill -0 $mc_pid 2>/dev/null; then
            success "Multicast test can start"
            kill $mc_pid 2>/dev/null || true
            wait $mc_pid 2>/dev/null || true
        else
            warn "Multicast test failed to start (may be OK if no network)"
        fi
    else
        fail "ros2 command not available"
    fi
}

print_summary() {
    echo ""
    echo "========================================"
    echo "  Test Summary"
    echo "========================================"
    echo -e "${GREEN}Passed:  $TESTS_PASSED${NC}"
    echo -e "${YELLOW}Warnings: $TESTS_WARNING${NC}"
    echo -e "${RED}Failed:  $TESTS_FAILED${NC}"
    echo "========================================"
    
    if [ $TESTS_FAILED -eq 0 ]; then
        if [ $TESTS_WARNING -eq 0 ]; then
            echo -e "${GREEN}✓ All tests passed!${NC}"
            echo "System is ready to use."
            return 0
        else
            echo -e "${YELLOW}⚠ Tests passed with warnings${NC}"
            echo "Review warnings above."
            return 0
        fi
    else
        echo -e "${RED}✗ Some tests failed${NC}"
        echo "Please fix failed tests before proceeding."
        return 1
    fi
}

main() {
    print_header
    
    test_ros2
    test_isaac_ros
    test_realsense
    test_cuda
    test_network
    test_chrony
    test_dds
    test_workspace
    test_jetson
    test_ros2_tools
    
    print_summary
}

main "$@"
exit_code=$?
exit $exit_code