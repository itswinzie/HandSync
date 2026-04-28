#!/bin/bash
# ============================================================
# MasterPi ROS2 Auto-Start Script
# ============================================================

# Terminal colors
RED='\033[0;31m'
GREEN='\033[0;32m'
CYAN='\033[0;36m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
WHITE='\033[1;37m'
NC='\033[0m'

# ── CYTRON ASCII Art (big font) ───────────────────────────────
clear
echo ""
echo -e "${CYAN}   _______     _________ _____   ____  _   _ "
echo -e "  / ____\\ \\   / /__   __|  __ \\ / __ \\| \\ | |"
echo -e " | |     \\ \\_/ /   | |  | |__) | |  | |  \\| |"
echo -e " | |      \\   /    | |  |  _  /| |  | | . \` |"
echo -e " | |____   | |     | |  | | \\ \\| |__| | |\\  |"
echo -e "  \\_____|  |_|     |_|  |_|  \\_\\\\____/|_| \\_|${NC}"
echo ""
echo -e "${YELLOW}  ══════════════════════════════════════════════════${NC}"
echo -e "${WHITE}       MasterPi Robot Control System — ROS2        ${NC}"
echo -e "${YELLOW}  ══════════════════════════════════════════════════${NC}"
echo ""

# ── Helper functions ──────────────────────────────────────────
log_info()  { echo -e "  ${GREEN}[  OK  ]${NC}  $1"; }
log_error() { echo -e "  ${RED}[ FAIL ]${NC}  $1"; }
log_step()  { echo -e "  ${CYAN}[  >>  ]${NC}  $1"; }

# ── Wait for system ───────────────────────────────────────────
log_step "Waiting for system to be ready..."
sleep 5

# ── Stop masterpi.service ─────────────────────────────────────
log_step "Stopping masterpi.service..."
systemctl stop masterpi.service 2>/dev/null
log_info "masterpi.service stopped"

# ── Start Docker container ────────────────────────────────────
log_step "Starting Docker container ros2_masterpi..."
docker start ros2_masterpi 2>/dev/null
if [ $? -eq 0 ]; then
    log_info "Container ros2_masterpi is running"
else
    log_error "Failed to start container!"
    exit 1
fi

# ── Copy latest files into container ─────────────────────────
log_step "Copying latest files into container..."
docker cp /home/pi/board_demo/ros_robot_controller_sdk.py ros2_masterpi:/root/ 2>/dev/null
docker cp /home/pi/masterpi_subscriber.py ros2_masterpi:/root/ 2>/dev/null
docker cp /home/pi/masterpi_camera_pub.py ros2_masterpi:/root/ 2>/dev/null
log_info "Files updated"

# ── Run Subscriber ────────────────────────────────────────────
log_step "Starting masterpi_subscriber.py..."
docker exec -d ros2_masterpi bash -c "
    source /opt/ros/humble/setup.bash && \
    export ROS_DOMAIN_ID=42 && \
    python3 /root/masterpi_subscriber.py
"
log_info "Subscriber running in background"

# ── Run Camera Publisher ──────────────────────────────────────
log_step "Starting masterpi_camera_pub.py..."
docker exec -d ros2_masterpi bash -c "
    source /opt/ros/humble/setup.bash && \
    export ROS_DOMAIN_ID=42 && \
    python3 /root/masterpi_camera_pub.py
"
log_info "Camera publisher running in background"

# ── Summary ───────────────────────────────────────────────────
echo ""
echo -e "${YELLOW}  ══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}       ✓  ROS2 MasterPi System is Ready!          ${NC}"
echo -e "${YELLOW}  ══════════════════════════════════════════════════${NC}"
echo ""
echo -e "  ${WHITE}Active Topics:${NC}"
echo -e "  ${BLUE}•${NC} /masterpi_command"
echo -e "  ${BLUE}•${NC} /masterpi/camera/compressed"
echo ""
echo -e "  ${WHITE}ROS_DOMAIN_ID :${NC} ${CYAN}42${NC}"
echo -e "  ${WHITE}Container     :${NC} ${CYAN}ros2_masterpi${NC}"
echo ""
