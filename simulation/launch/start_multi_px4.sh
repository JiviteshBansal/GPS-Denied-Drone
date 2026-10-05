#!/usr/bin/env bash
# ==============================================================================
# Multi-Drone PX4 SITL Launcher (3 Iris Quadcopters on Isolated UDP Ports)
# Track: Multi-Drone Simulation, Physics Environment & Cloud Deployment
#
# Ports:
#   drone_1: MAVLink UDP 14540 @ (-4.0, -6.0)
#   drone_2: MAVLink UDP 14541 @ ( 0.0, -6.0)
#   drone_3: MAVLink UDP 14542 @ ( 4.0, -6.0)
# ==============================================================================

set -e

PX4_SOURCE_DIR="${PX4_SOURCE_DIR:=$HOME/src/PX4-Autopilot}"
PX4_BUILD_DIR="${PX4_SOURCE_DIR}/build/px4_sitl_default"

echo "=========================================================="
echo " Starting 3-Drone PX4 SITL Fleet for Curved Gridroom"
echo " drone_1: Instance 1 | Port 14540"
echo " drone_2: Instance 2 | Port 14541"
echo " drone_3: Instance 3 | Port 14542"
echo "=========================================================="

if [ ! -d "$PX4_SOURCE_DIR" ]; then
    echo "PX4 source not found at $PX4_SOURCE_DIR. Configure PX4_SOURCE_DIR as needed."
    exit 1
fi

cd "$PX4_SOURCE_DIR"

export PX4_SYS_AUTOSTART=4001
export PX4_ESTIMATOR=ekf2

# Launch Instance 1 (drone_1)
echo "[PX4 SITL] Launching drone_1 on UDP port 14540..."
./build/px4_sitl_default/bin/px4 -i 1 -d -s px4.commands -m iris &

sleep 2

# Launch Instance 2 (drone_2)
echo "[PX4 SITL] Launching drone_2 on UDP port 14541..."
./build/px4_sitl_default/bin/px4 -i 2 -d -s px4.commands -m iris &

sleep 2

# Launch Instance 3 (drone_3)
echo "[PX4 SITL] Launching drone_3 on UDP port 14542..."
./build/px4_sitl_default/bin/px4 -i 3 -d -s px4.commands -m iris &

wait
