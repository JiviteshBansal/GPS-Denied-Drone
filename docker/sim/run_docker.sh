#!/usr/bin/env bash
# ==============================================================================
# Cloud GPU Simulation Launcher (Headless on RunPod / AWS EC2)
# Track: Multi-Drone Simulation, Physics Environment & Cloud Deployment
# ==============================================================================

set -e

IMAGE_NAME="gps_denied_sim:latest"
CONTAINER_NAME="gps_denied_sim_runner"

echo "=========================================================="
echo " Starting Headless Cloud GPU Simulation Environment"
echo " Target: RunPod / AWS EC2 (NVIDIA GPU Instance)"
echo " Mode: Single-Drone Physics Validation (Drone 1: Port 14540)"
echo "=========================================================="

# Check for NVIDIA Container Toolkit
if command -v nvidia-smi &> /dev/null; then
    echo "[GPU Check] NVIDIA GPU detected:"
    nvidia-smi --query-gpu=gpu_name,memory.total --format=csv,noheader
    GPU_FLAGS="--gpus all"
else
    echo "[WARN] No NVIDIA GPU detected. Falling back to CPU mode."
    GPU_FLAGS=""
fi

# Build Docker image if not present
if [[ "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" == "" ]]; then
    echo "[Docker] Building simulation image ${IMAGE_NAME}..."
    docker build -t ${IMAGE_NAME} -f docker/sim/Dockerfile .
fi

# Run container with hardware acceleration and MAVLink UDP port forwards
echo "[Docker] Spawning container ${CONTAINER_NAME}..."
docker run -it --rm \
    ${GPU_FLAGS} \
    --name ${CONTAINER_NAME} \
    --ipc=host \
    --net=host \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY="" \
    -e HEADLESS=1 \
    -v "$(pwd)":/root/workspace \
    -p 14540:14540/udp \
    -p 14541:14541/udp \
    -p 14542:14542/udp \
    -p 8888:8888/udp \
    ${IMAGE_NAME} \
    bash
