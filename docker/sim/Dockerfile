# ==============================================================================
# Cloud GPU Simulation Environment (Gazebo Harmonic + ROS 2 Humble + PX4 SITL)
# Target Deployment: Headless Cloud Instances (RunPod, AWS EC2 G4/G5 instances)
# ==============================================================================
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,video,display
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install foundational packages and graphics/EGL acceleration dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    curl \
    git \
    gnupg2 \
    lsb-release \
    net-tools \
    python3-pip \
    python3-dev \
    software-properties-common \
    sudo \
    udev \
    wget \
    xvfb \
    libegl1 \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libvulkan1 \
    mesa-utils \
    vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

# Add Gazebo Harmonic repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list

# Install ROS 2 Humble base & Gazebo Harmonic headless packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-dev-tools \
    python3-colcon-common-extensions \
    gz-harmonic \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

# Install Micro-XRCE-DDS-Agent for PX4 <-> ROS 2 communication
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/Micro-XRCE-DDS-Agent && \
    cd /tmp/Micro-XRCE-DDS-Agent && \
    mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/Micro-XRCE-DDS-Agent

# Install Python robotics & simulation packages
RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    opencv-python-headless \
    pymavlink \
    pyyaml

# Workspace setup
WORKDIR /root/workspace
COPY . /root/workspace/

# Environment sourcing
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default headless port exposures
# MAVLink UDP: 14540 (drone 1), 14541 (drone 2), 14542 (drone 3)
# MicroXRCEAgent DDS: 8888
EXPOSE 14540/udp 14541/udp 14542/udp 8888/udp

CMD ["/bin/bash"]
