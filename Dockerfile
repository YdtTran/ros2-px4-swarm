FROM osrf/ros:jazzy-desktop-full

# Cài đặt các thư viện cơ bản
RUN apt-get update && apt-get install -y \
    git cmake build-essential wget curl nano python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/env

# 1. Micro-XRCE-DDS-Agent
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/

# 2. PX4-Autopilot (SITL + Gazebo Harmonic)
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
RUN cd PX4-Autopilot && make px4_sitl_default

# 3. MAVROS — cài qua apt, không cần build từ source
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    ros-jazzy-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib dataset (bắt buộc cho MAVROS)
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# 4. Python deps cho thuật toán swarm
RUN pip3 install pyrvo pathfinding --break-system-packages

# 5. Auto-source
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /workspace/ros2_ws
