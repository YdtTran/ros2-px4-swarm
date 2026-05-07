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

# 3. px4_msgs — build từ source để match đúng PX4 version trong image
RUN mkdir -p /opt/px4_msgs_ws/src \
    && git clone https://github.com/PX4/px4_msgs.git /opt/px4_msgs_ws/src/px4_msgs
RUN . /opt/ros/jazzy/setup.sh \
    && cd /opt/px4_msgs_ws \
    && colcon build --symlink-install

# 4. Python deps cho thuật toán swarm
RUN pip3 install pyrvo pathfinding --break-system-packages

# 5. Auto-source
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
    && echo "source /opt/px4_msgs_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

WORKDIR /workspace/ros2_ws
