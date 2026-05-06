FROM osrf/ros:jazzy-desktop-full

# Cài đặt các thư viện cơ bản
RUN apt-get update && apt-get install -y \
    git cmake build-essential wget curl nano python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/env

# 1. Cài đặt Micro-XRCE-DDS-Agent
RUN git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
    && cd Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/

# 2. Cài đặt PX4-Autopilot (Hỗ trợ SITL & Gazebo Harmonic)
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
# --no-nuttx: bỏ qua toolchain phần cứng, giữ lại Gazebo để gz_bridge được build
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx
RUN cd PX4-Autopilot && make px4_sitl_default

# Thiết lập Workspace của dự án là thư mục mặc định
WORKDIR /workspace/ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc