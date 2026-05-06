# 🛸 ROS 2 PX4 Swarm Simulation (Dockerized)

Nền tảng mô phỏng phân tán dành cho điều khiển bầy đàn UAV. Dự án được đóng gói 100% bằng Docker, sử dụng **ROS 2 Jazzy**, **PX4 SITL**, **Gazebo Harmonic** và giao thức **Micro XRCE-DDS**.

Kiến trúc Docker giúp loại bỏ hoàn toàn các lỗi xung đột môi trường, bảo vệ máy chủ (host) luôn sạch sẽ và hỗ trợ render đồ họa bằng CPU (Software Rendering) cho các máy không có card rời.

## 🏗 Kiến trúc Hệ thống

![Kiến trúc Hệ thống Bầy đàn UAV](docs/architecture.png)
*(Lưu ý: Mở file `docs/architecture.html` trên trình duyệt để xem sơ đồ tương tác với hiệu ứng luồng dữ liệu).* 

## ⚠️ Yêu cầu Hệ thống (Prerequisites)

*   **Hệ điều hành:** Ubuntu 24.04 LTS.
*   **Môi trường hiển thị:** Bắt buộc sử dụng **X11** (Không dùng Wayland).

> **Cách chuyển Wayland sang X11 trên Ubuntu 24.04:**
> 1. Mở terminal, gõ: `sudo nano /etc/gdm3/custom.conf`
> 2. Tìm dòng `#WaylandEnable=false` và bỏ dấu thăng để thành `WaylandEnable=false`.
> 3. Lưu file (Ctrl+O, Enter, Ctrl+X) và khởi động lại máy tính (`reboot`).
> 4. Kiểm tra lại bằng lệnh: `echo $XDG_SESSION_TYPE`. Nếu kết quả in ra `x11` là thành công.

---

## 🚀 Hướng dẫn Cài đặt (Step-by-Step)

### Bước 1: Cài đặt Docker Engine
Nếu máy bạn chưa có Docker, hãy chạy chuỗi lệnh sau để cài đặt và cấp quyền:
```bash
# Cài đặt Docker
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL [https://download.docker.com/linux/ubuntu/gpg](https://download.docker.com/linux/ubuntu/gpg) -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] [https://download.docker.com/linux/ubuntu](https://download.docker.com/linux/ubuntu) \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Cấp quyền cho user (Không cần gõ sudo khi dùng docker)
sudo usermod -aG docker $USER
newgrp docker
```

### Bước 2: Khởi tạo Cấu trúc Dự án
Tạo không gian làm việc (Workspace) tại thư mục Home của bạn:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

Tạo file `Dockerfile` tại `~/ros2_ws/Dockerfile`:
```dockerfile
FROM osrf/ros:jazzy-desktop-full

# Cài đặt các thư viện cơ bản
RUN apt-get update && apt-get install -y \
    git cmake build-essential wget curl nano python3-pip \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/env

# 1. Cài đặt Micro-XRCE-DDS-Agent
RUN git clone -b v2.4.3 [https://github.com/eProsima/Micro-XRCE-DDS-Agent.git](https://github.com/eProsima/Micro-XRCE-DDS-Agent.git) \
    && cd Micro-XRCE-DDS-Agent && mkdir build && cd build \
    && cmake .. && make -j$(nproc) && make install && ldconfig /usr/local/lib/

# 2. Cài đặt PX4-Autopilot (Hỗ trợ SITL & Gazebo Harmonic)
RUN git clone [https://github.com/PX4/PX4-Autopilot.git](https://github.com/PX4/PX4-Autopilot.git) --recursive
RUN bash PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools
RUN cd PX4-Autopilot && make px4_sitl_default

# Thiết lập Workspace của dự án là thư mục mặc định
WORKDIR /workspace/ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

Tạo file `docker-compose.yml` tại `~/ros2_ws/docker-compose.yml`:
```yaml
services:
  swarm_env:
    build: .
    container_name: px4_swarm_jazzy
    network_mode: "host"       # Chia sẻ mạng lưới DDS với host
    privileged: true
    ipc: host                  # Tối ưu hóa bộ nhớ chia sẻ cho GUI
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
      - LIBGL_ALWAYS_SOFTWARE=1 # Ép dùng CPU để render 3D (Cho máy không có card rời)
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw # Cấp quyền hiển thị đồ họa
      - .:/workspace/ros2_ws             # Đồng bộ mã nguồn theo thời gian thực
    command: tail -f /dev/null           # Giữ container chạy ngầm
```

### Bước 3: Build và Khởi động Môi trường (Docker)
Tại thư mục `~/ros2_ws`, chạy lệnh sau để Docker tải và đóng gói môi trường. *(Lưu ý: Quá trình này có thể mất 15-20 phút trong lần chạy đầu tiên, các lần sau sẽ khởi động ngay lập tức).*
```bash
docker compose up -d
```

---

## 🎮 Luồng Làm Việc Hàng Ngày (Daily Workflow)

Mỗi khi bạn bật máy tính lên để làm việc, chỉ cần thực hiện 2 bước cực kỳ đơn giản:

**1. Cấp quyền xuất hình ảnh đồ họa (Chạy trên Terminal của máy thật):**
```bash
xhost +local:root
```

**2. Chui vào không gian mô phỏng của Docker:**
```bash
docker exec -it px4_swarm_jazzy bash
```

Lúc này, terminal của bạn đã ở bên trong Docker tại đường dẫn `/workspace/ros2_ws`. Mã nguồn bạn lưu ở máy thật bằng VS Code sẽ lập tức xuất hiện tại đây.

**Các lệnh thông dụng bên trong Docker:**
*   **Biên dịch code ROS 2:** 
    `colcon build`
*   **Cập nhật môi trường:** 
    `source install/setup.bash`
*   **Chạy file Launch (Ví dụ khởi tạo bầy đàn):** 
    `ros2 launch <tên_package> <tên_file_launch.py>`

> **Mẹo hiệu năng:** Nếu máy tính (CPU) chạy Gazebo quá giật lag, hãy thêm cờ `-s` vào lệnh gọi mô phỏng để tắt giao diện 3D (chạy Headless) và chỉ theo dõi máy bay thông qua QGroundControl.

---

## 🛑 Dọn dẹp & Tắt hệ thống
Khi làm việc xong, bạn có thể tắt môi trường Docker để giải phóng tài nguyên hệ thống (Mã nguồn code của bạn tại `~/ros2_ws` vẫn được giữ nguyên an toàn):
```bash
cd ~/ros2_ws
docker compose down
```
