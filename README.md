# 🛸 ROS 2 PX4 Swarm Simulation (ros2-px4-swarm-sim)

Một nền tảng mô phỏng phân tán dành cho điều khiển bầy đàn UAV (Multi-Agent), sử dụng **ROS 2 Jazzy**, **PX4 SITL**, **Gazebo Harmonic** và giao thức **Micro XRCE-DDS**.

## 🏗 Kiến trúc Hệ thống

![Kiến trúc Hệ thống Bầy đàn UAV](docs/architecture.png)
*(Lưu ý: Mở file `docs/architecture.html` trên trình duyệt để xem sơ đồ tương tác với hiệu ứng luồng dữ liệu).* 

Hệ thống được chia thành 5 tầng giao tiếp cốt lõi:
1. **Trạm mặt đất (QGroundControl):** Giám sát trạng thái, la bàn, GPS ảo và bắt bệnh lỗi (MAVLink).
2. **Lô-gic Điều khiển (ROS 2 Jazzy):** Não bộ trung tâm xử lý thuật toán bầy đàn, cấp phát tọa độ cho từng UAV.
3. **Cầu nối (Middleware):** Sử dụng `MicroXRCEAgent` để kết nối ROS 2 Topics với uORB của PX4, và `ros_gz_bridge` cho dữ liệu cảm biến trực tiếp.
4. **Firmware Bay (PX4 SITL):** Chạy độc lập cho từng UAV, tính toán RPM cho động cơ để giữ thăng bằng.
5. **Môi trường Vật lý (Gazebo Harmonic):** Mô phỏng trọng lực, gió, va chạm và xuất hình ảnh camera.

---

## 🛠 Yêu cầu Hệ thống (Prerequisites)

Dự án này được phát triển và kiểm thử trên môi trường:
* **OS:** Ubuntu 24.04 LTS
* **ROS 2:** Jazzy Jalisco
* **Gazebo:** Harmonic
* **PX4 Autopilot:** Phiên bản mã nguồn mới nhất (hỗ trợ SITL)
* **QGroundControl:** v4.4.0 (AppImage)

---

## 🚀 Hướng dẫn Cài đặt (Installation)

### 1. Cài đặt ROS 2 Jazzy & Gazebo
Đảm bảo bạn đã cài ROS 2 Jazzy. Cài đặt Gazebo Harmonic và Cầu nối:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz
```

### 2. Cài đặt PX4 Autopilot
Clone bộ mã nguồn PX4 và cài đặt các thư viện lõi (Lưu ý: Quá trình này có thể mất vài phút).
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```
*(Khởi động lại máy tính sau khi chạy xong script này).*

### 3. Cài đặt Micro XRCE-DDS Agent
Cầu nối bắt buộc để ROS 2 giao tiếp với PX4.
```bash
cd ~
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

---

## 🎮 Hướng dẫn Khởi chạy (How to Run)

Để hệ thống hoạt động, bạn cần mở **4 Terminal** khác nhau và chạy lần lượt theo đúng thứ tự sau:

### Terminal 1: Mở QGroundControl
*(Giám sát và vượt qua các lỗi pre-flight check của PX4)*
```bash
cd ~
./QGroundControl.AppImage
```

### Terminal 2: Khởi động Micro XRCE-DDS Agent
*(Tạo đường truyền UDP)*
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 3: Khởi động PX4 SITL & Gazebo
*(Bật máy bay và môi trường vật lý)*
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
> **Mẹo:** Nếu bị lỗi `tone_alarm` (tiếng bíp) do chưa có GPS, hãy mở QGroundControl để kiểm tra lỗi, hoặc gõ `param set COM_ARM_WO_GPS 1` trong terminal của PX4.

### Terminal 4: Điều khiển bằng ROS 2
*(Xác nhận kết nối)*
```bash
ros2 topic list
```
*(Nếu bạn thấy các topic `/fmu/...` hiện ra, hệ thống đã thông suốt. Bạn có thể bắt đầu chạy các Node điều khiển bầy đàn của mình).* 

---

## 👥 Tác giả & Đóng góp
Dự án được xây dựng phục vụ nghiên cứu điều khiển phân tán. Mọi đóng góp (Pull Requests) liên quan đến thuật toán tránh va chạm (Collision Avoidance) hoặc lập kế hoạch quỹ đạo (Trajectory Planning) đều được hoan nghênh.
