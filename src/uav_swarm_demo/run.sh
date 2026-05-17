#!/bin/bash
# UAV Swarm Demo — Gazebo + 3 UAV PX4 + XRCE-DDS + Swarm Algorithm

SHOW_GUI=0
DO_BUILD=1
for arg in "$@"; do
    case "$arg" in
        --gui)      SHOW_GUI=1  ;;
        --no-build) DO_BUILD=0  ;;
    esac
done

# ── Cấu hình map ──────────────────────────────────────────────────────────────
# Chỉ cần đổi dòng này để thay world Gazebo.
# Các world có sẵn trong PX4: default | baylands | lawn | sonoma_raceway | ...
GZ_WORLD=default
VIRTUAL_WALL_ENABLED=1
VIRTUAL_WALL_POSE="40 25 1 0 0 0"
VIRTUAL_WALL_SIZE="0.3 30.0 2.0"

# ── Dọn dẹp tiến trình cũ ─────────────────────────────────────────────────────
pkill -9 -x px4             2>/dev/null || true
pkill -9 -x MicroXRCEAgent  2>/dev/null || true
pkill -9 -f "gz sim"        2>/dev/null || true
pkill -9 -f "px4_swarm"     2>/dev/null || true
pkill -9 -f "mavlink_proxy" 2>/dev/null || true
sleep 2  # wait for killed processes to release ports and file locks

trap '
    echo -e "\n🛑 Dừng toàn bộ..."
    pkill -9 -x px4 2>/dev/null
    pkill -9 -x MicroXRCEAgent 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "px4_swarm" 2>/dev/null
    pkill -9 -f "mavlink_proxy" 2>/dev/null
    sleep 1
    rm -f /tmp/uav_swarm_server.config
    rm -f /tmp/uav_swarm_*_virtual_wall.sdf
    rm -rf "$BUILD_PATH/instance_0" "$BUILD_PATH/instance_1" "$BUILD_PATH/instance_2"
    exit
' SIGINT SIGTERM

PX4_PATH=/opt/env/PX4-Autopilot
BUILD_PATH=$PX4_PATH/build/px4_sitl_default

# Xóa thư mục instance cũ để tránh dataman stale ("Mission rejected: empty")
rm -rf "$BUILD_PATH/instance_0" "$BUILD_PATH/instance_1" "$BUILD_PATH/instance_2" 2>/dev/null || true

# ── Vá SDF ────────────────────────────────────────────────────────────────────
sed -i '/<gz_frame_id>/d' \
    $PX4_PATH/Tools/simulation/gz/models/x500_base/model.sdf 2>/dev/null
sed -i '/<plugin filename="MotorFailurePlugin"/,/<\/plugin>/d' \
    $PX4_PATH/Tools/simulation/gz/models/x500/model.sdf 2>/dev/null

# Mỗi UAV gửi MAVLink về port riêng: UAV0→14550, UAV1→14560, UAV2→14570
# -p: broadcast mode (QGC trên LAN tự discover)
# -o: remote port riêng cho từng instance
PX4_MAVLINK_RC=$BUILD_PATH/etc/init.d-posix/px4-rc.mavlink
sed -i 's/mavlink start -x -u \$udp_gcs_port_local -r 4000000 -f.*/mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -p -o $((14550 + px4_instance * 10))/' "$PX4_MAVLINK_RC"
sed -i '/^param set MAV_0_BROADCAST/d' "$PX4_MAVLINK_RC"

# SITL param tuning — remove stale lines then re-inject cleanly each run.
# Uses s/// so \n expands to real newlines in GNU sed (unlike /i text\nmore).
sed -i '/^param set EKF2_GPS_CHECK\|^param set EKF2_GPS_V_NOISE\|^param set EKF2_BARO_NOISE\|^param set EKF2_REQ_\|^param set FD_FAIL/d' "$PX4_MAVLINK_RC"
sed -i 's/^mavlink start/param set EKF2_GPS_CHECK 0\nparam set EKF2_GPS_V_NOISE 0.5\nparam set EKF2_BARO_NOISE 3.5\nparam set EKF2_REQ_VDRIFT 0.5\nparam set EKF2_REQ_HDRIFT 0.5\nparam set FD_FAIL_P 0.9\nparam set FD_FAIL_R 0.9\nmavlink start/' "$PX4_MAVLINK_RC"

# ── Bước 1/4: XRCE-DDS Agent ──────────────────────────────────────────────────
echo "🚀 [1/4] Micro XRCE-DDS Agent (port 8888)..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &
AGENT_PID=$!

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Start MAVLink proxy before PX4 — proxy binds :14560/:14570 and sends GCS
# heartbeats so PX4 UAV1/2 adopt those as partner ports.
# UAV0 connects to QGC directly (:14550); proxy must NOT touch UAV0.
python3 -u "$SCRIPT_DIR/../../scripts/mavlink_proxy.py" > /tmp/mavlink_proxy.log 2>&1 &
sleep 2  # proxy must bind its UDP ports before PX4 starts and picks a partner port

# ── Bước 2/4: Gazebo ──────────────────────────────────────────────────────────
echo "🌍 [2/4] Gazebo server..."
export GZ_SIM_RESOURCE_PATH=$PX4_PATH/Tools/simulation/gz/models:$PX4_PATH/Tools/simulation/gz/worlds
# Prefer the discrete NVIDIA GPU on PRIME / hybrid graphics systems.
export __NV_PRIME_RENDER_OFFLOAD=${__NV_PRIME_RENDER_OFFLOAD:-1}
export __GLX_VENDOR_LIBRARY_NAME=${__GLX_VENDOR_LIBRARY_NAME:-nvidia}
export __VK_LAYER_NV_optimus=${__VK_LAYER_NV_optimus:-NVIDIA_only}
export NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
export NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all}
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset MESA_GLSL_VERSION_OVERRIDE

if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader > /tmp/uav_swarm_nvidia_gpu.log 2>&1 || true
fi

CUSTOM_SERVER_CONFIG=/tmp/uav_swarm_server.config
grep -v 'OpticalFlow\|GstCamera' $PX4_PATH/Tools/simulation/gz/server.config \
    > $CUSTOM_SERVER_CONFIG
export GZ_SIM_SERVER_CONFIG_PATH=$CUSTOM_SERVER_CONFIG

WORLD_PATH=$PX4_PATH/Tools/simulation/gz/worlds/${GZ_WORLD}.sdf
if [ "$VIRTUAL_WALL_ENABLED" = "1" ]; then
    VIRTUAL_WORLD_PATH=/tmp/uav_swarm_${GZ_WORLD}_virtual_wall.sdf
    python3 - "$WORLD_PATH" "$VIRTUAL_WORLD_PATH" "$VIRTUAL_WALL_POSE" "$VIRTUAL_WALL_SIZE" <<'PY'
import sys
from pathlib import Path

src = Path(sys.argv[1])
dst = Path(sys.argv[2])
pose = sys.argv[3]
size = sys.argv[4]

world = src.read_text()
wall = f"""
    <model name="virtual_wall">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{size}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

if "name=\"virtual_wall\"" not in world:
    world = world.replace("</world>", f"{wall}\n  </world>", 1)
dst.write_text(world)
PY
    WORLD_PATH=$VIRTUAL_WORLD_PATH
    echo "🧱 Virtual wall enabled: pose=[$VIRTUAL_WALL_POSE], size=[$VIRTUAL_WALL_SIZE]"
fi

gz sim -s -r --headless-rendering \
    $WORLD_PATH 2>/dev/null &
GZ_PID=$!

if [ $SHOW_GUI -eq 1 ]; then
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p $XDG_RUNTIME_DIR
    _dnum="${DISPLAY##*:}"; _dnum="${_dnum%%.*}"
    if [ -S "/tmp/.X11-unix/X${_dnum}" ]; then
        echo "🖥️  Gazebo GUI..."
        # Strip ROS2 vendor libs so system OGRE is used with the NVIDIA GLX stack.
        _clean_ld=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':')
        LD_LIBRARY_PATH="$_clean_ld" gz sim -g --render-engine ogre > /tmp/gz_gui.log 2>&1 &
    else
        echo "⚠️  GUI yêu cầu X11. Chạy lệnh này trên HOST trước rồi restart container:"
        echo "     xhost +local:"
        echo "     docker compose down && docker compose up -d"
    fi
fi

echo "⏳ Chờ Gazebo sẵn sàng..."
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/.*/clock"; then
        echo "✅ Gazebo sẵn sàng! (${i}s)"
        break
    fi
    sleep 1
done

# ── Bước 3/4: 3 UAV PX4 ──────────────────────────────────────────────────────
echo "🛸 [3/4] Khởi động 3 UAV PX4..."
export PX4_SIM_LOCKSTEP=0
export PX4_GZ_STANDALONE=1
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=$GZ_WORLD
export PATH=$BUILD_PATH/bin:$PATH

if [ ! -f "$BUILD_PATH/bin/gz_bridge" ]; then
    ln -s "$BUILD_PATH/bin/px4-gz_bridge" "$BUILD_PATH/bin/gz_bridge"
fi

for i in 0 1 2; do
    INSTANCE_DIR="$BUILD_PATH/instance_$i"
    mkdir -p "$INSTANCE_DIR"
    pushd "$INSTANCE_DIR" > /dev/null
    echo "  → UAV $i tại x=$((i * 2))m"
    PX4_GZ_MODEL_POSE="$((i * 2)),0,0,0,0,0" \
        $BUILD_PATH/bin/px4 -i $i -d $BUILD_PATH/etc > out.log 2> err.log &
    popd > /dev/null
    sleep 3  # stagger spawns so Gazebo physics settles before the next model is inserted
done

# Chờ UAV spawn và kết nối XRCE-DDS
echo "⏳ Chờ UAV kết nối XRCE-DDS..."
source /opt/ros/jazzy/setup.bash
source /opt/px4_msgs_ws/install/setup.bash

for i in 0 1 2; do
    echo -n "  UAV ${i}: "
    # Instance 0 → bare /fmu/..., instance N → /px4_N/fmu/...
    if [ $i -eq 0 ]; then
        TOPIC_PATTERN="/fmu/out/vehicle_local_position"
    else
        TOPIC_PATTERN="/px4_${i}/fmu"
    fi
    for attempt in $(seq 1 30); do
        if ros2 topic list 2>/dev/null | grep -qF "$TOPIC_PATTERN"; then
            echo "✅ connected (${attempt}s)"
            break
        fi
        sleep 1
        if [ $attempt -eq 30 ]; then
            echo "❌ timeout — xem /tmp/xrce_agent.log"
        fi
    done
done

echo "⏳ Chờ EKF2 hội tụ (pre_flight_checks_pass, cần 3 lần liên tiếp)..."
for i in 0 1 2; do
    if [ $i -eq 0 ]; then NS=""; else NS="/px4_${i}"; fi
    TOPIC="${NS}/fmu/out/vehicle_status_v4"
    echo -n "  UAV ${i}: "
    ok=0; consec=0
    for t in $(seq 1 90); do
        val=$(ros2 topic echo --once "$TOPIC" 2>/dev/null | grep "pre_flight_checks_pass" | grep -o "true" || true)
        if [ "$val" = "true" ]; then
            consec=$((consec + 1))
            if [ $consec -ge 3 ]; then
                echo "✅ preflight stably OK (${t}s)"; ok=1; break
            fi
        else
            consec=0
        fi
        sleep 1
    done
    [ $ok -eq 0 ] && echo "⚠️  timeout 90s — tiếp tục dù preflight chưa ổn định"
done

# ── Bước 4/4: Build + Swarm Node ──────────────────────────────────────────────
echo "🤖 [4/4] Swarm Algorithm Node..."

cd /workspace/ros2_ws
if [ $DO_BUILD -eq 1 ]; then
    colcon build --packages-select uav_swarm_demo --symlink-install --packages-ignore px4_msgs
fi
source install/setup.bash

ros2 run uav_swarm_demo px4_swarm &
SWARM_PID=$!

echo ""
echo "======================================================="
echo "✅ UAV Swarm Demo đang chạy!"
echo "   3 UAV: ARM → TAKEOFF 5m → Bay đội hình V-shape"
echo "   Mở QGroundControl để theo dõi."
echo "   Nhấn Ctrl+C để dừng toàn bộ."
echo "======================================================="

wait $AGENT_PID $GZ_PID $SWARM_PID 2>/dev/null
