#!/bin/bash
# UAV Swarm Demo — Gazebo + 3 UAV PX4 + XRCE-DDS + Swarm Algorithm

SHOW_GUI=0
if [ "$1" = "--gui" ]; then
    SHOW_GUI=1
fi

# ── Dọn dẹp tiến trình cũ ─────────────────────────────────────────────────────
pkill -9 -x px4             2>/dev/null || true
pkill -9 -x MicroXRCEAgent  2>/dev/null || true
pkill -9 -f "gz sim"        2>/dev/null || true
pkill -9 -f "px4_swarm"     2>/dev/null || true
pkill -9 -f "gcs_heartbeat" 2>/dev/null || true
sleep 2

trap '
    echo -e "\n🛑 Dừng toàn bộ..."
    pkill -9 -x px4 2>/dev/null
    pkill -9 -x MicroXRCEAgent 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "px4_swarm" 2>/dev/null
    pkill -9 -f "gcs_heartbeat" 2>/dev/null
    sleep 1
    rm -f /tmp/uav_swarm_server.config
    rm -rf "$BUILD_PATH/instance_0" "$BUILD_PATH/instance_1" "$BUILD_PATH/instance_2"
    exit
' SIGINT SIGTERM

PX4_PATH=/opt/env/PX4-Autopilot
BUILD_PATH=$PX4_PATH/build/px4_sitl_default

# ── Vá SDF ────────────────────────────────────────────────────────────────────
sed -i '/<gz_frame_id>/d' \
    $PX4_PATH/Tools/simulation/gz/models/x500_base/model.sdf 2>/dev/null
sed -i '/<plugin filename="MotorFailurePlugin"/,/<\/plugin>/d' \
    $PX4_PATH/Tools/simulation/gz/models/x500/model.sdf 2>/dev/null
sed -i 's/<shadows>1<\/shadows>/<shadows>0<\/shadows>/g' \
    $PX4_PATH/Tools/simulation/gz/worlds/baylands.sdf 2>/dev/null

# ── Bước 1/4: XRCE-DDS Agent ──────────────────────────────────────────────────
echo "🚀 [1/4] Micro XRCE-DDS Agent (port 8888)..."
MicroXRCEAgent udp4 -p 8888 > /tmp/xrce_agent.log 2>&1 &
AGENT_PID=$!

# Fake GCS heartbeats so PX4 SITL passes the "No connection to GCS" preflight check
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$SCRIPT_DIR/../../scripts/gcs_heartbeat.py" > /dev/null 2>&1 &
sleep 2

# ── Bước 2/4: Gazebo ──────────────────────────────────────────────────────────
echo "🌍 [2/4] Gazebo server..."
export GZ_SIM_RESOURCE_PATH=$PX4_PATH/Tools/simulation/gz/models:$PX4_PATH/Tools/simulation/gz/worlds
export LIBGL_ALWAYS_SOFTWARE=1

CUSTOM_SERVER_CONFIG=/tmp/uav_swarm_server.config
grep -v 'OpticalFlow\|GstCamera' $PX4_PATH/Tools/simulation/gz/server.config \
    > $CUSTOM_SERVER_CONFIG
export GZ_SIM_SERVER_CONFIG_PATH=$CUSTOM_SERVER_CONFIG

gz sim -s -r --headless-rendering \
    $PX4_PATH/Tools/simulation/gz/worlds/baylands.sdf 2>/dev/null &
GZ_PID=$!

if [ $SHOW_GUI -eq 1 ]; then
    echo "🖥️  Gazebo GUI..."
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p $XDG_RUNTIME_DIR
    DRI_PRIME=1 gz sim -g --render-engine ogre > /dev/null 2>&1 &
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
export PX4_GZ_WORLD=baylands
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
    sleep 3
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

echo "⏳ Chờ EKF2 hội tụ (15s)..."
sleep 15

# ── Bước 4/4: Build + Swarm Node ──────────────────────────────────────────────
echo "🤖 [4/4] Swarm Algorithm Node..."

cd /workspace/ros2_ws
colcon build --packages-select uav_swarm_demo --symlink-install
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
