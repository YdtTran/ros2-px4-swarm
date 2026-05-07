#!/bin/bash
# UAV Swarm Demo — script khởi động độc lập
# Khởi động toàn bộ: XRCE-DDS + Gazebo + 3 UAV PX4 + Swarm Algorithm Node

SHOW_GUI=0
if [ "$1" = "--gui" ]; then
    SHOW_GUI=1
fi

# Dọn dẹp tiến trình cũ
pkill -x px4             2>/dev/null || true
pkill -x MicroXRCEAgent  2>/dev/null || true
pkill -f "gz sim"        2>/dev/null || true
pkill -f "px4_swarm"     2>/dev/null || true
sleep 1

trap '
    echo -e "\n🛑 Dừng toàn bộ..."
    pkill -x px4 2>/dev/null
    pkill -x MicroXRCEAgent 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "px4_swarm" 2>/dev/null
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

# ── Bước 1: XRCE-DDS Agent ────────────────────────────────────────────────────
echo "🚀 [1/4] Micro XRCE-DDS Agent (port 8888)..."
MicroXRCEAgent udp4 -p 8888 -v 3 &
AGENT_PID=$!
sleep 2

# ── Bước 2: Gazebo ────────────────────────────────────────────────────────────
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

# ── Bước 3: 3 UAV PX4 ────────────────────────────────────────────────────────
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

echo "⏳ Chờ EKF2 hội tụ (15s)..."
sleep 15

# ── Bước 4: Build + chạy Swarm Algorithm Node ────────────────────────────────
echo "🤖 [4/4] Swarm Algorithm Node (A* + Formation + ORCA)..."

source /opt/ros/jazzy/setup.bash
source /opt/px4_msgs_ws/install/setup.bash

echo "📦 Building uav_swarm_demo..."
cd /workspace/ros2_ws
colcon build --packages-select uav_swarm_demo --symlink-install
source install/setup.bash

echo "🚀 Khởi động px4_swarm node..."
ros2 run uav_swarm_demo px4_swarm &
SWARM_PID=$!

echo ""
echo "======================================================="
echo "✅ UAV Swarm Demo đang chạy!"
echo "   3 UAV sẽ: ARM → TAKEOFF 5m → Bay đội hình V-shape"
echo "   Mở QGroundControl để theo dõi."
echo "   Nhấn Ctrl+C để dừng toàn bộ."
echo "======================================================="

wait $AGENT_PID $GZ_PID $SWARM_PID
