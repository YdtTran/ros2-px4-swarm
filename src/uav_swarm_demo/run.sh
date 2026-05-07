#!/bin/bash
# UAV Swarm Demo — tự chứa, không phụ thuộc demo1
# Gazebo + 3 UAV PX4 + MAVROS + Swarm Algorithm

SHOW_GUI=0
if [ "$1" = "--gui" ]; then
    SHOW_GUI=1
fi

# ── Dọn dẹp tiến trình cũ ─────────────────────────────────────────────────────
pkill -9 -x px4            2>/dev/null || true
pkill -9 -f "gz sim"       2>/dev/null || true
pkill -9 -f "mavros_node"  2>/dev/null || true
pkill -9 -f "px4_swarm"    2>/dev/null || true
sleep 3   # chờ process chết hẳn và giải phóng port

trap '
    echo -e "\n🛑 Dừng toàn bộ..."
    pkill -9 -x px4 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    pkill -9 -f "mavros_node" 2>/dev/null
    pkill -9 -f "px4_swarm" 2>/dev/null
    [ -n "$FOLLOW_PID" ] && kill $FOLLOW_PID 2>/dev/null
    sleep 1
    rm -f /tmp/uav_swarm_server.config /tmp/mavros_*.log
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

# ── Bước 1/4: Gazebo ──────────────────────────────────────────────────────────
echo "🌍 [1/4] Gazebo server..."
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

# ── Bước 2/4: 3 UAV PX4 ──────────────────────────────────────────────────────
echo "🛸 [2/4] Khởi động 3 UAV PX4..."
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

# Kiểm tra UAV spawn thành công
echo "⏳ Chờ UAV xuất hiện trong Gazebo..."
for i in 0 1 2; do
    for attempt in $(seq 1 10); do
        if gz topic -l 2>/dev/null | grep -q "x500_${i}"; then
            echo "  ✅ UAV ${i} đã spawn"
            break
        fi
        sleep 2
        if [ $attempt -eq 10 ]; then
            echo "  ❌ UAV ${i} không spawn được. Xem log:"
            tail -5 "$BUILD_PATH/instance_${i}/err.log"
        fi
    done
done

# Chase camera: luôn nhìn từ phía sau UAV 0 theo hướng bay
if [ $SHOW_GUI -eq 1 ]; then
    echo "📷 Chase camera UAV 0..."
    ros2 run uav_swarm_demo gz_chase_cam &
    FOLLOW_PID=$!
    echo "  ✅ Chase cam started (PID=$FOLLOW_PID)"
fi

echo "⏳ Chờ EKF2 hội tụ (15s)..."
sleep 15

source /opt/ros/jazzy/setup.bash

# ── Bước 3/4: MAVROS ──────────────────────────────────────────────────────────
echo "📡 [3/4] Khởi động MAVROS (3 UAV)..."
# PX4 SITL MAVLink ports:
#   Instance i → gửi đến 14540+i, nhận từ 14580+i
# fcu_url format: udp://:LOCAL@localhost:REMOTE
#   LOCAL  = port MAVROS lắng nghe (PX4 gửi đến đây)
#   REMOTE = port PX4 lắng nghe    (MAVROS gửi đến đây)
for i in 0 1 2; do
    LOCAL_PORT=$((14540 + i))
    REMOTE_PORT=$((14580 + i))
    ros2 run mavros mavros_node --ros-args \
        -r __ns:=/uav${i} \
        -p fcu_url:="udp://:${LOCAL_PORT}@localhost:${REMOTE_PORT}" \
        -p tgt_system:=$((i + 1)) \
        > /tmp/mavros_${i}.log 2>&1 &
    echo "  → MAVROS /uav${i} (fcu: :${LOCAL_PORT}@${REMOTE_PORT}, tgt_system=$((i+1)))"
    sleep 2
done

echo "⏳ Chờ MAVROS kết nối FCU..."
for i in 0 1 2; do
    echo -n "  UAV ${i}: "
    for attempt in $(seq 1 30); do
        if grep -q "detected remote address" /tmp/mavros_${i}.log 2>/dev/null; then
            echo "✅ kết nối (${attempt}s)"
            break
        fi
        sleep 1
        if [ $attempt -eq 30 ]; then
            echo "❌ timeout — xem: /tmp/mavros_${i}.log"
        fi
    done
done

# ── Bước 4/4: Swarm Algorithm Node ───────────────────────────────────────────
echo "🤖 [4/4] Swarm Algorithm Node..."

cd /workspace/ros2_ws
colcon build --packages-select uav_swarm_demo --symlink-install
source install/setup.bash

# Restart MAVROS để tránh stale link sau khi swarm node cũ tắt
echo "🔄 Restart MAVROS (tránh stale link)..."
pkill -f mavros_node 2>/dev/null; sleep 2
for i in 0 1 2; do
    LOCAL_PORT=$((14540 + i))
    REMOTE_PORT=$((14580 + i))
    ros2 run mavros mavros_node --ros-args \
        -r __ns:=/uav${i} \
        -p fcu_url:="udp://:${LOCAL_PORT}@localhost:${REMOTE_PORT}" \
        -p tgt_system:=$((i + 1)) \
        >> /tmp/mavros_${i}.log 2>&1 &
    sleep 1
done

# Đợi cả 3 UAV báo connected=True trên state topic
echo "⏳ Chờ FCU connected sau build..."
for i in 0 1 2; do
    echo -n "  UAV ${i}: "
    for attempt in $(seq 1 60); do
        connected=$(ros2 topic echo /uav${i}/state \
            --once --field connected 2>/dev/null | grep -m1 -iE '^true')
        if [ -n "$connected" ]; then
            echo "✅ connected (${attempt}s)"
            break
        fi
        sleep 1
        if [ $attempt -eq 60 ]; then
            echo "❌ timeout — xem /tmp/mavros_${i}.log"
        fi
    done
done

ros2 run uav_swarm_demo px4_swarm &
SWARM_PID=$!

echo ""
echo "======================================================="
echo "✅ UAV Swarm Demo đang chạy!"
echo "   3 UAV: ARM → TAKEOFF 5m → Bay đội hình V-shape"
echo "   Mở QGroundControl để theo dõi."
echo "   Nhấn Ctrl+C để dừng toàn bộ."
echo "======================================================="

wait $GZ_PID $SWARM_PID 2>/dev/null
