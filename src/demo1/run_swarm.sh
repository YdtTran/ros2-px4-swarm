#!/bin/bash

# Kiểm tra flag --gui
SHOW_GUI=0
if [ "$1" = "--gui" ]; then
    SHOW_GUI=1
fi

# Dọn dẹp các tiến trình cũ
pkill -x px4 || true
pkill -x MicroXRCEAgent || true
pkill -f "gz sim" || true
sleep 1

trap 'echo -e "\n🛑 Đang dọn dẹp hệ thống..."; pkill -x px4; pkill -x MicroXRCEAgent; pkill -f "gz sim"; exit' SIGINT SIGTERM

PX4_PATH=/opt/env/PX4-Autopilot
BUILD_PATH=$PX4_PATH/build/px4_sitl_default

# ── Vá SDF: xoá warnings + tối ưu hiệu năng ──────────────────
X500_BASE=$PX4_PATH/Tools/simulation/gz/models/x500_base/model.sdf
X500=$PX4_PATH/Tools/simulation/gz/models/x500/model.sdf
BAYLANDS=$PX4_PATH/Tools/simulation/gz/worlds/baylands.sdf

# Xoá gz_frame_id (non-standard) và MotorFailurePlugin (không tồn tại)
sed -i '/<gz_frame_id>/d' $X500_BASE 2>/dev/null
sed -i '/<plugin filename="MotorFailurePlugin"/,/<\/plugin>/d' $X500 2>/dev/null

# Giảm update rate của gpu_lidar xuống 5Hz (mặc định thường 10-20Hz)
find $PX4_PATH/Tools/simulation/gz/models/ -name "*.sdf" \
    -exec sed -i '/<sensor.*gpu_lidar/,/<\/sensor>/ s|<update_rate>[0-9.]*</update_rate>|<update_rate>5</update_rate>|g' {} \; 2>/dev/null

# Tắt shadows trong baylands (tốn GPU/CPU khi render)
sed -i 's/<shadows>1<\/shadows>/<shadows>0<\/shadows>/g' $BAYLANDS 2>/dev/null

# ── Bước 1: Micro XRCE-DDS Agent ──────────────────────────────
echo "🚀 [1/3] Bật Micro XRCE-DDS Agent ở port 8888..."
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
sleep 2

# ── Bước 2: Khởi động Gazebo server (headless, không cần GUI) ──
echo "🌍 [2/3] Khởi động Gazebo server (headless)..."
export GZ_SIM_RESOURCE_PATH=$PX4_PATH/Tools/simulation/gz/models:$PX4_PATH/Tools/simulation/gz/worlds
export LIBGL_ALWAYS_SOFTWARE=1

# Tạo server.config không có 2 plugin tuỳ chọn (OpticalFlow, GstCamera)
CUSTOM_SERVER_CONFIG=/tmp/px4_server.config
grep -v 'OpticalFlow\|GstCamera' $PX4_PATH/Tools/simulation/gz/server.config > $CUSTOM_SERVER_CONFIG
export GZ_SIM_SERVER_CONFIG_PATH=$CUSTOM_SERVER_CONFIG

# -s = server only, -r = tự chạy ngay, --headless-rendering = software render cho sensors
gz sim -s -r --headless-rendering $PX4_PATH/Tools/simulation/gz/worlds/baylands.sdf 2>/dev/null &
GZ_PID=$!

# Khởi động GUI nếu có flag --gui
if [ $SHOW_GUI -eq 1 ]; then
    echo "🖥️  Khởi động Gazebo GUI (render engine: ogre)..."
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    mkdir -p $XDG_RUNTIME_DIR
    DRI_PRIME=1 gz sim -g --render-engine ogre > /dev/null 2>&1 &
fi

# Đợi Gazebo world sẵn sàng (tối đa 30 giây)
echo "⏳ Đợi Gazebo world khởi động..."
READY=0
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/.*/clock"; then
        echo "✅ Gazebo world sẵn sàng! (${i}s)"
        READY=1
        break
    fi
    sleep 1
done

if [ $READY -eq 0 ]; then
    echo "❌ Gazebo không khởi động được sau 30 giây."
    exit 1
fi

# ── Bước 3: Khởi động 3 UAV PX4 ───────────────────────────────
echo "🛸 [3/3] Khởi động 3 UAV..."

export PX4_SIM_LOCKSTEP=0
export PX4_GZ_STANDALONE=1
export PX4_SIM_MODEL=gz_x500
export PX4_GZ_WORLD=baylands
export PATH=$BUILD_PATH/bin:$PATH

# px4-rc.gzsim tìm "gz_bridge" nhưng binary tên là "px4-gz_bridge" → tạo symlink
if [ ! -f "$BUILD_PATH/bin/gz_bridge" ]; then
    ln -s "$BUILD_PATH/bin/px4-gz_bridge" "$BUILD_PATH/bin/gz_bridge"
fi

for i in 0 1 2; do
    INSTANCE_DIR="$BUILD_PATH/instance_$i"
    mkdir -p "$INSTANCE_DIR"
    pushd "$INSTANCE_DIR" > /dev/null
    echo "  → UAV $i tại vị trí x=$((i * 2))m"
    PX4_GZ_MODEL_POSE="$((i * 2)),0,0,0,0,0" \
        $BUILD_PATH/bin/px4 -i $i -d $BUILD_PATH/etc > out.log 2> err.log &
    popd > /dev/null
    sleep 3
done

echo ""
echo "-------------------------------------------------------"
echo "✅ Bầy đàn 3 UAV đã sẵn sàng!"
echo ""
echo "👉 QGroundControl tự động kết nối cả 3 UAV."
echo "👉 Nhấn Ctrl+C để tắt toàn bộ mô phỏng."
echo "-------------------------------------------------------"

wait $AGENT_PID $GZ_PID
