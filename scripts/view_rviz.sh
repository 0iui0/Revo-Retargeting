#!/usr/bin/env bash
# Launch MuJoCo viewer + robot_state_publisher + RViz2 for Revo3 hand.
# The viewer publishes joint_states that RViz reads.
#
# Usage:
#   ./scripts/enter_jazzy.sh -- bash scripts/view_rviz.sh right
#   ./scripts/enter_jazzy.sh -- bash scripts/view_rviz.sh left
set -euo pipefail

SIDE="${1:-right}"
WS="/workspaces/isaac_ros-dev/Revo-Retargeting"

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

# ── Protocol config ──────────────────────────────────────────────────────────
SLAVE_ID=$([ "$SIDE" = "right" ] && echo 127 || echo 126)
cat > /tmp/protocol_${SIDE}.yaml << EOF
hardware:
  slave_id: ${SLAVE_ID}
  log_level: info
  port: /dev/ttyUSB1
  baudrate: 5000000
  auto_detect: false
  read_decimation: 4
EOF

# ── URDF ─────────────────────────────────────────────────────────────────────
echo "[rviz] Generating URDF..."
DRIVER_SHARE=$(ros2 pkg prefix revo3_driver 2>/dev/null)/share/revo3_driver
xacro "${WS}/src/brainco_revo3_ros2/revo3_description/urdf/revo3.single.system.xacro" \
    hand_side:=${SIDE} \
    protocol_config_file:=/tmp/protocol_${SIDE}.yaml \
    if_sim:=false \
    initial_positions_file:=${DRIVER_SHARE}/config/initial_positions_${SIDE}.yaml \
    > /tmp/revo3_${SIDE}.urdf 2>&1
echo "[rviz] URDF: $(wc -c < /tmp/revo3_${SIDE}.urdf) bytes"

# ── Robot State Publisher ────────────────────────────────────────────────────
echo "[rviz] Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -r __ns:=/revo3_${SIDE} \
    -p robot_description:="$(cat /tmp/revo3_${SIDE}.urdf)" \
    --remap joint_states:=/revo3_${SIDE}/revo3_joint_state/joint_states &
RSP_PID=$!
sleep 1.5

# ── RViz ─────────────────────────────────────────────────────────────────────
RVIZ_CFG="${WS}/src/brainco_revo3_ros2/revo3_description/rviz/revo3_hand.rviz"
echo "[rviz] Starting RViz..."
rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

# ── MuJoCo Viewer (publishes joint_states that RViz reads) ──────────────────
echo "[rviz] Starting MuJoCo viewer..."
python3 "${WS}/scripts/view_revo3_ros2.py" --hand "$SIDE" &
VIEWER_PID=$!

echo "[rviz] All running. Close RViz or Ctrl-C to stop."
trap "kill $RSP_PID $RVIZ_PID $VIEWER_PID 2>/dev/null; exit 0" INT

# Wait for any to exit
wait -n $RSP_PID $RVIZ_PID $VIEWER_PID 2>/dev/null
kill $RSP_PID $RVIZ_PID $VIEWER_PID 2>/dev/null
