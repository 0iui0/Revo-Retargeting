#!/usr/bin/env bash
# Launch MuJoCo viewer + robot_state_publisher + RViz2 for Revo3 hand.
#
# Usage:
#   ./scripts/enter_jazzy.sh -- bash scripts/view_rviz.sh right
set -eo pipefail

SIDE="${1:-right}"
WS="/workspaces/isaac_ros-dev/Revo-Retargeting"
MJCF="${WS}/scripts/revo3_right.xml"

source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"

# ── Generate MJCF from URDF if not already done ──────────────────────────────
if [ ! -f "$MJCF" ]; then
    echo "[rviz] Converting URDF to MJCF..."
    python3 -c "
import mujoco, os
urdf = '${WS}/src/brainco_revo3_ros2/revo3_description/urdf/revo3_${SIDE}_may3.urdf'
m = mujoco.MjModel.from_xml_path(urdf)
os.makedirs(os.path.dirname('${MJCF}'), exist_ok=True)
mujoco.mj_saveLastXML('${MJCF}', m)
print(f'MJCF saved: {m.njnt} joints')
" 2>&1
fi

# ── Generate URDF ────────────────────────────────────────────────────────────
cat > /tmp/protocol_${SIDE}.yaml << EOF
hardware:
  slave_id: $([ "$SIDE" = "right" ] && echo 127 || echo 126)
  log_level: info
  port: /dev/ttyUSB1
  baudrate: 5000000
  auto_detect: false
  read_decimation: 4
EOF

DRIVER_SHARE=$(ros2 pkg prefix revo3_driver)/share/revo3_driver
xacro "${WS}/src/brainco_revo3_ros2/revo3_description/urdf/revo3.single.system.xacro" \
    hand_side:=${SIDE} protocol_config_file:=/tmp/protocol_${SIDE}.yaml if_sim:=false \
    initial_positions_file:=${DRIVER_SHARE}/config/initial_positions_${SIDE}.yaml \
    > /tmp/revo3_${SIDE}.urdf 2>&1
echo "[rviz] URDF: $(wc -c < /tmp/revo3_${SIDE}.urdf) bytes"

# ── Robot State Publisher ────────────────────────────────────────────────────
echo "[rviz] Starting robot_state_publisher..."
python3 -c "
import rclpy, yaml, os
from rclpy.node import Node
class RSPLauncher(Node):
    def __init__(self):
        super().__init__('rsp_launcher')
        with open('/tmp/revo3_${SIDE}.urdf') as f:
            urdf = f.read()
        self.declare_parameter('robot_description', urdf)
rclpy.init()
n = RSPLauncher()
rclpy.spin(n)
" 2>&1 &
RSP_PID=$!

# Actually use ros2 run with proper params
kill $RSP_PID 2>/dev/null
python3 -c "
import subprocess, os
urdf = open('/tmp/revo3_${SIDE}.urdf').read()
# Write minimal YAML for params
with open('/tmp/rsp_params.yaml', 'w') as f:
    f.write('/revo3_${SIDE}/robot_state_publisher:\\n')
    f.write('  ros__parameters:\\n')
    f.write('    robot_description: \"' + urdf.replace('\\\\', '\\\\\\\\').replace('\"', '\\\\\"').replace('\\n', '\\\\n') + '\"\\n')
print('Params written')
" 2>&1

ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -r __ns:=/revo3_${SIDE} \
    --params-file /tmp/rsp_params.yaml \
    --remap joint_states:=/revo3_${SIDE}/revo3_joint_state/joint_states &
RSP_PID=$!
sleep 1.5

# ── RViz ─────────────────────────────────────────────────────────────────────
RVIZ_CFG="${WS}/src/brainco_revo3_ros2/revo3_description/rviz/revo3_hand.rviz"
echo "[rviz] Starting RViz..."
rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!

# ── MuJoCo Viewer ────────────────────────────────────────────────────────────
echo "[rviz] Starting MuJoCo viewer..."
python3 "${WS}/scripts/view_revo3_ros2.py" --hand "$SIDE" &
VIEWER_PID=$!

echo "[rviz] All running. Ctrl-C to stop."
trap "kill $RSP_PID $RVIZ_PID $VIEWER_PID 2>/dev/null; exit 0" INT
wait -n 2>/dev/null || wait
