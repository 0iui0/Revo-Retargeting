# Revo3 Retargeting — 架构与使用指南

## 数据流

```
Manus 手套 (MetaglovePro)                    MuJoCo Viewer (3D 可视化)
    │ ~95Hz                                          │
    ▼                                                │
manus_data_publisher (C++)                            │
    │ /manus_glove_0 (keypoints + ergonomics)         │
    ▼                                                │
retarget_node (C++, Pinocchio IK)                     │
    │ /revo3_right/joint_forward_mit_controller/commands (Revo3MITCommand, rad)
    │ /revo3_right/joint_forward_mit_controller/retarget_targets
    ▼                                                │
revo3_hw_bridge.py (Python, bc-stark-sdk)             │
    │ EMA 低通滤波 (--smoothing 0.3)                  │
    │ Modbus RS485 → Revo3 真机                       │
    ▼                                                │
┌──────────────────────────┐                         │
│ Revo3 21-DOF 灵巧手       │ ◄── joint_states ──────┘
│ /dev/ttyUSB1, slave 127  │      /revo3_right/revo3_joint_state/joint_states
│ MIT 阻抗控制模式           │
└──────────────────────────┘
```

## 代码结构

```
Revo-Retargeting/
├── scripts/
│   ├── enter_jazzy.sh         # Docker 容器管理 (一键进入 Jazzy 环境)
│   ├── teleop.sh              # 遥操作启动 (官方)
│   ├── teleop_revo3.sh        # Revo3 遥操作 (官方)
│   ├── start_revo3_driver.sh  # 硬件驱动启动 (需要 ros2_control)
│   ├── view_revo3_ros2.py     # MuJoCo 3D 可视化 + 本地 retarget
│   ├── thumb_ik_pinocchio.py  # Python Pinocchio 拇指 IK (与 C++ 对称)
│   ├── revo3_hw_bridge.py     # 真机桥接 (retarget → bc-stark-sdk → 硬件)
│   └── view_rviz.sh           # RViz 可视化启动
│
├── src/
│   ├── manus_ros2/            # Manus SDK 桥接 (C++)
│   ├── manus_ros2_msgs/       # ManusGlove 消息定义
│   ├── manus_revo3_retarget/  # ★ C++ retarget 管线
│   │   ├── src/
│   │   │   ├── retarget_node_cpp.cpp  # 主节点: spread + four_finger + thumb IK
│   │   │   ├── thumb_retarget.cpp     # 拇指 Pinocchio DLS IK
│   │   │   ├── spread_retarget.cpp    # 四指侧摆 (MiddleSpread 动态基准)
│   │   │   └── four_finger_retarget.cpp  # 四指屈伸
│   │   ├── config/             # YAML 参数
│   │   ├── launch/             # 启动文件
│   │   └── resource/           # Jazzy 适配补丁
│   │
│   └── brainco_revo3_ros2/    # ★ Revo3 驱动子模块
│       ├── revo3_driver/       # ros2_control 硬件插件 (C++)
│       ├── revo3_mit_controller/  # MIT 控制器
│       ├── revo3_mit_controller_msgs/  # Revo3MITCommand 消息
│       └── revo3_description/  # URDF/Xacro 手部模型
│
├── ARCHITECTURE.md             # 本文档
└── README.md                   # 官方说明
```

## 包依赖关系

```
manus_ros2_msgs ─────────────────────────┐
manus_ros2 ──────────────┤               │
revo3_mit_controller_msgs ───────────────┤
revo3_description ───────────────────────┤
revo3_mit_controller ─────┤              │
revo3_driver ─────────────┤              │
manus_revo3_retarget ─────┴──────────────┘
    │ (libmanus_revo3_retarget_thumb_pinocchio.so)
    └── 动态加载 Pinocchio IK 插件
```

## Retarget 算法

### Spread (侧摆) — spread_retarget.cpp

```
                   finger_spread_sign = -1.0
                   middle_dynamic = true

MiddleSpread ──────┬──→ middle_ref (动态基准)
                   │
IndexSpread ───────┤
RingSpread ────────┤   每指 = (SENSOR - middle_ref - offset) × scale × finger_spread_sign
PinkySpread ───────┘
                         offset: index=15°, middle=-5°, ring=-5°, pinky=0°
```

### Four-Finger Flexion (屈伸) — four_finger_retarget.cpp

```
Stretch → deg × per_finger_scale × mcp_scale (for MCP joints)
         deg × per_finger_scale             (for PIP joints)
         deg × per_finger_scale × dip_scale (for DIP joints)
```

### Thumb IK — thumb_retarget.cpp

```
Manus keypoints (25 nodes)
    │
    ├─ 4-finger tips → 中心点 center
    │
    ├─ Thumb tip (node 4) → transform → reach-scale → EMA filter → target
    │   EMA: right=0.4/0.6, left=0.9/0.1
    │
    ├─ Pinocchio DLS IK:
    │   ┌─ Tip position task  (weight 2.0)
    │   ├─ PIP position task  (weight 0.1)
    │   ├─ DIP position task  (weight 0.1)
    │   ├─ Posture prior      (weight 0.1)
    │   └─ Temporal smooth    (weight 0.1)
    │
    └─ Output: 5 motor angles (CMP, CMR, MCP, PIP, DIP) → physical calibration
```

## 官方可视化工具

| 工具 | 命令 | 说明 |
|------|------|------|
| MuJoCo 3D Viewer | `python3 scripts/view_revo3_ros2.py` | 手部模型 + 手套骨架 + 实时角度 |
| Tk 时序曲线 | `ros2 run manus_revo3_retarget command_state_viewer --hand-mode right` | 每个关节 command vs state |
| Tk 调参面板 | `ros2 run manus_revo3_retarget retarget_tuning_panel` | 在线调参 |
| RViz2 | `rviz2 -d src/brainco_revo3_ros2/revo3_description/rviz/revo3_hand.rviz` | 标准 ROS 可视化 |
| 录制/回放 | `ros2 run manus_revo3_retarget manus_record / ros2 run manus_revo3_retarget manus_replay` | Manus 数据录制回放 |

---

## 快速开始

### 1. 进入容器

```bash
./scripts/enter_jazzy.sh
```

首次运行会自动创建容器、安装依赖（pinocchio、mujoco、ros2_control 等），需要几分钟。之后秒进。

### 2. 启动遥操作（3 个终端）

```bash
# 终端 1: Manus 数据采集
./scripts/enter_jazzy.sh -- ros2 run manus_ros2 manus_data_publisher

# 终端 2: C++ Retarget 管线
./scripts/enter_jazzy.sh -- ros2 run manus_revo3_retarget retarget_node --ros-args -p hand_mode:=right

# 终端 3: 真机驱动 (跳过 ros2_control)
./scripts/enter_jazzy.sh -- python3 scripts/revo3_hw_bridge.py --hand right --smoothing 0.5
```

### 3. 可视化（终端 4）

```bash
# MuJoCo 3D (推荐)
./scripts/enter_jazzy.sh -- python3 scripts/view_revo3_ros2.py

# 或 Tk 时序曲线
./scripts/enter_jazzy.sh -- ros2 run manus_revo3_retarget command_state_viewer --hand-mode right
```

### 4. 一键启动（使用官方脚本）

```bash
# 跳过 ros2_control，复用 manus + retarget
START_REVO3_DRIVER=0 ./scripts/teleop.sh right

# 另一个终端：真机桥接
./scripts/enter_jazzy.sh -- python3 scripts/revo3_hw_bridge.py --hand right --smoothing 0.5
```

---

## 参数调优

### 抖动控制

| 参数 | 默认值 | 效果 |
|------|--------|------|
| `--smoothing 0.0` | | 无滤波，响应最快但可能抖动 |
| `--smoothing 0.3` | ★ 默认 | 轻度滤波 |
| `--smoothing 0.5` | | 较强滤波，更平滑但稍有延迟 |

### retarget_node 参数

```bash
# 完整参数
ros2 run manus_revo3_retarget retarget_node --ros-args \
  -p hand_mode:=right \
  -p mit_command_publish_hz:=200.0 \
  -p mit_default_kp:=0.4 \
  -p mit_default_kd:=0.05
```

---

## 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `START_REVO3_DRIVER` | 1 | `teleop.sh` 是否启动 ros2_control 驱动 |
| `START_MANUS_PUBLISHER` | 1 | `teleop.sh` 是否启动 Manus 发布器 |
| `RMW_IMPLEMENTATION` | (fastrtps) | DDS 中间件 (建议 Cyclone DDS) |
| `ISAAC_ROS_WS` | `$HOME/workspaces/isaac_ros-dev` | 主机工作区路径 |

---

## 常见问题

**Q: retarget_node 报 `libpinocchio_parsers.so not found`**
A: `enter_jazzy.sh` 已自动处理 LD_LIBRARY_PATH。如手动 source 需加：
```bash
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

**Q: bc_stark_sdk 找不到**
A: `enter_jazzy.sh` 会在每次进入时自动检测并复制。确保主机 `~/.local` 下有 bc-stark-sdk。

**Q: MuJoCo viewer 找不到 MJCF**
A: 确保 `brainco-description` 已挂载。`enter_jazzy.sh` 自动挂载 `$ISAAC_ROS_WS/brainco-description`。

**Q: 为什么不直接用 `ros2 launch` 启动 revo3_driver**
A: Jazzy 版 `controller_manager` 与官方 Humble 模板有兼容性问题（参数加载 + joint_state_broadcaster 崩溃）。`revo3_hw_bridge.py` 是等效替代。
