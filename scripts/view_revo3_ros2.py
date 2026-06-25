#!/usr/bin/env python3
"""Revo3 + Manus Skeleton ROS2 MuJoCo Visualization.

Uses the official brainco-description MJCF (with proper visual/collision meshes).
Subscribes to /revo3/right/set_motor_multi and /manus_glove_0.

Usage (inside container):
  source install/setup.bash
  python3 scripts/view_revo3_ros2.py
  python3 scripts/view_revo3_ros2.py --hand right
"""

import argparse
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove
from sensor_msgs.msg import JointState

# Optional: old Stark SDK motor topic (may not exist in official build)
try:
    from ros2_stark_interfaces.msg import SetMotorMulti
    _has_stark_msgs = True
except ImportError:
    SetMotorMulti = None
    _has_stark_msgs = False

# Official C++ retarget publishes Revo3MITCommand
try:
    from revo3_mit_controller_msgs.msg import Revo3MITCommand
    _has_mit_msgs = True
except ImportError:
    Revo3MITCommand = None
    _has_mit_msgs = False


# ── MJCF path ────────────────────────────────────────────────────────────────
_MJCF_CANDIDATES = [
    "/workspaces/brainco-description/revo3_system/mjcf/revo3_right.xml",
    "/workspaces/isaac_ros-dev/brainco-description/revo3_system/mjcf/revo3_right.xml",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "revo3_right.xml"),
]

MJCF_PATH = None
for p in _MJCF_CANDIDATES:
    if os.path.exists(p):
        MJCF_PATH = Path(p)
        break
if MJCF_PATH is None:
    raise FileNotFoundError(f"No MJCF found. Tried: {_MJCF_CANDIDATES}")


# ── Manus → MuJoCo coordinate transform ─────────────────────────────────────
# Manus SDK: X→right, Y→up, Z→forward
# MuJoCo:     X→forward, Y→right, Z→up
# Manus coords: X→right, Y→up, Z→forward
# MJCF coords:    X→forward, Y→right, Z→up
# Flip Manus X (right-hand → match MJCF right-hand thumb side)
R_MANUS2MJ = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)

# ── Revo3 joint names (thumb→pinky, matches MJCF) ────────────────────────────
JOINT_NAMES = [
    "right_thumb_CMP_joint", "right_thumb_CMR_joint", "right_thumb_MCP_joint",
    "right_thumb_PIP_joint", "right_thumb_DIP_joint",
    "right_index_MPR_joint", "right_index_MCP_joint", "right_index_PIP_joint",
    "right_index_DIP_joint",
    "right_middle_MPR_joint", "right_middle_MCP_joint", "right_middle_PIP_joint",
    "right_middle_DIP_joint",
    "right_ring_MPR_joint", "right_ring_MCP_joint", "right_ring_PIP_joint",
    "right_ring_DIP_joint",
    "right_little_MPR_joint", "right_little_MCP_joint", "right_little_PIP_joint",
    "right_little_DIP_joint",
]

# Motor order from retarget: little(0-3), ring(4-7), middle(8-11), index(12-15), thumb(16-20)
# Thumb motors: 19=CMP, 20=CMR, 16=MCP, 17=PIP, 18=DIP
MOTOR_TO_JOINT = [
    19, 20, 16, 17, 18,   # thumb
    12, 13, 14, 15,        # index
    8,  9,  10, 11,        # middle
    4,  5,  6,  7,         # ring
    0,  1,  2,  3,         # little
]

# ── Manus skeleton ───────────────────────────────────────────────────────────
FINGER_CHAINS = {
    "thumb":  [1, 2, 3, 4],
    "index":  [5, 6, 7, 8, 9],
    "middle": [10, 11, 12, 13, 14],
    "ring":   [15, 16, 17, 18, 19],
    "pinky":  [20, 21, 22, 23, 24],
}
NODES_COUNT = 25

# Bright, opaque colors
FINGER_COLORS = np.array([
    [1.0, 0.15, 0.15, 1.0],   # thumb - red
    [0.15, 0.85, 0.15, 1.0],  # index - green
    [0.15, 0.40, 1.0, 1.0],   # middle - blue
    [1.0, 0.85, 0.10, 1.0],   # ring - yellow
    [1.0, 0.35, 1.0, 1.0],    # pinky - magenta
])


class SkeletonViz:
    """Renders Manus skeleton as colored spheres + capsules."""

    def __init__(self, max_geoms: int = 500):
        self._max = max_geoms
        self._kp: Optional[np.ndarray] = None

    def set(self, kp: np.ndarray):
        self._kp = kp.copy()

    def render(self, scene):
        scene.ngeom = 0

        # XYZ axes at origin (MJCF frame): X=red(5cm), Y=green(5cm), Z=blue(5cm)
        o = np.zeros(3)
        self._axis(scene, o, np.array([0.05, 0, 0]), [1,0,0,1])   # +X red
        self._axis(scene, o, np.array([0, 0.05, 0]), [0,1,0,1])   # +Y green
        self._axis(scene, o, np.array([0, 0, 0.05]), [0,0,1,1])   # +Z blue

        if self._kp is None or np.all(np.abs(self._kp) < 1e-9):
            return

        # Mirror Manus X (right-hand → match model right-hand)
        kp_m = self._kp.copy()
        kp_m[:, 0] *= -1.0
        nodes = kp_m @ R_MANUS2MJ.T
        # User-calibrated: X 180°, Y 180°, Y 90°, Y -180°
        R_ALIGN = np.array([[0, 0, -1], [0, -1, 0], [-1, 0, 0]], dtype=float)
        nodes = nodes @ R_ALIGN.T
        # Translation offset
        T_OFFSET = np.array([0.0, 0.025, 0.025])
        nodes = nodes + T_OFFSET

        # Wrist
        self._sph(scene, nodes[0], 0.006, [1, 1, 1, 0.95])

        for fi, (_, chain) in enumerate(FINGER_CHAINS.items()):
            color = FINGER_COLORS[fi]

            # Joint spheres — all same size
            for nid in chain:
                pos = nodes[nid]
                if np.any(np.isnan(pos)):
                    continue
                self._sph(scene, pos, 0.006, color)


    def _axis(self, scene, start, end, rgba):
        """Draw a thin capsule from start to end (coordinate axis)."""
        if scene.ngeom >= self._max:
            return
        d_vec = end - start
        d = np.linalg.norm(d_vec)
        if d < 1e-9:
            return
        direction = d_vec / d
        mid = (start + end) / 2
        z = np.array([0.0, 0.0, 1.0])
        v = np.cross(z, direction)
        s = np.linalg.norm(v)
        if s < 1e-9:
            rmat = np.eye(3) if direction[2] > 0 else np.diag([1.0, -1.0, -1.0])
        else:
            v /= s
            c = np.dot(z, direction)
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            rmat = np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
        scene.ngeom += 1
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
                            size=np.array([0.0015, d / 2, 0], dtype=np.float64),
                            pos=np.array(mid, dtype=np.float64),
                            mat=np.array(rmat, dtype=np.float64).flatten(),
                            rgba=np.array(rgba, dtype=np.float32))

    def _sph(self, scene, pos, r, rgba):
        if scene.ngeom >= self._max:
            return
        scene.ngeom += 1
        mujoco.mjv_initGeom(scene.geoms[scene.ngeom - 1],
                            type=mujoco.mjtGeom.mjGEOM_SPHERE,
                            size=np.array([r, 0, 0], dtype=np.float64),
                            pos=np.array(pos, dtype=np.float64),
                            mat=np.eye(3, dtype=np.float64).flatten(),
                            rgba=np.array(rgba, dtype=np.float32))

# ── Local retarget (same logic as C++ retarget_node) ────────────────────────

# Motor indices in output array (little→thumb order)
MOTOR = {
    "little_MPR": 0, "little_MCP": 1, "little_PIP": 2, "little_DIP": 3,
    "ring_MPR": 4, "ring_MCP": 5, "ring_PIP": 6, "ring_DIP": 7,
    "middle_MPR": 8, "middle_MCP": 9, "middle_PIP": 10, "middle_DIP": 11,
    "index_MPR": 12, "index_MCP": 13, "index_PIP": 14, "index_DIP": 15,
    "thumb_MCP": 16, "thumb_PIP": 17, "thumb_DIP": 18, "thumb_CMP": 19, "thumb_CMR": 20,
}

# Right-hand physical calibration (from retarget configs)
CALIB = {
    # Flexion scale+offset
    "little_MCP": (1.0, 4.0), "little_PIP": (1.2, 12.0), "little_DIP": (1.0, 9.0),
    "ring_MCP": (1.0, 5.0), "ring_PIP": (1.2, 5.0), "ring_DIP": (1.0, 9.0),
    "middle_MCP": (1.0, 5.0), "middle_PIP": (1.2, 5.0), "middle_DIP": (1.0, 0.0),
    "index_MCP": (1.0, 8.0), "index_PIP": (1.2, 12.0), "index_DIP": (1.0, 9.0),
    # MPR (spread) scale+offset
    "little_MPR": (1.0, 0.0), "ring_MPR": (1.0, 0.0), "middle_MPR": (1.0, 0.0), "index_MPR": (1.0, 0.0),
    # Thumb scale+offset
    "thumb_MCP": (1.0, -4.0), "thumb_PIP": (1.2, 10.0), "thumb_DIP": (1.0, 10.0),
    "thumb_CMP": (1.0, 0.0), "thumb_CMR": (1.0, 0.0),
}


def retarget_manus_to_motor(ergo: dict) -> list[float]:
    """Convert Manus ergonomics → 21 motor positions.

    Matches the official C++ retarget pipeline:
      four_finger_retarget.cpp + spread_retarget.cpp + thumb fallback.
    The Pinocchio thumb IK (when available) overrides the thumb entries later.
    """
    mp = [0.0] * 21

    def erg(name, default=0.0):
        return float(ergo.get(name, default))

    # ── Spread (MPR) — MiddleSpread-relative (matches spread_retarget.cpp + DV1 tuning) ──
    # DV1 tuning params: middle_dynamic=true, finger_spread_sign=-1.0
    # offsets: index=0°, middle=-10°, ring=0°, pinky=0°
    FINGER_SPREAD_SIGN = -1.0
    middle_ref = erg("MiddleSpread", 0.0)
    middle_dynamic = True
    middle_offset = -10.0

    index_val = (erg("IndexSpread", 0.0) - middle_ref - 0.0) * 1.0
    middle_val = (middle_ref - middle_offset) * 1.0 if middle_dynamic else -middle_offset
    ring_val = (erg("RingSpread", 0.0) - middle_ref - 0.0) * 1.0
    # ring asymmetric scaling (forward/backward)
    if ring_val > 0.0:
        ring_val *= 1.0   # ring_forward_scale
    elif ring_val < 0.0:
        ring_val *= 1.0   # ring_backward_scale
    pinky_val = (erg("PinkySpread", 0.0) - middle_ref - 0.0) * 1.0

    mp[MOTOR["index_MPR"]] = FINGER_SPREAD_SIGN * index_val
    mp[MOTOR["middle_MPR"]] = FINGER_SPREAD_SIGN * middle_val
    mp[MOTOR["ring_MPR"]] = FINGER_SPREAD_SIGN * ring_val
    mp[MOTOR["little_MPR"]] = FINGER_SPREAD_SIGN * pinky_val

    # ── Flexion (MCP/PIP/DIP) — matches four_finger_retarget.cpp ──────
    # Index
    mp[MOTOR["index_MCP"]] = erg("IndexMCPStretch") * 1.0 * 1.0      # index_angle_scale * four_finger_mcp_scale
    mp[MOTOR["index_PIP"]] = erg("IndexPIPStretch") * 1.0             # index_angle_scale
    mp[MOTOR["index_DIP"]] = erg("IndexDIPStretch") * 1.0             # index_angle_scale
    # Middle
    mp[MOTOR["middle_MCP"]] = erg("MiddleMCPStretch") * 1.0 * 1.0    # all_finger * four_finger_mcp
    mp[MOTOR["middle_PIP"]] = erg("MiddlePIPStretch") * 1.0           # all_finger
    mp[MOTOR["middle_DIP"]] = erg("MiddleDIPStretch") * 1.0 * 1.0    # all_finger * middle_ring_dip
    # Ring
    mp[MOTOR["ring_MCP"]] = erg("RingMCPStretch") * 1.0 * 1.0        # all_finger * four_finger_mcp
    mp[MOTOR["ring_PIP"]] = erg("RingPIPStretch") * 1.0               # all_finger
    mp[MOTOR["ring_DIP"]] = erg("RingDIPStretch") * 1.0 * 1.0        # all_finger * middle_ring_dip
    # Little
    mp[MOTOR["little_MCP"]] = erg("PinkyMCPStretch") * 1.0 * 1.0 * 1.0  # pinky_angle * pinky_mcp * four_finger_mcp
    mp[MOTOR["little_PIP"]] = erg("PinkyPIPStretch") * 1.0 * 1.0        # pinky_angle * pinky_dip_pip
    mp[MOTOR["little_DIP"]] = erg("PinkyDIPStretch") * 1.0 * 1.0        # pinky_angle * pinky_dip_pip

    # ── Thumb (simplified ergonomics — fallback when Pinocchio IK unavailable) ─
    flex = max(0.0, erg("ThumbMCPStretch"))
    abd = erg("ThumbMCPSpread")
    pip = max(0.0, erg("ThumbPIPStretch"))
    mp[MOTOR["thumb_CMP"]] = flex * 0.70
    mp[MOTOR["thumb_CMR"]] = abd * 0.80
    mp[MOTOR["thumb_MCP"]] = pip * 0.40
    mp[MOTOR["thumb_PIP"]] = pip * 0.40 * 0.5
    mp[MOTOR["thumb_DIP"]] = 0.0

    # ── Apply physical calibration: pos = pos * scale + offset (matches C++ apply_output_calibration) ──
    for name, motor_id in MOTOR.items():
        scale, offset = CALIB.get(name, (1.0, 0.0))
        mp[motor_id] = mp[motor_id] * scale + offset

    return mp


class Revo3Viewer(Node):
    def __init__(self, hand: str = "right"):
        super().__init__("revo3_viewer")
        self._hand = hand

        # Load MJCF
        mjcf_dir = str(MJCF_PATH.parent)
        os.chdir(mjcf_dir)
        self.get_logger().info(f"Loading: {MJCF_PATH}")
        self._model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
        self._data = mujoco.MjData(self._model)
        self.get_logger().info(f"{self._model.njnt} joints, {self._model.nbody} bodies, {self._model.ngeom} geoms")

        # Joint qpos addresses
        self._adr = []
        for jn in JOINT_NAMES:
            jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            self._adr.append(self._model.jnt_qposadr[jid] if jid >= 0 else -1)

        # Pinocchio thumb IK (fallback when retarget_node not running)
        try:
            sys.path.insert(0, str(Path(__file__).resolve().parent))
            from thumb_ik_pinocchio import PinocchioThumbIK
            self._thumb_ik = PinocchioThumbIK()
            self.get_logger().info("Pinocchio thumb IK ready")
        except Exception as e:
            self._thumb_ik = None
            self.get_logger().warning(f"No Pinocchio thumb IK: {e}")

        # Subscriptions
        self.create_subscription(ManusGlove, "/manus_glove_0", self._on_manus, 10)
        # Listen to official C++ retarget_node output (Revo3MITCommand, positions in radians)
        if _has_mit_msgs:
            self.create_subscription(Revo3MITCommand,
                f"/revo3_{hand}/joint_forward_mit_controller/retarget_targets",
                self._on_mit_command, 10)
            self.create_subscription(Revo3MITCommand,
                f"/revo3_{hand}/joint_forward_mit_controller/commands",
                self._on_mit_command, 10)
        # Legacy: Stark SDK motor commands (positions in degrees)
        if _has_stark_msgs:
            self.create_subscription(SetMotorMulti, f"/revo3/{hand}/set_motor_multi", self._on_motor, 10)

        # Publish joint_states so RViz can visualize
        self._joint_state_pub = self.create_publisher(
            JointState, f"/revo3_{hand}/revo3_joint_state/joint_states", 10)

        self._motor_pos: Optional[np.ndarray] = None  # from retarget_node (preferred)
        self._local_motor: Optional[np.ndarray] = None  # local retarget (fallback)
        self._last_ergo: dict = {}
        self._skel = SkeletonViz()
        self._fps = deque(maxlen=60)
        self._t = time.time()

    def _on_manus(self, msg):
        kp = np.zeros((NODES_COUNT, 3), dtype=float)
        for node in msg.raw_nodes:
            nid = node.node_id
            if 0 <= nid < NODES_COUNT:
                kp[nid] = [node.pose.position.x, node.pose.position.y, node.pose.position.z]
        if not np.all(np.abs(kp) < 1e-9):
            self._skel.set(kp)

        # Extract ergonomics and run local retarget
        ergo = {}
        for e in msg.ergonomics:
            ergo[str(e.type)] = float(e.value)
        if ergo:
            self._last_ergo = ergo
            motor = np.array(retarget_manus_to_motor(ergo), dtype=float)
            # Use Pinocchio IK for thumb (when retarget_node not supplying motor data)
            if self._thumb_ik is not None and self._motor_pos is None:
                try:
                    thumb_deg = self._thumb_ik.solve(ergo, kp)
                    motor[MOTOR["thumb_CMP"]] = thumb_deg[0]
                    motor[MOTOR["thumb_CMR"]] = thumb_deg[1]
                    motor[MOTOR["thumb_MCP"]] = thumb_deg[2]
                    motor[MOTOR["thumb_PIP"]] = thumb_deg[3]
                    motor[MOTOR["thumb_DIP"]] = thumb_deg[4]
                except Exception:
                    pass  # IK failed, keep simplified thumb
            self._local_motor = motor

    def _on_motor(self, msg):
        if len(msg.positions) >= 21:
            self._motor_pos = np.array(msg.positions[:21], dtype=float)

    def _on_mit_command(self, msg):
        """Receive from official C++ retarget_node (Revo3MITCommand, positions in radians)."""
        if msg.joint_names and len(msg.position) >= 21:
            # C++ joint order (retarget_common.hpp): little(0-3)→ring(4-7)→middle(8-11)→index(12-15)→thumb(16-20)
            # Thumb: MCP=16,PIP=17,DIP=18,CMP=19,CMR=20 — same as our MOTOR dict
            # Position is in radians — convert to degrees for viewer
            self._motor_pos = np.rad2deg(np.array(msg.position[:21], dtype=float))

    def _publish_joint_states(self):
        """Publish current MJCF joint positions as JointState for RViz."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        positions_rad = []
        for i, jn in enumerate(JOINT_NAMES):
            adr = self._adr[i] if i < len(self._adr) else -1
            if adr >= 0:
                positions_rad.append(float(self._data.qpos[adr]))
            else:
                positions_rad.append(0.0)
        js.position = positions_rad
        self._joint_state_pub.publish(js)

    def spin(self):
        self.get_logger().info("Launching MuJoCo viewer...")
        with mujoco.viewer.launch_passive(self._model, self._data) as v:
            v.cam.azimuth = 160
            v.cam.elevation = -20
            v.cam.distance = 0.5
            v.cam.lookat = np.array([0.0, 0.0, 0.06])

            lp = 0.0
            while v.is_running() and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)

                # Update joints: prefer retarget_node, fallback to local retarget
                motor = self._motor_pos if self._motor_pos is not None else self._local_motor
                if motor is not None:
                    for ji, adr in enumerate(self._adr):
                        if adr >= 0 and ji < len(MOTOR_TO_JOINT):
                            self._data.qpos[adr] = np.deg2rad(motor[MOTOR_TO_JOINT[ji]])

                mujoco.mj_forward(self._model, self._data)
                self._skel.render(v.user_scn)

                # Publish joint states for RViz
                self._publish_joint_states()

                v.sync()

                now = time.time()
                self._fps.append(1.0 / max(now - self._t, 0.0001))
                self._t = now
                if now - lp >= 2.0:
                    lp = now
                    fps = sum(self._fps) / max(len(self._fps), 1)
                    motor = self._motor_pos if self._motor_pos is not None else self._local_motor
                    src = "retarget_node" if self._motor_pos is not None else ("local" if self._local_motor is not None else "none")
                    if motor is not None:
                        mp = motor
                        s = f"[{src}] thumb[CMP={mp[19]:.0f} CMR={mp[20]:.0f} MCP={mp[16]:.0f} PIP={mp[17]:.0f} DIP={mp[18]:.0f}] "
                        s += f"idx=[{mp[12]:.0f},{mp[13]:.0f},{mp[14]:.0f},{mp[15]:.0f}] "
                        s += f"spread[idx={mp[MOTOR['index_MPR']]:.0f} mid={mp[MOTOR['middle_MPR']]:.0f} ring={mp[MOTOR['ring_MPR']]:.0f}]"
                    else:
                        s = "(waiting for Manus)"
                    sk = "YES" if self._skel._kp is not None else "no"
                    self.get_logger().info(f"{fps:.0f}FPS | skel:{sk} | {s}")

        self.get_logger().info("Viewer closed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    args = parser.parse_args()
    rclpy.init()
    node = Revo3Viewer(hand=args.hand)
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
