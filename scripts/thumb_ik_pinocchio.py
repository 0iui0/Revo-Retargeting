#!/usr/bin/env python3
"""Pinocchio-based thumb IK — direct port of C++ thumb_retarget.cpp."""

import numpy as np
import pinocchio as pin
from pathlib import Path

# ── URDF path ────────────────────────────────────────────────────────────────
_URDF_DIR = Path("/workspaces/isaac_ros-dev/Revo-Retargeting/src/revo3_description/urdf")
URDF_RIGHT = str(_URDF_DIR / "revo3_right_may3.urdf")

# ── Manus keypoint node IDs ──────────────────────────────────────────────────
THUMB_TIP_NODE = 4
THUMB_DIP_NODE = 3
THUMB_PIP_NODE = 2
FOUR_FINGER_TIP_NODES = [9, 14, 19, 24]

# Posture weights: [CMP, CMR, MCP, PIP, DIP]
POSTURE_WEIGHTS = np.array([0.0, 0.25, 0.9, 1.2, 1.0])

# Right-hand thumb calibration (from thumb_retarget.yaml + physical)
# apply_output_calibration: output = q * scale + offset_deg
THUMB_CALIB = {
    "cmp_scale": 1.04, "cmp_offset_deg": 0.0,
    "cmr_offset_deg": 0.0,
    "mcp_scale": 1.0, "mcp_offset_deg": 0.0,
    "pip_scale": 1.2,
    "dip_scale": 1.2,
}
# Physical calibration applied AFTER output calibration
THUMB_PHYSICAL = {
    "cmp_scale": 1.0, "cmp_offset_deg": 0.0,
    "cmr_scale": 1.0, "cmr_offset_deg": 0.0,
    "mcp_scale": 1.0, "mcp_offset_deg": -4.0,
    "pip_scale": 1.2, "pip_offset_deg": 10.0,
    "dip_scale": 1.0, "dip_offset_deg": 10.0,
}


class PinocchioThumbIK:
    """Pinocchio-based thumb IK, matching C++ thumb_retarget.cpp exactly."""

    def __init__(self, urdf_path: str = URDF_RIGHT):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        # Frame IDs
        self.tip_frame = self.model.getFrameId("right_thumb_tip_Link")
        self.dip_frame = self.model.getFrameId("right_thumb_DIP_Link")
        self.pip_frame = self.model.getFrameId("right_thumb_PIP_Link")

        # Thumb joint qpos/dof addresses
        thumb_joints = [
            "right_thumb_CMP_joint", "right_thumb_CMR_joint",
            "right_thumb_MCP_joint", "right_thumb_PIP_joint",
            "right_thumb_DIP_joint",
        ]
        self.qpos_adrs = []
        self.dof_adrs = []
        for jn in thumb_joints:
            jid = self.model.getJointId(jn)
            self.qpos_adrs.append(self.model.idx_qs[jid])
            self.dof_adrs.append(self.model.idx_vs[jid])
        self.n = len(self.qpos_adrs)

        # Joint limits
        self.jlow = np.array([self.model.lowerPositionLimit[i] for i in range(self.model.nq)])
        self.jhigh = np.array([self.model.upperPositionLimit[i] for i in range(self.model.nq)])
        for i in range(self.model.nq):
            if not np.isfinite(self.jlow[i]) or not np.isfinite(self.jhigh[i]) or self.jlow[i] >= self.jhigh[i]:
                self.jlow[i] = -np.pi
                self.jhigh[i] = np.pi

        # Start at zero (or joint limit if zero not in range)
        self.current_q = np.zeros(self.model.nq)
        for i in range(self.model.nq):
            if not (self.jlow[i] <= 0.0 <= self.jhigh[i]):
                self.current_q[i] = self.jlow[i]

        # EMA filter state
        self._filtered_thumb: np.ndarray | None = None

        # Config (matching official C++ thumb_retarget.cpp + thumb_retarget.yaml)
        # Right-hand EMA: faster tracking (0.4/0.6), left-hand: smoother (0.9/0.1)
        self.manus_z_rotation_rad = np.pi / 2.0
        self.manus_scale_xz = 1.0
        self.manus_out_y_sign = -1.0
        self.spread_sign = -1.0           # right-hand
        self.reach_scale = 1.0
        self.ik_position_scale = 1.0
        self.pip_ik_scale = 1.0
        self.dip_ik_scale = 1.0
        self.ema_prev = 0.4               # official right-hand
        self.ema_cur = 0.6               # official right-hand
        self.ik_posture_weight = 0.1
        self.ik_smooth_weight = 0.1
        self.ik_max_iterations = 15       # official default
        self.ik_max_step_rad = np.deg2rad(3.0)
        self.ik_max_frame_delta_rad = np.deg2rad(6.0)
        self.ik_damping = 0.02
        self.ik_step_size = 0.30
        self.ik_tolerance = 5e-4
        self.calib = THUMB_CALIB.copy()
        self.physical = THUMB_PHYSICAL.copy()

    def _transform_manus(self, xyz: np.ndarray) -> np.ndarray:
        """Manus coords → model coords: Z rotation + scale + Y sign."""
        c = np.cos(self.manus_z_rotation_rad)
        s = np.sin(self.manus_z_rotation_rad)
        rx = c * xyz[0] - s * xyz[1]
        ry = s * xyz[0] + c * xyz[1]
        return np.array([
            rx * self.manus_scale_xz,
            self.manus_out_y_sign * ry * self.manus_scale_xz,
            xyz[2] * self.manus_scale_xz,
        ])

    def _reach_scale(self, thumb: np.ndarray, center: np.ndarray) -> np.ndarray:
        return center + (thumb - center) * self.reach_scale

    def _posture_target(self, ergonomics: dict) -> tuple[np.ndarray, np.ndarray]:
        target = np.full(self.n, np.nan)
        weights = np.zeros(self.n)
        sources = [
            (1, "ThumbMCPSpread", False),
            (2, "ThumbMCPStretch", True),
            (3, "ThumbPIPStretch", True),
            (4, "ThumbDIPStretch", True),
        ]
        for idx, name, clamp_pos in sources:
            if idx >= self.n:
                continue
            v = ergonomics.get(name)
            if v is None:
                continue
            v_deg = float(v)
            if clamp_pos:
                v_deg = max(0.0, v_deg)
            else:
                v_deg = float(self.spread_sign) * v_deg
            adr = self.qpos_adrs[idx]
            target[idx] = np.clip(np.deg2rad(v_deg), self.jlow[adr], self.jhigh[adr])
            weights[idx] = POSTURE_WEIGHTS[idx]
        return target, weights

    def solve(self, ergonomics: dict, keypoints: np.ndarray) -> np.ndarray:
        """Run thumb IK. keypoints: (25, 3) Manus positions. Returns (5,) motor degrees."""
        thumb_raw = keypoints[THUMB_TIP_NODE]
        if np.any(np.isnan(thumb_raw)) or np.all(np.abs(thumb_raw) < 1e-9):
            return self._get_motor_degrees()

        # Compute 4-finger center
        four = []
        for nid in FOUR_FINGER_TIP_NODES:
            if not np.any(np.isnan(keypoints[nid])) and not np.all(np.abs(keypoints[nid]) < 1e-9):
                four.append(self._transform_manus(keypoints[nid]))
        if len(four) != 4:
            return self._get_motor_degrees()
        center = np.mean(four, axis=0)

        # Transform + reach-scale thumb targets
        thumb_target = self._reach_scale(self._transform_manus(thumb_raw), center)
        dip_raw = keypoints[THUMB_DIP_NODE]
        pip_raw = keypoints[THUMB_PIP_NODE]
        dip_target = self._reach_scale(self._transform_manus(dip_raw), center) if not np.any(np.isnan(dip_raw)) else None
        pip_target = self._reach_scale(self._transform_manus(pip_raw), center) if not np.any(np.isnan(pip_raw)) else None

        # EMA filter
        if self._filtered_thumb is not None:
            self._filtered_thumb = self.ema_prev * self._filtered_thumb + self.ema_cur * thumb_target
        else:
            self._filtered_thumb = thumb_target

        # Scale targets
        tip = self._filtered_thumb * self.ik_position_scale
        dip = dip_target * self.ik_position_scale if dip_target is not None else None
        pip = pip_target * self.ik_position_scale if pip_target is not None else None

        # DLS IK
        q_prev = self.current_q.copy()
        q = q_prev.copy()
        posture, posture_w = self._posture_target(ergonomics)

        for _ in range(self.ik_max_iterations):
            pin.forwardKinematics(self.model, self.data, q)
            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)

            residuals = []
            jacs = []

            # Tip task (weight 2.0)
            def _add_task(frame_id, target_xyz, weight):
                if target_xyz is None:
                    return
                current = self.data.oMf[frame_id].translation.copy()
                residuals.append(weight * (target_xyz - current))
                J_full = pin.getFrameJacobian(self.model, self.data, frame_id, pin.LOCAL_WORLD_ALIGNED)
                J = np.zeros((3, self.n))
                for r in range(3):
                    for c in range(self.n):
                        J[r, c] = weight * J_full[r, self.dof_adrs[c]]
                jacs.append(J)

            _add_task(self.tip_frame, tip, 2.0)
            _add_task(self.dip_frame, dip, 0.1 * self.dip_ik_scale)
            _add_task(self.pip_frame, pip, 0.1 * self.pip_ik_scale)

            # Posture prior
            if self.ik_posture_weight > 0:
                for i in range(self.n):
                    if not np.isfinite(posture[i]) or posture_w[i] <= 0:
                        continue
                    w = self.ik_posture_weight * posture_w[i]
                    residuals.append(np.array([w * (posture[i] - q[self.qpos_adrs[i]])]))
                    J = np.zeros((1, self.n))
                    J[0, i] = w
                    jacs.append(J)

            # Temporal smoothness
            if self.ik_smooth_weight > 0:
                res = np.zeros(self.n)
                for i in range(self.n):
                    res[i] = self.ik_smooth_weight * (q_prev[self.qpos_adrs[i]] - q[self.qpos_adrs[i]])
                residuals.append(res)
                jacs.append(np.eye(self.n) * self.ik_smooth_weight)

            # Assemble
            residual = np.concatenate(residuals)
            jacobian = np.vstack(jacs)
            if np.linalg.norm(residual) < self.ik_tolerance:
                break
            if not np.all(np.isfinite(residual)) or not np.all(np.isfinite(jacobian)):
                break

            lhs = jacobian @ jacobian.T + (self.ik_damping ** 2) * np.eye(jacobian.shape[0])
            try:
                dq = jacobian.T @ np.linalg.solve(lhs, residual)
            except np.linalg.LinAlgError:
                break
            dq *= self.ik_step_size
            dq = np.clip(dq, -self.ik_max_step_rad, self.ik_max_step_rad)
            if np.linalg.norm(dq) < 1e-5:
                break
            for i in range(self.n):
                adr = self.qpos_adrs[i]
                q[adr] = np.clip(q[adr] + dq[i], self.jlow[adr], self.jhigh[adr])

        # Per-frame delta clamp
        for i in range(self.n):
            adr = self.qpos_adrs[i]
            delta = np.clip(q[adr] - q_prev[adr], -self.ik_max_frame_delta_rad, self.ik_max_frame_delta_rad)
            q[adr] = np.clip(q_prev[adr] + delta, self.jlow[adr], self.jhigh[adr])

        self.current_q = q
        return self._get_motor_degrees()

    def _get_motor_degrees(self) -> np.ndarray:
        """Convert current_q → 5 motor degrees with calibration applied."""
        q = self.current_q
        cal = self.calib
        phy = self.physical

        def _calib(idx, scale, offset_deg):
            adr = self.qpos_adrs[idx]
            return np.clip(q[adr] * scale + np.deg2rad(offset_deg), self.jlow[adr], self.jhigh[adr])

        # Output calibration (thumb_retarget.cpp apply_output_calibration)
        cmp = _calib(0, cal["cmp_scale"], cal["cmp_offset_deg"])
        cmr = _calib(1, 1.0, cal["cmr_offset_deg"])
        mcp = _calib(2, cal["mcp_scale"], cal["mcp_offset_deg"])
        pip = _calib(3, cal["pip_scale"], 0.0)
        dip = _calib(4, cal["dip_scale"], 0.0)

        # Convert to degrees + apply physical calibration
        result = np.array([
            np.rad2deg(cmp) * phy["cmp_scale"] + phy["cmp_offset_deg"],
            np.rad2deg(cmr) * phy["cmr_scale"] + phy["cmr_offset_deg"],
            np.rad2deg(mcp) * phy["mcp_scale"] + phy["mcp_offset_deg"],
            np.rad2deg(pip) * phy["pip_scale"] + phy["pip_offset_deg"],
            np.rad2deg(dip) * phy["dip_scale"] + phy["dip_offset_deg"],
        ])
        return result
