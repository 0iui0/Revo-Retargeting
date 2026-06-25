#!/usr/bin/env python3
"""Diagnose index finger spread bias — print raw Manus ergonomics + computed MPR.

Usage (inside container):
  python3 scripts/diag_spread.py
"""
import rclpy
from rclpy.node import Node
from manus_ros2_msgs.msg import ManusGlove


# DV1 tuning (same as retarget_tuning_right_DV1.yaml)
FINGER_SPREAD_SIGN = -1.0
MIDDLE_DYNAMIC = True
MIDDLE_OFFSET = -10.0   # middle_spread_offset_deg
INDEX_OFFSET = 0.0      # index_spread_offset_deg
RING_OFFSET = 0.0       # ring_spread_offset_deg
PINKY_OFFSET = 0.0      # pinky_spread_offset_deg


def compute_mpr(ergo: dict) -> dict:
    middle_ref = float(ergo.get("MiddleSpread", 0.0))

    if MIDDLE_DYNAMIC:
        middle_val = (middle_ref - MIDDLE_OFFSET) * 1.0
    else:
        middle_val = -MIDDLE_OFFSET

    index_val = (float(ergo.get("IndexSpread", 0.0)) - middle_ref - INDEX_OFFSET) * 1.0
    ring_val = (float(ergo.get("RingSpread", 0.0)) - middle_ref - RING_OFFSET) * 1.0
    pinky_val = (float(ergo.get("PinkySpread", 0.0)) - middle_ref - PINKY_OFFSET) * 1.0

    return {
        "IndexSpread": float(ergo.get("IndexSpread", 0.0)),
        "MiddleSpread": float(ergo.get("MiddleSpread", 0.0)),
        "RingSpread": float(ergo.get("RingSpread", 0.0)),
        "PinkySpread": float(ergo.get("PinkySpread", 0.0)),
        "middle_ref": middle_ref,
        "index_MPR": FINGER_SPREAD_SIGN * index_val,
        "middle_MPR": FINGER_SPREAD_SIGN * middle_val,
        "ring_MPR": FINGER_SPREAD_SIGN * ring_val,
        "little_MPR": FINGER_SPREAD_SIGN * pinky_val,
    }


class DiagNode(Node):
    def __init__(self):
        super().__init__("diag_spread")
        self.create_subscription(ManusGlove, "/manus_glove_0", self._cb, 10)
        self._count = 0

    def _cb(self, msg: ManusGlove):
        if msg.side.lower() not in ("right", "r"):
            return
        ergo = {str(e.type): float(e.value) for e in msg.ergonomics}
        r = compute_mpr(ergo)
        self._count += 1
        if self._count % 30 == 0:  # ~1Hz
            self.get_logger().info(
                f"[spread] raw: idx={r['IndexSpread']:.1f} mid={r['MiddleSpread']:.1f} "
                f"ring={r['RingSpread']:.1f} pinky={r['PinkySpread']:.1f} | "
                f"mid_ref={r['middle_ref']:.1f} → "
                f"MPR: idx={r['index_MPR']:+.1f}° mid={r['middle_MPR']:+.1f}° "
                f"ring={r['ring_MPR']:+.1f}° little={r['little_MPR']:+.1f}°"
            )


def main():
    rclpy.init()
    node = DiagNode()
    print("=" * 70)
    print("Spread diagnostic running. Straighten all 4 fingers and observe.")
    print("  idx_MPR > 0  = index moves toward thumb (abduction)")
    print("  idx_MPR < 0  = index moves toward middle (adduction)")
    print("=" * 70)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
