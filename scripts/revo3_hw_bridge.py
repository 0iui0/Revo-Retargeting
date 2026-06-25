#!/usr/bin/env python3
"""Bridge: retarget_node (Revo3MITCommand) → real Revo3 hardware via bc-stark-sdk.

Usage:
  source install/setup.bash
  python3 scripts/revo3_hw_bridge.py --hand right
"""

import argparse
import asyncio
import importlib
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from revo3_mit_controller_msgs.msg import Revo3MITCommand

MOTOR_COUNT = 21
DEFAULT_KP = 0.4
DEFAULT_KD = 0.05


def _baudrate_from_int(sdk, value: int):
    mapping = {
        115200: sdk.Baudrate.Baud115200,
        57600: sdk.Baudrate.Baud57600,
        19200: sdk.Baudrate.Baud19200,
        460800: sdk.Baudrate.Baud460800,
        1000000: sdk.Baudrate.Baud1Mbps,
        2000000: sdk.Baudrate.Baud2Mbps,
        5000000: sdk.Baudrate.Baud5Mbps,
    }
    return mapping.get(value, sdk.Baudrate.Baud460800)


class Revo3HWBridge(Node):
    def __init__(self, hand: str = "right", port: str = "/dev/ttyUSB1",
                 slave_id: int = 127, baudrate: int = 5000000):
        super().__init__("revo3_hw_bridge")
        self._port = port
        self._slave_id = slave_id
        self._baudrate = baudrate
        self._device = None
        self._sdk = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._connected = threading.Event()
        self._pending_positions: Optional[list] = None
        self._lock = threading.Lock()
        self._cmd_count = 0
        self._last_log = 0.0

        # Subscribe to C++ retarget output
        self._cmd_sub = self.create_subscription(
            Revo3MITCommand,
            f"/revo3_{hand}/joint_forward_mit_controller/commands",
            self._on_command,
            10,
        )
        self._target_sub = self.create_subscription(
            Revo3MITCommand,
            f"/revo3_{hand}/joint_forward_mit_controller/retarget_targets",
            self._on_command,
            10,
        )

        # Start async event loop thread
        self._thread = threading.Thread(target=self._async_loop, daemon=True)
        self._thread.start()

        if not self._connected.wait(timeout=10.0):
            self.get_logger().error("Device connection timeout — check power and serial port")
        else:
            self.get_logger().info(
                f"Revo3 HW bridge ready: {hand} hand, port={port}, slave={slave_id}"
            )

    def _async_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._connect())
            self._loop.create_task(self._send_loop())
            self._connected.set()
            self._loop.run_forever()
        finally:
            self._loop.close()

    async def _connect(self):
        try:
            self._sdk = importlib.import_module('bc_stark_sdk.main_mod')
        except Exception:
            # Fallback: try namespace package
            self._sdk = __import__('bc_stark_sdk.main_mod', fromlist=['main_mod'])

        baud = _baudrate_from_int(self._sdk, self._baudrate)
        self._device = await self._sdk.modbus_open(self._port, baud)

        # Set MIT control mode
        try:
            await self._device.v3_set_ctrl_mode_all(self._slave_id, 4)
            await asyncio.sleep(0.1)
        except Exception:
            pass  # MIT mode may already be set

    async def _send_loop(self):
        while True:
            with self._lock:
                positions = self._pending_positions
                self._pending_positions = None
            if positions is not None:
                try:
                    await self._send_mit(positions)
                except Exception:
                    pass  # logged in _send_mit
            await asyncio.sleep(0.001)

    async def _send_mit(self, positions_deg: list):
        count = MOTOR_COUNT
        kp = [DEFAULT_KP] * count
        kd = [DEFAULT_KD] * count
        vel = [0.0] * count
        torque = [0.0] * count
        await self._device.revo3_set_all_mit_batch(
            self._slave_id, kp, kd, positions_deg, vel, torque
        )
        self._cmd_count += 1

    def _on_command(self, msg: Revo3MITCommand):
        if self._device is None or self._loop is None:
            return
        if len(msg.position) < MOTOR_COUNT:
            return

        # Convert radians → degrees
        positions_deg = [float(p) * 180.0 / 3.141592653589793
                         for p in msg.position[:MOTOR_COUNT]]

        with self._lock:
            self._pending_positions = positions_deg

        # Log at ~1 Hz
        now = time.time()
        if now - self._last_log >= 2.0:
            self._last_log = now
            self.get_logger().info(
                f"HW: {self._cmd_count} cmds, "
                f"thumb[CMP={positions_deg[19]:.0f} CMR={positions_deg[20]:.0f} "
                f"MCP={positions_deg[16]:.0f} PIP={positions_deg[17]:.0f}] "
                f"idx[MPR={positions_deg[12]:.0f} MCP={positions_deg[13]:.0f}]"
            )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", default="right", choices=["left", "right"])
    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--slave-id", type=int, default=127)
    parser.add_argument("--baudrate", type=int, default=5000000)
    args = parser.parse_args()

    rclpy.init()
    node = Revo3HWBridge(
        hand=args.hand, port=args.port,
        slave_id=args.slave_id, baudrate=args.baudrate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
