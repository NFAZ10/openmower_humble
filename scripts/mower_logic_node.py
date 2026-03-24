#!/usr/bin/env python3
"""
Mowing State Machine Node for OpenMower Humble

Ports the high-level state logic from the original open_mower_ros (by Clemens Elflein)
to ROS2/Humble, adapted for the openmowernext / rosmower architecture.

Original work:
  Copyright (c) 2022 Clemens Elflein - https://github.com/ClemensElflein/open_mower_ros
  Licensed under the Open Source License

ROS2 adaptation for openmowernext + rosmower by NFAZ10.

Responsibilities:
  - Subscribe to /robot_mode_cmd (std_msgs/String) for UI/bridge commands
  - Publish /robot_mode (std_msgs/String) as transient-local so late-joining nodes
    receive the current state immediately
  - Enable/disable the mow blade (ros2_control effort controller) on mode transitions
  - Auto-switch to 'charging' when charger is detected on /power/charger_present
  - Auto-switch to 'idle' when battery drops below threshold on /power

Supported mode values (mirroring OG mower_logic.cpp):
  idle      - not mowing, not charging
  mowing    - actively mowing (blade ON)
  charging  - docked and charging (blade OFF)
  docking   - in transit to dock (blade OFF, reports as 'idle' to UI)

Topics:
  Subscribed:
    /robot_mode_cmd       (std_msgs/String)
    /power/charger_present (std_msgs/Bool)
    /power                (sensor_msgs/BatteryState)

  Published:
    /robot_mode           (std_msgs/String, transient-local)
    /mower_controller/commands  (std_msgs/Float64MultiArray) [if OM_BLADE_ENABLED=true]

Environment variables (all optional):
  OM_MODE_TOPIC              default: /robot_mode
  OM_MODE_CMD_TOPIC          default: /robot_mode_cmd
  OM_CHARGER_TOPIC           default: /power/charger_present
  OM_BATTERY_TOPIC           default: /power
  OM_BLADE_TOPIC             default: /mower_controller/commands
  OM_BLADE_ENABLED           default: true  (set false to skip blade commands)
  OM_BLADE_SPEED             default: 1.0   (normalised effort 0.0–1.0)
  OM_BATTERY_LOW_THRESHOLD   default: 20.0  (percent)
"""

import math
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64MultiArray, String

# ── Mode constants ──────────────────────────────────────────────────────────
MODE_IDLE = "idle"
MODE_MOWING = "mowing"
MODE_CHARGING = "charging"
MODE_DOCKING = "docking"


class MowerLogicNode(Node):
    """Lightweight ROS2 state machine — the 'mower_logic' equivalent from OG."""

    def __init__(self) -> None:
        super().__init__("mower_logic")

        # ── Config from env ─────────────────────────────────────────────────
        self._mode_topic = os.getenv("OM_MODE_TOPIC", "/robot_mode")
        self._mode_cmd_topic = os.getenv("OM_MODE_CMD_TOPIC", "/robot_mode_cmd")
        self._charger_topic = os.getenv("OM_CHARGER_TOPIC", "/power/charger_present")
        self._battery_topic = os.getenv("OM_BATTERY_TOPIC", "/power")
        self._blade_topic = os.getenv("OM_BLADE_TOPIC", "/mower_controller/commands")
        self._blade_enabled = os.getenv("OM_BLADE_ENABLED", "true").lower() == "true"
        self._blade_speed = float(os.getenv("OM_BLADE_SPEED", "1.0"))
        self._battery_low_threshold = float(os.getenv("OM_BATTERY_LOW_THRESHOLD", "20.0"))

        # ── Internal state (all access under _lock) ──────────────────────────
        self._lock = threading.Lock()
        self._current_mode: str = MODE_IDLE
        self._charger_present: bool = False
        self._battery_pct: float = 100.0
        self._battery_low_triggered: bool = False

        # ── QoS: transient-local so late subscribers get current mode ────────
        latching_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._mode_pub = self.create_publisher(String, self._mode_topic, latching_qos)
        self._blade_pub = self.create_publisher(Float64MultiArray, self._blade_topic, 10)

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(String, self._mode_cmd_topic, self._on_mode_cmd, 10)
        self.create_subscription(Bool, self._charger_topic, self._on_charger, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._on_battery, 10)

        # Heartbeat: re-publish current mode every 2 s so the UI stays in sync
        self.create_timer(2.0, self._heartbeat)

        self._publish_mode(MODE_IDLE)
        self.get_logger().info(
            f"Mower Logic Node started | mode={self._current_mode} "
            f"blade_enabled={self._blade_enabled} "
            f"battery_low_threshold={self._battery_low_threshold}%"
        )

    # ── Public helpers ────────────────────────────────────────────────────────

    def _transition(self, new_mode: str) -> None:
        """Thread-safe mode transition; no-op if already in that mode."""
        with self._lock:
            if new_mode == self._current_mode:
                return
            old = self._current_mode
            self._current_mode = new_mode

        self.get_logger().info(f"State: {old} → {new_mode}")
        self._control_blade(new_mode == MODE_MOWING)
        self._publish_mode(new_mode)

    def _publish_mode(self, mode: str) -> None:
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _control_blade(self, enable: bool) -> None:
        if not self._blade_enabled:
            return
        msg = Float64MultiArray()
        msg.data = [self._blade_speed if enable else 0.0]
        self._blade_pub.publish(msg)
        self.get_logger().info(f"Mow blade: {'ON (speed={self._blade_speed})' if enable else 'OFF'}")

    # ── Timer ─────────────────────────────────────────────────────────────────

    def _heartbeat(self) -> None:
        with self._lock:
            mode = self._current_mode
        self._publish_mode(mode)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _on_mode_cmd(self, msg: String) -> None:
        """Handle commands from the UI bridge on /robot_mode_cmd."""
        cmd = msg.data.strip().lower()
        self.get_logger().debug(f"Mode command received: {cmd!r}")

        with self._lock:
            charger = self._charger_present

        if cmd == MODE_MOWING:
            if charger:
                self.get_logger().warning("Cannot start mowing while charger is connected")
                return
            with self._lock:
                self._battery_low_triggered = False
            self._transition(MODE_MOWING)

        elif cmd == MODE_IDLE:
            self._transition(MODE_IDLE)

        elif cmd == MODE_CHARGING:
            self._transition(MODE_CHARGING)

        elif cmd == MODE_DOCKING:
            # The bridge handles actual docking navigation; we just reflect state
            self._transition(MODE_IDLE)

        else:
            self.get_logger().warning(f"Unknown mode command: {cmd!r}")

    def _on_charger(self, msg: Bool) -> None:
        """Auto-transition based on charger presence (mirrors OG IdleBehavior logic)."""
        connected = bool(msg.data)
        with self._lock:
            prev = self._charger_present
            self._charger_present = connected
            current = self._current_mode

        if connected and not prev:
            self.get_logger().info("Charger connected — switching to charging")
            self._transition(MODE_CHARGING)
        elif not connected and prev and current == MODE_CHARGING:
            self.get_logger().info("Charger disconnected — switching to idle")
            self._transition(MODE_IDLE)

    def _on_battery(self, msg: BatteryState) -> None:
        """Auto-stop mowing when battery is low (mirrors OG MowingBehavior logic)."""
        if math.isnan(msg.percentage) or msg.percentage < 0:
            return

        pct = msg.percentage * 100.0  # BatteryState uses 0.0–1.0

        with self._lock:
            self._battery_pct = pct
            current = self._current_mode
            already_triggered = self._battery_low_triggered

        if pct < self._battery_low_threshold and current == MODE_MOWING and not already_triggered:
            self.get_logger().warning(
                f"Battery low ({pct:.1f}% < {self._battery_low_threshold}%) — stopping mow"
            )
            with self._lock:
                self._battery_low_triggered = True
            self._transition(MODE_IDLE)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MowerLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
