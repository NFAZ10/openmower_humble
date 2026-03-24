#!/usr/bin/env python3
# Mowing State Machine Node for OpenMower Humble
#
# Ports the high-level state logic from the original open_mower_ros
# Copyright (c) 2022 Clemens Elflein - https://github.com/ClemensElflein/open_mower_ros
# ROS2 adaptation for openmowernext + rosmower by NFAZ10.
#
# Responsibilities:
#   - Publish /robot_mode (transient-local) so bridge/UI always see current state
#   - Subscribe to /robot_mode_cmd for UI commands
#   - Enable/disable mow blade (ros2_control effort controller) on transitions
#   - Auto-switch to charging when charger detected on /power/charger_present
#   - Auto-stop mowing when battery low (from /power)
#   - Monitor GPS quality via odometry covariance; block/pause mow if lost
#   - Handle emergency stop via /emergency topic
#   - Support pause/resume commands
#
# Env vars (all optional):
#   OM_MODE_TOPIC              default: /robot_mode
#   OM_MODE_CMD_TOPIC          default: /robot_mode_cmd
#   OM_CHARGER_TOPIC           default: /power/charger_present
#   OM_BATTERY_TOPIC           default: /power
#   OM_ODOM_TOPIC              default: /odometry/filtered
#   OM_EMERGENCY_TOPIC         default: /emergency
#   OM_BLADE_TOPIC             default: /mower_controller/commands
#   OM_BLADE_ENABLED           default: true
#   OM_BLADE_SPEED             default: 1.0  (0.0-1.0)
#   OM_BLADE_SPINUP_DELAY      default: 2.0  (seconds)
#   OM_BATTERY_LOW_THRESHOLD   default: 20.0 (percent)
#   OM_GPS_QUALITY_MIN         default: 30.0 (percent)
#   OM_GPS_TIMEOUT             default: 5.0  (seconds)

import math
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64MultiArray, String

MODE_IDLE = "idle"
MODE_MOWING = "mowing"
MODE_CHARGING = "charging"
MODE_DOCKING = "docking"
MODE_EMERGENCY = "emergency"


class MowerLogicNode(Node):

    def __init__(self):
        super().__init__("mower_logic")

        self._mode_topic = os.getenv("OM_MODE_TOPIC", "/robot_mode")
        self._mode_cmd_topic = os.getenv("OM_MODE_CMD_TOPIC", "/robot_mode_cmd")
        self._charger_topic = os.getenv("OM_CHARGER_TOPIC", "/power/charger_present")
        self._battery_topic = os.getenv("OM_BATTERY_TOPIC", "/power")
        self._odom_topic = os.getenv("OM_ODOM_TOPIC", "/odometry/filtered")
        self._emergency_topic = os.getenv("OM_EMERGENCY_TOPIC", "/emergency")
        self._blade_topic = os.getenv("OM_BLADE_TOPIC", "/mower_controller/commands")
        self._blade_enabled = os.getenv("OM_BLADE_ENABLED", "true").lower() == "true"
        self._blade_speed = float(os.getenv("OM_BLADE_SPEED", "1.0"))
        self._blade_spinup_delay = float(os.getenv("OM_BLADE_SPINUP_DELAY", "2.0"))
        self._battery_low_threshold = float(os.getenv("OM_BATTERY_LOW_THRESHOLD", "20.0"))
        self._gps_quality_min = float(os.getenv("OM_GPS_QUALITY_MIN", "30.0"))
        self._gps_timeout = float(os.getenv("OM_GPS_TIMEOUT", "5.0"))

        self._lock = threading.Lock()
        self._current_mode = MODE_IDLE
        self._charger_present = False
        self._battery_pct = 100.0
        self._battery_low_triggered = False
        self._mowing_paused = False
        self._emergency_active = False
        self._gps_quality_pct = 100.0
        self._last_gps_time = time.time()
        self._gps_lost_while_mowing = False

        latching_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._mode_pub = self.create_publisher(String, self._mode_topic, latching_qos)
        self._blade_pub = self.create_publisher(Float64MultiArray, self._blade_topic, 10)

        self.create_subscription(String, self._mode_cmd_topic, self._on_mode_cmd, 10)
        self.create_subscription(Bool, self._charger_topic, self._on_charger, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._on_battery, 10)
        self.create_subscription(Odometry, self._odom_topic, self._on_odometry, 10)
        self.create_subscription(Bool, self._emergency_topic, self._on_emergency, 10)

        self.create_timer(2.0, self._heartbeat)
        self.create_timer(1.0, self._gps_watchdog)

        self._publish_mode(MODE_IDLE)
        self.get_logger().info(
            "Mower Logic started | blade=%s battery_low=%.0f%% gps_min=%.0f%% gps_timeout=%.0fs" % (
                self._blade_enabled, self._battery_low_threshold,
                self._gps_quality_min, self._gps_timeout
            )
        )

    def _transition(self, new_mode):
        with self._lock:
            if new_mode == self._current_mode:
                return
            old = self._current_mode
            self._current_mode = new_mode
        self.get_logger().info("State: %s -> %s" % (old, new_mode))
        if new_mode == MODE_MOWING:
            self._schedule_blade(True)
        else:
            self._control_blade(False)
        self._publish_mode(new_mode)

    def _publish_mode(self, mode):
        msg = String()
        msg.data = mode
        self._mode_pub.publish(msg)

    def _control_blade(self, enable):
        if not self._blade_enabled:
            return
        msg = Float64MultiArray()
        msg.data = [self._blade_speed if enable else 0.0]
        self._blade_pub.publish(msg)
        self.get_logger().info("Blade: %s" % ("ON" if enable else "OFF"))

    def _schedule_blade(self, enable):
        def _delayed():
            with self._lock:
                still_mowing = self._current_mode == MODE_MOWING
            if still_mowing:
                self._control_blade(enable)
        t = threading.Timer(self._blade_spinup_delay, _delayed)
        t.daemon = True
        t.start()

    def _heartbeat(self):
        with self._lock:
            mode = self._current_mode
        self._publish_mode(mode)

    def _gps_watchdog(self):
        with self._lock:
            mode = self._current_mode
            last = self._last_gps_time
            already_lost = self._gps_lost_while_mowing

        if mode != MODE_MOWING:
            return

        age = time.time() - last
        if age > self._gps_timeout and not already_lost:
            self.get_logger().warning("GPS timeout (%.1fs) -- pausing mowing" % age)
            with self._lock:
                self._gps_lost_while_mowing = True
                self._mowing_paused = True
            self._control_blade(False)
            self._publish_mode(MODE_IDLE)
        elif age <= self._gps_timeout and already_lost:
            self.get_logger().info("GPS recovered -- resuming mowing")
            with self._lock:
                self._gps_lost_while_mowing = False
                self._mowing_paused = False
            self._schedule_blade(True)
            self._publish_mode(MODE_MOWING)

    def _on_mode_cmd(self, msg):
        cmd = msg.data.strip().lower()
        self.get_logger().debug("Mode command: %r" % cmd)

        with self._lock:
            charger = self._charger_present
            emergency = self._emergency_active
            current = self._current_mode
            gps_ok = self._gps_quality_pct >= self._gps_quality_min

        if cmd == MODE_MOWING:
            if emergency:
                self.get_logger().warning("Cannot mow during emergency")
                return
            if charger:
                self.get_logger().warning("Cannot mow while charger connected")
                return
            if not gps_ok:
                self.get_logger().warning(
                    "Cannot mow -- GPS quality too low (%.0f%% < %.0f%%)" % (
                        self._gps_quality_pct, self._gps_quality_min))
                return
            with self._lock:
                self._battery_low_triggered = False
                self._gps_lost_while_mowing = False
                self._mowing_paused = False
            self._transition(MODE_MOWING)

        elif cmd == "pause":
            if current == MODE_MOWING:
                self.get_logger().info("Mowing paused")
                with self._lock:
                    self._mowing_paused = True
                self._control_blade(False)
                self._publish_mode(MODE_IDLE)

        elif cmd == "resume":
            with self._lock:
                was_paused = self._mowing_paused
            if was_paused:
                self.get_logger().info("Mowing resumed")
                with self._lock:
                    self._mowing_paused = False
                self._transition(MODE_MOWING)

        elif cmd == MODE_IDLE:
            with self._lock:
                self._mowing_paused = False
            self._transition(MODE_IDLE)

        elif cmd == MODE_CHARGING:
            self._transition(MODE_CHARGING)

        elif cmd == MODE_DOCKING:
            self._transition(MODE_IDLE)

        elif cmd == "reset_emergency":
            with self._lock:
                self._emergency_active = False
            self.get_logger().info("Emergency reset")
            self._transition(MODE_IDLE)

        else:
            self.get_logger().warning("Unknown mode command: %r" % cmd)

    def _on_charger(self, msg):
        connected = bool(msg.data)
        with self._lock:
            prev = self._charger_present
            self._charger_present = connected
            current = self._current_mode
        if connected and not prev:
            self.get_logger().info("Charger connected -> charging")
            self._transition(MODE_CHARGING)
        elif not connected and prev and current == MODE_CHARGING:
            self.get_logger().info("Charger disconnected -> idle")
            self._transition(MODE_IDLE)

    def _on_battery(self, msg):
        if math.isnan(msg.percentage) or msg.percentage < 0:
            return
        pct = msg.percentage * 100.0  # BatteryState is 0.0-1.0
        with self._lock:
            self._battery_pct = pct
            current = self._current_mode
            already_triggered = self._battery_low_triggered
        if pct < self._battery_low_threshold and current == MODE_MOWING and not already_triggered:
            self.get_logger().warning(
                "Battery low (%.1f%% < %.1f%%) -- stopping mow" % (
                    pct, self._battery_low_threshold))
            with self._lock:
                self._battery_low_triggered = True
            self._transition(MODE_IDLE)

    def _on_odometry(self, msg):
        with self._lock:
            self._last_gps_time = time.time()
        cov_x = msg.pose.covariance[0]
        cov_y = msg.pose.covariance[7]
        if cov_x <= 0 or cov_y <= 0 or math.isnan(cov_x) or math.isnan(cov_y):
            return
        avg_std = math.sqrt((cov_x + cov_y) / 2.0)
        quality = max(0.0, min(100.0, (1.0 - avg_std) * 100.0))
        with self._lock:
            self._gps_quality_pct = quality

    def _on_emergency(self, msg):
        active = bool(msg.data)
        with self._lock:
            was_active = self._emergency_active
            self._emergency_active = active
            current = self._current_mode
        if active and not was_active:
            self.get_logger().error("EMERGENCY STOP -- halting all motion")
            self._control_blade(False)
            self._publish_mode(MODE_EMERGENCY)
        elif not active and was_active:
            self.get_logger().info("Emergency cleared -> idle")
            self._transition(MODE_IDLE)


def main(args=None):
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
