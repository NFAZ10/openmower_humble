#!/usr/bin/env python3

import asyncio
import json
import math
import os
import signal
import threading
import time
from pathlib import Path
from typing import Any

import paho.mqtt.client as mqtt
import rclpy
import websockets
from bson import BSON
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
from open_mower_next.action import DockRobotNearest, RecordAreaBoundary, RecordDockingStation
from open_mower_next.msg import Area, Map as MowerMap
from open_mower_next.srv import SaveArea
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


LEGACY_STATE_AREA_RECORDING = "AREA_RECORDING"
LEGACY_STATE_PAUSED = "paused"
DEFAULT_AREA_DISTANCE_THRESHOLD = 0.05
ROBOT_MODE_IDLE = "idle"
ROBOT_MODE_CHARGING = "charging"
ROBOT_MODE_MOWING = "mowing"
ROBOT_MODE_FULL = "full"
MOWING_PATH_NAME = "bridge_boustrophedon"
DEFAULT_MOW_STRIPE_SPACING = 0.4
DEFAULT_MOW_BOUNDARY_MARGIN = 0.15
DEFAULT_MOW_WAYPOINT_TOLERANCE = 0.35

ACTION_START_MOWING = "mower_logic:idle/start_mowing"
ACTION_START_AREA_RECORDING = "mower_logic:idle/start_area_recording"
ACTION_PAUSE_MOWING = "mower_logic:mowing/pause"
ACTION_CONTINUE_MOWING = "mower_logic:mowing/continue"
ACTION_START_RECORDING = "mower_logic:area_recording/start_recording"
ACTION_STOP_RECORDING = "mower_logic:area_recording/stop_recording"
ACTION_FINISH_MOWING_AREA = "mower_logic:area_recording/finish_mowing_area"
ACTION_FINISH_NAVIGATION_AREA = "mower_logic:area_recording/finish_navigation_area"
ACTION_FINISH_DISCARD = "mower_logic:area_recording/finish_discard"
ACTION_RECORD_DOCK = "mower_logic:area_recording/record_dock"
ACTION_EXIT_RECORDING_MODE = "mower_logic:area_recording/exit_recording_mode"
ACTION_AUTO_POINT_COLLECTING_ENABLE = "mower_logic:area_recording/auto_point_collecting_enable"
ACTION_AUTO_POINT_COLLECTING_DISABLE = "mower_logic:area_recording/auto_point_collecting_disable"
ACTION_COLLECT_POINT = "mower_logic:area_recording/collect_point"
ACTION_START_MANUAL_MOWING = "mower_logic:area_recording/start_manual_mowing"
ACTION_STOP_MANUAL_MOWING = "mower_logic:area_recording/stop_manual_mowing"
ACTION_ABORT_MOWING = "mower_logic:mowing/abort_mowing"


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class OpenMowerUiBridge(Node):
    def __init__(self) -> None:
        super().__init__("openmower_ui_bridge")

        self._state_lock = threading.RLock()
        self._have_pose = False
        self._last_pose = {
            "x": 0.0,
            "y": 0.0,
            "heading": 0.0,
            "pos_accuracy": 0.0,
            "heading_accuracy": 0.0,
            "heading_valid": 0,
        }
        self._charger_present = False
        self._battery_percentage = 0.0
        self._last_teleop_time = 0.0
        self._robot_mode = ""
        self._mowing_paused = False
        self._mowing_nav_goal_handle: Any | None = None
        self._mowing_nav_goal_pending = False
        self._mowing_plan_poses: list[PoseStamped] = []
        self._mowing_goal_offset = 0
        self._mowing_goal_length = 0
        self._mowing_resume_index = 0
        self._mowing_progress_index = 0
        self._active_mowing_area_id = ""
        self._active_mowing_area_name = ""

        self._legacy_area_recording_mode = False
        self._area_recording_goal_handle: Any | None = None
        self._area_recording_auto_enabled = True
        self._area_recording_paused = False
        self._area_recording_finalizing = False
        self._area_recording_pending_type: int | None = None
        self._area_recording_cancel_reason: str | None = None
        self._record_docking_goal_handle: Any | None = None
        self._dock_nearest_goal_handle: Any | None = None

        self._mqtt_host = os.getenv("MQTT_HOST", "127.0.0.1")
        self._mqtt_port = int(os.getenv("MQTT_PORT", os.getenv("OM_MQTT_PORT", "1884")))
        self._mqtt_username = os.getenv("MQTT_USERNAME", "")
        self._mqtt_password = os.getenv("MQTT_PASSWORD", "")
        self._ws_port = int(os.getenv("OM_UI_BRIDGE_WS_PORT", "9002"))
        self._teleop_topic = os.getenv("OM_UI_BRIDGE_TELEOP_TOPIC", "/cmd_vel_joy")
        self._odom_topic = os.getenv("OM_UI_BRIDGE_ODOM_TOPIC", "/odometry/filtered")
        self._map_topic = os.getenv("OM_UI_BRIDGE_MAP_TOPIC", "/mowing_map")
        self._charger_topic = os.getenv("OM_UI_BRIDGE_CHARGER_TOPIC", "/power/charger_present")
        self._battery_topic = os.getenv("OM_UI_BRIDGE_BATTERY_TOPIC", "/power")
        self._mode_topic = os.getenv("OM_UI_BRIDGE_MODE_TOPIC", "/robot_mode")
        self._mode_cmd_topic = os.getenv("OM_UI_BRIDGE_MODE_CMD_TOPIC", "/robot_mode_cmd")
        self._version_string = os.getenv("OM_UI_BRIDGE_VERSION", "openmower-humble")
        self._map_path = Path(os.getenv("OM_MAP_PATH", "/data/map.json"))
        self._mow_stripe_spacing = max(0.1, float(os.getenv("OM_UI_BRIDGE_MOW_STRIPE_SPACING", DEFAULT_MOW_STRIPE_SPACING)))
        self._mow_boundary_margin = max(0.0, float(os.getenv("OM_UI_BRIDGE_MOW_BOUNDARY_MARGIN", DEFAULT_MOW_BOUNDARY_MARGIN)))
        self._mow_waypoint_tolerance = max(
            0.05, float(os.getenv("OM_UI_BRIDGE_MOW_WAYPOINT_TOLERANCE", DEFAULT_MOW_WAYPOINT_TOLERANCE))
        )

        self._map_payload = {"d": {"areas": [], "docking_stations": []}}
        self._overlay_payload = {"d": {"polygons": []}}
        self._sensor_infos_payload = {"d": []}
        self._actions_payload = {"d": []}
        self._last_robot_state_error = ""

        self._load_map_fallback()

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._cmd_vel_publisher = self.create_publisher(Twist, self._teleop_topic, 10)
        self._mode_cmd_publisher = self.create_publisher(String, self._mode_cmd_topic, 10)
        self.create_subscription(Odometry, self._odom_topic, self._odom_callback, 10)
        self.create_subscription(MowerMap, self._map_topic, self._map_callback, map_qos)
        self.create_subscription(Bool, self._charger_topic, self._charger_callback, 10)
        self.create_subscription(BatteryState, self._battery_topic, self._battery_callback, 10)
        self.create_subscription(String, self._mode_topic, self._mode_callback, 10)
        self.create_timer(0.5, self._publish_robot_state_safe)

        self._callback_group = ReentrantCallbackGroup()
        self._record_area_client = ActionClient(
            self,
            RecordAreaBoundary,
            "record_area_boundary",
            callback_group=self._callback_group,
        )
        self._record_docking_client = ActionClient(
            self,
            RecordDockingStation,
            "record_docking_station",
            callback_group=self._callback_group,
        )
        self._dock_nearest_client = ActionClient(
            self,
            DockRobotNearest,
            "dock_robot_nearest",
            callback_group=self._callback_group,
        )
        self._navigate_through_poses_client = ActionClient(
            self,
            NavigateThroughPoses,
            "navigate_through_poses",
            callback_group=self._callback_group,
        )
        self._set_recording_mode_client = self.create_client(
            SetBool,
            "set_recording_mode",
            callback_group=self._callback_group,
        )
        self._add_boundary_point_client = self.create_client(
            Trigger,
            "add_boundary_point",
            callback_group=self._callback_group,
        )
        self._finish_area_recording_client = self.create_client(
            Trigger,
            "finish_area_recording",
            callback_group=self._callback_group,
        )
        self._save_area_client = self.create_client(
            SaveArea,
            "save_area",
            callback_group=self._callback_group,
        )

        self._mqtt_client = mqtt.Client(client_id="openmower-ui-bridge", clean_session=True)
        if self._mqtt_username:
            self._mqtt_client.username_pw_set(self._mqtt_username, self._mqtt_password)
        self._mqtt_client.enable_logger()
        self._mqtt_client.on_connect = self._on_mqtt_connect
        self._mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self._mqtt_client.on_message = self._on_mqtt_message
        self._mqtt_client.reconnect_delay_set(min_delay=1, max_delay=30)
        self._mqtt_client.connect_async(self._mqtt_host, self._mqtt_port, keepalive=20)
        self._mqtt_client.loop_start()

        self._refresh_ui_state()

    def _load_map_fallback(self) -> None:
        if not self._map_path.exists():
            return

        try:
            raw_map = json.loads(self._map_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"Failed to read map file fallback {self._map_path}: {exc}")
            return

        if isinstance(raw_map, dict) and isinstance(raw_map.get("d"), dict):
            self._map_payload = raw_map
            return

        if raw_map.get("type") != "FeatureCollection":
            return

        areas: list[dict[str, Any]] = []
        docking_stations: list[dict[str, Any]] = []

        for index, feature in enumerate(raw_map.get("features", [])):
            geometry = feature.get("geometry") or {}
            properties = feature.get("properties") or {}
            geometry_type = geometry.get("type")

            if geometry_type == "Polygon":
                coordinates = geometry.get("coordinates") or []
                if not coordinates:
                    continue

                outline = []
                for point in coordinates[0]:
                    if len(point) < 2:
                        continue
                    outline.append({"x": float(point[0]), "y": float(point[1])})

                if len(outline) > 1 and outline[0] == outline[-1]:
                    outline.pop()

                area_type = str(properties.get("type", "mow")).lower()
                if area_type in {"operation", "work", "working", "mowing"}:
                    area_type = "mow"
                elif area_type in {"navigation", "nav"}:
                    area_type = "nav"
                elif area_type in {"exclusion", "obstacle"}:
                    area_type = "obstacle"

                areas.append(
                    {
                        "id": str(properties.get("id", f"area_{index}")),
                        "name": str(properties.get("name", properties.get("id", f"area_{index}"))),
                        "outline": outline,
                        "properties": {"type": area_type},
                    }
                )
            elif geometry_type == "Point":
                coordinates = geometry.get("coordinates") or []
                point_type = str(properties.get("type", "")).lower()
                if len(coordinates) < 2 or point_type not in {"dock", "docking_station", "charging_station"}:
                    continue

                docking_stations.append(
                    {
                        "id": str(properties.get("id", f"dock_{index}")),
                        "name": str(properties.get("name", properties.get("id", f"dock_{index}"))),
                        "position": {"x": float(coordinates[0]), "y": float(coordinates[1])},
                        "heading": float(properties.get("heading", 0.0)),
                    }
                )

        self._map_payload = {"d": {"areas": areas, "docking_stations": docking_stations}}

    def _next_generated_name(self, prefix: str) -> str:
        return f"{prefix}_{time.strftime('%Y%m%d_%H%M%S')}"

    def _start_background_task(self, target: Any, *args: Any) -> None:
        threading.Thread(target=target, args=args, daemon=True).start()

    def _wait_for_action_server(self, client: ActionClient, action_name: str, timeout_sec: float = 2.0) -> bool:
        if client.wait_for_server(timeout_sec=timeout_sec):
            return True
        self.get_logger().warning(f"Action server '{action_name}' is not available")
        return False

    def _wait_for_service(self, client: Any, service_name: str, timeout_sec: float = 2.0) -> bool:
        if client.wait_for_service(timeout_sec=timeout_sec):
            return True
        self.get_logger().warning(f"Service '{service_name}' is not available")
        return False

    def _clear_mowing_execution_locked(self) -> None:
        self._mowing_nav_goal_handle = None
        self._mowing_nav_goal_pending = False
        self._mowing_plan_poses = []
        self._mowing_goal_offset = 0
        self._mowing_goal_length = 0
        self._mowing_resume_index = 0
        self._mowing_progress_index = 0
        self._active_mowing_area_id = ""
        self._active_mowing_area_name = ""

    def _normalize_outline(self, outline: list[dict[str, float]]) -> list[tuple[float, float]]:
        normalized = [(float(point["x"]), float(point["y"])) for point in outline if "x" in point and "y" in point]
        if len(normalized) > 1 and normalized[0] == normalized[-1]:
            normalized.pop()
        return normalized

    def _point_in_polygon(self, x: float, y: float, outline: list[tuple[float, float]]) -> bool:
        inside = False
        point_count = len(outline)
        if point_count < 3:
            return False

        for index in range(point_count):
            x1, y1 = outline[index]
            x2, y2 = outline[(index + 1) % point_count]
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-9) + x1):
                inside = not inside

        return inside

    def _polygon_centroid(self, outline: list[tuple[float, float]]) -> tuple[float, float]:
        if not outline:
            return 0.0, 0.0

        signed_area = 0.0
        centroid_x = 0.0
        centroid_y = 0.0

        for index in range(len(outline)):
            x1, y1 = outline[index]
            x2, y2 = outline[(index + 1) % len(outline)]
            cross = x1 * y2 - x2 * y1
            signed_area += cross
            centroid_x += (x1 + x2) * cross
            centroid_y += (y1 + y2) * cross

        if abs(signed_area) < 1e-9:
            avg_x = sum(point[0] for point in outline) / len(outline)
            avg_y = sum(point[1] for point in outline) / len(outline)
            return avg_x, avg_y

        signed_area *= 0.5
        return centroid_x / (6.0 * signed_area), centroid_y / (6.0 * signed_area)

    def _select_mowing_area_locked(self, x: float, y: float) -> dict[str, Any] | None:
        map_data = self._map_payload.get("d", {})
        mow_areas = [
            area
            for area in map_data.get("areas", [])
            if isinstance(area, dict) and (area.get("properties") or {}).get("type") == "mow"
        ]

        if not mow_areas:
            return None

        containing_areas = []
        nearest_area = mow_areas[0]
        nearest_distance_sq = math.inf

        for area in mow_areas:
            outline = self._normalize_outline(area.get("outline", []))
            if len(outline) < 3:
                continue

            if self._point_in_polygon(x, y, outline):
                containing_areas.append(area)

            centroid_x, centroid_y = self._polygon_centroid(outline)
            distance_sq = (centroid_x - x) ** 2 + (centroid_y - y) ** 2
            if distance_sq < nearest_distance_sq:
                nearest_distance_sq = distance_sq
                nearest_area = area

        if containing_areas:
            if len(containing_areas) == 1:
                return containing_areas[0]

            containing_areas.sort(
                key=lambda area: (area.get("name") or area.get("id") or "", area.get("id") or "")
            )
            return containing_areas[0]

        return nearest_area

    def _scanline_intersections(
        self,
        outline: list[tuple[float, float]],
        scan_value: float,
        horizontal_rows: bool,
    ) -> list[float]:
        intersections = []
        for index in range(len(outline)):
            x1, y1 = outline[index]
            x2, y2 = outline[(index + 1) % len(outline)]

            if horizontal_rows:
                if abs(y1 - y2) < 1e-9:
                    continue
                if not ((y1 <= scan_value < y2) or (y2 <= scan_value < y1)):
                    continue
                ratio = (scan_value - y1) / (y2 - y1)
                intersections.append(x1 + ratio * (x2 - x1))
            else:
                if abs(x1 - x2) < 1e-9:
                    continue
                if not ((x1 <= scan_value < x2) or (x2 <= scan_value < x1)):
                    continue
                ratio = (scan_value - x1) / (x2 - x1)
                intersections.append(y1 + ratio * (y2 - y1))

        intersections.sort()
        return intersections

    def _deduplicate_path_points(self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        deduplicated = []
        for point in points:
            if deduplicated:
                prev_x, prev_y = deduplicated[-1]
                if math.hypot(point[0] - prev_x, point[1] - prev_y) < 0.05:
                    continue
            deduplicated.append(point)
        return deduplicated

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.z = math.sin(yaw * 0.5)
        quaternion.w = math.cos(yaw * 0.5)
        return quaternion

    def _make_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = self._yaw_to_quaternion(yaw)
        return pose

    def _build_mowing_route_points(
        self,
        outline: list[tuple[float, float]],
        start_x: float,
        start_y: float,
    ) -> list[tuple[float, float]]:
        if len(outline) < 3:
            return []

        xs = [point[0] for point in outline]
        ys = [point[1] for point in outline]
        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)
        width = max_x - min_x
        height = max_y - min_y
        horizontal_rows = width >= height
        scan_min = min_y if horizontal_rows else min_x
        scan_max = max_y if horizontal_rows else max_x
        margin = self._mow_boundary_margin
        stripe_spacing = self._mow_stripe_spacing
        start_scan = scan_min + margin
        end_scan = scan_max - margin

        if end_scan <= start_scan:
            start_scan = scan_min
            end_scan = scan_max

        scan_values = []
        scan_value = start_scan
        while scan_value <= end_scan + 1e-9:
            scan_values.append(scan_value)
            scan_value += stripe_spacing

        if not scan_values:
            midpoint = (scan_min + scan_max) * 0.5
            scan_values.append(midpoint)

        route_points = []
        reverse_row = False

        for scan_value in scan_values:
            intersections = self._scanline_intersections(outline, scan_value, horizontal_rows)
            if len(intersections) < 2:
                continue

            row_points = []
            for pair_index in range(0, len(intersections) - 1, 2):
                segment_start = intersections[pair_index]
                segment_end = intersections[pair_index + 1]
                if segment_end <= segment_start:
                    continue

                trim = min(margin, max(0.0, 0.5 * (segment_end - segment_start) - 1e-3))
                segment_start += trim
                segment_end -= trim
                if segment_end <= segment_start:
                    continue

                if horizontal_rows:
                    row_points.extend([(segment_start, scan_value), (segment_end, scan_value)])
                else:
                    row_points.extend([(scan_value, segment_start), (scan_value, segment_end)])

            if not row_points:
                continue

            if reverse_row:
                row_points.reverse()

            route_points.extend(row_points)
            reverse_row = not reverse_row

        route_points = self._deduplicate_path_points(route_points)
        if len(route_points) > 1:
            first_distance = math.hypot(route_points[0][0] - start_x, route_points[0][1] - start_y)
            last_distance = math.hypot(route_points[-1][0] - start_x, route_points[-1][1] - start_y)
            if last_distance < first_distance:
                route_points.reverse()

        return route_points

    def _generate_mowing_plan(
        self,
        area: dict[str, Any],
        start_x: float,
        start_y: float,
        start_heading: float,
    ) -> list[PoseStamped]:
        outline = self._normalize_outline(area.get("outline", []))
        route_points = self._build_mowing_route_points(outline, start_x, start_y)
        if not route_points:
            return []

        poses = []
        fallback_yaw = start_heading
        for index, (point_x, point_y) in enumerate(route_points):
            if index + 1 < len(route_points):
                next_x, next_y = route_points[index + 1]
                yaw = math.atan2(next_y - point_y, next_x - point_x)
                fallback_yaw = yaw
            else:
                yaw = fallback_yaw
            poses.append(self._make_pose_stamped(point_x, point_y, yaw))

        return poses

    def _estimate_next_resume_index_locked(self, x: float, y: float) -> int:
        if not self._mowing_plan_poses:
            return 0

        search_start = min(self._mowing_progress_index, len(self._mowing_plan_poses) - 1)
        best_index = search_start
        best_distance_sq = math.inf

        for index in range(search_start, len(self._mowing_plan_poses)):
            pose = self._mowing_plan_poses[index].pose.position
            distance_sq = (pose.x - x) ** 2 + (pose.y - y) ** 2
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_index = index

        if best_distance_sq <= self._mow_waypoint_tolerance**2:
            return min(best_index + 1, len(self._mowing_plan_poses))

        return best_index

    def _send_mowing_navigation_goal(self, poses: list[PoseStamped], goal_offset: int) -> None:
        with self._state_lock:
            self._mowing_goal_offset = goal_offset
            self._mowing_goal_length = len(poses)

        goal = NavigateThroughPoses.Goal()
        goal.poses = poses
        if hasattr(goal, "behavior_tree"):
            goal.behavior_tree = ""

        future = self._navigate_through_poses_client.send_goal_async(
            goal,
            feedback_callback=self._on_mowing_nav_feedback,
        )
        future.add_done_callback(self._on_mowing_nav_goal_response)

    def _cancel_mowing_navigation_goal(self) -> None:
        goal_handle = None
        with self._state_lock:
            goal_handle = self._mowing_nav_goal_handle

        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def _build_actions_payload_locked(self) -> dict[str, Any]:
        action_ids: set[str] = set()
        mowing_controls_available = self._mode_cmd_publisher.get_subscription_count() > 0 or bool(self._robot_mode)
        mowing_active = self._robot_mode == ROBOT_MODE_MOWING

        if self._legacy_area_recording_mode:
            if self._record_docking_goal_handle is not None:
                action_ids.add(ACTION_EXIT_RECORDING_MODE)
            elif self._area_recording_goal_handle is None:
                action_ids.add(ACTION_EXIT_RECORDING_MODE)
                if self._have_pose and not self._charger_present:
                    action_ids.add(ACTION_START_RECORDING)
                    action_ids.add(ACTION_RECORD_DOCK)
                    if mowing_controls_available:
                        action_ids.add(ACTION_START_MANUAL_MOWING)
            elif self._area_recording_finalizing:
                pass
            else:
                if self._area_recording_paused:
                    action_ids.add(ACTION_START_RECORDING)
                else:
                    action_ids.add(ACTION_STOP_RECORDING)

                action_ids.update(
                    {
                        ACTION_FINISH_MOWING_AREA,
                        ACTION_FINISH_NAVIGATION_AREA,
                        ACTION_FINISH_DISCARD,
                        ACTION_EXIT_RECORDING_MODE,
                    }
                )

                if mowing_controls_available:
                    if mowing_active:
                        action_ids.add(ACTION_STOP_MANUAL_MOWING)
                    else:
                        action_ids.add(ACTION_START_MANUAL_MOWING)

                if self._area_recording_auto_enabled:
                    action_ids.add(ACTION_AUTO_POINT_COLLECTING_DISABLE)
                else:
                    action_ids.add(ACTION_AUTO_POINT_COLLECTING_ENABLE)
                    if not self._area_recording_paused:
                        action_ids.add(ACTION_COLLECT_POINT)
        elif self._dock_nearest_goal_handle is not None:
            pass
        elif self._mowing_paused and mowing_controls_available:
            action_ids.add(ACTION_CONTINUE_MOWING)
            action_ids.add(ACTION_ABORT_MOWING)
        elif mowing_active and mowing_controls_available:
            action_ids.add(ACTION_PAUSE_MOWING)
            action_ids.add(ACTION_ABORT_MOWING)
        elif self._have_pose and not self._charger_present:
            action_ids.add(ACTION_START_AREA_RECORDING)
            if mowing_controls_available:
                action_ids.add(ACTION_START_MOWING)

        return {"d": [{"action_id": action_id, "enabled": 1} for action_id in sorted(action_ids)]}

    def _refresh_ui_state(self) -> None:
        with self._state_lock:
            self._actions_payload = self._build_actions_payload_locked()
            actions_payload = self._actions_payload

        self._publish_bson("actions/bson", actions_payload, retain=True, qos=1)
        self._publish_robot_state()

    def _on_mqtt_connect(self, client: mqtt.Client, _userdata: Any, _flags: dict[str, Any], rc: int) -> None:
        if rc != 0:
            self.get_logger().error(f"MQTT connect failed with code {rc}")
            return

        self.get_logger().info(
            f"Connected to MQTT broker at {self._mqtt_host}:{self._mqtt_port}, publishing legacy UI topics"
        )
        client.subscribe(
            [
                ("teleop", 0),
                ("/teleop", 0),
                ("action", 1),
                ("/action", 1),
                ("command", 0),
                ("/command", 0),
            ]
        )
        self._start_background_task(self._publish_mqtt_state_after_connect)

    def _on_mqtt_disconnect(self, _client: mqtt.Client, _userdata: Any, rc: int) -> None:
        if rc != 0:
            self.get_logger().warning("Disconnected from MQTT broker, waiting for reconnect")
        else:
            self.get_logger().info("Disconnected from MQTT broker")

    def _publish_bson(self, topic: str, payload: dict[str, Any], retain: bool = False, qos: int = 0) -> None:
        if not self._mqtt_client.is_connected():
            return
        self._mqtt_client.publish(topic, BSON.encode(payload), qos=qos, retain=retain)

    def _publish_static_payloads(self) -> None:
        self._publish_bson("version", {"version": self._version_string}, retain=True, qos=1)
        self._publish_bson("actions/bson", self._actions_payload, retain=True, qos=1)
        self._publish_bson("sensor_infos/bson", self._sensor_infos_payload, retain=True, qos=1)
        self._publish_bson("map/bson", self._map_payload, retain=True, qos=1)
        self._publish_bson("map_overlay/bson", self._overlay_payload, retain=True, qos=1)

    def _publish_mqtt_state_after_connect(self) -> None:
        self._publish_static_payloads()
        self._refresh_ui_state()

    def _odom_callback(self, msg: Odometry) -> None:
        refresh_ui = False
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        covariance = list(msg.pose.covariance)
        pos_accuracy = 0.0
        heading_accuracy = 0.0

        if len(covariance) >= 36:
            pos_accuracy = max(math.sqrt(abs(covariance[0])), math.sqrt(abs(covariance[7])))
            heading_accuracy = math.sqrt(abs(covariance[35]))

        with self._state_lock:
            if not self._have_pose:
                refresh_ui = True
            self._have_pose = True
            self._last_pose = {
                "x": float(msg.pose.pose.position.x),
                "y": float(msg.pose.pose.position.y),
                "heading": float(yaw),
                "pos_accuracy": float(pos_accuracy),
                "heading_accuracy": float(heading_accuracy),
                "heading_valid": 1,
            }

        if refresh_ui:
            self._refresh_ui_state()

    def _map_callback(self, msg: MowerMap) -> None:
        areas = []
        for area in msg.areas:
            if area.type == Area.TYPE_OPERATION:
                area_type = "mow"
            elif area.type == Area.TYPE_NAVIGATION:
                area_type = "nav"
            else:
                area_type = "obstacle"

            areas.append(
                {
                    "id": area.id,
                    "name": area.name,
                    "outline": [{"x": float(point.x), "y": float(point.y)} for point in area.area.polygon.points],
                    "properties": {"type": area_type},
                }
            )

        docking_stations = []
        for station in msg.docking_stations:
            pose = station.pose.pose
            yaw = quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            docking_stations.append(
                {
                    "id": station.id,
                    "name": station.name,
                    "position": {"x": float(pose.position.x), "y": float(pose.position.y)},
                    "heading": float(yaw),
                    }
                )

        map_payload = {"d": {"areas": areas, "docking_stations": docking_stations}}
        with self._state_lock:
            self._map_payload = map_payload
        self._publish_bson("map/bson", map_payload, retain=True, qos=1)

    def _charger_callback(self, msg: Bool) -> None:
        refresh_ui = False
        with self._state_lock:
            charger_present = bool(msg.data)
            refresh_ui = charger_present != self._charger_present
            self._charger_present = charger_present

        if refresh_ui:
            self._refresh_ui_state()

    def _battery_callback(self, msg: BatteryState) -> None:
        percentage = msg.percentage
        if math.isnan(percentage):
            return
        with self._state_lock:
            self._battery_percentage = clamp(float(percentage) * 100.0, 0.0, 100.0)

    def _mode_callback(self, msg: String) -> None:
        refresh_ui = False
        robot_mode = msg.data.strip().lower()

        with self._state_lock:
            refresh_ui = robot_mode != self._robot_mode
            self._robot_mode = robot_mode

            if robot_mode != ROBOT_MODE_IDLE and self._mowing_paused:
                self._mowing_paused = False
                refresh_ui = True

        if refresh_ui:
            self._refresh_ui_state()

    def _publish_robot_state(self) -> None:
        with self._state_lock:
            pose = dict(self._last_pose)
            is_charging = self._charger_present
            battery_percentage = self._battery_percentage
            have_pose = self._have_pose
            last_teleop_time = self._last_teleop_time
            robot_mode = self._robot_mode
            mowing_paused = self._mowing_paused
            legacy_area_recording_mode = self._legacy_area_recording_mode
            current_area = self._active_mowing_area_name or self._active_mowing_area_id
            current_path = MOWING_PATH_NAME if self._mowing_plan_poses else ""
            current_path_index = self._mowing_progress_index

        if legacy_area_recording_mode:
            current_state = LEGACY_STATE_AREA_RECORDING
        elif is_charging or robot_mode == ROBOT_MODE_CHARGING:
            current_state = "charging"
        elif time.monotonic() - last_teleop_time < 1.0:
            current_state = "remote_control"
        elif mowing_paused:
            current_state = LEGACY_STATE_PAUSED
        elif robot_mode == ROBOT_MODE_MOWING:
            current_state = "mowing"
        elif have_pose or robot_mode in {ROBOT_MODE_IDLE, ROBOT_MODE_FULL}:
            current_state = "idle"
        else:
            current_state = "starting"

        payload = {
            "d": {
                "pose": pose,
                "emergency": 0,
                "is_charging": 1 if is_charging else 0,
                "rain_detected": 0,
                "current_state": current_state,
                "gps_percentage": 100 if have_pose else 0,
                "battery_percentage": battery_percentage,
                "current_area": current_area,
                "current_path": current_path,
                "current_path_index": current_path_index,
            }
        }
        self._publish_bson("robot_state/bson", payload)

    def _publish_robot_state_safe(self) -> None:
        try:
            self._publish_robot_state()
        except Exception as exc:
            message = str(exc)
            if message != self._last_robot_state_error:
                self.get_logger().error(f"Failed to publish robot state: {exc}")
                self._last_robot_state_error = message
        else:
            if self._last_robot_state_error:
                self.get_logger().info("Robot state publishing recovered")
                self._last_robot_state_error = ""

    def _publish_teleop(self, vx: float, vz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(vz)
        self._cmd_vel_publisher.publish(msg)

        with self._state_lock:
            self._last_teleop_time = time.monotonic()

    def _handle_teleop_payload(self, payload: bytes) -> None:
        try:
            command = BSON(payload).decode()
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode teleop BSON payload: {exc}")
            return

        self._publish_teleop(float(command.get("vx", 0.0)), float(command.get("vz", 0.0)))

    def _dispatch_legacy_action(self, action: str) -> None:
        handlers = {
            ACTION_START_MOWING: self._handle_start_mowing,
            ACTION_START_AREA_RECORDING: self._handle_start_area_recording_mode,
            ACTION_PAUSE_MOWING: self._handle_pause_mowing,
            ACTION_CONTINUE_MOWING: self._handle_continue_mowing,
            ACTION_START_RECORDING: self._handle_start_or_resume_recording,
            ACTION_STOP_RECORDING: self._handle_stop_recording,
            ACTION_FINISH_MOWING_AREA: lambda: self._handle_finish_area(Area.TYPE_OPERATION),
            ACTION_FINISH_NAVIGATION_AREA: lambda: self._handle_finish_area(Area.TYPE_NAVIGATION),
            ACTION_FINISH_DISCARD: self._handle_finish_discard,
            ACTION_RECORD_DOCK: self._handle_record_docking_station,
            ACTION_EXIT_RECORDING_MODE: self._handle_exit_recording_mode,
            ACTION_AUTO_POINT_COLLECTING_ENABLE: lambda: self._handle_set_auto_point_collecting(True),
            ACTION_AUTO_POINT_COLLECTING_DISABLE: lambda: self._handle_set_auto_point_collecting(False),
            ACTION_COLLECT_POINT: self._handle_collect_point,
            ACTION_START_MANUAL_MOWING: self._handle_start_manual_mowing,
            ACTION_STOP_MANUAL_MOWING: self._handle_stop_manual_mowing,
            ACTION_ABORT_MOWING: self._handle_abort_mowing,
        }

        handler = handlers.get(action)
        if handler is None:
            self.get_logger().warning(f"Legacy UI requested unsupported action '{action}'")
            return

        self.get_logger().info(f"Legacy UI requested action '{action}'")
        handler()

    def _publish_mode_command(self, mode: str) -> None:
        if self._mode_cmd_publisher.get_subscription_count() == 0:
            self.get_logger().warning(
                f"No subscribers are listening on '{self._mode_cmd_topic}', published mode command anyway"
            )

        msg = String()
        msg.data = mode
        self._mode_cmd_publisher.publish(msg)
        self.get_logger().info(f"Published robot mode command '{mode}' to {self._mode_cmd_topic}")

    def _handle_start_mowing(self) -> None:
        selected_area = None
        start_pose = None
        selected_area_name = ""

        with self._state_lock:
            if self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot start mowing while area recording mode is active")
                return
            if self._dock_nearest_goal_handle is not None:
                self.get_logger().warning("Cannot start mowing while docking is active")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot start mowing without a valid pose")
                return
            if self._charger_present:
                self.get_logger().warning("Cannot start mowing while charging")
                return
            if (self._robot_mode == ROBOT_MODE_MOWING and not self._mowing_paused) or self._mowing_nav_goal_pending:
                self.get_logger().info("Mowing is already active")
                return
            if self._mowing_nav_goal_handle is not None and not self._mowing_paused:
                self.get_logger().info("A mowing navigation goal is already active")
                return
            selected_area = self._select_mowing_area_locked(self._last_pose["x"], self._last_pose["y"])
            start_pose = dict(self._last_pose)
            self._mowing_paused = False

        if selected_area is None:
            self.get_logger().warning("Cannot start mowing because no mowing area is available in the current map")
            return

        selected_area_name = str(selected_area.get("name", selected_area.get("id", "")))

        plan_poses = self._generate_mowing_plan(
            selected_area,
            float(start_pose["x"]),
            float(start_pose["y"]),
            float(start_pose["heading"]),
        )
        if not plan_poses:
            self.get_logger().warning(
                f"Failed to generate a mowing route for area '{selected_area.get('name') or selected_area.get('id')}'"
            )
            return

        if not self._wait_for_action_server(self._navigate_through_poses_client, "navigate_through_poses"):
            return

        with self._state_lock:
            self._mowing_plan_poses = plan_poses
            self._mowing_resume_index = 0
            self._mowing_progress_index = 0
            self._active_mowing_area_id = str(selected_area.get("id", ""))
            self._active_mowing_area_name = str(selected_area.get("name", self._active_mowing_area_id))
            self._mowing_nav_goal_pending = True

        self.get_logger().info(
            f"Starting bridge-generated mowing route in area '{selected_area_name}' with {len(plan_poses)} poses"
        )
        self._refresh_ui_state()
        self._send_mowing_navigation_goal(plan_poses, 0)

    def _handle_start_manual_mowing(self) -> None:
        with self._state_lock:
            if not self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot start manual mowing outside area recording mode")
                return
            if self._dock_nearest_goal_handle is not None:
                self.get_logger().warning("Cannot start manual mowing while docking is active")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot start manual mowing without a valid pose")
                return
            if self._charger_present:
                self.get_logger().warning("Cannot start manual mowing while charging")
                return
            if self._robot_mode == ROBOT_MODE_MOWING:
                self.get_logger().info("Manual mowing is already active")
                return

        self._publish_mode_command(ROBOT_MODE_MOWING)

    def _handle_pause_mowing(self) -> None:
        should_cancel_goal = False
        with self._state_lock:
            if self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot pause mowing while area recording mode is active")
                return
            if self._robot_mode != ROBOT_MODE_MOWING or self._mowing_paused:
                self.get_logger().warning("Cannot pause mowing because mowing is not active")
                return
            self._mowing_paused = True
            self._mowing_resume_index = self._estimate_next_resume_index_locked(
                self._last_pose["x"], self._last_pose["y"]
            )
            self._mowing_progress_index = self._mowing_resume_index
            should_cancel_goal = self._mowing_nav_goal_handle is not None

        self._refresh_ui_state()
        if should_cancel_goal:
            self._cancel_mowing_navigation_goal()
        self._publish_mode_command(ROBOT_MODE_IDLE)

    def _handle_stop_manual_mowing(self) -> None:
        with self._state_lock:
            if not self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot stop manual mowing outside area recording mode")
                return
            if self._robot_mode != ROBOT_MODE_MOWING:
                self.get_logger().warning("Cannot stop manual mowing because it is not active")
                return

        self._publish_mode_command(ROBOT_MODE_IDLE)

    def _handle_continue_mowing(self) -> None:
        remaining_plan = []
        area_name = ""
        with self._state_lock:
            if self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot continue mowing while area recording mode is active")
                return
            if not self._mowing_paused:
                self.get_logger().warning("Cannot continue mowing because mowing is not paused")
                return
            if self._charger_present:
                self.get_logger().warning("Cannot continue mowing while charging")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot continue mowing without a valid pose")
                return
            if self._mowing_nav_goal_pending or self._mowing_nav_goal_handle is not None:
                self.get_logger().warning("Cannot continue mowing while a mowing navigation goal is still active")
                return
            if not self._mowing_plan_poses:
                self.get_logger().warning("Cannot continue mowing because no bridge-generated mowing plan is available")
                return
            resume_index = self._estimate_next_resume_index_locked(self._last_pose["x"], self._last_pose["y"])
            self._mowing_resume_index = resume_index
            self._mowing_progress_index = resume_index
            remaining_plan = self._mowing_plan_poses[resume_index:]
            self._mowing_nav_goal_pending = True
            area_name = self._active_mowing_area_name or self._active_mowing_area_id

        if not remaining_plan:
            self.get_logger().info("No mowing waypoints remain, treating continue as completed")
            with self._state_lock:
                self._mowing_paused = False
                self._clear_mowing_execution_locked()
            self._refresh_ui_state()
            self._publish_mode_command(ROBOT_MODE_IDLE)
            return

        if not self._wait_for_action_server(self._navigate_through_poses_client, "navigate_through_poses"):
            with self._state_lock:
                self._mowing_nav_goal_pending = False
            self._refresh_ui_state()
            return

        self.get_logger().info(
            f"Continuing bridge-generated mowing route in area '{area_name}' "
            f"from waypoint {self._mowing_resume_index}"
        )
        self._refresh_ui_state()
        self._send_mowing_navigation_goal(remaining_plan, self._mowing_resume_index)

    def _handle_abort_mowing(self) -> None:
        should_cancel_goal = False
        goal_handle = None
        with self._state_lock:
            if self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot stop mowing while area recording mode is active")
                return
            if self._robot_mode != ROBOT_MODE_MOWING and not self._mowing_paused:
                self.get_logger().warning("Cannot stop mowing because mowing is not active")
                return
            self._mowing_paused = False
            goal_handle = self._mowing_nav_goal_handle
            should_cancel_goal = goal_handle is not None
            self._clear_mowing_execution_locked()

        self._refresh_ui_state()
        if should_cancel_goal and goal_handle is not None:
            goal_handle.cancel_goal_async()
        self._publish_mode_command(ROBOT_MODE_IDLE)

    def _handle_start_area_recording_mode(self) -> None:
        with self._state_lock:
            if self._legacy_area_recording_mode:
                return
            if self._dock_nearest_goal_handle is not None:
                self.get_logger().warning("Cannot enter area recording mode while docking is active")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot enter area recording mode without a valid pose")
                return
            if self._charger_present:
                self.get_logger().warning("Cannot enter area recording mode while charging")
                return

            self._legacy_area_recording_mode = True
            self._mowing_paused = False
            self._area_recording_auto_enabled = True
            self._area_recording_paused = False
            self._area_recording_finalizing = False
            self._area_recording_pending_type = None
            self._area_recording_cancel_reason = None

        self._refresh_ui_state()

    def _handle_exit_recording_mode(self) -> None:
        area_goal_to_cancel = None
        record_docking_goal_to_cancel = None

        with self._state_lock:
            if not self._legacy_area_recording_mode:
                return

            self._legacy_area_recording_mode = False
            self._area_recording_finalizing = False
            self._area_recording_pending_type = None
            self._area_recording_cancel_reason = "exit"
            area_goal_to_cancel = self._area_recording_goal_handle
            record_docking_goal_to_cancel = self._record_docking_goal_handle

        self._refresh_ui_state()

        if area_goal_to_cancel is not None:
            area_goal_to_cancel.cancel_goal_async()

        if record_docking_goal_to_cancel is not None:
            record_docking_goal_to_cancel.cancel_goal_async()

    def _handle_start_or_resume_recording(self) -> None:
        with self._state_lock:
            if not self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot start recording outside area recording mode")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot start recording without a valid pose")
                return
            if self._charger_present:
                self.get_logger().warning("Cannot start recording while charging")
                return
            if self._record_docking_goal_handle is not None:
                self.get_logger().warning("Cannot start area recording while docking station recording is active")
                return

            goal_handle = self._area_recording_goal_handle
            paused = self._area_recording_paused
            auto_enabled = self._area_recording_auto_enabled

        if goal_handle is None:
            if not self._wait_for_action_server(self._record_area_client, "record_area_boundary"):
                return

            goal = RecordAreaBoundary.Goal()
            goal.name = self._next_generated_name("area")
            goal.type = Area.TYPE_OPERATION
            goal.auto_recording = True
            goal.distance_threshold = DEFAULT_AREA_DISTANCE_THRESHOLD

            send_future = self._record_area_client.send_goal_async(
                goal,
                feedback_callback=self._on_area_recording_feedback,
            )
            send_future.add_done_callback(self._on_area_recording_goal_response)
            return

        if not paused:
            self.get_logger().info("Area recording is already active")
            return

        with self._state_lock:
            self._area_recording_paused = False

        self._refresh_ui_state()

        if auto_enabled:
            self._call_set_recording_mode(True)

    def _handle_stop_recording(self) -> None:
        with self._state_lock:
            if self._area_recording_goal_handle is None:
                self.get_logger().warning("Cannot stop recording because no area recording is active")
                return
            if self._area_recording_finalizing:
                self.get_logger().warning("Cannot stop recording while area save is already in progress")
                return
            if self._area_recording_paused:
                return

            auto_enabled = self._area_recording_auto_enabled
            self._area_recording_paused = True

        self._refresh_ui_state()

        if auto_enabled:
            self._call_set_recording_mode(False)

    def _handle_set_auto_point_collecting(self, enabled: bool) -> None:
        with self._state_lock:
            if self._area_recording_goal_handle is None:
                self.get_logger().warning("Cannot toggle point collecting without an active area recording")
                return
            if self._area_recording_finalizing:
                self.get_logger().warning("Cannot toggle point collecting while area save is in progress")
                return

            paused = self._area_recording_paused
            self._area_recording_auto_enabled = enabled

        self._refresh_ui_state()

        if not paused:
            self._call_set_recording_mode(enabled)

    def _handle_collect_point(self) -> None:
        with self._state_lock:
            if self._area_recording_goal_handle is None:
                self.get_logger().warning("Cannot add a boundary point without an active area recording")
                return
            if self._area_recording_finalizing:
                self.get_logger().warning("Cannot add a boundary point while area save is in progress")
                return
            if self._area_recording_paused:
                self.get_logger().warning("Cannot add a boundary point while recording is paused")
                return
            if self._area_recording_auto_enabled:
                self.get_logger().warning("Cannot add a boundary point while automatic collection is enabled")
                return

        if not self._wait_for_service(self._add_boundary_point_client, "add_boundary_point"):
            return

        request = Trigger.Request()
        future = self._add_boundary_point_client.call_async(request)
        future.add_done_callback(self._on_add_boundary_point_response)

    def _handle_finish_area(self, area_type: int) -> None:
        with self._state_lock:
            if self._area_recording_goal_handle is None:
                self.get_logger().warning("Cannot finish area recording because no area recording is active")
                return
            if self._area_recording_finalizing:
                return

            self._area_recording_finalizing = True
            self._area_recording_pending_type = area_type

        self._refresh_ui_state()

        if not self._wait_for_service(self._finish_area_recording_client, "finish_area_recording"):
            with self._state_lock:
                self._area_recording_finalizing = False
                self._area_recording_pending_type = None
            self._refresh_ui_state()
            return

        request = Trigger.Request()
        future = self._finish_area_recording_client.call_async(request)
        future.add_done_callback(self._on_finish_area_recording_response)

    def _handle_finish_discard(self) -> None:
        goal_handle = None

        with self._state_lock:
            if self._area_recording_goal_handle is None:
                self._legacy_area_recording_mode = False
                self._area_recording_finalizing = False
                self._area_recording_pending_type = None
                self._area_recording_cancel_reason = None
            else:
                self._legacy_area_recording_mode = False
                self._area_recording_finalizing = False
                self._area_recording_pending_type = None
                self._area_recording_cancel_reason = "discard"
                goal_handle = self._area_recording_goal_handle

        self._refresh_ui_state()

        if goal_handle is not None:
            goal_handle.cancel_goal_async()

    def _handle_record_docking_station(self) -> None:
        with self._state_lock:
            if not self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot record a docking station outside area recording mode")
                return
            if self._area_recording_goal_handle is not None:
                self.get_logger().warning("Finish or discard the active area recording before recording a dock")
                return
            if self._record_docking_goal_handle is not None:
                self.get_logger().info("Docking station recording is already active")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot record a docking station without a valid pose")
                return

        if not self._wait_for_action_server(self._record_docking_client, "record_docking_station"):
            return

        goal = RecordDockingStation.Goal()
        goal.name = self._next_generated_name("dock")

        send_future = self._record_docking_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_record_docking_goal_response)

    def _handle_dock_robot_nearest(self) -> None:
        with self._state_lock:
            if self._legacy_area_recording_mode:
                self.get_logger().warning("Cannot start docking while the bridge is in area recording mode")
                return
            if self._dock_nearest_goal_handle is not None:
                self.get_logger().info("Docking is already active")
                return
            if not self._have_pose:
                self.get_logger().warning("Cannot dock without a valid pose")
                return
            if self._charger_present:
                self.get_logger().info("Robot is already charging")
                return

        if not self._wait_for_action_server(self._dock_nearest_client, "dock_robot_nearest"):
            return

        goal = DockRobotNearest.Goal()
        send_future = self._dock_nearest_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_dock_robot_nearest_goal_response)

    def _call_set_recording_mode(self, enabled: bool) -> None:
        if not self._wait_for_service(self._set_recording_mode_client, "set_recording_mode"):
            return

        request = SetBool.Request()
        request.data = enabled
        future = self._set_recording_mode_client.call_async(request)
        future.add_done_callback(lambda response_future: self._on_set_recording_mode_response(response_future, enabled))

    def _on_set_recording_mode_response(self, future: Any, enabled: bool) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to call set_recording_mode: {exc}")
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(f"set_recording_mode rejected: {response.message}")

    def _on_add_boundary_point_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to call add_boundary_point: {exc}")
            return

        if response.success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(f"add_boundary_point rejected: {response.message}")

    def _on_finish_area_recording_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to call finish_area_recording: {exc}")
            with self._state_lock:
                self._area_recording_finalizing = False
                self._area_recording_pending_type = None
            self._refresh_ui_state()
            return

        if response.success:
            self.get_logger().info(response.message)
            return

        self.get_logger().warning(f"finish_area_recording rejected: {response.message}")
        with self._state_lock:
            self._area_recording_finalizing = False
            self._area_recording_pending_type = None
        self._refresh_ui_state()

    def _on_area_recording_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to send RecordAreaBoundary goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("RecordAreaBoundary goal was rejected")
            return

        with self._state_lock:
            self._area_recording_goal_handle = goal_handle
            self._area_recording_auto_enabled = True
            self._area_recording_paused = False
            self._area_recording_finalizing = False
            self._area_recording_pending_type = None
            self._area_recording_cancel_reason = None

        self._refresh_ui_state()
        goal_handle.get_result_async().add_done_callback(self._on_area_recording_result)

    def _on_area_recording_feedback(self, _feedback: Any) -> None:
        return

    def _on_area_recording_result(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self.get_logger().warning(f"Failed to receive RecordAreaBoundary result: {exc}")
            with self._state_lock:
                self._area_recording_goal_handle = None
                self._area_recording_finalizing = False
            self._refresh_ui_state()
            return

        desired_type = None
        cancel_reason = None

        with self._state_lock:
            desired_type = self._area_recording_pending_type
            cancel_reason = self._area_recording_cancel_reason

            if cancel_reason in {"discard", "exit"}:
                self._legacy_area_recording_mode = False
            elif desired_type is not None and result.code == RecordAreaBoundary.Result.CODE_SUCCESS:
                self._legacy_area_recording_mode = False

            self._area_recording_goal_handle = None
            self._area_recording_auto_enabled = True
            self._area_recording_paused = False
            self._area_recording_finalizing = False
            self._area_recording_pending_type = None
            self._area_recording_cancel_reason = None

        if result.code == RecordAreaBoundary.Result.CODE_SUCCESS:
            self.get_logger().info(result.message or "Area boundary recording completed successfully")
            if desired_type is not None and desired_type != result.area.type:
                self._save_area_with_updated_type(result.area, desired_type)
        elif result.code == RecordAreaBoundary.Result.CODE_CANCELED:
            self.get_logger().info(result.message or "Area boundary recording was canceled")
        else:
            self.get_logger().warning(f"Area boundary recording failed: {result.message} (code={result.code})")

        self._refresh_ui_state()

    def _save_area_with_updated_type(self, area: Area, desired_type: int) -> None:
        if not self._wait_for_service(self._save_area_client, "save_area"):
            return

        request = SaveArea.Request()
        request.area = area
        request.area.type = desired_type
        future = self._save_area_client.call_async(request)
        future.add_done_callback(self._on_save_area_response)

    def _on_save_area_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to call save_area: {exc}")
            return

        if response.code == SaveArea.Response.CODE_SUCCESS:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warning(f"save_area failed: {response.message} (code={response.code})")

    def _on_mowing_nav_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to send NavigateThroughPoses goal: {exc}")
            with self._state_lock:
                self._mowing_nav_goal_pending = False
                self._mowing_goal_offset = 0
                self._mowing_goal_length = 0
                if not self._mowing_paused:
                    self._clear_mowing_execution_locked()
            self._refresh_ui_state()
            return

        if not goal_handle.accepted:
            self.get_logger().warning("NavigateThroughPoses goal was rejected")
            with self._state_lock:
                self._mowing_nav_goal_pending = False
                self._mowing_goal_offset = 0
                self._mowing_goal_length = 0
                if not self._mowing_paused:
                    self._clear_mowing_execution_locked()
            self._refresh_ui_state()
            return

        with self._state_lock:
            self._mowing_nav_goal_handle = goal_handle
            self._mowing_nav_goal_pending = False
            self._mowing_paused = False

        self._refresh_ui_state()
        self._publish_mode_command(ROBOT_MODE_MOWING)
        goal_handle.get_result_async().add_done_callback(self._on_mowing_nav_result)

    def _on_mowing_nav_feedback(self, feedback_msg: Any) -> None:
        feedback = getattr(feedback_msg, "feedback", feedback_msg)

        with self._state_lock:
            if not self._mowing_plan_poses:
                return

            remaining = getattr(feedback, "number_of_poses_remaining", None)
            goal_offset = self._mowing_goal_offset
            goal_length = self._mowing_goal_length or max(0, len(self._mowing_plan_poses) - goal_offset)
            if remaining is not None:
                next_index = max(
                    goal_offset,
                    min(
                        len(self._mowing_plan_poses),
                        goal_offset + goal_length - int(remaining),
                    ),
                )
            else:
                current_pose = getattr(feedback, "current_pose", None)
                if current_pose is None:
                    return
                next_index = self._estimate_next_resume_index_locked(
                    float(current_pose.pose.position.x),
                    float(current_pose.pose.position.y),
                )

            self._mowing_progress_index = next_index
            if not self._mowing_paused:
                self._mowing_resume_index = next_index

    def _on_mowing_nav_result(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as exc:
            self.get_logger().warning(f"Failed to receive NavigateThroughPoses result: {exc}")
            with self._state_lock:
                self._mowing_nav_goal_handle = None
                self._mowing_nav_goal_pending = False
                should_publish_idle = not self._mowing_paused
                if should_publish_idle:
                    self._clear_mowing_execution_locked()
            self._refresh_ui_state()
            if should_publish_idle:
                self._publish_mode_command(ROBOT_MODE_IDLE)
            return

        error_code = getattr(result, "error_code", 0)
        error_msg = getattr(result, "error_msg", "")

        with self._state_lock:
            self._mowing_nav_goal_handle = None
            self._mowing_nav_goal_pending = False
            self._mowing_goal_offset = 0
            self._mowing_goal_length = 0
            paused = self._mowing_paused

            if status == GoalStatus.STATUS_SUCCEEDED:
                completion = "succeeded"
                self._mowing_progress_index = len(self._mowing_plan_poses)
                self._clear_mowing_execution_locked()
                should_publish_idle = True
            elif status == GoalStatus.STATUS_CANCELED and paused:
                completion = "paused"
                self._mowing_resume_index = max(self._mowing_resume_index, self._mowing_progress_index)
                should_publish_idle = False
            elif status == GoalStatus.STATUS_CANCELED:
                completion = "canceled"
                self._clear_mowing_execution_locked()
                should_publish_idle = True
            else:
                completion = "failed"
                self._clear_mowing_execution_locked()
                should_publish_idle = True

        if completion == "succeeded":
            self.get_logger().info("Bridge-generated mowing route completed successfully")
        elif completion == "paused":
            self.get_logger().info("Bridge-generated mowing route paused")
        elif completion == "canceled":
            self.get_logger().info("Bridge-generated mowing route canceled")
        else:
            self.get_logger().warning(
                f"Bridge-generated mowing route failed with status={status}, error_code={error_code}, error_msg='{error_msg}'"
            )

        self._refresh_ui_state()
        if should_publish_idle:
            self._publish_mode_command(ROBOT_MODE_IDLE)

    def _on_record_docking_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to send RecordDockingStation goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("RecordDockingStation goal was rejected")
            return

        with self._state_lock:
            self._record_docking_goal_handle = goal_handle

        self._refresh_ui_state()
        goal_handle.get_result_async().add_done_callback(self._on_record_docking_result)

    def _on_record_docking_result(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self.get_logger().warning(f"Failed to receive RecordDockingStation result: {exc}")
            with self._state_lock:
                self._record_docking_goal_handle = None
            self._refresh_ui_state()
            return

        with self._state_lock:
            self._record_docking_goal_handle = None

        if result.code == RecordDockingStation.Result.CODE_SUCCESS:
            self.get_logger().info(result.message or "Docking station recording completed successfully")
        elif result.code == RecordDockingStation.Result.CODE_CANCELED:
            self.get_logger().info(result.message or "Docking station recording was canceled")
        else:
            self.get_logger().warning(f"Docking station recording failed: {result.message} (code={result.code})")

        self._refresh_ui_state()

    def _on_dock_robot_nearest_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to send DockRobotNearest goal: {exc}")
            return

        if not goal_handle.accepted:
            self.get_logger().warning("DockRobotNearest goal was rejected")
            return

        with self._state_lock:
            self._dock_nearest_goal_handle = goal_handle

        self._refresh_ui_state()
        goal_handle.get_result_async().add_done_callback(self._on_dock_robot_nearest_result)

    def _on_dock_robot_nearest_result(self, future: Any) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
        except Exception as exc:
            self.get_logger().warning(f"Failed to receive DockRobotNearest result: {exc}")
            with self._state_lock:
                self._dock_nearest_goal_handle = None
            self._refresh_ui_state()
            return

        with self._state_lock:
            self._dock_nearest_goal_handle = None

        if result.code == DockRobotNearest.Result.CODE_SUCCESS:
            self.get_logger().info(result.message or "Docking completed successfully")
        else:
            self.get_logger().warning(f"Docking failed: {result.message} (code={result.code})")

        self._refresh_ui_state()

    def _on_mqtt_message(self, _client: mqtt.Client, _userdata: Any, msg: mqtt.MQTTMessage) -> None:
        topic = msg.topic
        if topic.endswith("teleop"):
            self._handle_teleop_payload(msg.payload)
            return

        if topic.endswith("action"):
            action = msg.payload.decode("utf-8", errors="replace").strip()
            if action:
                self._start_background_task(self._dispatch_legacy_action, action)
            return

        if topic.endswith("command"):
            command = msg.payload.decode("utf-8", errors="replace").strip()
            if command:
                self.get_logger().warning(f"Legacy UI requested unsupported command '{command}'")

    async def websocket_handler(self, websocket: websockets.WebSocketServerProtocol, _path: str = "") -> None:
        peer = getattr(websocket, "remote_address", None)
        self.get_logger().info(f"Legacy joystick socket connected: {peer}")
        try:
            async for message in websocket:
                if isinstance(message, str):
                    message = message.encode("utf-8")
                self._handle_teleop_payload(message)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._publish_teleop(0.0, 0.0)
            self.get_logger().info(f"Legacy joystick socket disconnected: {peer}")

    def shutdown(self) -> None:
        self._publish_teleop(0.0, 0.0)
        self._mqtt_client.loop_stop()
        self._mqtt_client.disconnect()


async def serve_websocket(node: OpenMowerUiBridge) -> None:
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    async with websockets.serve(node.websocket_handler, "0.0.0.0", node._ws_port, ping_interval=None):
        node.get_logger().info(f"Legacy joystick WebSocket listening on 0.0.0.0:{node._ws_port}")
        await stop_event.wait()


def main() -> None:
    rclpy.init()
    node = OpenMowerUiBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def spin_executor() -> None:
        try:
            executor.spin()
        except Exception as exc:
            node.get_logger().error(f"ROS executor stopped: {exc}")
            raise

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(serve_websocket(node))
    finally:
        node.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
