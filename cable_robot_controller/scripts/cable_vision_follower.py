#!/usr/bin/env python3
"""
Camera-only finite state machine for underwater cable inspection in Gazebo.

Mission flow:
  SEARCH -> ALIGN -> DETECTION_CONFIRMATION -> FOLLOW -> FAULT_DETECTION
  -> TURNAROUND -> RETURN -> MISSION_COMPLETE
"""

from __future__ import annotations

import csv
import math
from datetime import datetime
from dataclasses import dataclass
from enum import Enum
from itertools import combinations
from pathlib import Path
from threading import Lock
from typing import Optional

import cv2
import gz.transport13 as gz_transport
import numpy as np
import rclpy
from cable_runtime_utils import (
    build_cumulative_distances,
    integrate_body_twist,
    load_cable_polyline,
    load_robot_initial_pose_3d,
    project_on_polyline,
)
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from gz.msgs10 import boolean_pb2
from gz.msgs10 import pose_v_pb2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def wrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def pressure_from_depth(robot_z: float) -> float:
    depth = abs(robot_z)
    return 101325.0 + (1000.0 * 9.81 * depth)


class MissionState(str, Enum):
    SEARCH = "SEARCH"
    ALIGN = "ALIGN"
    DETECTION_CONFIRMATION = "DETECTION_CONFIRMATION"
    FOLLOW = "FOLLOW"
    FAULT_DETECTION = "FAULT_DETECTION"
    REVERSE = "REVERSE"
    TURNAROUND = "TURNAROUND"
    MISSION_COMPLETE = "MISSION_COMPLETE"


@dataclass
class FaultReport:
    fault_type: str
    image_point: tuple[int, int]
    details: str = ""


@dataclass
class FaultInfo:
    type: str
    x: float
    y: float
    z: float
    depth: float


@dataclass
class CableCandidate:
    contour: np.ndarray
    points: np.ndarray
    mask: np.ndarray
    area_px: float
    aspect_ratio: float
    centroid: tuple[int, int]
    near_point: tuple[int, int]
    control_point: tuple[int, int]
    center_error_norm: float
    heading_error_norm: float
    run_span_px: float
    vertical_span_norm: float
    track_points: list[tuple[int, int]]
    score: float
    axis: np.ndarray


@dataclass
class VisionResult:
    cable_visible: bool = False
    candidate: Optional[CableCandidate] = None
    debug_frame: Optional[np.ndarray] = None
    corrosion_fault: Optional[FaultReport] = None
    break_fault: Optional[FaultReport] = None


@dataclass
class LoggedFault:
    time: str
    robot_x: float
    robot_y: float
    robot_z: float
    pressure: float
    fault_type: str
    cable_distance: float
    fault_x: float
    fault_y: float
    repair_status: str = ""


class CableVisionFollower(Node):
    def __init__(self) -> None:
        super().__init__("cable_vision_follower")

        self.declare_parameter("camera_topic", "/cable_repair_robot/camera")
        self.declare_parameter("cmd_topic", "/model/cable_repair_robot/cmd_vel")
        self.declare_parameter("control_rate_hz", 15.0)
        self.declare_parameter("search_forward_speed", 0.28)
        self.declare_parameter("search_rotation_speed", 0.10)
        self.declare_parameter("follow_speed", 0.35)
        self.declare_parameter("align_forward_speed", 0.08)
        self.declare_parameter("align_forward_error_limit", 0.35)
        self.declare_parameter("align_kp", 1.4)
        self.declare_parameter("follow_kp", 0.7)
        self.declare_parameter("follow_heading_kp", 0.14)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("center_threshold", 0.08)
        self.declare_parameter("realign_threshold", 0.16)
        self.declare_parameter("detection_frames_required", 5)
        self.declare_parameter("aligned_frames_required", 3)
        self.declare_parameter("confirmation_stop_sec", 2.0)
        self.declare_parameter("lost_cable_timeout_sec", 1.5)
        self.declare_parameter("fault_frames_required", 2)
        self.declare_parameter("fault_report_cooldown_sec", 3.0)
        self.declare_parameter("fault_detection_min_follow_sec", 1.5)
        self.declare_parameter("fault_detection_min_travel_m", 0.75)
        # Positive linear.x should move the robot toward the camera-facing front.
        self.declare_parameter("drive_direction", 1.0)
        self.declare_parameter("slide_direction", -1.0)
        self.declare_parameter("steering_direction", -1.0)
        self.declare_parameter("show_debug_window", True)
        self.declare_parameter("min_contour_area_px", 500.0)
        self.declare_parameter("min_aspect_ratio", 2.4)
        self.declare_parameter("roi_start_fraction", 0.30)
        self.declare_parameter("search_entry_max_center_error", 0.55)
        self.declare_parameter("ridge_threshold", 18.0)
        self.declare_parameter("max_heading_error_norm", 0.35)
        self.declare_parameter("side_view_min_contour_area_px", 120.0)
        self.declare_parameter("side_view_min_span_fraction", 0.12)
        self.declare_parameter("source_sdf", "/home/kousik/ocean_ecosystem_full.sdf")
        self.declare_parameter("robot_model_name", "cable_repair_robot")
        self.declare_parameter("inspection_log_file", "/home/kousik/inspection_log.csv")
        self.declare_parameter("station_reach_distance_m", 2.0)
        self.declare_parameter("fault_dedup_distance_m", 1.5)
        self.declare_parameter("turnaround_yaw_tolerance_deg", 10.0)
        self.declare_parameter("return_reverse_speed", 0.15)
        self.declare_parameter("return_reverse_duration_sec", 1.25)
        self.declare_parameter("world_name", "ocean_world")
        self.declare_parameter("stop_marker_name", "stop_marker")
        self.declare_parameter("stop_marker_reach_distance_m", 1.0)
        self.declare_parameter(
            "inspection_complete_topic",
            "/model/cable_repair_robot/inspection_complete",
        )

        self.camera_topic = str(self.get_parameter("camera_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.search_forward_speed = float(self.get_parameter("search_forward_speed").value)
        self.search_rotation_speed = float(self.get_parameter("search_rotation_speed").value)
        self.follow_speed = float(self.get_parameter("follow_speed").value)
        self.align_forward_speed = float(
            self.get_parameter("align_forward_speed").value
        )
        self.align_forward_error_limit = float(
            self.get_parameter("align_forward_error_limit").value
        )
        self.align_kp = float(self.get_parameter("align_kp").value)
        self.follow_kp = float(self.get_parameter("follow_kp").value)
        self.follow_heading_kp = float(self.get_parameter("follow_heading_kp").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.center_threshold = float(self.get_parameter("center_threshold").value)
        self.realign_threshold = float(self.get_parameter("realign_threshold").value)
        self.detection_frames_required = int(
            self.get_parameter("detection_frames_required").value
        )
        self.aligned_frames_required = int(
            self.get_parameter("aligned_frames_required").value
        )
        self.confirmation_stop_sec = float(
            self.get_parameter("confirmation_stop_sec").value
        )
        self.lost_cable_timeout_sec = float(
            self.get_parameter("lost_cable_timeout_sec").value
        )
        self.fault_frames_required = int(self.get_parameter("fault_frames_required").value)
        self.fault_report_cooldown_sec = float(
            self.get_parameter("fault_report_cooldown_sec").value
        )
        self.fault_detection_min_follow_sec = float(
            self.get_parameter("fault_detection_min_follow_sec").value
        )
        self.fault_detection_min_travel_m = float(
            self.get_parameter("fault_detection_min_travel_m").value
        )
        self.drive_direction = float(self.get_parameter("drive_direction").value)
        self.slide_direction = float(self.get_parameter("slide_direction").value)
        self.steering_direction = float(self.get_parameter("steering_direction").value)
        self.show_debug_window = bool(self.get_parameter("show_debug_window").value)
        self.min_contour_area_px = float(self.get_parameter("min_contour_area_px").value)
        self.min_aspect_ratio = float(self.get_parameter("min_aspect_ratio").value)
        self.roi_start_fraction = float(self.get_parameter("roi_start_fraction").value)
        self.search_entry_max_center_error = float(
            self.get_parameter("search_entry_max_center_error").value
        )
        self.ridge_threshold = float(self.get_parameter("ridge_threshold").value)
        self.max_heading_error_norm = float(
            self.get_parameter("max_heading_error_norm").value
        )
        self.side_view_min_contour_area_px = float(
            self.get_parameter("side_view_min_contour_area_px").value
        )
        self.side_view_min_span_fraction = float(
            self.get_parameter("side_view_min_span_fraction").value
        )
        self.source_sdf = Path(str(self.get_parameter("source_sdf").value))
        self.robot_model_name = str(self.get_parameter("robot_model_name").value)
        self.inspection_log_file = Path(
            str(self.get_parameter("inspection_log_file").value)
        )
        self.station_reach_distance_m = float(
            self.get_parameter("station_reach_distance_m").value
        )
        self.fault_dedup_distance_m = float(
            self.get_parameter("fault_dedup_distance_m").value
        )
        self.turnaround_yaw_tolerance_rad = math.radians(
            float(self.get_parameter("turnaround_yaw_tolerance_deg").value)
        )
        self.return_reverse_speed = float(
            self.get_parameter("return_reverse_speed").value
        )
        self.return_reverse_duration_sec = float(
            self.get_parameter("return_reverse_duration_sec").value
        )
        self.world_name = str(self.get_parameter("world_name").value)
        self.stop_marker_name = str(self.get_parameter("stop_marker_name").value)
        self.stop_marker_reach_distance_m = float(
            self.get_parameter("stop_marker_reach_distance_m").value
        )
        self.inspection_complete_topic = str(
            self.get_parameter("inspection_complete_topic").value
        )

        self.dark_lower_hsv = np.array([0, 0, 0], dtype=np.uint8)
        self.dark_upper_hsv = np.array([180, 255, 115], dtype=np.uint8)
        self.brown_lower_hsv = np.array([5, 90, 40], dtype=np.uint8)
        self.brown_upper_hsv = np.array([25, 255, 210], dtype=np.uint8)

        self.bridge = CvBridge()
        self.latest_frame: Optional[np.ndarray] = None
        self.last_image_time_sec = -1e9
        self.last_detection_time_sec = -1e9
        self.last_fault_report_time_sec = -1e9
        self.last_status_log_time_sec = -1e9
        self.confirmation_deadline_sec = 0.0
        self.debug_window_available = self.show_debug_window
        self.image_count = 0

        self.state = MissionState.SEARCH
        self.detect_streak = 0
        self.aligned_streak = 0
        self.corrosion_streak = 0
        self.break_streak = 0
        self.corrosionCount = 0
        self.cableLostCounter = 0
        self.wasFollowingCable = False
        self.breakDetectedDuringCurrentLoss = False
        self.pending_fault: Optional[FaultReport] = None
        self.last_turn_direction = 1.0
        init_x, init_y, init_z, init_yaw = load_robot_initial_pose_3d(
            self.source_sdf,
            self.robot_model_name,
            -33.8,
            2.67,
            1.10,
            1.22,
        )
        self.segment_points = load_cable_polyline(self.source_sdf)
        self.segment_cumulative_dist = build_cumulative_distances(self.segment_points)
        self.total_cable_distance = (
            self.segment_cumulative_dist[-1] if self.segment_cumulative_dist else 0.0
        )
        self.robot_x_est = init_x
        self.robot_y_est = init_y
        self.robot_z_est = init_z
        self.robot_yaw_est = init_yaw
        self.last_motion_update_sec = self.now_sec()
        self.pose_last_update_sec = -1e9
        self.pose_lock = Lock()
        self.follow_state_enter_sec = -1e9
        self.travel_since_start_m = 0.0
        self.last_cmd_sent = Twist()
        self.outbound_complete = False
        self.return_trip_started = False
        self.mission_complete = False
        self.inspectionComplete = False
        self.inspectionReportPrinted = False
        self.stopOnlyOnSecondCorrosion = True
        self.return_reverse_until_sec = -1e9
        self.turnaround_target_yaw: Optional[float] = None
        self.detectedFaults: list[FaultInfo] = []
        self.logged_faults: list[LoggedFault] = []
        stop_marker_x, stop_marker_y, stop_marker_z, _ = load_robot_initial_pose_3d(
            self.source_sdf,
            self.stop_marker_name,
            38.7970,
            -0.1226,
            3.0,
            0.0,
        )
        self.stop_marker_x = stop_marker_x
        self.stop_marker_y = stop_marker_y
        self.stop_marker_z = stop_marker_z
        self.fault_log_fieldnames = [
            "time",
            "robot_x",
            "robot_y",
            "robot_z",
            "pressure",
            "fault_type",
            "cable_distance",
            "repair_status",
        ]
        self.pose_transport = gz_transport.Node()
        self.control_transport = gz_transport.Node()
        self.inspection_complete_pub = None
        self.pose_topics = [
            f"/world/{self.world_name}/dynamic_pose/info",
            f"/world/{self.world_name}/pose/info",
        ]
        for pose_topic in self.pose_topics:
            try:
                self.pose_transport.subscribe(
                    pose_v_pb2.Pose_V,
                    pose_topic,
                    self.on_pose_update,
                )
            except Exception as exc:
                self.get_logger().warn(f"Pose subscription failed for {pose_topic}: {exc}")

        try:
            self.inspection_complete_pub = self.control_transport.advertise(
                self.inspection_complete_topic,
                boolean_pb2.Boolean,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Inspection-complete publisher setup failed: {exc}"
            )

        self._initialize_fault_log()

        self.create_subscription(
            Image, self.camera_topic, self.on_image, qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.control_step)

        self.get_logger().info(
            f"Started cable_vision_follower | camera={self.camera_topic} cmd={self.cmd_topic}"
        )
        self.publish_inspection_complete_flag(False)
        self.log_state(self.state)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_image(self, msg: Image) -> None:
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_image_time_sec = self.now_sec()
            self.image_count += 1
            if self.image_count == 1:
                self.get_logger().info(f"Received first image on {self.camera_topic}")
        except Exception as exc:
            self.get_logger().warn(f"Image conversion failed: {exc}")

    def integrate_motion_estimate(self, now_sec: float) -> None:
        dt = clamp(now_sec - self.last_motion_update_sec, 0.0, 0.25)
        self.last_motion_update_sec = now_sec
        if dt <= 0.0:
            return

        prev_x = self.robot_x_est
        prev_y = self.robot_y_est
        self.robot_x_est, self.robot_y_est, self.robot_yaw_est = integrate_body_twist(
            self.robot_x_est,
            self.robot_y_est,
            self.robot_yaw_est,
            float(self.last_cmd_sent.linear.x),
            float(self.last_cmd_sent.linear.y),
            float(self.last_cmd_sent.angular.z),
            dt,
            linear_x_scale=1.0,
            linear_y_scale=1.0,
        )
        self.travel_since_start_m += float(
            np.hypot(self.robot_x_est - prev_x, self.robot_y_est - prev_y)
        )

    def _initialize_fault_log(self) -> None:
        self.inspection_log_file.parent.mkdir(parents=True, exist_ok=True)
        with self.inspection_log_file.open("w", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=self.fault_log_fieldnames)
            writer.writeheader()

    def on_pose_update(self, msg: pose_v_pb2.Pose_V) -> None:
        preferred_names = (
            self.robot_model_name,
            f"{self.robot_model_name}::base_link",
        )
        for target_name in preferred_names:
            for pose in msg.pose:
                if pose.name != target_name:
                    continue
                yaw = quaternion_to_yaw(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                with self.pose_lock:
                    self.robot_x_est = float(pose.position.x)
                    self.robot_y_est = float(pose.position.y)
                    self.robot_z_est = float(pose.position.z)
                    self.robot_yaw_est = yaw
                    self.pose_last_update_sec = self.now_sec()
                return

    def current_pose(self) -> tuple[float, float, float, float]:
        with self.pose_lock:
            return (
                self.robot_x_est,
                self.robot_y_est,
                self.robot_z_est,
                self.robot_yaw_est,
            )

    def current_projection(self):
        robot_x, robot_y, _, _ = self.current_pose()
        return project_on_polyline(
            robot_x,
            robot_y,
            self.segment_points,
            self.segment_cumulative_dist,
        )

    def emit_terminal_message(self, message: str) -> None:
        print(message, flush=True)

    def emit_fault_report(self, logged_fault: LoggedFault) -> None:
        self.emit_terminal_message("FAULT DETECTED")
        self.emit_terminal_message(f"Type: {logged_fault.fault_type}")
        self.emit_terminal_message(
            f"Coordinates: ({logged_fault.fault_x:.2f}, {logged_fault.fault_y:.2f})"
        )
        self.emit_terminal_message(f"Depth: {logged_fault.robot_z:.2f} meters")
        self.emit_terminal_message(f"Pressure: {logged_fault.pressure:.0f} Pascals")
        self.get_logger().info(
            (
                "FAULT_EVENT"
                f" type={logged_fault.fault_type}"
                f" world=({logged_fault.fault_x:.2f},{logged_fault.fault_y:.2f})"
                f" robot_z={logged_fault.robot_z:.2f}"
                f" pressure={logged_fault.pressure:.0f}"
            )
        )

    def append_fault_log_row(self, logged_fault: LoggedFault) -> None:
        row = {
            "time": logged_fault.time,
            "robot_x": f"{logged_fault.robot_x:.3f}",
            "robot_y": f"{logged_fault.robot_y:.3f}",
            "robot_z": f"{logged_fault.robot_z:.3f}",
            "pressure": f"{logged_fault.pressure:.3f}",
            "fault_type": logged_fault.fault_type,
            "cable_distance": f"{logged_fault.cable_distance:.3f}",
            "repair_status": logged_fault.repair_status,
        }
        with self.inspection_log_file.open("a", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=self.fault_log_fieldnames)
            writer.writerow(row)

    def publish_inspection_complete_flag(self, completed: bool) -> None:
        if self.inspection_complete_pub is None:
            return
        try:
            message = boolean_pb2.Boolean()
            message.data = completed
            self.inspection_complete_pub.publish_raw(
                message.SerializeToString(),
                "gz.msgs.Boolean",
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Inspection-complete publish failed: {exc}"
            )

    def fault_already_logged(self, fault_type: str, cable_distance: float) -> bool:
        for logged_fault in self.logged_faults:
            if logged_fault.fault_type != fault_type:
                continue
            if abs(logged_fault.cable_distance - cable_distance) <= self.fault_dedup_distance_m:
                return True
        return False

    def normalize_fault_type(self, fault_type: str) -> str:
        return fault_type.strip().replace(" ", "_").upper()

    def record_fault(self, fault: FaultReport) -> bool:
        robot_x, robot_y, robot_z, _ = self.current_pose()
        projection = self.current_projection()
        normalized_fault_type = self.normalize_fault_type(fault.fault_type)
        if self.fault_already_logged(normalized_fault_type, projection.distance_m):
            return False

        fault_info = FaultInfo(
            type=normalized_fault_type,
            x=robot_x,
            y=robot_y,
            z=robot_z,
            depth=robot_z,
        )
        self.detectedFaults.append(fault_info)

        logged_fault = LoggedFault(
            time=datetime.now().isoformat(timespec="seconds"),
            robot_x=robot_x,
            robot_y=robot_y,
            robot_z=robot_z,
            pressure=pressure_from_depth(robot_z),
            fault_type=normalized_fault_type,
            cable_distance=projection.distance_m,
            fault_x=fault_info.x,
            fault_y=fault_info.y,
            repair_status="",
        )
        self.last_fault_report_time_sec = self.now_sec()
        self.logged_faults.append(logged_fault)
        self.append_fault_log_row(logged_fault)
        return True

    def detect_break_from_cable_loss(
        self, cable_detected: bool
    ) -> Optional[FaultReport]:
        if cable_detected:
            self.wasFollowingCable = True

        if self.state not in {MissionState.FOLLOW, MissionState.FAULT_DETECTION}:
            self.cableLostCounter = 0
            self.breakDetectedDuringCurrentLoss = False
            return None

        if cable_detected:
            self.cableLostCounter = 0
            self.breakDetectedDuringCurrentLoss = False
            return None

        self.cableLostCounter += 1
        if (
            self.cableLostCounter > 10
            and self.wasFollowingCable
            and not self.breakDetectedDuringCurrentLoss
        ):
            self.breakDetectedDuringCurrentLoss = True
            point = (0, 0)
            if self.latest_frame is not None:
                height, width = self.latest_frame.shape[:2]
                point = (width // 2, height // 2)
            return FaultReport("CABLE_BREAK", point, "cable_loss")

        return None

    def begin_turnaround(self) -> None:
        self.outbound_complete = True
        self.emit_terminal_message("STATION B REACHED")
        self.emit_terminal_message("Inspection half completed")
        self.get_logger().info("MISSION_EVENT reached_station_b")
        self.pending_fault = None
        self.detect_streak = 0
        self.aligned_streak = 0
        self.return_reverse_until_sec = self.now_sec() + max(0.1, self.return_reverse_duration_sec)
        self.set_state(MissionState.REVERSE)

    def turnaround_complete(self) -> bool:
        if self.turnaround_target_yaw is None:
            return False
        _, _, _, robot_yaw = self.current_pose()
        return (
            abs(wrap_angle(self.turnaround_target_yaw - robot_yaw))
            <= self.turnaround_yaw_tolerance_rad
        )

    def complete_turnaround(self) -> None:
        self.turnaround_target_yaw = None
        self.return_reverse_until_sec = -1e9
        self.return_trip_started = True
        self.last_detection_time_sec = -1e9
        self.set_state(MissionState.SEARCH)

    def emit_final_report(self) -> None:
        if self.inspectionReportPrinted:
            return

        self.emit_terminal_message("====================================")
        self.emit_terminal_message("CABLE INSPECTION REPORT")
        self.emit_terminal_message("====================================")
        self.emit_terminal_message("")
        for index, fault in enumerate(self.detectedFaults, start=1):
            self.emit_terminal_message(f"Fault {index}")
            self.emit_terminal_message(f"Type: {fault.type}")
            self.emit_terminal_message(
                f"Coordinates: ({fault.x:.3f}, {fault.y:.3f}, {fault.z:.3f})"
            )
            self.emit_terminal_message(f"Depth: {fault.depth:.3f} meters")
            self.emit_terminal_message("")
        self.emit_terminal_message("------------------------------------")
        self.emit_terminal_message(
            f"Total Faults Detected: {len(self.detectedFaults)}"
        )
        self.emit_terminal_message("Inspection Status: COMPLETED")
        self.emit_terminal_message("Robot stopped at marker location")
        self.emit_terminal_message("====================================")
        self.inspectionReportPrinted = True
        self.get_logger().info(
            f"MISSION_EVENT complete total_faults={len(self.detectedFaults)}"
        )

    def finish_inspection_at_marker(self) -> None:
        if self.inspectionComplete:
            return

        self.inspectionComplete = True
        stop_cmd = self.make_cmd(0.0, 0.0)
        self.cmd_pub.publish(stop_cmd)
        self.last_cmd_sent = stop_cmd
        self.publish_inspection_complete_flag(True)
        self.mission_complete = True
        self.pending_fault = None
        self.set_state(MissionState.MISSION_COMPLETE)
        self.emit_final_report()

    def stop_marker_distance(self) -> float:
        robot_x, robot_y, robot_z, _ = self.current_pose()
        return math.sqrt(
            ((robot_x - self.stop_marker_x) ** 2)
            + ((robot_y - self.stop_marker_y) ** 2)
            + ((robot_z - self.stop_marker_z) ** 2)
        )

    def reached_stop_marker(self) -> bool:
        return self.stop_marker_distance() < self.stop_marker_reach_distance_m

    def log_state(self, state: MissionState) -> None:
        message = f"STATE: {state.value}"
        self.get_logger().info(message)
        print(message, flush=True)

    def set_state(self, new_state: MissionState) -> None:
        if self.state == new_state:
            return
        self.state = new_state
        if new_state == MissionState.FOLLOW:
            self.follow_state_enter_sec = self.now_sec()
        elif new_state in {
            MissionState.SEARCH,
            MissionState.ALIGN,
            MissionState.DETECTION_CONFIRMATION,
            MissionState.REVERSE,
            MissionState.TURNAROUND,
            MissionState.MISSION_COMPLETE,
        }:
            self.follow_state_enter_sec = -1e9
        self.log_state(new_state)

    def fault_detection_armed(self, now_sec: float) -> bool:
        if self.state not in {MissionState.FOLLOW, MissionState.FAULT_DETECTION}:
            return False
        if self.follow_state_enter_sec <= -1e8:
            return False
        if now_sec - self.follow_state_enter_sec < self.fault_detection_min_follow_sec:
            return False
        if self.travel_since_start_m < self.fault_detection_min_travel_m:
            return False
        return True

    def make_cmd(self, linear_x: float, angular_z: float, linear_y: float = 0.0) -> Twist:
        cmd = Twist()
        max_linear = max(
            abs(self.search_forward_speed),
            abs(self.follow_speed),
            abs(self.return_reverse_speed),
        )
        cmd.linear.x = float(
            clamp(self.drive_direction * linear_x, -max_linear, max_linear)
        )
        cmd.linear.y = float(
            clamp(self.slide_direction * linear_y, -max_linear, max_linear)
        )
        cmd.angular.z = float(clamp(angular_z, -self.max_angular_speed, self.max_angular_speed))
        return cmd

    def search_cmd(self, center_error: Optional[float] = None) -> Twist:
        if center_error is None:
            linear = self.search_forward_speed
            angular = self.search_rotation_speed
        else:
            error_mag = abs(center_error)
            turn_gain = 1.8 if error_mag > 0.55 else 1.2
            guided_turn = self.steering_direction * clamp(
                turn_gain * center_error, -1.0, 1.0
            ) * self.max_angular_speed
            # Keep SEARCH as combined linear+angular motion, but slow down when the
            # cable sits near the edge so the robot can curve back onto it.
            linear_scale = 0.35 if error_mag > 0.70 else 0.65 if error_mag > 0.40 else 1.0
            linear = self.search_forward_speed * linear_scale
            if error_mag < 0.05:
                angular = 0.4 * guided_turn
            else:
                angular = guided_turn
        return self.make_cmd(linear, angular)

    def align_cmd(self, center_error: float) -> Twist:
        angular = self.steering_direction * self.align_kp * center_error
        error_mag = abs(center_error)
        if self.align_forward_error_limit <= 1e-6 or error_mag >= self.align_forward_error_limit:
            return self.make_cmd(0.0, angular)

        # Keep alignment mostly rotational, but add a small forward creep when the
        # cable is already reasonably centered so the robot does not appear stuck.
        linear_scale = 1.0 - (error_mag / self.align_forward_error_limit)
        return self.make_cmd(self.align_forward_speed * linear_scale, angular)

    def follow_cmd(self, center_error: float, heading_error: float) -> Twist:
        steer = (self.follow_kp * center_error) + (
            0.6 * self.follow_heading_kp * heading_error
        )
        angular = self.steering_direction * steer
        return self.make_cmd(self.follow_speed, angular)

    def short_reacquire_cmd(self) -> Twist:
        return self.make_cmd(
            0.35 * self.follow_speed,
            self.steering_direction * 0.18 * self.last_turn_direction,
        )

    def compute_track_points(
        self, mask: np.ndarray
    ) -> Optional[tuple[list[tuple[int, int]], list[int], int]]:
        height, width = mask.shape[:2]
        band_half_height = max(5, int(0.018 * height))
        band_centers = [int(r * height) for r in (0.92, 0.82, 0.72, 0.62, 0.52, 0.42)]

        points: list[tuple[int, int]] = []
        spans: list[int] = []
        prev_x = width // 2

        for band_idx, band_center_y in enumerate(band_centers):
            y0 = max(0, band_center_y - band_half_height)
            y1 = min(height, band_center_y + band_half_height + 1)
            band = mask[y0:y1, :]
            cols_filled = np.any(band > 0, axis=0).astype(np.uint8)

            runs: list[tuple[int, int]] = []
            run_start: Optional[int] = None
            for idx, filled in enumerate(cols_filled):
                if filled and run_start is None:
                    run_start = idx
                elif not filled and run_start is not None:
                    runs.append((run_start, idx - 1))
                    run_start = None
            if run_start is not None:
                runs.append((run_start, len(cols_filled) - 1))

            valid_runs = []
            for run_x0, run_x1 in runs:
                span_px = run_x1 - run_x0 + 1
                # Use a tighter width gate near the bottom of the frame to
                # reject self-occlusion / robot-body blobs while still allowing
                # wider cable projections farther away.
                max_run_fraction = 0.22 + (0.05 * band_idx)
                max_run_span_px = max(36, int(max_run_fraction * width))
                if 2 <= span_px <= max_run_span_px:
                    valid_runs.append((run_x0, run_x1, span_px))

            if not valid_runs:
                continue

            ref_x = (width // 2) if band_idx == 0 else prev_x
            best_run = min(
                valid_runs,
                key=lambda run: (
                    abs(0.5 * (run[0] + run[1]) - ref_x),
                    run[2],
                    -band_center_y,
                ),
            )
            run_center_x = int(0.5 * (best_run[0] + best_run[1]))
            points.append((run_center_x, band_center_y))
            spans.append(best_run[2])
            prev_x = run_center_x

        if len(points) < 3:
            return None

        return points, spans, band_half_height

    def build_candidate(
        self, contour: np.ndarray, image_shape: tuple[int, int]
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        area_px = float(cv2.contourArea(contour))
        if area_px < self.min_contour_area_px:
            return None

        rect = cv2.minAreaRect(contour)
        rect_w, rect_h = rect[1]
        major = max(rect_w, rect_h)
        minor = max(1.0, min(rect_w, rect_h))
        aspect_ratio = major / minor
        if aspect_ratio < self.min_aspect_ratio or major < 40.0:
            return None
        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(contour)
        bbox_area = float(max(1, bbox_w * bbox_h))
        fill_ratio = area_px / bbox_area
        if fill_ratio > 0.42 and (bbox_w > int(0.16 * width) or bbox_h > int(0.20 * height)):
            return None

        raw_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(raw_mask, [contour], -1, 255, thickness=cv2.FILLED)
        ys, xs = np.where(raw_mask > 0)
        if xs.size < 50:
            return None

        bottom_band = raw_mask[max(0, height - max(24, height // 10)) :, :]
        bottom_xs = np.where(np.any(bottom_band > 0, axis=0))[0]
        if bottom_xs.size > 0:
            bottom_span_norm = float(bottom_xs[-1] - bottom_xs[0] + 1) / max(float(width), 1.0)
            if bottom_span_norm > 0.26 and fill_ratio > 0.24:
                return None

        y_span = float(np.max(ys) - np.min(ys))
        x_span = float(np.max(xs) - np.min(xs))
        side_view_candidate = self.build_side_view_candidate(
            contour,
            raw_mask,
            (height, width),
            area_px,
            aspect_ratio,
            x_span,
            y_span,
            float(np.max(ys)) / max(float(height - 1), 1.0),
        )
        if side_view_candidate is not None:
            return side_view_candidate

        if max(x_span / max(width, 1), y_span / max(height, 1)) < 0.12:
            return None
        vertical_span_norm = y_span / max(float(height), 1.0)
        if vertical_span_norm < 0.12:
            return None
        bottom_bias = float(np.max(ys)) / max(float(height - 1), 1.0)
        if bottom_bias < 0.70:
            return None

        track_result = self.compute_track_points(raw_mask)
        if track_result is None:
            return None

        track_points, spans, band_half_height = track_result
        median_span_px = float(np.median(spans))
        corridor_thickness = int(
            clamp(median_span_px * 1.95, 14.0, max(22.0, 0.20 * width))
        )
        corridor_mask = np.zeros((height, width), dtype=np.uint8)
        if len(track_points) >= 2:
            cv2.polylines(
                corridor_mask,
                [np.array(track_points, dtype=np.int32)],
                False,
                255,
                thickness=corridor_thickness,
            )
        for point in track_points:
            cv2.circle(corridor_mask, point, max(3, corridor_thickness // 2), 255, -1)

        mask = cv2.bitwise_and(raw_mask, corridor_mask)
        # Expand the selected cable body inside the track corridor so a
        # one-sided dark seed still grows back to the full cable width.
        body_fill_radius = int(clamp(0.55 * median_span_px, 4.0, 18.0))
        body_fill_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (2 * body_fill_radius + 1, 2 * body_fill_radius + 1),
        )
        mask = cv2.bitwise_and(corridor_mask, cv2.dilate(mask, body_fill_kernel, iterations=1))
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
        )
        ys, xs = np.where(mask > 0)
        if xs.size < 60:
            return None

        pruned_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not pruned_contours:
            return None
        contour = max(pruned_contours, key=cv2.contourArea)
        area_px = float(cv2.contourArea(contour))
        if area_px < self.min_contour_area_px * 0.45:
            return None

        moments = cv2.moments(mask, binaryImage=True)
        if moments["m00"] <= 1e-6:
            return None
        centroid = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )

        near_point = track_points[0]
        upper_points = track_points[min(2, len(track_points) - 1) :]
        lookahead_x = int(np.mean([pt[0] for pt in upper_points]))
        lookahead_y = int(np.mean([pt[1] for pt in upper_points]))
        control_point = (lookahead_x, lookahead_y)
        center_error_norm = (control_point[0] - (width / 2.0)) / max(width / 2.0, 1.0)
        center_error_norm = clamp(float(center_error_norm), -1.0, 1.0)
        path_dx = float(control_point[0] - near_point[0])
        path_dy = float(max(1, near_point[1] - control_point[1]))
        heading_angle = np.arctan2(path_dx, path_dy)
        heading_error_norm = clamp(
            float(heading_angle / np.radians(45.0)),
            -self.max_heading_error_norm,
            self.max_heading_error_norm,
        )

        all_points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        run_center_x = near_point[0]
        ref_row = near_point[1]
        run_span_px = spans[0]
        run_x0 = max(0, int(run_center_x - 0.5 * run_span_px))
        run_x1 = min(width - 1, int(run_center_x + 0.5 * run_span_px))
        local_sel = (
            (ys >= max(0, ref_row - band_half_height))
            & (ys <= min(height - 1, ref_row + band_half_height))
            & (xs >= max(0, run_x0 - 10))
            & (xs <= min(width - 1, run_x1 + 10))
        )
        local_points = all_points[local_sel]
        if local_points.shape[0] < 12:
            return None

        points = local_points
        [vx], [vy], _, _ = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
        axis = np.array([float(vx), float(vy)], dtype=np.float32)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-6:
            return None
        axis /= axis_norm

        center_bias = 1.0 - min(
            1.0, abs(control_point[0] - (width / 2.0)) / max(width / 2.0, 1.0)
        )
        track_coverage = float(len(track_points)) / 6.0
        width_score = 1.0 - clamp(run_span_px / max(1.0, 0.35 * width), 0.0, 1.0)
        score = (
            area_px
            * aspect_ratio
            * (0.45 + 0.55 * bottom_bias)
            * (0.40 + 0.60 * track_coverage)
            * (0.55 + 0.45 * center_bias)
            * (0.4 + 0.6 * width_score)
        )

        return CableCandidate(
            contour=contour,
            points=all_points,
            mask=mask,
            area_px=area_px,
            aspect_ratio=aspect_ratio,
            centroid=centroid,
            near_point=near_point,
            control_point=control_point,
            center_error_norm=center_error_norm,
            heading_error_norm=heading_error_norm,
            run_span_px=float(run_span_px),
            vertical_span_norm=vertical_span_norm,
            track_points=track_points,
            score=score,
            axis=axis,
        )

    def build_break_fragment_candidate(
        self, contour: np.ndarray, image_shape: tuple[int, int]
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        area_px = float(cv2.contourArea(contour))
        if area_px < self.side_view_min_contour_area_px:
            return None

        rect = cv2.minAreaRect(contour)
        rect_w, rect_h = rect[1]
        major = max(rect_w, rect_h)
        minor = max(1.0, min(rect_w, rect_h))
        aspect_ratio = major / minor
        if aspect_ratio < max(1.6, 0.70 * self.min_aspect_ratio) or major < 24.0:
            return None

        bbox_x, bbox_y, bbox_w, bbox_h = cv2.boundingRect(contour)
        bbox_area = float(max(1, bbox_w * bbox_h))
        fill_ratio = area_px / bbox_area
        if fill_ratio > 0.72:
            return None

        raw_mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(raw_mask, [contour], -1, 255, thickness=cv2.FILLED)
        ys, xs = np.where(raw_mask > 0)
        if xs.size < 50:
            return None

        bottom_band = raw_mask[max(0, height - max(24, height // 10)) :, :]
        bottom_xs = np.where(np.any(bottom_band > 0, axis=0))[0]
        if bottom_xs.size > 0:
            bottom_span_norm = float(bottom_xs[-1] - bottom_xs[0] + 1) / max(float(width), 1.0)
            if bottom_span_norm > 0.45:
                return None

        x_span = float(np.max(xs) - np.min(xs))
        y_span = float(np.max(ys) - np.min(ys))
        bottom_bias = float(np.max(ys)) / max(float(height - 1), 1.0)
        relaxed_candidate = self.build_relaxed_side_view_candidate(
            contour,
            raw_mask,
            (height, width),
            area_px,
            aspect_ratio,
            x_span,
            y_span,
            bottom_bias,
        )
        if relaxed_candidate is not None:
            return relaxed_candidate

        return self.build_edge_clipped_break_candidate(
            contour,
            raw_mask,
            (height, width),
            area_px,
            aspect_ratio,
            x_span,
            y_span,
            bottom_bias,
            bbox_x,
            bbox_w,
        )

    def build_edge_clipped_break_candidate(
        self,
        contour: np.ndarray,
        raw_mask: np.ndarray,
        image_shape: tuple[int, int],
        area_px: float,
        aspect_ratio: float,
        x_span: float,
        y_span: float,
        bottom_bias: float,
        bbox_x: int,
        bbox_w: int,
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        border_touch = bbox_x <= 3 or (bbox_x + bbox_w) >= (width - 3)
        if not border_touch:
            return None

        x_span_norm = x_span / max(float(width), 1.0)
        vertical_span_norm = y_span / max(float(height), 1.0)
        if area_px < max(180.0, 0.28 * self.min_contour_area_px):
            return None
        if aspect_ratio < 1.12:
            return None
        if x_span_norm < 0.08 or x_span_norm > 0.42:
            return None
        if vertical_span_norm > 0.34:
            return None
        if bottom_bias < 0.58:
            return None

        return self.make_side_view_candidate(
            contour,
            raw_mask,
            image_shape,
            area_px,
            max(aspect_ratio, 1.12),
            x_span,
            y_span,
            bottom_bias,
            min_abs_axis_x=0.72,
        )

    def build_side_view_candidate(
        self,
        contour: np.ndarray,
        raw_mask: np.ndarray,
        image_shape: tuple[int, int],
        area_px: float,
        aspect_ratio: float,
        x_span: float,
        y_span: float,
        bottom_bias: float,
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        x_span_norm = x_span / max(float(width), 1.0)
        vertical_span_norm = y_span / max(float(height), 1.0)
        if area_px < self.min_contour_area_px:
            return None
        if aspect_ratio < self.min_aspect_ratio:
            return None
        if x_span_norm < 0.38:
            return None
        if vertical_span_norm > 0.18:
            return None
        if bottom_bias < 0.55:
            return None

        return self.make_side_view_candidate(
            contour,
            raw_mask,
            image_shape,
            area_px,
            aspect_ratio,
            x_span,
            y_span,
            bottom_bias,
        )

    def build_relaxed_side_view_candidate(
        self,
        contour: np.ndarray,
        raw_mask: np.ndarray,
        image_shape: tuple[int, int],
        area_px: float,
        aspect_ratio: float,
        x_span: float,
        y_span: float,
        bottom_bias: float,
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        x_span_norm = x_span / max(float(width), 1.0)
        vertical_span_norm = y_span / max(float(height), 1.0)
        if area_px < self.side_view_min_contour_area_px:
            return None
        if aspect_ratio < max(1.6, 0.70 * self.min_aspect_ratio):
            return None
        if x_span_norm < self.side_view_min_span_fraction:
            return None
        if vertical_span_norm > 0.20:
            return None
        if bottom_bias < 0.45:
            return None

        return self.make_side_view_candidate(
            contour,
            raw_mask,
            image_shape,
            area_px,
            aspect_ratio,
            x_span,
            y_span,
            bottom_bias,
        )

    def make_side_view_candidate(
        self,
        contour: np.ndarray,
        raw_mask: np.ndarray,
        image_shape: tuple[int, int],
        area_px: float,
        aspect_ratio: float,
        x_span: float,
        y_span: float,
        bottom_bias: float,
        min_abs_axis_x: float = 0.90,
    ) -> Optional[CableCandidate]:
        height, width = image_shape
        ys, xs = np.where(raw_mask > 0)
        if xs.size < 60:
            return None

        all_points = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
        [vx], [vy], [x0], [y0] = cv2.fitLine(all_points, cv2.DIST_L2, 0, 0.01, 0.01)
        axis = np.array([float(vx), float(vy)], dtype=np.float32)
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm < 1e-6:
            return None
        axis /= axis_norm
        if abs(float(axis[0])) < min_abs_axis_x:
            return None
        if axis[0] < 0.0:
            axis *= -1.0

        line_center = np.array([float(x0), float(y0)], dtype=np.float32)
        rel_points = all_points - line_center
        line_proj = rel_points @ axis
        t_min = float(np.min(line_proj))
        t_max = float(np.max(line_proj))

        def point_from_t(t_value: float) -> tuple[int, int]:
            point = line_center + (axis * t_value)
            return (
                int(clamp(float(point[0]), 0.0, float(width - 1))),
                int(clamp(float(point[1]), 0.0, float(height - 1))),
            )

        t_mid = 0.5 * (t_min + t_max)
        if abs(float(axis[0])) > 1e-3:
            t_center = clamp(
                ((0.5 * width) - float(line_center[0])) / float(axis[0]),
                t_min,
                t_max,
            )
        else:
            t_center = t_mid

        left_point = point_from_t(t_min)
        center_point = point_from_t(float(t_center))
        right_point = point_from_t(t_max)
        mid_point = point_from_t(t_mid)
        track_points = [left_point, mid_point, right_point]

        moments = cv2.moments(raw_mask, binaryImage=True)
        if moments["m00"] <= 1e-6:
            return None
        centroid = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )

        center_error_norm = (center_point[0] - (width / 2.0)) / max(width / 2.0, 1.0)
        center_error_norm = clamp(float(center_error_norm), -1.0, 1.0)
        heading_angle = float(np.arctan2(axis[1], max(abs(axis[0]), 1e-6)))
        heading_error_norm = clamp(
            heading_angle / np.radians(25.0),
            -self.max_heading_error_norm,
            self.max_heading_error_norm,
        )
        x_span_norm = x_span / max(float(width), 1.0)
        vertical_span_norm = y_span / max(float(height), 1.0)

        score = (
            area_px
            * aspect_ratio
            * (0.45 + 0.55 * x_span_norm)
            * (0.30 + 0.70 * bottom_bias)
            * (1.0 - 0.35 * min(1.0, abs(center_error_norm)))
        )

        return CableCandidate(
            contour=contour,
            points=all_points,
            mask=raw_mask,
            area_px=area_px,
            aspect_ratio=aspect_ratio,
            centroid=centroid,
            near_point=center_point,
            control_point=center_point,
            center_error_norm=center_error_norm,
            heading_error_norm=heading_error_norm,
            run_span_px=float(max(6.0, y_span + 1.0)),
            vertical_span_norm=vertical_span_norm,
            track_points=track_points,
            score=score,
            axis=axis,
        )

    def detect_corrosion(
        self, hsv_frame: np.ndarray, candidate: CableCandidate
    ) -> Optional[FaultReport]:
        brown_mask = cv2.inRange(hsv_frame, self.brown_lower_hsv, self.brown_upper_hsv)
        brown_mask = cv2.morphologyEx(
            brown_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        )
        cable_region = cv2.dilate(
            candidate.mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
            iterations=1,
        )
        overlap = cv2.bitwise_and(brown_mask, cable_region)
        contours, _ = cv2.findContours(overlap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        best = max(contours, key=cv2.contourArea)
        if cv2.contourArea(best) < 120.0:
            return None

        moments = cv2.moments(best)
        if moments["m00"] <= 1e-6:
            return None

        point = (
            int(moments["m10"] / moments["m00"]),
            int(moments["m01"] / moments["m00"]),
        )
        return FaultReport("CORROSION", point, "brown_segment")

    def detect_break(
        self, candidates: list[CableCandidate], image_shape: tuple[int, int]
    ) -> Optional[FaultReport]:
        if len(candidates) < 2:
            return None

        height, width = image_shape
        best_report: Optional[FaultReport] = None
        best_gap = 0.0

        for first, second in combinations(candidates, 2):
            axis = first.axis.copy()
            second_axis = second.axis.copy()
            if float(np.dot(axis, second_axis)) < 0.0:
                second_axis *= -1.0

            dot = clamp(float(np.dot(axis, second_axis)), -1.0, 1.0)
            angle_error_deg = abs(np.degrees(np.arccos(dot)))
            if angle_error_deg > 20.0:
                continue

            normal = np.array([-axis[1], axis[0]], dtype=np.float32)
            first_proj = first.points @ axis
            second_proj = second.points @ axis
            first_cross = first.points @ normal
            second_cross = second.points @ normal

            lateral_gap = abs(float(np.mean(first_cross) - np.mean(second_cross)))
            if lateral_gap > 60.0:
                continue

            first_min = float(np.min(first_proj))
            first_max = float(np.max(first_proj))
            second_min = float(np.min(second_proj))
            second_max = float(np.max(second_proj))

            if first_min <= second_min:
                gap_px = second_min - first_max
                gap_proj = 0.5 * (first_max + second_min)
            else:
                gap_px = first_min - second_max
                gap_proj = 0.5 * (first_min + second_max)

            if gap_px < 18.0 or gap_px > 160.0 or gap_px <= best_gap:
                continue

            cross_mid = 0.5 * (float(np.mean(first_cross)) + float(np.mean(second_cross)))
            gap_point = (axis * gap_proj) + (normal * cross_mid)
            x_px = int(clamp(float(gap_point[0]), 0.0, float(width - 1)))
            y_px = int(clamp(float(gap_point[1]), 0.0, float(height - 1)))

            best_gap = gap_px
            best_report = FaultReport(
                "CABLE BREAK",
                (x_px, y_px),
                f"gap={gap_px:.0f}px",
            )

        return best_report

    def render_debug_frame(
        self,
        frame: np.ndarray,
        state: MissionState,
        mask: np.ndarray,
        edges: np.ndarray,
        candidates: list[CableCandidate],
        chosen: Optional[CableCandidate],
        corrosion: Optional[FaultReport],
        cable_break: Optional[FaultReport],
    ) -> np.ndarray:
        debug = frame.copy()
        height, width = debug.shape[:2]

        for candidate in candidates[:4]:
            cv2.drawContours(debug, [candidate.contour], -1, (120, 120, 120), 1)

        cv2.line(debug, (width // 2, 0), (width // 2, height - 1), (255, 0, 0), 2)

        if chosen is not None:
            cv2.drawContours(debug, [chosen.contour], -1, (0, 255, 0), 2)
            cv2.circle(debug, chosen.centroid, 5, (0, 255, 255), -1)
            for idx in range(1, len(chosen.track_points)):
                cv2.line(
                    debug,
                    chosen.track_points[idx - 1],
                    chosen.track_points[idx],
                    (255, 255, 0),
                    2,
                )
            cv2.circle(debug, chosen.near_point, 5, (255, 255, 0), -1)
            cv2.circle(debug, chosen.control_point, 6, (0, 0, 255), -1)
            cv2.putText(
                debug,
                (
                    f"err={chosen.center_error_norm:+.2f} "
                    f"head={chosen.heading_error_norm:+.2f} "
                    f"trk={len(chosen.track_points)}"
                ),
                (12, 58),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.putText(
            debug,
            f"STATE: {state.value}",
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        if corrosion is not None:
            cv2.circle(debug, corrosion.image_point, 8, (0, 140, 255), 2)
            cv2.putText(
                debug,
                "CORROSION",
                (max(0, corrosion.image_point[0] - 40), max(20, corrosion.image_point[1] - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 140, 255),
                2,
                cv2.LINE_AA,
            )

        if cable_break is not None:
            cv2.circle(debug, cable_break.image_point, 8, (0, 0, 255), 2)
            cv2.putText(
                debug,
                "CABLE BREAK",
                (max(0, cable_break.image_point[0] - 55), max(20, cable_break.image_point[1] - 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        inset_w = max(120, width // 4)
        inset_h = max(90, height // 4)
        mask_small = cv2.resize(mask, (inset_w, inset_h), interpolation=cv2.INTER_NEAREST)
        edges_small = cv2.resize(edges, (inset_w, inset_h), interpolation=cv2.INTER_NEAREST)
        mask_small_bgr = cv2.cvtColor(mask_small, cv2.COLOR_GRAY2BGR)
        edges_small_bgr = cv2.cvtColor(edges_small, cv2.COLOR_GRAY2BGR)

        debug[0:inset_h, 0:inset_w] = mask_small_bgr
        debug[0:inset_h, inset_w : 2 * inset_w] = edges_small_bgr
        cv2.putText(debug, "mask", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(
            debug,
            "edges",
            (inset_w + 8, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1,
        )

        return debug

    def analyze_frame(self, frame: np.ndarray) -> VisionResult:
        height, width = frame.shape[:2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dark_mask = cv2.inRange(hsv, self.dark_lower_hsv, self.dark_upper_hsv)
        roi_y0 = int(clamp(self.roi_start_fraction, 0.0, 0.9) * height)
        dark_mask[:roi_y0, :] = 0
        dark_mask = cv2.morphologyEx(
            dark_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )
        dark_mask = cv2.morphologyEx(
            dark_mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)),
        )

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        blurred_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(blurred_gray, 40, 120)
        edges[:roi_y0, :] = 0
        edge_seed = cv2.dilate(
            dark_mask,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
            iterations=1,
        )
        edges_on_dark = cv2.bitwise_and(edges, edge_seed)

        # Black-hat picks out thin dark cable-like ridges even when the raw HSV
        # threshold is dominated by self-shadow or misses anti-aliased pixels.
        ridge_mask = cv2.morphologyEx(
            blurred_gray,
            cv2.MORPH_BLACKHAT,
            cv2.getStructuringElement(cv2.MORPH_RECT, (17, 17)),
        )
        _, ridge_mask = cv2.threshold(
            ridge_mask, self.ridge_threshold, 255, cv2.THRESH_BINARY
        )
        ridge_mask[:roi_y0, :] = 0
        ridge_mask = cv2.morphologyEx(
            ridge_mask,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )
        ridge_mask = cv2.morphologyEx(
            ridge_mask,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)),
        )

        support_mask = cv2.bitwise_or(edges_on_dark, ridge_mask)
        support_mask = cv2.dilate(
            support_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13)),
            iterations=1,
        )
        # Keep dark regions only where there is local cable-like structure nearby.
        # This suppresses large seabed/shadow blobs that would otherwise swallow the
        # cable into one rejected contour.
        supported_dark = cv2.bitwise_and(dark_mask, support_mask)
        # Recover the cable body thickness from the thin ridge/edge seed without
        # letting the mask jump arbitrarily into the background.
        grow_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
        for _ in range(2):
            supported_dark = cv2.bitwise_and(
                dark_mask,
                cv2.dilate(supported_dark, grow_kernel, iterations=1),
            )

        combined = cv2.bitwise_or(supported_dark, edges_on_dark)
        combined = cv2.bitwise_or(combined, ridge_mask)
        combined = cv2.morphologyEx(
            combined,
            cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)),
        )
        combined = cv2.morphologyEx(
            combined,
            cv2.MORPH_OPEN,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        )

        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[CableCandidate] = []
        break_candidates: list[CableCandidate] = []
        for contour in contours:
            candidate = self.build_candidate(contour, (height, width))
            if candidate is not None:
                candidates.append(candidate)
                break_candidates.append(candidate)
                continue
            fragment_candidate = self.build_break_fragment_candidate(contour, (height, width))
            if fragment_candidate is not None:
                break_candidates.append(fragment_candidate)

        candidates.sort(key=lambda item: item.score, reverse=True)
        break_candidates.sort(key=lambda item: item.score, reverse=True)
        chosen = candidates[0] if candidates else None
        corrosion = self.detect_corrosion(hsv, chosen) if chosen is not None else None
        cable_break = self.detect_break(break_candidates, (height, width))
        debug_frame = self.render_debug_frame(
            frame,
            self.state,
            combined,
            edges,
            break_candidates,
            chosen,
            corrosion,
            cable_break,
        )

        return VisionResult(
            cable_visible=chosen is not None,
            candidate=chosen,
            debug_frame=debug_frame,
            corrosion_fault=corrosion,
            break_fault=cable_break,
        )

    def show_debug(self, frame: Optional[np.ndarray]) -> None:
        if frame is None or not self.debug_window_available:
            return

        try:
            cv2.imshow("cable_detection", frame)
            cv2.waitKey(1)
        except cv2.error as exc:
            self.debug_window_available = False
            self.get_logger().warn(f"Disabling OpenCV debug window: {exc}")

    def maybe_log_status(
        self, now_sec: float, vision: Optional[VisionResult], center_error: float
    ) -> None:
        if now_sec - self.last_status_log_time_sec < 1.5:
            return

        self.last_status_log_time_sec = now_sec
        if self.latest_frame is None:
            self.get_logger().info(f"Waiting for images on {self.camera_topic}")
            return

        if vision is None or not vision.cable_visible:
            self.get_logger().info(f"STATE: {self.state.value} | cable=not_visible")
            return

        self.get_logger().info(
            (
                f"STATE: {self.state.value} | cable=visible | center_error={center_error:+.2f}"
                f" | heading={vision.candidate.heading_error_norm:+.2f}"
                f" | streak={self.detect_streak}"
            )
        )

    def maybe_queue_fault(
        self,
        now_sec: float,
        corrosion: Optional[FaultReport],
        cable_break: Optional[FaultReport],
    ) -> Optional[FaultReport]:
        if not self.fault_detection_armed(now_sec):
            self.corrosion_streak = 0
            self.break_streak = 0
            return None

        self.corrosion_streak = self.corrosion_streak + 1 if corrosion else 0

        if cable_break is not None:
            return cable_break
        if corrosion is not None and self.corrosion_streak >= self.fault_frames_required:
            return corrosion
        return None

    def report_fault(self, fault: FaultReport) -> None:
        if not self.record_fault(fault):
            return

        if self.normalize_fault_type(fault.fault_type) == "CORROSION":
            self.corrosionCount += 1

    def control_step(self) -> None:
        now_sec = self.now_sec()
        self.integrate_motion_estimate(now_sec)

        if not self.inspectionComplete and self.reached_stop_marker():
            self.finish_inspection_at_marker()

        if (
            self.inspectionComplete
            or self.mission_complete
            or self.state == MissionState.MISSION_COMPLETE
        ):
            cmd = self.make_cmd(0.0, 0.0)
            self.cmd_pub.publish(cmd)
            self.last_cmd_sent = cmd
            return

        if self.state == MissionState.REVERSE:
            if now_sec >= self.return_reverse_until_sec:
                _, _, _, robot_yaw = self.current_pose()
                self.turnaround_target_yaw = wrap_angle(robot_yaw + math.pi)
                self.set_state(MissionState.TURNAROUND)
                cmd = self.make_cmd(0.0, 0.0)
            else:
                cmd = self.make_cmd(-self.return_reverse_speed, 0.0)
            self.cmd_pub.publish(cmd)
            self.last_cmd_sent = cmd
            return

        if self.state == MissionState.TURNAROUND:
            if self.turnaround_complete():
                self.complete_turnaround()
                cmd = self.make_cmd(0.0, 0.0)
            else:
                _, _, _, robot_yaw = self.current_pose()
                yaw_error = wrap_angle((self.turnaround_target_yaw or robot_yaw) - robot_yaw)
                cmd = self.make_cmd(0.0, 1.6 * yaw_error)
            self.cmd_pub.publish(cmd)
            self.last_cmd_sent = cmd
            return

        if self.latest_frame is None:
            self.maybe_log_status(now_sec, None, 0.0)
            cmd = self.make_cmd(0.0, 0.0)
            self.cmd_pub.publish(cmd)
            self.last_cmd_sent = cmd
            return

        vision = self.analyze_frame(self.latest_frame)
        self.show_debug(vision.debug_frame)

        cable_visible = vision.cable_visible and vision.candidate is not None
        center_error = vision.candidate.center_error_norm if cable_visible else 0.0
        heading_error = vision.candidate.heading_error_norm if cable_visible else 0.0
        projection = self.current_projection()
        stable_search_detection = cable_visible and (
            abs(center_error) <= self.search_entry_max_center_error
        )

        if stable_search_detection:
            self.last_detection_time_sec = now_sec
            self.detect_streak += 1
            if abs(center_error) > 1e-3:
                self.last_turn_direction = 1.0 if center_error > 0.0 else -1.0
        else:
            self.detect_streak = 0

        if cable_visible and abs(center_error) < self.center_threshold:
            self.aligned_streak += 1
        else:
            self.aligned_streak = 0

        self.maybe_log_status(now_sec, vision, center_error)

        cable_break_fault = self.detect_break_from_cable_loss(cable_visible)
        fault_candidate = self.maybe_queue_fault(
            now_sec,
            vision.corrosion_fault,
            cable_break_fault,
        )

        if self.state == MissionState.FOLLOW and fault_candidate is not None:
            self.pending_fault = fault_candidate
            self.set_state(MissionState.FAULT_DETECTION)

        cmd = self.make_cmd(0.0, 0.0)
        cable_lost = (now_sec - self.last_detection_time_sec) > self.lost_cable_timeout_sec

        if self.state == MissionState.SEARCH:
            if stable_search_detection and self.detect_streak >= self.detection_frames_required:
                self.set_state(MissionState.ALIGN)
                cmd = self.align_cmd(center_error)
            else:
                cmd = self.search_cmd(center_error if cable_visible else None)

        elif self.state == MissionState.ALIGN:
            if not cable_visible:
                if cable_lost:
                    self.set_state(MissionState.SEARCH)
                    cmd = self.search_cmd()
                else:
                    cmd = self.make_cmd(
                        0.0, self.steering_direction * 0.25 * self.last_turn_direction
                    )
            else:
                cmd = self.align_cmd(center_error)
                if self.aligned_streak >= self.aligned_frames_required:
                    self.confirmation_deadline_sec = now_sec + self.confirmation_stop_sec
                    self.set_state(MissionState.DETECTION_CONFIRMATION)
                    cmd = self.make_cmd(0.0, 0.0)

        elif self.state == MissionState.DETECTION_CONFIRMATION:
            cmd = self.make_cmd(0.0, 0.0)
            if not cable_visible and cable_lost:
                self.set_state(MissionState.SEARCH)
                cmd = self.search_cmd()
            elif cable_visible and abs(center_error) > self.realign_threshold:
                self.set_state(MissionState.ALIGN)
                cmd = self.align_cmd(center_error)
            elif now_sec >= self.confirmation_deadline_sec:
                self.get_logger().info("CABLE DETECTED")
                print("CABLE DETECTED", flush=True)
                self.set_state(MissionState.FOLLOW)
                if cable_visible:
                    cmd = self.follow_cmd(center_error, heading_error)

        elif self.state == MissionState.FOLLOW:
            if not cable_visible:
                if cable_lost:
                    self.set_state(MissionState.SEARCH)
                    cmd = self.search_cmd()
                else:
                    cmd = self.short_reacquire_cmd()
            elif abs(center_error) > self.realign_threshold:
                self.set_state(MissionState.ALIGN)
                cmd = self.align_cmd(center_error)
            else:
                cmd = self.follow_cmd(center_error, heading_error)

        elif self.state == MissionState.FAULT_DETECTION:
            if self.pending_fault is not None:
                self.report_fault(self.pending_fault)
                self.pending_fault = None
                if self.inspectionComplete:
                    cmd = self.make_cmd(0.0, 0.0)
                    self.cmd_pub.publish(cmd)
                    self.last_cmd_sent = cmd
                    return

            if not cable_visible:
                if cable_lost:
                    self.set_state(MissionState.SEARCH)
                    cmd = self.search_cmd()
                else:
                    cmd = self.short_reacquire_cmd()
            elif abs(center_error) > self.realign_threshold:
                self.set_state(MissionState.ALIGN)
                cmd = self.align_cmd(center_error)
            else:
                cmd = self.follow_cmd(center_error, heading_error)
                self.set_state(MissionState.FOLLOW)

        else:
            self.set_state(MissionState.SEARCH)
            cmd = self.search_cmd()

        if not self.stopOnlyOnSecondCorrosion:
            if (
                not self.outbound_complete
                and self.state in {MissionState.FOLLOW, MissionState.FAULT_DETECTION}
                and self.total_cable_distance > 0.0
                and projection.distance_m
                >= (self.total_cable_distance - self.station_reach_distance_m)
            ):
                self.begin_turnaround()
                cmd = self.make_cmd(0.0, 0.0)
            elif (
                self.return_trip_started
                and not self.mission_complete
                and self.state in {MissionState.FOLLOW, MissionState.FAULT_DETECTION}
                and projection.distance_m <= self.station_reach_distance_m
            ):
                self.mission_complete = True
                self.set_state(MissionState.MISSION_COMPLETE)
                self.emit_final_report()
                cmd = self.make_cmd(0.0, 0.0)

        self.cmd_pub.publish(cmd)
        self.last_cmd_sent = cmd


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CableVisionFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
