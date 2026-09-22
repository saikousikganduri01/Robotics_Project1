#!/usr/bin/env python3
"""
Add-on monitor node for cable inspection simulation.

This script does not change robot control logic. It only:
- listens to existing logs/events from cable_vision_follower,
- simulates landing-station notifications,
- simulates cable data transmission status,
- logs inspection data to CSV for offline graphing.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import math
import matplotlib
from pathlib import Path
import re
from typing import Optional

import gz.transport13 as gz_transport
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from gz.msgs10 import boolean_pb2, entity_factory_pb2, entity_pb2
import rclpy
from generate_inspection_graphs import FIELDNAMES, generate_graphs, rows_from_records, write_excel_report
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from cable_runtime_utils import (
    build_cumulative_distances,
    clamp,
    integrate_body_twist,
    load_cable_polyline,
    load_robot_initial_pose,
    make_repair_segment_sdf,
    project_on_polyline,
)


@dataclass(frozen=True)
class CableCollisionSpec:
    x: float
    y: float
    z: float
    yaw: float
    radius: float
    length: float


@dataclass(frozen=True)
class SpawnedVisualSpec:
    model_name: str
    x: float
    y: float
    z: float
    yaw: float
    radius: float
    length: float
    color_rgba: tuple[float, float, float, float]


class CableStationMonitor(Node):
    def __init__(self) -> None:
        super().__init__("cable_station_monitor")

        # Inputs / outputs
        self.declare_parameter("cmd_topic", "/model/cable_repair_robot/cmd_vel")
        self.declare_parameter("rosout_topic", "/rosout")
        self.declare_parameter("source_sdf", "/home/kousik/ocean_ecosystem_full.sdf")
        self.declare_parameter("log_file", "/home/kousik/inspection_log.csv")
        self.declare_parameter("excel_file", "/home/kousik/inspection_points.xlsx")
        self.declare_parameter("graph_output_dir", "/home/kousik")
        self.declare_parameter("auto_export_on_shutdown", False)
        self.declare_parameter("robot_model_name", "cable_repair_robot")
        self.declare_parameter("world_name", "ocean_world")
        self.declare_parameter("clock_topic", "/clock")

        # Motion integration (estimated path from existing cmd_vel only)
        self.declare_parameter("initial_robot_x", -9.0)
        self.declare_parameter("initial_robot_y", 1.2)
        self.declare_parameter("initial_robot_yaw", 1.57)
        self.declare_parameter("linear_command_scale", -1.0)
        self.declare_parameter("lateral_command_scale", 1.0)

        # Periodic behavior
        self.declare_parameter("motion_update_hz", 20.0)
        self.declare_parameter("log_period_sec", 1.0)
        self.declare_parameter("tx_status_period_sec", 8.0)
        self.declare_parameter("fault_trigger_delay_sec", 0.0)
        self.declare_parameter("fault_service_timeout_ms", 2000)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.rosout_topic = str(self.get_parameter("rosout_topic").value)
        self.source_sdf = Path(str(self.get_parameter("source_sdf").value))
        self.log_file = Path(str(self.get_parameter("log_file").value))
        self.excel_file = Path(str(self.get_parameter("excel_file").value))
        self.graph_output_dir = Path(str(self.get_parameter("graph_output_dir").value))
        self.auto_export_on_shutdown = bool(
            self.get_parameter("auto_export_on_shutdown").value
        )
        self.robot_model_name = str(self.get_parameter("robot_model_name").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.clock_topic = str(self.get_parameter("clock_topic").value)
        initial_robot_x = float(self.get_parameter("initial_robot_x").value)
        initial_robot_y = float(self.get_parameter("initial_robot_y").value)
        initial_robot_yaw = float(self.get_parameter("initial_robot_yaw").value)
        self.robot_x, self.robot_y, self.robot_yaw = load_robot_initial_pose(
            self.source_sdf,
            self.robot_model_name,
            initial_robot_x,
            initial_robot_y,
            initial_robot_yaw,
        )
        self.linear_command_scale = float(self.get_parameter("linear_command_scale").value)
        self.lateral_command_scale = float(self.get_parameter("lateral_command_scale").value)

        self.motion_update_hz = max(1.0, float(self.get_parameter("motion_update_hz").value))
        self.log_period_sec = max(0.2, float(self.get_parameter("log_period_sec").value))
        self.tx_status_period_sec = max(
            1.0, float(self.get_parameter("tx_status_period_sec").value)
        )
        self.fault_trigger_delay_sec = max(
            0.0, float(self.get_parameter("fault_trigger_delay_sec").value)
        )
        self.fault_service_timeout_ms = max(
            100, int(self.get_parameter("fault_service_timeout_ms").value)
        )

        self.segment_points: list[tuple[float, float]] = load_cable_polyline(self.source_sdf)
        self.segment_cumulative_dist: list[float] = build_cumulative_distances(
            self.segment_points
        )
        self.cable_collision_specs = self._load_cable_collision_specs()

        self.last_cmd = Twist()
        self.last_motion_time_sec = self.now_sec()
        self.pending_repair = False
        self.repair_completed = False
        self.transmission_active = True
        self.simulated_faults_active = False
        self.activity_started = False
        self.awaiting_fault_type_line = False
        self.last_fault_type_for_log = ""
        self.next_repair_status_for_log = ""
        self.export_completed = False
        self.run_records: list[dict[str, str]] = []
        self.robot_positions: list[tuple[float, float]] = []
        self.detected_fault_coordinates: list[tuple[float, float]] = []
        self.wall_start_time_sec = self.now_sec()
        self.latest_sim_time_sec: Optional[float] = None
        self.last_gazebo_warning_sec = -1e9
        self.gz_node = gz_transport.Node()
        self.gz_create_service = f"/world/{self.world_name}/create"
        self.gz_remove_service = f"/world/{self.world_name}/remove"
        self.spawned_visual_models: set[str] = set()
        self.startup_patch_spec = self._build_startup_patch_spec()
        self.startup_patch_ready = self.startup_patch_spec is None
        self.fault_visual_specs = self._build_fault_visual_specs()
        self.fault_visuals_ready = False

        self._ensure_log_file()

        self.create_subscription(Clock, self.clock_topic, self.on_clock, 10)
        self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.create_subscription(Log, self.rosout_topic, self.on_rosout, 200)
        self.create_timer(1.0 / self.motion_update_hz, self.motion_step)
        self.create_timer(self.log_period_sec, self.write_log_row)
        self.create_timer(self.tx_status_period_sec, self.print_transmission_status)
        self.create_timer(0.5, self.sync_simulation_state)

        if not self.startup_patch_ready:
            self.startup_patch_ready = self.ensure_startup_cable_health()
        self.sync_simulation_state()
        self.print_transmission_status()

        self.get_logger().info(
            f"cable_station_monitor started | cmd_topic={self.cmd_topic} log_file={self.log_file}"
        )

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_clock(self, msg: Clock) -> None:
        self.latest_sim_time_sec = float(msg.clock.sec) + (float(msg.clock.nanosec) / 1e9)

    def elapsed_simulation_sec(self) -> float:
        if self.latest_sim_time_sec is not None:
            return max(0.0, self.latest_sim_time_sec)
        return max(0.0, self.now_sec() - self.wall_start_time_sec)

    def _ensure_log_file(self) -> None:
        self.log_file.parent.mkdir(parents=True, exist_ok=True)
        if self.log_file.exists() and self.log_file.stat().st_size > 0:
            return

        with self.log_file.open("w", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(
                file_obj,
                fieldnames=FIELDNAMES,
            )
            writer.writeheader()

    def _load_cable_collision_specs(self) -> dict[str, CableCollisionSpec]:
        if not self.source_sdf.exists():
            return {}

        text = self.source_sdf.read_text(encoding="utf-8")
        pattern = re.compile(
            r'<collision name="([^"]+)">\s*'
            r"<pose>([^<]+)</pose>\s*"
            r"<geometry><cylinder><radius>([^<]+)</radius><length>([^<]+)</length>",
            re.MULTILINE,
        )

        specs: dict[str, CableCollisionSpec] = {}
        for match in pattern.finditer(text):
            pose_values = match.group(2).split()
            if len(pose_values) < 6:
                continue
            specs[match.group(1)] = CableCollisionSpec(
                x=float(pose_values[0]),
                y=float(pose_values[1]),
                z=float(pose_values[2]),
                yaw=float(pose_values[5]),
                radius=float(match.group(3)),
                length=float(match.group(4)),
            )
        return specs

    def _build_startup_patch_spec(self) -> Optional[SpawnedVisualSpec]:
        return None

    def _build_fault_visual_specs(self) -> list[SpawnedVisualSpec]:
        return []

    def _warn_gazebo(self, message: str) -> None:
        now = self.now_sec()
        if now - self.last_gazebo_warning_sec < 5.0:
            return
        self.last_gazebo_warning_sec = now
        self.get_logger().warn(message)

    def _request_boolean_service(
        self,
        service_name: str,
        request,
        request_type,
        *,
        log_failure: bool,
    ) -> bool:
        try:
            success, response = self.gz_node.request(
                service_name,
                request,
                request_type,
                boolean_pb2.Boolean,
                self.fault_service_timeout_ms,
            )
        except Exception as exc:
            if log_failure:
                self._warn_gazebo(f"Gazebo request failed for {service_name}: {exc}")
            return False

        if success and getattr(response, "data", False):
            return True

        if log_failure:
            self._warn_gazebo(f"Gazebo request failed for {service_name}")
        return False

    def remove_visual_model(self, model_name: str) -> bool:
        request = entity_pb2.Entity()
        request.name = model_name
        request.type = entity_pb2.Entity.MODEL
        removed = self._request_boolean_service(
            self.gz_remove_service,
            request,
            entity_pb2.Entity,
            log_failure=False,
        )
        if removed:
            self.spawned_visual_models.discard(model_name)
        return removed

    def spawn_visual_model(self, spec: SpawnedVisualSpec) -> bool:
        if spec.model_name in self.spawned_visual_models:
            self.remove_visual_model(spec.model_name)

        request = entity_factory_pb2.EntityFactory()
        request.sdf = make_repair_segment_sdf(
            spec.model_name,
            spec.radius,
            spec.length,
            spec.color_rgba,
        )
        request.name = spec.model_name
        request.allow_renaming = False
        request.pose.name = spec.model_name
        request.pose.position.x = spec.x
        request.pose.position.y = spec.y
        request.pose.position.z = spec.z
        half_yaw = 0.5 * spec.yaw
        request.pose.orientation.z = math.sin(half_yaw)
        request.pose.orientation.w = math.cos(half_yaw)
        created = self._request_boolean_service(
            self.gz_create_service,
            request,
            entity_factory_pb2.EntityFactory,
            log_failure=True,
        )
        if created:
            self.spawned_visual_models.add(spec.model_name)
        return created

    def ensure_startup_cable_health(self) -> bool:
        if self.startup_patch_spec is None:
            return True
        return self.spawn_visual_model(self.startup_patch_spec)

    def ensure_fault_visuals(self) -> bool:
        if not self.fault_visual_specs:
            return True
        for spec in self.fault_visual_specs:
            if not self.spawn_visual_model(spec):
                return False
        return True

    def sync_simulation_state(self) -> None:
        if not self.startup_patch_ready:
            self.startup_patch_ready = self.ensure_startup_cable_health()
            if not self.startup_patch_ready:
                return

        if self.fault_visuals_ready:
            return
        if self.elapsed_simulation_sec() < self.fault_trigger_delay_sec:
            return

        if not self.ensure_fault_visuals():
            return

        self.fault_visuals_ready = True
        self.simulated_faults_active = True
        self.transmission_active = False
        print("SIMULATION EVENT", flush=True)
        print("Cable faults detected on transmission line", flush=True)
        self.print_transmission_status()

    def on_cmd_vel(self, msg: Twist) -> None:
        self.last_cmd = msg
        if self._is_nonzero_cmd(msg):
            self._mark_activity_started()

    def on_rosout(self, msg: Log) -> None:
        # Only consume existing follower output; no controller edits needed.
        if msg.name != "cable_vision_follower":
            return

        text = msg.msg.strip()
        if text.startswith("FAULT DETECTED:"):
            self.handle_fault_message(text)
            self.awaiting_fault_type_line = False
            return
        if text == "FAULT DETECTED":
            self.awaiting_fault_type_line = True
            return
        if self.awaiting_fault_type_line and text.startswith("Type:"):
            self.awaiting_fault_type_line = False
            self.handle_fault_message(text)
            return
        if text == "REPAIR COMPLETED":
            self.handle_repair_completed_message()
            return

    def handle_fault_message(self, text: str) -> None:
        if "CABLE BREAK" not in text:
            return

        self._mark_activity_started()
        self.detected_fault_coordinates.append((self.robot_x, self.robot_y))
        self.last_fault_type_for_log = "CABLE BREAK"
        self.pending_repair = True
        self.next_repair_status_for_log = "in_progress"

    def handle_repair_completed_message(self) -> None:
        self._mark_activity_started()
        self.pending_repair = False
        self.repair_completed = True
        self.next_repair_status_for_log = "repaired"
        print("STATION UPDATE", flush=True)
        print("Cable repaired successfully", flush=True)
        if not self.transmission_active and not self.simulated_faults_active:
            self.transmission_active = True
            print("DATA TRANSMISSION RESTORED", flush=True)
            print("Source: Station A", flush=True)
            print("Destination: Station B", flush=True)
        self.append_event_log_row("CABLE BREAK", "repaired")

    def _is_nonzero_cmd(self, msg: Twist) -> bool:
        return any(
            abs(value) > 1e-4
            for value in (
                float(msg.linear.x),
                float(msg.linear.y),
                float(msg.angular.z),
            )
        )

    def _mark_activity_started(self) -> None:
        if self.activity_started:
            return

        self.activity_started = True
        self.robot_positions.append((self.robot_x, self.robot_y))

    def motion_step(self) -> None:
        if not self.activity_started:
            self.last_motion_time_sec = self.now_sec()
            return

        now = self.now_sec()
        dt = clamp(now - self.last_motion_time_sec, 0.0, 0.25)
        self.last_motion_time_sec = now
        if dt <= 0.0:
            return

        self.robot_x, self.robot_y, self.robot_yaw = integrate_body_twist(
            self.robot_x,
            self.robot_y,
            self.robot_yaw,
            float(self.last_cmd.linear.x),
            float(self.last_cmd.linear.y),
            float(self.last_cmd.angular.z),
            dt,
            linear_x_scale=self.linear_command_scale,
            linear_y_scale=self.lateral_command_scale,
        )

    def print_transmission_status(self) -> None:
        if self.transmission_active:
            print("DATA TRANSMISSION ACTIVE", flush=True)
        else:
            print("DATA TRANSMISSION INTERRUPTED", flush=True)
        print("Source: Station A", flush=True)
        print("Destination: Station B", flush=True)

    def _build_log_row(self, fault_type: str, repair_status: str) -> dict[str, str]:
        projection = project_on_polyline(
            self.robot_x,
            self.robot_y,
            self.segment_points,
            self.segment_cumulative_dist,
        )
        return {
            "time": datetime.now().isoformat(timespec="seconds"),
            "robot_x": f"{self.robot_x:.3f}",
            "robot_y": f"{self.robot_y:.3f}",
            "fault_type": fault_type,
            "cable_distance": f"{projection.distance_m:.3f}",
            "repair_status": repair_status,
        }

    def _append_log_row(self, row: dict[str, str]) -> None:
        self.run_records.append(dict(row))
        self.robot_positions.append((self.robot_x, self.robot_y))
        with self.log_file.open("a", newline="", encoding="utf-8") as file_obj:
            writer = csv.DictWriter(file_obj, fieldnames=FIELDNAMES)
            writer.writerow(row)

    def write_log_row(self) -> None:
        if not self.activity_started:
            return

        repair_status = self.next_repair_status_for_log

        if not repair_status:
            repair_status = "in_progress" if self.pending_repair else "none"

        row = self._build_log_row(self.last_fault_type_for_log, repair_status)
        self._append_log_row(row)

        self.last_fault_type_for_log = ""
        if self.next_repair_status_for_log in {"repaired", "in_progress"}:
            self.next_repair_status_for_log = ""

    def append_event_log_row(self, fault_type: str, repair_status: str) -> None:
        row = self._build_log_row(fault_type, repair_status)
        self._append_log_row(row)

    def finalize_outputs(self) -> None:
        if self.export_completed:
            return
        self.export_completed = True

        if not self.auto_export_on_shutdown:
            return
        if not self.run_records:
            self.get_logger().info("No points captured in this run; skipping Excel/graph export.")
            return

        try:
            rows = rows_from_records(self.run_records)
            excel_path = write_excel_report(rows, self.excel_file)
            graph_paths = generate_graphs(rows, self.graph_output_dir)
            print("REPORT EXPORT COMPLETED", flush=True)
            print(f"Excel: {excel_path}", flush=True)
            for graph_path in graph_paths:
                print(f"Graph: {graph_path}", flush=True)
            self.get_logger().info(f"Saved Excel report: {excel_path}")
            for graph_path in graph_paths:
                self.get_logger().info(f"Saved graph: {graph_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to export Excel/graphs: {exc}")

    def plot_inspection_path(self) -> None:
        if not self.robot_positions and not self.detected_fault_coordinates:
            self.get_logger().info("No inspection path or fault points captured; skipping plot.")
            return

        try:
            self.graph_output_dir.mkdir(parents=True, exist_ok=True)
            plot_path = self.graph_output_dir / "underwater_cable_inspection_path.png"

            plt.figure()
            if self.robot_positions:
                path_x, path_y = zip(*self.robot_positions)
                plt.plot(path_x, path_y, label="Robot Path")
            if self.detected_fault_coordinates:
                fault_x, fault_y = zip(*self.detected_fault_coordinates)
                plt.scatter(fault_x, fault_y, color="red", label="Faults")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title("Underwater Cable Inspection Path")
            plt.legend()
            plt.savefig(plot_path, bbox_inches="tight")
            plt.close()
            self.get_logger().info(f"Saved inspection path plot: {plot_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to generate inspection path plot: {exc}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CableStationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize_outputs()
        node.plot_inspection_path()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
