#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class PolylineProjection:
    distance_m: float
    lateral_error_m: float
    point_x: float
    point_y: float
    tangent_yaw: float
    segment_index: int


LEGACY_MONITOR_FIELDNAMES = [
    "time",
    "robot_x",
    "robot_y",
    "fault_type",
    "cable_distance",
    "repair_status",
]


class RedirectingDictWriter:
    """Keep the legacy monitor's periodic CSV separate from fault-only inspection logs."""

    def __init__(self, file_obj, fieldnames, *args, **kwargs) -> None:
        self._file_obj = file_obj
        self._fieldnames = list(fieldnames)
        self._args = args
        self._kwargs = kwargs
        self._redirect_path: Optional[Path] = None

        file_name = Path(getattr(file_obj, "name", "")).expanduser()
        if (
            file_name == Path("/home/kousik/inspection_log.csv")
            and self._fieldnames == LEGACY_MONITOR_FIELDNAMES
        ):
            self._redirect_path = Path("/home/kousik/inspection_runtime_log.csv")

    def _build_writer(self, mode: str):
        if self._redirect_path is None:
            return csv._inspection_original_dict_writer(
                self._file_obj,
                self._fieldnames,
                *self._args,
                **self._kwargs,
            ), None

        self._redirect_path.parent.mkdir(parents=True, exist_ok=True)
        redirected = self._redirect_path.open(mode, newline="", encoding="utf-8")
        writer = csv._inspection_original_dict_writer(
            redirected,
            self._fieldnames,
            *self._args,
            **self._kwargs,
        )
        return writer, redirected

    def writeheader(self) -> None:
        mode = "w" if "w" in getattr(self._file_obj, "mode", "") else "a"
        writer, redirected = self._build_writer(mode)
        try:
            writer.writeheader()
        finally:
            if redirected is not None:
                redirected.close()

    def writerow(self, rowdict) -> None:
        mode = "a"
        writer, redirected = self._build_writer(mode)
        try:
            writer.writerow(rowdict)
        finally:
            if redirected is not None:
                redirected.close()

    def writerows(self, rowdicts) -> None:
        mode = "a"
        writer, redirected = self._build_writer(mode)
        try:
            writer.writerows(rowdicts)
        finally:
            if redirected is not None:
                redirected.close()


def install_csv_redirect() -> None:
    if hasattr(csv, "_inspection_original_dict_writer"):
        return

    csv._inspection_original_dict_writer = csv.DictWriter
    csv.DictWriter = RedirectingDictWriter


install_csv_redirect()


def load_cable_polyline(sdf_path: Path) -> list[tuple[float, float]]:
    if not sdf_path.exists():
        return []

    text = sdf_path.read_text(encoding="utf-8")
    pattern = re.compile(
        r'<collision name="segment_collision_(\d+)">\s*'
        r"<pose>([^<]+)</pose>",
        re.MULTILINE,
    )

    segments: list[tuple[int, float, float]] = []
    for match in pattern.finditer(text):
        segment_id = int(match.group(1))
        pose_values = match.group(2).split()
        if len(pose_values) < 2:
            continue
        segments.append((segment_id, float(pose_values[0]), float(pose_values[1])))

    segments.sort(key=lambda item: item[0])
    return [(x, y) for _, x, y in segments]


def build_cumulative_distances(points: list[tuple[float, float]]) -> list[float]:
    if not points:
        return []

    cumulative = [0.0]
    for idx in range(len(points) - 1):
        ax, ay = points[idx]
        bx, by = points[idx + 1]
        cumulative.append(cumulative[-1] + math.hypot(bx - ax, by - ay))
    return cumulative


def load_robot_initial_pose(
    sdf_path: Path,
    model_name: str,
    default_x: float,
    default_y: float,
    default_yaw: float,
) -> tuple[float, float, float]:
    if not sdf_path.exists():
        return default_x, default_y, default_yaw

    text = sdf_path.read_text(encoding="utf-8")
    pattern = re.compile(
        rf'<model name="{re.escape(model_name)}">.*?<pose>([^<]+)</pose>',
        re.DOTALL,
    )
    match = pattern.search(text)
    if not match:
        return default_x, default_y, default_yaw

    pose_values = match.group(1).split()
    if len(pose_values) < 6:
        return default_x, default_y, default_yaw

    return float(pose_values[0]), float(pose_values[1]), float(pose_values[5])


def load_robot_initial_pose_3d(
    sdf_path: Path,
    model_name: str,
    default_x: float,
    default_y: float,
    default_z: float,
    default_yaw: float,
) -> tuple[float, float, float, float]:
    if not sdf_path.exists():
        return default_x, default_y, default_z, default_yaw

    text = sdf_path.read_text(encoding="utf-8")
    pattern = re.compile(
        rf'<model name="{re.escape(model_name)}">.*?<pose>([^<]+)</pose>',
        re.DOTALL,
    )
    match = pattern.search(text)
    if not match:
        return default_x, default_y, default_z, default_yaw

    pose_values = match.group(1).split()
    if len(pose_values) < 6:
        return default_x, default_y, default_z, default_yaw

    return (
        float(pose_values[0]),
        float(pose_values[1]),
        float(pose_values[2]),
        float(pose_values[5]),
    )


def sample_polyline_at_distance(
    points: list[tuple[float, float]],
    cumulative: list[float],
    distance_m: float,
) -> Optional[tuple[float, float, float, int]]:
    if len(points) < 2 or len(cumulative) != len(points):
        return None

    target = clamp(distance_m, 0.0, cumulative[-1])
    for idx in range(len(points) - 1):
        start_dist = cumulative[idx]
        end_dist = cumulative[idx + 1]
        if target > end_dist and idx != len(points) - 2:
            continue

        ax, ay = points[idx]
        bx, by = points[idx + 1]
        seg_len = max(1e-6, end_dist - start_dist)
        t = clamp((target - start_dist) / seg_len, 0.0, 1.0)
        x = ax + (t * (bx - ax))
        y = ay + (t * (by - ay))
        yaw = math.atan2(by - ay, bx - ax)
        return x, y, yaw, idx

    ax, ay = points[-2]
    bx, by = points[-1]
    return bx, by, math.atan2(by - ay, bx - ax), len(points) - 2


def project_on_polyline(
    x: float,
    y: float,
    points: list[tuple[float, float]],
    cumulative: list[float],
) -> PolylineProjection:
    if len(points) < 2:
        return PolylineProjection(
            distance_m=0.0,
            lateral_error_m=0.0,
            point_x=x,
            point_y=y,
            tangent_yaw=0.0,
            segment_index=0,
        )

    best = PolylineProjection(
        distance_m=0.0,
        lateral_error_m=float("inf"),
        point_x=x,
        point_y=y,
        tangent_yaw=0.0,
        segment_index=0,
    )

    for idx in range(len(points) - 1):
        ax, ay = points[idx]
        bx, by = points[idx + 1]
        vx = bx - ax
        vy = by - ay
        seg_len_sq = (vx * vx) + (vy * vy)
        if seg_len_sq <= 1e-9:
            continue

        t = ((x - ax) * vx + (y - ay) * vy) / seg_len_sq
        t = clamp(t, 0.0, 1.0)
        proj_x = ax + (t * vx)
        proj_y = ay + (t * vy)
        dx = x - proj_x
        dy = y - proj_y
        lateral_error = math.hypot(dx, dy)

        if lateral_error < best.lateral_error_m:
            seg_len = math.sqrt(seg_len_sq)
            best = PolylineProjection(
                distance_m=cumulative[idx] + (t * seg_len),
                lateral_error_m=lateral_error,
                point_x=proj_x,
                point_y=proj_y,
                tangent_yaw=math.atan2(vy, vx),
                segment_index=idx,
            )

    return best


def integrate_body_twist(
    x: float,
    y: float,
    yaw: float,
    linear_x: float,
    linear_y: float,
    angular_z: float,
    dt: float,
    linear_x_scale: float = -1.0,
    linear_y_scale: float = 1.0,
) -> tuple[float, float, float]:
    vx = linear_x_scale * linear_x
    vy = linear_y_scale * linear_y
    yaw_next = yaw + (angular_z * dt)
    x_next = x + ((vx * math.cos(yaw_next)) - (vy * math.sin(yaw_next))) * dt
    y_next = y + ((vx * math.sin(yaw_next)) + (vy * math.cos(yaw_next))) * dt
    return x_next, y_next, yaw_next


def make_repair_segment_sdf(
    model_name: str,
    radius: float,
    length: float,
    color_rgba: tuple[float, float, float, float],
) -> str:
    r, g, b, a = color_rgba
    return (
        "<sdf version='1.10'>"
        f"<model name='{model_name}'>"
        "<static>true</static>"
        "<link name='link'>"
        "<visual name='visual'>"
        "<pose>0 0 0 0 1.5708 0</pose>"
        f"<geometry><cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder></geometry>"
        "<material>"
        f"<ambient>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</ambient>"
        f"<diffuse>{r:.4f} {g:.4f} {b:.4f} {a:.4f}</diffuse>"
        "</material>"
        "</visual>"
        "</link>"
        "</model>"
        "</sdf>"
    )


def spawn_sdf_model(
    sdf_xml: str,
    x: float,
    y: float,
    z: float,
    yaw: float,
    preferred_worlds: Optional[list[str]] = None,
    timeout_ms: int = 3000,
) -> tuple[bool, str]:
    worlds = list(preferred_worlds or [])
    for fallback in ("underwater_world", "default", "ocean_world"):
        if fallback not in worlds:
            worlds.append(fallback)

    half_yaw = 0.5 * yaw
    req_text = (
        f"sdf: {json.dumps(sdf_xml)} "
        "allow_renaming: true "
        "pose { "
        f"position {{ x: {x:.5f} y: {y:.5f} z: {z:.5f} }} "
        f"orientation {{ x: 0 y: 0 z: {math.sin(half_yaw):.8f} w: {math.cos(half_yaw):.8f} }} "
        "}"
    )

    last_error = "spawn request not executed"
    for world in worlds:
        cmd = [
            "gz",
            "service",
            "-s",
            f"/world/{world}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            req_text,
        ]
        completed = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False,
        )
        combined_output = (completed.stdout or "") + (completed.stderr or "")
        if completed.returncode == 0 and "data: true" in combined_output.lower():
            return True, world
        last_error = combined_output.strip() or f"returncode={completed.returncode}"

    return False, last_error
