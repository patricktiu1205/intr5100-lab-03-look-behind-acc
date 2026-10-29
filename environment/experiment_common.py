# -*- coding: utf-8 -*-
"""Shared infrastructure for INTR5100 experiments 1--3.

The goal of this module is to make safety/reliability behavior identical across
experiments while leaving only the vehicle topology/controller choice to the
small experiment entry files.
"""
from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence

from idm_controller import IDMController, IDMParams
from route_map_utils import (
    RouteAnalyzer,
    RouteSection,
    control_section_index,
    validate_route_for_experiments,
    visualize_route_zones,
)
from vehicle_io import AsyncVehicleCommandClient, VehicleState, VehicleStateClient


STATE_PORT = 10200
CONTROL_PORT = 10100


@dataclass
class SafetyConfig:
    vehicle_length: float = 250.0
    emergency_spacing: float = 80.0
    center_sensor_threshold: float = 0.15
    use_center_sensor: bool = True
    max_command_velocity: float = 100.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def bumper_spacing(
    route: RouteAnalyzer,
    front_s: float,
    rear_s: float,
    vehicle_length: float,
) -> tuple[float, float]:
    """Return (center/path gap, bumper spacing) using one convention everywhere."""
    gap = route.forward_gap(front_s, rear_s)
    spacing = max(0.0, gap - float(vehicle_length))
    return gap, spacing


def apply_safety_shield(
    desired_velocity: float,
    rear_state: VehicleState,
    spacing: float,
    config: SafetyConfig,
) -> tuple[float, bool, str]:
    """Apply TA-owned safety rules outside the student/IDM controller."""
    v_cmd = clamp(desired_velocity, 0.0, config.max_command_velocity)
    if spacing < config.emergency_spacing:
        return 0.0, True, "spacing_emergency"
    if config.use_center_sensor:
        center = rear_state.get_float("center_distance", 0.0)
        if 0.0 < center < config.center_sensor_threshold:
            return 0.0, True, "center_sensor_emergency"
    return v_cmd, False, ""


class TrajectoryRecorder:
    """Sequence-deduplicated telemetry and command/event storage."""

    def __init__(self) -> None:
        self.rows: List[Dict[str, Any]] = []
        self.command_rows: List[Dict[str, Any]] = []
        self.event_rows: List[Dict[str, Any]] = []
        self.latest_seq_seen: Dict[str, int] = {}

    def reset(self) -> None:
        self.rows.clear()
        self.command_rows.clear()
        self.event_rows.clear()
        self.latest_seq_seen.clear()

    def record_state(
        self,
        role: str,
        state: VehicleState,
        route: RouteAnalyzer,
        run_start_wall: float,
    ) -> bool:
        key = f"{role}:{state.host}"
        if self.latest_seq_seen.get(key) == state.seq:
            return False
        self.latest_seq_seen[key] = state.seq
        proj = route.project_xy(state.x, state.y)
        row: Dict[str, Any] = {
            "role": role,
            "host": state.host,
            "recv_time": state.recv_time,
            "t_from_start": state.recv_time - run_start_wall,
            "seq": state.seq,
            "x": state.x,
            "y": state.y,
            "heading": state.heading,
            "velocity": state.velocity,
            "s": proj.s,
            "segment_i": proj.segment_i,
            "lateral_error": proj.lateral_error,
            "route_distance": proj.distance,
            "zone_type": proj.zone_type,
            "corner_id": proj.corner_id,
            "corner_name": proj.corner_name,
            "raw_json": json.dumps(state.data, ensure_ascii=False),
        }
        for k in [
            "left_distance", "center_distance", "right_distance", "control_mode",
            "path_index", "position_index", "target_index", "map_version",
            "map_load", "pose_ready", "pose_age",
        ]:
            if k in state.data:
                row[k] = state.data[k]
        self.rows.append(row)
        return True

    @staticmethod
    def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        if not rows:
            path.write_text("", encoding="utf-8")
            return
        keys = sorted({k for row in rows for k in row.keys()})
        preferred = [
            "role", "host", "recv_time", "t_from_start", "seq", "x", "y", "heading", "velocity",
            "s", "segment_i", "lateral_error", "zone_type", "corner_id", "corner_name",
            "mode", "reason", "front_role", "rear_role", "gap", "spacing",
            "desired_velocity", "v_cmd", "a_idm", "emergency",
        ]
        fieldnames = [k for k in preferred if k in keys] + [k for k in keys if k not in preferred]
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)


class LeaderPerturbationManager:
    """Three deterministic one-shot perturbations on the three selected straights.

    Each event captures the *actual/current command* when it first triggers, so
    no discontinuity is introduced by a stale hard-coded ``from_speed``. Events
    are one-shot; on later laps they remain completed instead of jumping to the
    event target.
    """

    def __init__(self) -> None:
        self.sections: List[RouteSection] = []
        self.events: List[Dict[str, Any]] = []
        self.current_command: Optional[float] = None

    def reset(self, sections: Sequence[RouteSection], base_speed: float) -> None:
        self.sections = list(sections)
        targets = [
            (base_speed + 15.0, 8.0, "acceleration"),
            (max(base_speed - 15.0, 20.0), 8.0, "braking"),
            (max(base_speed - 20.0, 20.0), 10.0, "stop_go"),
        ]
        self.events = []
        for i, section in enumerate(self.sections[:3]):
            target, duration, kind = targets[i]
            self.events.append({
                "id": i + 1,
                "section": section,
                "target_speed": float(target),
                "duration": float(duration),
                "type": kind,
                "trigger_time": None,
                "start_speed": None,
                "completed": False,
                "was_inside": False,
            })
        self.current_command = float(base_speed)

    def step(
        self,
        *,
        now_mono: float,
        leader_s: float,
        observed_velocity: float,
        base_speed: float,
        loop_length: float,
    ) -> tuple[float, Optional[Dict[str, Any]]]:
        if not self.events:
            self.current_command = float(base_speed)
            return float(base_speed), None

        active: Optional[Dict[str, Any]] = None
        for event in self.events:
            inside = event["section"].contains(leader_s, loop_length)
            if inside and not event["completed"]:
                active = event
                if event["trigger_time"] is None:
                    event["trigger_time"] = now_mono
                    event["start_speed"] = float(
                        self.current_command if self.current_command is not None else observed_velocity
                    )
                elapsed = max(0.0, now_mono - float(event["trigger_time"]))
                ratio = min(1.0, elapsed / max(float(event["duration"]), 1e-6))
                start_v = float(event["start_speed"])
                target_v = float(event["target_speed"])
                cmd = start_v + ratio * (target_v - start_v)
                self.current_command = cmd
                event["was_inside"] = True
                return cmd, {
                    "event_id": event["id"],
                    "event_type": event["type"],
                    "leader_s": leader_s,
                    "elapsed": elapsed,
                    "command_speed": cmd,
                }

            if event["was_inside"] and not inside and event["trigger_time"] is not None:
                # Event is complete once the vehicle has left its section.
                event["completed"] = True
                event["was_inside"] = False

        self.current_command = float(base_speed)
        return float(base_speed), None


def build_route(
    map_path: str | Path,
    line_name: str,
    *,
    threshold_deg: float,
    window_points: int,
    max_corners: int,
    min_corner_gap: float,
    expected_corners: int,
    output_dir: str | Path,
) -> tuple[RouteAnalyzer, List[RouteSection], List[str]]:
    route = RouteAnalyzer.from_map(
        map_path,
        line_name=line_name,
        threshold_deg=threshold_deg,
        window_points=window_points,
        max_corners=max_corners,
        min_corner_gap=min_corner_gap,
    )
    ok, messages, sections = validate_route_for_experiments(
        route,
        expected_corners=expected_corners,
        detector_max_corners=max_corners,
        required_control_sections=3,
    )

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    route.export_zones(
        out_dir / f"corner_zones_{line_name}.json",
        map_name=Path(map_path).stem,
        line_name=line_name,
    )
    try:
        visualize_route_zones(
            route,
            out_dir / f"route_zones_{line_name}.png",
            title=f"{Path(map_path).stem} {line_name} zones",
        )
    except Exception as exc:
        messages.append(f"WARNING: route visualization could not be written: {exc}")
    if not ok:
        raise ValueError("\n".join(messages))
    return route, sections, messages


def make_idm(params: IDMParams) -> IDMController:
    return IDMController(params)


def same_control_section(
    sections: Sequence[RouteSection],
    route: RouteAnalyzer,
    s1: float,
    s2: float,
) -> bool:
    i1 = control_section_index(sections, s1, route.loop_length)
    i2 = control_section_index(sections, s2, route.loop_length)
    return i1 >= 0 and i1 == i2


def in_control_section(sections: Sequence[RouteSection], route: RouteAnalyzer, s: float) -> bool:
    return control_section_index(sections, s, route.loop_length) >= 0
