# -*- coding: utf-8 -*-
"""Shared Qt GUI for Experiments 2 and 3."""
from __future__ import annotations

import json
import sys
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence

from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    import pyqtgraph as pg
except Exception:  # pragma: no cover
    pg = None

from experiment_common import (
    CONTROL_PORT,
    STATE_PORT,
    LeaderPerturbationManager,
    SafetyConfig,
    TrajectoryRecorder,
    apply_safety_shield,
    build_route,
    bumper_spacing,
    in_control_section,
    same_control_section,
)
from idm_controller import IDMController, IDMParams
from route_map_utils import RouteAnalyzer, RouteSection
from vehicle_io import AsyncVehicleCommandClient, VehicleState, VehicleStateClient


class ThreeVehicleExperimentGUI(QMainWindow):
    """One implementation for Exp2/Exp3; subclasses only define topology."""

    experiment_name = "experiment"
    window_title = "Three Vehicle Experiment"
    roles: Sequence[str] = ("leader", "middle", "student")
    idm_front_role = "leader"
    idm_ego_role = "middle"
    student_role = "student"
    student_front_role = "middle"

    def __init__(self, student_control: Callable[..., float]) -> None:
        super().__init__()
        self.student_control = student_control
        self.setWindowTitle(self.window_title)
        self.resize(1240, 820)

        self.route: Optional[RouteAnalyzer] = None
        self.control_sections: List[RouteSection] = []
        self.route_valid = False
        self.state_clients: Dict[str, VehicleStateClient] = {}
        self.command_clients: Dict[str, AsyncVehicleCommandClient] = {}

        self.recording = False
        self.current_run_dir: Optional[Path] = None
        self.run_start_wall = 0.0
        self.run_start_mono = 0.0
        self.last_control_mono: Optional[float] = None
        self.idm_controller: Optional[IDMController] = None
        self.recorder = TrajectoryRecorder()
        self.leader_manager = LeaderPerturbationManager()
        self.last_modes: Dict[str, str] = {}

        self._init_ui()
        self._load_default_route()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._timer_tick)
        self.ui_timer.start(100)

    # ------------------------------------------------------------------ UI
    def _init_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        common = QGroupBox("Common settings")
        grid = QGridLayout(common)
        layout.addWidget(common)

        self.ip_edits: Dict[str, QLineEdit] = {role: QLineEdit("") for role in self.roles}
        self.map_path = QLineEdit(str(Path(__file__).resolve().parent / "6e68_map.yaml"))
        self.output_dir = QLineEdit(str(Path(__file__).resolve().parent / "runs"))
        self.line_name = QLineEdit("line1")

        self.leader_speed = QSpinBox(); self.leader_speed.setRange(0, 9999); self.leader_speed.setValue(40)
        self.control_period_ms = QSpinBox(); self.control_period_ms.setRange(50, 1000); self.control_period_ms.setValue(150)
        self.enable_perturb = QCheckBox("Enable leader acceleration/braking events"); self.enable_perturb.setChecked(True)

        # Corner detector is visible/tunable instead of silently relying on defaults.
        self.corner_threshold = QDoubleSpinBox(); self.corner_threshold.setRange(1, 180); self.corner_threshold.setValue(30.0)
        self.corner_window = QSpinBox(); self.corner_window.setRange(1, 200); self.corner_window.setValue(8)
        self.max_corners = QSpinBox(); self.max_corners.setRange(1, 64); self.max_corners.setValue(8)
        self.min_corner_gap = QDoubleSpinBox(); self.min_corner_gap.setRange(0, 100000); self.min_corner_gap.setValue(500.0)
        self.expected_corners = QSpinBox(); self.expected_corners.setRange(0, 64); self.expected_corners.setValue(0)
        self.expected_corners.setToolTip("0 = no exact-count requirement; still rejects zero corners and detector saturation")

        r = 0
        for i, role in enumerate(self.roles):
            grid.addWidget(QLabel(f"{role.capitalize()} IP"), r, 2 * i)
            grid.addWidget(self.ip_edits[role], r, 2 * i + 1)
        r += 1
        grid.addWidget(QLabel("Map YAML"), r, 0); grid.addWidget(self.map_path, r, 1, 1, 4)
        self.browse_btn = QPushButton("Browse map"); self.browse_btn.clicked.connect(self._browse_map); grid.addWidget(self.browse_btn, r, 5)
        r += 1
        grid.addWidget(QLabel("Output dir"), r, 0); grid.addWidget(self.output_dir, r, 1, 1, 2)
        grid.addWidget(QLabel("Line"), r, 3); grid.addWidget(self.line_name, r, 4)
        self.load_map_btn = QPushButton("Load / validate map"); self.load_map_btn.clicked.connect(self._load_route_from_ui); grid.addWidget(self.load_map_btn, r, 5)
        r += 1
        grid.addWidget(QLabel("Leader speed"), r, 0); grid.addWidget(self.leader_speed, r, 1)
        grid.addWidget(QLabel("Control period ms"), r, 2); grid.addWidget(self.control_period_ms, r, 3)
        grid.addWidget(self.enable_perturb, r, 4, 1, 2)
        r += 1
        grid.addWidget(QLabel("Corner threshold deg"), r, 0); grid.addWidget(self.corner_threshold, r, 1)
        grid.addWidget(QLabel("Corner window points"), r, 2); grid.addWidget(self.corner_window, r, 3)
        grid.addWidget(QLabel("Max corners"), r, 4); grid.addWidget(self.max_corners, r, 5)
        r += 1
        grid.addWidget(QLabel("Min corner gap"), r, 0); grid.addWidget(self.min_corner_gap, r, 1)
        grid.addWidget(QLabel("Expected corners (0=auto)"), r, 2); grid.addWidget(self.expected_corners, r, 3)

        # IDM + safety parameters share the exact same ranges/convention in Exp2/Exp3.
        params = QGroupBox("IDM and safety")
        pgrid = QGridLayout(params)
        layout.addWidget(params)
        self.idm_v0 = QDoubleSpinBox(); self.idm_v0.setRange(0, 9999); self.idm_v0.setValue(70)
        self.idm_T = QDoubleSpinBox(); self.idm_T.setRange(0, 20); self.idm_T.setDecimals(2); self.idm_T.setValue(1.2)
        self.idm_s0 = QDoubleSpinBox(); self.idm_s0.setRange(0, 10000); self.idm_s0.setValue(250)
        self.idm_a = QDoubleSpinBox(); self.idm_a.setRange(0, 1000); self.idm_a.setValue(20)
        self.idm_b = QDoubleSpinBox(); self.idm_b.setRange(0, 1000); self.idm_b.setValue(30)
        self.idm_delta = QDoubleSpinBox(); self.idm_delta.setRange(1, 10); self.idm_delta.setValue(4)
        self.vehicle_length = QDoubleSpinBox(); self.vehicle_length.setRange(0, 5000); self.vehicle_length.setValue(250)
        self.emergency_spacing = QDoubleSpinBox(); self.emergency_spacing.setRange(0, 5000); self.emergency_spacing.setValue(80)
        self.sensor_emergency = QDoubleSpinBox(); self.sensor_emergency.setRange(0, 10); self.sensor_emergency.setDecimals(3); self.sensor_emergency.setValue(0.15)
        self.max_command_velocity = QDoubleSpinBox(); self.max_command_velocity.setRange(0, 9999); self.max_command_velocity.setValue(100)
        self.use_center_sensor = QCheckBox("Use center-distance emergency stop"); self.use_center_sensor.setChecked(True)

        vals = [
            ("v0", self.idm_v0), ("T", self.idm_T), ("s0", self.idm_s0),
            ("a", self.idm_a), ("b", self.idm_b), ("delta", self.idm_delta),
            ("vehicle length", self.vehicle_length), ("emergency spacing", self.emergency_spacing),
            ("sensor emergency", self.sensor_emergency), ("max command velocity", self.max_command_velocity),
        ]
        for i, (name, widget) in enumerate(vals):
            rr, cc = divmod(i, 5)
            pgrid.addWidget(QLabel(name), rr * 2, cc)
            pgrid.addWidget(widget, rr * 2 + 1, cc)
        pgrid.addWidget(self.use_center_sensor, 4, 0, 1, 5)

        middle = QHBoxLayout()
        layout.addLayout(middle, stretch=1)
        controls = QVBoxLayout(); middle.addLayout(controls, stretch=0)
        self.connect_btn = QPushButton("Connect state / command"); self.connect_btn.clicked.connect(self._connect_clients)
        self.check_btn = QPushButton("Check pose"); self.check_btn.clicked.connect(self._check_pose)
        self.start_btn = QPushButton("Start experiment"); self.start_btn.clicked.connect(self._start_experiment)
        self.stop_btn = QPushButton("Stop & save"); self.stop_btn.clicked.connect(self._stop_and_save); self.stop_btn.setEnabled(False)
        self.emergency_btn = QPushButton("EMERGENCY STOP ALL"); self.emergency_btn.clicked.connect(self._emergency_stop_all)
        for w in [self.connect_btn, self.check_btn, self.start_btn, self.stop_btn, self.emergency_btn]: controls.addWidget(w)
        controls.addStretch(1)

        right = QVBoxLayout(); middle.addLayout(right, stretch=1)
        self.status_label = QLabel("Ready"); self.status_label.setAlignment(Qt.AlignmentFlag.AlignLeft); right.addWidget(self.status_label)

        if pg is not None:
            self.plot = pg.PlotWidget(); self.plot.setBackground("w"); self.plot.setAspectLocked(True); self.plot.showGrid(x=True, y=True, alpha=0.25); self.plot.addLegend()
            self.route_curve = self.plot.plot(pen=pg.mkPen(width=1.5), name="route")
            self.trace_curves: Dict[str, Any] = {}
            self.dots: Dict[str, Any] = {}
            symbols = ["o", "s", "t"]
            for i, role in enumerate(self.roles):
                self.trace_curves[role] = self.plot.plot(pen=pg.mkPen(width=2), name=f"{role} trace")
                self.dots[role] = self.plot.plot(pen=None, symbol=symbols[i % len(symbols)], symbolSize=11, name=role)
            right.addWidget(self.plot, stretch=3)
        else:
            self.plot = None
            right.addWidget(QLabel("pyqtgraph not installed; live monitor disabled."), stretch=1)

        self.log_box = QTextEdit(); self.log_box.setReadOnly(True); self.log_box.setMinimumHeight(150); right.addWidget(self.log_box, stretch=1)

    # ---------------------------------------------------------------- route
    def _browse_map(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Select map YAML", str(Path.cwd()), "YAML (*.yaml *.yml);;All files (*)")
        if path: self.map_path.setText(path)

    def _load_default_route(self) -> None:
        try:
            self._load_route_from_ui()
        except Exception as exc:
            self.route_valid = False
            self._log(f"Default route not loaded/validated: {exc}")

    def _load_route_from_ui(self) -> None:
        if self.recording:
            QMessageBox.warning(self, "Experiment running", "Stop the experiment before reloading the route.")
            return
        path = Path(self.map_path.text()).expanduser()
        if not path.exists():
            raise FileNotFoundError(path)
        try:
            route, sections, messages = build_route(
                path,
                self.line_name.text().strip() or "line1",
                threshold_deg=self.corner_threshold.value(),
                window_points=self.corner_window.value(),
                max_corners=self.max_corners.value(),
                min_corner_gap=self.min_corner_gap.value(),
                expected_corners=self.expected_corners.value(),
                output_dir=self.output_dir.text(),
            )
        except Exception as exc:
            self.route_valid = False
            self._log(str(exc))
            QMessageBox.warning(self, "Route validation failed", str(exc))
            return
        self.route = route
        self.control_sections = sections
        self.route_valid = True
        for msg in messages: self._log(msg)
        for i, sec in enumerate(self.control_sections, start=1):
            self._log(f"Control straight {i}: start={sec.start_s:.1f}, end={sec.end_s:.1f}, length={sec.length:.1f}, wraps={sec.wraps}")
        self._draw_route()

    def _draw_route(self) -> None:
        if self.plot is None or self.route is None: return
        xs = [p.x for p in self.route.points] + [self.route.points[0].x]
        ys = [p.y for p in self.route.points] + [self.route.points[0].y]
        self.route_curve.setData(xs, ys)
        self.plot.enableAutoRange()

    # ------------------------------------------------------------- connection
    def _disconnect_clients(self) -> None:
        for c in self.state_clients.values(): c.stop()
        for c in self.command_clients.values(): c.close()
        self.state_clients = {}; self.command_clients = {}

    def _connect_clients(self) -> None:
        if self.recording:
            QMessageBox.warning(self, "Experiment running", "Stop the experiment before reconnecting vehicles.")
            return
        ips = {role: self.ip_edits[role].text().strip() for role in self.roles}
        missing = [role for role, ip in ips.items() if not ip]
        if missing:
            QMessageBox.warning(self, "Missing IP", "Enter IP for: " + ", ".join(missing))
            return
        self._disconnect_clients()
        for role, ip in ips.items():
            sc = VehicleStateClient(role, ip, STATE_PORT); sc.start(); self.state_clients[role] = sc
            self.command_clients[role] = AsyncVehicleCommandClient(ip, CONTROL_PORT, timeout=0.25)
        self._log("State clients and asynchronous command workers started.")

    def _latest(self, role: str) -> Optional[VehicleState]:
        c = self.state_clients.get(role)
        return c.latest() if c else None

    def _check_pose(self) -> None:
        lines = []
        for role in self.roles:
            state = self._latest(role); client = self.state_clients.get(role)
            if state and self.route:
                p = self.route.project_xy(state.x, state.y)
                lines.append(f"{role}: x={state.x:.1f}, y={state.y:.1f}, v={state.velocity:.1f}, s={p.s:.1f}, zone={p.zone_type}")
            elif client:
                lines.append(f"{role}: {client.status}, err={client.last_error}")
            else:
                lines.append(f"{role}: not connected")
        QMessageBox.information(self, "Pose check", "\n".join(lines)); self._log(" | ".join(lines))

    def _ensure_ready(self) -> bool:
        if self.recording:
            QMessageBox.warning(self, "Already running", "Experiment is already running."); return False
        if self.route is None or not self.route_valid:
            QMessageBox.warning(self, "Route not ready", "Load and validate the map first."); return False
        if any(role not in self.command_clients for role in self.roles):
            QMessageBox.warning(self, "Not connected", "Connect all three vehicles first."); return False
        missing_state = [role for role in self.roles if self._latest(role) is None]
        if missing_state:
            QMessageBox.warning(self, "No telemetry", "No state received yet from: " + ", ".join(missing_state)); return False
        return True

    # ------------------------------------------------------------- parameters
    def _idm_params(self) -> IDMParams:
        return IDMParams(
            v0=self.idm_v0.value(), T=self.idm_T.value(), s0=self.idm_s0.value(),
            a=self.idm_a.value(), b=self.idm_b.value(), delta=self.idm_delta.value(),
            vehicle_length=self.vehicle_length.value(), emergency_spacing=self.emergency_spacing.value(),
        )

    def _safety_config(self) -> SafetyConfig:
        return SafetyConfig(
            vehicle_length=self.vehicle_length.value(),
            emergency_spacing=self.emergency_spacing.value(),
            center_sensor_threshold=self.sensor_emergency.value(),
            use_center_sensor=self.use_center_sensor.isChecked(),
            max_command_velocity=self.max_command_velocity.value(),
        )

    # ------------------------------------------------------------ run control
    def _new_run(self) -> None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_run_dir = Path(self.output_dir.text()).expanduser() / f"{self.experiment_name}_{stamp}"
        self.current_run_dir.mkdir(parents=True, exist_ok=True)
        self.recorder.reset(); self.run_start_wall = time.time(); self.run_start_mono = time.monotonic(); self.last_control_mono = None
        self.last_modes = {}

    def _start_experiment(self) -> None:
        if not self._ensure_ready(): return
        self._new_run()
        self.idm_controller = IDMController(self._idm_params())
        self.leader_manager.reset(self.control_sections, float(self.leader_speed.value()))
        for role in self.roles:
            self.command_clients[role].start_line1(int(self.leader_speed.value()))
        self.recording = True
        self.start_btn.setEnabled(False); self.stop_btn.setEnabled(True)
        self._log(f"{self.experiment_name} started. Vehicle order: {' -> '.join(self.roles)}")

    def _stop_and_save(self) -> None:
        # Safe on ordinary misclicks.
        if not self.recording:
            self._log("Stop requested while idle; no run to save.")
            return
        self.recording = False
        for c in self.command_clients.values(): c.stop_vehicle()
        self._save_run()
        self.start_btn.setEnabled(True); self.stop_btn.setEnabled(False)

    def _emergency_stop_all(self) -> None:
        was_running = self.recording
        self.recording = False
        for c in self.command_clients.values(): c.emergency_stop()
        self._log("EMERGENCY STOP queued for all vehicles.")
        if was_running and self.current_run_dir is not None:
            self._save_run()
        self.start_btn.setEnabled(True); self.stop_btn.setEnabled(False)

    # --------------------------------------------------------------- controls
    def _leader_behavior_step(self) -> None:
        if not self.recording or self.route is None or not self.enable_perturb.isChecked(): return
        leader = self._latest("leader")
        if not leader: return
        p = self.route.project_xy(leader.x, leader.y)
        cmd, event_row = self.leader_manager.step(
            now_mono=time.monotonic(), leader_s=p.s, observed_velocity=leader.velocity,
            base_speed=float(self.leader_speed.value()), loop_length=self.route.loop_length,
        )
        self.command_clients["leader"].set_velocity(int(round(cmd)))
        if event_row is not None:
            event_row = dict(event_row)
            event_row["time"] = time.time(); event_row["t_from_start"] = time.time() - self.run_start_wall
            self.recorder.event_rows.append(event_row)

    def _student_args(self, states: Dict[str, VehicleState]) -> tuple[Any, ...]:
        """Subclasses define the argument order expected by their student controller."""
        raise NotImplementedError

    def _student_immediate_front_role(self) -> str:
        return self.student_front_role

    def _control_step(self) -> None:
        if not self.recording or self.route is None or self.idm_controller is None: return
        now_mono = time.monotonic()
        period = self.control_period_ms.value() / 1000.0
        if self.last_control_mono is not None and now_mono - self.last_control_mono < period: return
        dt = period if self.last_control_mono is None else max(1e-3, now_mono - self.last_control_mono)
        self.last_control_mono = now_mono

        states = {role: self._latest(role) for role in self.roles}
        if any(s is None for s in states.values()): return
        states = {k: v for k, v in states.items() if v is not None}
        proj = {role: self.route.project_xy(s.x, s.y) for role, s in states.items()}
        safety = self._safety_config()

        # IDM vehicle: only active when front and ego are on the same selected straight.
        front_role, ego_role = self.idm_front_role, self.idm_ego_role
        gap, spacing = bumper_spacing(self.route, proj[front_role].s, proj[ego_role].s, safety.vehicle_length)
        if same_control_section(self.control_sections, self.route, proj[front_role].s, proj[ego_role].s):
            a_idm = self.idm_controller.acceleration(states[ego_role].velocity, states[front_role].velocity, spacing)
            desired_idm = self.idm_controller.next_velocity(states[ego_role].velocity, states[front_role].velocity, spacing, dt, v_max=self.idm_v0.value())
            idm_mode = "idm_straight"; idm_reason = "same_control_straight"
        else:
            desired_idm = states["leader"].velocity
            self.idm_controller.last_cmd_v = float(desired_idm)
            a_idm = None; idm_mode = "leader_follow_noncontrol"; idm_reason = proj[ego_role].zone_type
        v_idm, emergency, emergency_reason = apply_safety_shield(desired_idm, states[ego_role], spacing, safety)
        if emergency: idm_reason = emergency_reason
        self.command_clients[ego_role].set_velocity(int(round(v_idm)))

        # Student vehicle: algorithm is active only on selected straight and, for
        # predictable transitions, when its immediate front is on the same one.
        srole = self.student_role; sfrole = self._student_immediate_front_role()
        s_gap, s_spacing = bumper_spacing(self.route, proj[sfrole].s, proj[srole].s, safety.vehicle_length)
        if same_control_section(self.control_sections, self.route, proj[sfrole].s, proj[srole].s):
            try:
                desired_student = float(self.student_control(*self._student_args(states), history=self.recorder.rows))
                student_mode = "student_straight"; student_reason = "same_control_straight"
            except Exception as exc:
                desired_student = states["leader"].velocity
                student_mode = "student_fallback"; student_reason = f"controller_error:{exc}"
                self._log(f"Student controller error; using leader velocity: {exc}")
        else:
            desired_student = states["leader"].velocity
            student_mode = "leader_follow_noncontrol"; student_reason = proj[srole].zone_type
        v_student, s_emergency, s_emergency_reason = apply_safety_shield(desired_student, states[srole], s_spacing, safety)
        if s_emergency: student_reason = s_emergency_reason
        self.command_clients[srole].set_velocity(int(round(v_student)))

        now_wall = time.time()
        self.recorder.command_rows.extend([
            {
                "time": now_wall, "t_from_start": now_wall - self.run_start_wall,
                "role": ego_role, "mode": idm_mode, "reason": idm_reason,
                "front_role": front_role, "rear_role": ego_role,
                "gap": gap, "spacing": spacing, "desired_velocity": desired_idm,
                "v_cmd": v_idm, "a_idm": a_idm, "dt": dt, "emergency": int(emergency),
            },
            {
                "time": now_wall, "t_from_start": now_wall - self.run_start_wall,
                "role": srole, "mode": student_mode, "reason": student_reason,
                "front_role": sfrole, "rear_role": srole,
                "gap": s_gap, "spacing": s_spacing, "desired_velocity": desired_student,
                "v_cmd": v_student, "a_idm": None, "dt": dt, "emergency": int(s_emergency),
            },
        ])
        self.last_modes[ego_role] = idm_mode; self.last_modes[srole] = student_mode

    # -------------------------------------------------------------- telemetry
    def _record_states(self) -> None:
        if not self.recording or self.route is None: return
        for role in self.roles:
            state = self._latest(role)
            if state: self.recorder.record_state(role, state, self.route, self.run_start_wall)

    def _timer_tick(self) -> None:
        self._update_status(); self._update_plot()
        if self.recording:
            self._record_states(); self._leader_behavior_step(); self._control_step()

    # ---------------------------------------------------------------- saving
    def _save_run(self) -> None:
        if self.current_run_dir is None: return
        TrajectoryRecorder.write_csv(self.current_run_dir / "trajectory.csv", self.recorder.rows)
        for role in self.roles:
            TrajectoryRecorder.write_csv(self.current_run_dir / f"{role}_trajectory.csv", [r for r in self.recorder.rows if r.get("role") == role])
        TrajectoryRecorder.write_csv(self.current_run_dir / "commands.csv", self.recorder.command_rows)
        TrajectoryRecorder.write_csv(self.current_run_dir / "leader_events.csv", self.recorder.event_rows)
        straight_rows = [r for r in self.recorder.rows if in_control_section(self.control_sections, self.route, float(r.get("s", 0.0)))] if self.route else []
        TrajectoryRecorder.write_csv(self.current_run_dir / "control_straight_trajectory.csv", straight_rows)

        metadata = {
            "experiment": self.experiment_name,
            "vehicle_order": list(self.roles),
            "ips": {role: self.ip_edits[role].text().strip() for role in self.roles},
            "map_path": self.map_path.text(), "line_name": self.line_name.text(),
            "leader_speed": self.leader_speed.value(), "control_period_ms": self.control_period_ms.value(),
            "idm_params": asdict(self._idm_params()), "safety": asdict(self._safety_config()),
            "corner_detection": {
                "threshold_deg": self.corner_threshold.value(), "window_points": self.corner_window.value(),
                "max_corners": self.max_corners.value(), "min_corner_gap": self.min_corner_gap.value(),
                "expected_corners": self.expected_corners.value(),
            },
            "control_sections": [s.to_dict() for s in self.control_sections],
            "trajectory_rows": len(self.recorder.rows), "command_rows": len(self.recorder.command_rows),
        }
        if self.route:
            metadata["route_loop_length"] = self.route.loop_length
            metadata["corner_zones"] = [z.to_dict() for z in self.route.corner_zones]
        (self.current_run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
        self._log(f"Saved run to {self.current_run_dir}")
        QMessageBox.information(self, "Saved", f"Saved to:\n{self.current_run_dir}")

    # --------------------------------------------------------------- display
    def _update_status(self) -> None:
        parts = []
        for role in self.roles:
            s = self._latest(role); c = self.state_clients.get(role)
            if s and self.route:
                p = self.route.project_xy(s.x, s.y); age = time.time() - s.recv_time
                mode = self.last_modes.get(role, "leader" if role == "leader" else "-")
                parts.append(f"{role}: v={s.velocity:.0f} s={p.s:.0f} {p.zone_type} {mode} age={age:.1f}s")
            elif c: parts.append(f"{role}: {c.status}")
            else: parts.append(f"{role}: not connected")
        self.status_label.setText(("RUN | " if self.recording else "idle | ") + " | ".join(parts))

    def _update_plot(self) -> None:
        if self.plot is None: return
        for role in self.roles:
            s = self._latest(role)
            if s: self.dots[role].setData([s.x], [s.y])
            rows = [r for r in self.recorder.rows if r.get("role") == role][-300:]
            if rows: self.trace_curves[role].setData([r["x"] for r in rows], [r["y"] for r in rows])

    def _log(self, text: str) -> None:
        self.log_box.append(f"[{datetime.now().strftime('%H:%M:%S')}] {text}")

    def closeEvent(self, event) -> None:  # type: ignore[override]
        if self.recording:
            for c in self.command_clients.values(): c.emergency_stop()
        self._disconnect_clients(); event.accept()
