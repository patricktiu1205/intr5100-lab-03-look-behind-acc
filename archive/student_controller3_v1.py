"""Lab III submission: bounded bilateral speed feedback with front priority.

Copy only this file beside the unmodified TA Experiment III files. Units: route
positions/gaps in coordinate units, reported velocity and output in command
units. GAMMA and BETA are distinct, unverified Lab II working assumptions.
No vehicle communication, files written, GUI imports or optimization solver.
"""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

GAMMA = 2.8  # coordinate velocity / reported velocity; not a physical calibration
BETA = 2.8   # steady coordinate velocity / command; independently uncertain
_CLOCK = time.monotonic
_WALL_CLOCK = time.time


def _clip(value, lo, hi):
    return max(lo, min(hi, value))


def _load_route():
    """Read supplied geometry; neither duplicate nor modify TA support files."""
    here = Path(__file__).resolve().parent
    candidates = [here, here / 'environment', here.parent / 'Shared Environment']
    for folder in candidates:
        if (folder / '6e68_map.yaml').is_file() and (folder / 'route_map_utils.py').is_file():
            if str(folder) not in sys.path:
                sys.path.insert(0, str(folder))
            from route_map_utils import RouteAnalyzer, longest_straight_sections
            route = RouteAnalyzer.from_map(folder / '6e68_map.yaml')
            return route, longest_straight_sections(route, 3)
    raise FileNotFoundError('Expected the supplied map and route_map_utils.py beside this submission')


class LookBehindController:
    """Transparent speed policy; rear feedback can never relax the front cap."""

    def __init__(self, route=None, sections=None, gamma=GAMMA, beta=BETA, rear_weight=0.35):
        if gamma <= 0 or beta <= 0 or not all(map(math.isfinite, (gamma, beta, rear_weight))):
            raise ValueError('Finite positive speed conversions are required')
        self.route, self.sections = route, sections
        self.gamma, self.beta = gamma, beta
        self.rear_weight = _clip(rear_weight, 0.0, 0.5)
        self.length = 250.0
        self.standstill, self.headway = 300.0, 1.8
        self.rear_standstill, self.rear_headway = 300.0, 1.2
        self.k_gap, self.k_closing = 0.10, 0.25
        self.k_rear_gap, self.k_rear_velocity = 0.025, 0.20
        self.reserve, self.reaction, self.braking = 120.0, 0.5, 45.0
        self.reset()

    def reset(self):
        self.last_time = self.last_output = self.last_history_time = None
        self.history_source = None
        self.history_length = 0
        self.filtered_rear = 0.0
        self.debug = {}

    def _stop(self, reason, now):
        self.last_time, self.last_output = now, 0.0
        self.filtered_rear = 0.0
        self.debug = {'reason': reason, 'command': 0.0}
        return 0.0

    def _section(self, s):
        return next((i for i, sec in enumerate(self.sections)
                     if sec.contains(s, self.route.loop_length)), -1)

    def _read(self, state, wall_now):
        vals = tuple(float(getattr(state, k)) for k in ('x', 'y', 'velocity'))
        if not all(map(math.isfinite, vals)) or vals[2] < 0:
            raise ValueError('Invalid telemetry')
        received = getattr(state, 'recv_time', None)
        age = 0.0 if received is None else wall_now - float(received)
        if not math.isfinite(age) or age > 0.6 or age < -0.1:
            raise ValueError('Stale or future telemetry')
        projection = self.route.project_xy(vals[0], vals[1])
        s = float(projection.s)
        distance = float(getattr(projection, 'distance', 0.0))
        if not math.isfinite(s) or not math.isfinite(distance) or distance > 150.0:
            raise ValueError('Pose is not on the route')
        return s, vals[2] * self.gamma, age

    def step(self, leader_state, student_state, following_state, history=None, now=None, wall_now=None):
        now = _CLOCK() if now is None else float(now)
        wall_now = _WALL_CLOCK() if wall_now is None else float(wall_now)
        if not math.isfinite(now) or not math.isfinite(wall_now):
            return self._stop('invalid_clock', 0.0)
        try:
            if self.route is None:
                self.route, self.sections = _load_route()
        except Exception:
            # Do not raise into TA's exception handler, which falls back to leader speed.
            return self._stop('geometry_unavailable', now)
        raw_dt = None if self.last_time is None else now - self.last_time
        reentry = raw_dt is None or raw_dt <= 0 or raw_dt > 0.65
        dt = 0.15 if reentry else _clip(raw_dt, 0.02, 0.35)
        if history is not None:
            n = len(history)
            latest = None
            for row in reversed(history[-30:]):
                try:
                    stamp = float(row['t_from_start'])
                    if math.isfinite(stamp):latest = stamp if latest is None else max(latest, stamp)
                except (ValueError, TypeError, KeyError):
                    continue
            if (self.history_source is not history or n < self.history_length
                    or (latest is not None and self.last_history_time is not None
                        and latest < self.last_history_time)):
                reentry = True
            self.history_source, self.history_length = history, n
            self.last_history_time = latest
        try:
            front = self._read(leader_state, wall_now)
            own = self._read(student_state, wall_now)
        except (ValueError, TypeError, AttributeError, OverflowError):
            return self._stop('invalid_or_stale_front_state', now)
        section = self._section(own[0])
        if section < 0 or self._section(front[0]) != section:
            self.filtered_rear = 0.0
            self.last_time, self.last_output = now, None
            self.debug = {'reason': 'inactive_section', 'command': 0.0}
            return 0.0  # The supplied GUI owns bypass commands and does not call us here.
        # Nominal gap uses raw route projections, matching the TA wrapper convention.
        gap = self.route.forward_gap(front[0], own[0]) - self.length
        if not math.isfinite(gap) or gap > self.route.loop_length / 2:
            return self._stop('ambiguous_front_order', now)
        v0, v1 = front[1], own[1]
        # Conservative asynchrony allowance, never count an unobserved leader advance.
        protected_gap = gap - v1 * max(0.0, own[2] - front[2])
        closing = max(0.0, v1 - v0)
        ttc = protected_gap / closing if closing > 1e-9 else None
        if protected_gap <= self.reserve or (ttc is not None and ttc < 1.5):
            return self._stop('front_emergency', now)
        if reentry:
            self.last_output = v1 / self.beta
            self.filtered_rear = 0.0
        target = self.standstill + self.headway * v1
        error = gap - target
        front_reference = v0 + self.k_gap * error + self.k_closing * (v0 - v1)
        rear_raw = 0.0
        rear_valid = False
        rear_gap = None
        try:
            rear = self._read(following_state, wall_now)
            rear_gap = self.route.forward_gap(own[0], rear[0]) - self.length
            rear_valid = (self._section(rear[0]) == section and 0 <= rear_gap < self.route.loop_length / 2)
            if rear_valid:
                rear_target = self.rear_standstill + self.rear_headway * rear[1]
                rear_raw = _clip(self.k_rear_velocity * (rear[1] - v1)
                                 + self.k_rear_gap * (rear_target - rear_gap), -12.0, 12.0)
        except (ValueError, TypeError, AttributeError, OverflowError):
            rear_valid = False
        # Downweight rear pressure unless there is front headroom. Invalid/bypassed rear
        # immediately disables the term, including previously filtered information.
        gate = _clip((protected_gap - self.reserve) / max(target - self.reserve, 1.0), 0.0, 1.0)
        if rear_valid:
            self.filtered_rear += dt / (0.5 + dt) * (rear_raw - self.filtered_rear)
        else:
            self.filtered_rear = 0.0
        rear_term = self.rear_weight * gate * self.filtered_rear
        desired = _clip((front_reference + rear_term) / self.beta, 0.0, 70.0)
        previous = v1 / self.beta if self.last_output is None else self.last_output
        smooth = _clip(desired, previous - 12.0 * dt, previous + 6.0 * dt)
        # Conservative stopping-distance cap with common braking, zero road grade and
        # reaction delay assumptions. A constraint, not a validated safety guarantee.
        b, reaction = self.braking, self.reaction
        safe_speed = max(0.0, -b * reaction + math.sqrt((b * reaction)**2
                         + v0*v0 + 2*b*max(0.0, protected_gap-self.reserve)))
        command = _clip(min(smooth, safe_speed / self.beta), 0.0, 70.0)
        self.last_time, self.last_output = now, command
        self.debug = dict(reason='front_cap' if command < smooth - 1e-9 else 'feedback',
                          command=command, gap=gap, protected_gap=protected_gap,
                          target=target, error=error, rear_gap=rear_gap,
                          rear_valid=rear_valid, rear_term=rear_term, front_ttc=ttc,
                          safe_command=safe_speed/self.beta, reentry=reentry)
        return float(command)


_CONTROLLER = LookBehindController()


def student_control(leader_state, student_state, following_state, history=None) -> float:
    """Required TA callback; returns a desired speed command for the middle car."""
    result = _CONTROLLER.step(leader_state, student_state, following_state, history)
    student_control.last_debug = dict(_CONTROLLER.debug)
    return float(result)
