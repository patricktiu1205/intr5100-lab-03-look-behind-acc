"""Lab III submission: equilibrium-referenced leading cruise feedback with front priority.

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

    def __init__(self, route=None, sections=None, gamma=GAMMA, beta=BETA, rear_weight=1.0):
        if gamma <= 0 or beta <= 0 or not all(map(math.isfinite, (gamma, beta, rear_weight))):
            raise ValueError('Finite positive speed conversions are required')
        self.route, self.sections = route, sections
        self.gamma, self.beta = gamma, beta
        self.rear_weight = _clip(rear_weight, 0.0, 1.0)
        self.length = 250.0
        self.standstill, self.headway = 300.0, 1.8
        self.k_gap, self.k_closing = 0.15, 0.50
        self.fresh_age, self.prediction_limit = 0.15, 1.2
        self.prediction_accel_bound = 45.0
        self.rear_guard_horizon = 1.5
        self.command_response_tau = 0.5
        self.rear_washout_tau = 60.0
        self.k_rear_gap, self.k_rear_velocity = 0.35, 1.5
        self.reference_tau, self.rear_filter_tau, self.rear_limit = 6.0, 0.35, 30.0
        # Public TA default IDM equilibrium, queried in reported-speed units.
        self.idm_v0, self.idm_T, self.idm_s0 = 70.0, 1.2, 250.0
        self.reserve, self.reaction, self.braking = 120.0, 0.5, 45.0
        self.reset()

    def reset(self):
        self.last_time = self.last_output = self.last_history_time = None
        self.history_source = None
        self.history_length = 0
        self.filtered_rear = 0.0
        self.rear_gap_bias = 0.0
        self.reference_speed = None
        self.last_section = None
        self.observations = {}
        self.packet_status = {}
        self.command_tape = []
        self.current_wall = None
        self.debug = {}

    def _stop(self, reason, now):
        self.last_time, self.last_output = now, 0.0
        self.filtered_rear = 0.0
        self.rear_gap_bias = 0.0
        self.reference_speed = None
        self.last_section = None
        self.debug = {'reason': reason, 'command': 0.0}
        self._remember_command(0.0)
        return 0.0

    def _section(self, s):
        return next((i for i, sec in enumerate(self.sections)
                     if sec.contains(s, self.route.loop_length)), -1)

    def _remember_command(self, command):
        if self.current_wall is not None:
            self.command_tape.append((self.current_wall,float(round(command))))
            self.command_tape=self.command_tape[-32:]

    def _predict_own(self, s, velocity, stamp, wall_now):
        """Predict using requested commands and an assumed first-order response.

        Fresh telemetry overrides prediction. These commands are not actuator
        acknowledgements; unobserved TA overrides remain a source of uncertainty.
        """
        command=velocity/self.beta
        events=[]
        for t,u in self.command_tape:
            if t<=stamp:command=u
            elif t<wall_now:events.append((t,u))
        current=stamp
        for end,next_command in events+[(wall_now,command)]:
            duration=max(0.0,end-current)
            decay=math.exp(-duration/self.command_response_tau)
            steady=self.beta*command
            s+=steady*duration+(velocity-steady)*self.command_response_tau*(1-decay)
            velocity=steady+(velocity-steady)*decay
            current=end;command=next_command
        return s%self.route.loop_length,max(0.0,velocity)

    def _confidence(self, age):
        return _clip((self.prediction_limit-age) /
                     (self.prediction_limit-self.fresh_age),0.0,1.0)

    def _read(self, state, wall_now, role=None):
        """Validate, reject reordering, and propagate to a common receiver time.

        recv_time is a reception timestamp, not a certified sensor capture time.
        Short extrapolation uses bounded, filtered speed trend; long loss fails
        closed for front/own state. No claim about unseen transport latency.
        """
        cached = self.observations.get(role) if role else None
        reused = state is None
        packet_source='cached_missing' if reused else 'received'
        if state is None:
            if cached is None:raise ValueError('No telemetry')
            s, velocity, stamp, acceleration = cached
        else:
            vals = tuple(float(getattr(state,k)) for k in ('x','y','velocity'))
            if not all(map(math.isfinite,vals)) or vals[2]<0:
                raise ValueError('Invalid telemetry')
            received=getattr(state,'recv_time',None)
            stamp=wall_now if received is None else float(received)
            if not math.isfinite(stamp) or stamp>wall_now+0.1:
                raise ValueError('Invalid timestamp')
            if cached is not None and stamp<cached[2]-1e-9:
                # Older packets never overwrite a newer accepted observation.
                s,velocity,stamp,acceleration=cached
                reused=True
                packet_source='out_of_order'
            else:
                projection=self.route.project_xy(vals[0],vals[1])
                s=float(projection.s);distance=float(getattr(projection,'distance',0.0))
                if not math.isfinite(s) or not math.isfinite(distance) or distance>150:
                    raise ValueError('Pose is not on the route')
                velocity=vals[2]*self.gamma
                if not math.isfinite(velocity):raise ValueError('Invalid scaled velocity')
                acceleration=0.0 if cached is None else cached[3]
                if cached is not None:
                    elapsed=stamp-cached[2]
                    if 0.04<=elapsed<=1.0:
                        trend=_clip((velocity-cached[1])/elapsed,
                                    -self.prediction_accel_bound,self.prediction_accel_bound)
                        acceleration+=elapsed/(0.4+elapsed)*(trend-acceleration)
                if cached is not None and 0.04<=stamp-cached[2]<=self.prediction_limit:
                    elapsed=stamp-cached[2]
                    displacement=(s-cached[0]+self.route.loop_length/2)%self.route.loop_length-self.route.loop_length/2
                    expected=0.5*(velocity+cached[1])*elapsed
                    if abs(displacement-expected)>max(120.0,120.0*elapsed):
                        s,velocity,stamp,acceleration=cached
                        reused=True
                        packet_source='jump_rejected'
        age=max(0.0,wall_now-stamp)
        if age>self.prediction_limit or (cached is None and age>0.6):
            raise ValueError('Telemetry prediction horizon exhausted')
        if role:self.observations[role]=(s,velocity,stamp,acceleration)
        # Predict only while velocity remains nonnegative. Cap propagation age to
        # the declared finite horizon; modulo handles a route seam, not overtaking.
        horizon=age
        if acceleration<0:horizon=min(horizon,velocity/-acceleration)
        projected=(s+velocity*horizon+0.5*acceleration*horizon*horizon)%self.route.loop_length
        predicted_velocity=max(0.0,velocity+acceleration*horizon)
        if role=='student' and self.command_tape and age>0:
            projected,predicted_velocity=self._predict_own(s,velocity,stamp,wall_now)
        if role:self.packet_status[role]=dict(age=age,confidence=self._confidence(age),reused=reused,source=packet_source)
        return projected,predicted_velocity,age

    def step(self, leader_state, student_state, following_state, history=None, now=None, wall_now=None):
        now = _CLOCK() if now is None else float(now)
        wall_now = _WALL_CLOCK() if wall_now is None else float(wall_now)
        self.current_wall=wall_now if math.isfinite(wall_now) else None
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
        reset_observer = raw_dt is not None and (raw_dt<=0 or raw_dt>0.65)
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
                reset_observer = True
            self.history_source, self.history_length = history, n
            self.last_history_time = latest
        if reset_observer:
            self.observations.clear()
            self.command_tape.clear()
        self.packet_status = {}
        try:
            front = self._read(leader_state, wall_now, 'leader')
            own = self._read(student_state, wall_now, 'student')
        except (ValueError, TypeError, AttributeError, OverflowError):
            return self._stop('invalid_or_stale_front_state', now)
        section = self._section(own[0])
        if section < 0 or self._section(front[0]) != section:
            self.filtered_rear = 0.0
            self.last_time, self.last_output = now, None
            self.reference_speed, self.last_section = None, None
            self.command_tape.clear()
            self.debug = {'reason': 'inactive_section', 'command': 0.0}
            return 0.0  # The supplied GUI owns bypass commands and does not call us here.
        if section != self.last_section:
            reentry = True
        self.last_section = section
        # Nominal feedback uses receiver-time-aligned predicted route positions.
        # The unmodified TA shield still uses its original raw pose convention.
        gap = self.route.forward_gap(front[0], own[0]) - self.length
        if not math.isfinite(gap) or gap > self.route.loop_length / 2:
            return self._stop('ambiguous_front_order', now)
        v0, v1 = front[1], own[1]
        # Conservative asynchrony allowance, never count an unobserved leader advance.
        uncertainty = 0.5*self.prediction_accel_bound*(front[2]**2+own[2]**2)
        protected_gap = gap - uncertainty
        closing = max(0.0, v1 - v0)
        ttc = protected_gap / closing if closing > 1e-9 else None
        if protected_gap <= self.reserve or (ttc is not None and ttc < 1.5):
            return self._stop('front_emergency', now)
        if reentry:
            self.last_output = v1 / self.beta
            self.filtered_rear = 0.0
            self.rear_gap_bias = 0.0
            self.reference_speed = v0
        target = self.standstill + self.headway * v1
        error = gap - target
        front_reference = v0 + self.k_gap * error + self.k_closing * (v0 - v1)
        rear_raw = 0.0
        rear_valid = False
        rear_gap = None
        rear_deceleration_gate = 1.0
        rear_confidence = 0.0
        rear_floor = 0.0
        if self.reference_speed is None:
            self.reference_speed = v0
        self.reference_speed += dt / (self.reference_tau + dt) * (v0 - self.reference_speed)
        try:
            rear = self._read(following_state, wall_now, 'following')
            rear_gap = self.route.forward_gap(own[0], rear[0]) - self.length
            rear_valid = (self._section(rear[0]) == section and 0 <= rear_gap < self.route.loop_length / 2)
            if rear_valid:
                rear_confidence = self._confidence(rear[2])
                # Kinematic lower command bound for rear approach. It is applied
                # only when compatible with the higher-priority front speed cap.
                rear_margin=rear_gap-80.0-0.5*self.prediction_accel_bound*(rear[2]**2+own[2]**2)
                rear_floor=(max(0.0,rear[1]-rear_margin/self.rear_guard_horizon)/self.beta
                            if self.rear_weight>0 else 0.0)
                # LCC Eq. 27 uses rear spacing and speed deviations from equilibrium.
                # Its demonstrated negative speed feedback differs from chasing v2-v1.
                qref = _clip(self.reference_speed / self.gamma, 0.0, 0.85*self.idm_v0)
                rear_target = (self.idm_s0 + self.idm_T*qref) / math.sqrt(1-(qref/self.idm_v0)**4)
                rear_error=rear_gap-rear_target
                if self.rear_washout_tau>0:
                    self.rear_gap_bias+=rear_confidence*dt/(self.rear_washout_tau+dt)*(
                        _clip(rear_error,-500.0,500.0)-self.rear_gap_bias)
                else:self.rear_gap_bias=0.0
                # Wash out persistent rear spacing bias while retaining dynamic
                # rear disturbances; do not integrate a spacing offset into speed.
                rear_raw = _clip(-self.k_rear_gap * (rear_error-self.rear_gap_bias)
                                 - self.k_rear_velocity * (rear[1]-self.reference_speed),
                                 -self.rear_limit, self.rear_limit)
                # Do not use rear feedback to request extra slowing when rear space
                # is scarce. This limits the correction, not the necessary front brake.
                rear_deceleration_gate = _clip((rear_gap-self.reserve
                    -0.5*max(0.0,rear[1]-v1))/180.0,0.0,1.0)
                if rear_raw < 0:rear_raw *= rear_deceleration_gate
        except (ValueError, TypeError, AttributeError, OverflowError):
            rear_valid = False
        # Downweight rear pressure unless there is front headroom. Invalid/bypassed rear
        # immediately disables the term, including previously filtered information.
        gate = _clip((protected_gap - self.reserve) / max(target - self.reserve, 1.0), 0.0, 1.0)
        if rear_valid:
            self.filtered_rear += dt / (self.rear_filter_tau + dt) * (rear_raw - self.filtered_rear)
        else:
            self.filtered_rear = 0.0
            self.rear_gap_bias = 0.0
        rear_term = self.rear_weight * gate * rear_confidence * self.filtered_rear
        if rear_term < 0:rear_term *= rear_deceleration_gate
        desired = _clip((front_reference + rear_term) / self.beta, 0.0, 70.0)
        previous = v1 / self.beta if self.last_output is None else self.last_output
        if max(front[2],own[2])>0.6:
            desired=min(desired,previous)  # No blind acceleration on old front data.
        smooth = _clip(desired, previous - 12.0 * dt, previous + 6.0 * dt)
        # Conservative stopping-distance cap with common braking, zero road grade and
        # reaction delay assumptions. A constraint, not a validated safety guarantee.
        b, reaction = self.braking, self.reaction
        safe_speed = max(0.0, -b * reaction + math.sqrt((b * reaction)**2
                         + v0*v0 + 2*b*max(0.0, protected_gap-self.reserve)))
        front_cap = min(70.0,safe_speed/self.beta)
        if max(front[2],own[2])>0.6:front_cap=min(front_cap,previous)
        two_sided_feasible = rear_floor <= front_cap
        guarded=max(smooth,min(rear_floor,front_cap))
        command = _clip(min(guarded,front_cap),0.0,70.0)
        self.last_time, self.last_output = now, command
        self.debug = dict(reason=('two_sided_conflict' if not two_sided_feasible else
                                  'front_cap' if command < smooth-1e-9 else
                                  'rear_guard' if command > smooth+1e-9 else 'feedback'),
                          command=command, gap=gap, protected_gap=protected_gap,
                          target=target, error=error, rear_gap=rear_gap,
                          rear_valid=rear_valid, rear_term=rear_term, front_ttc=ttc,
                          reference_speed=self.reference_speed, rear_deceleration_gate=rear_deceleration_gate,
                          safe_command=front_cap, rear_floor=rear_floor,
                          two_sided_feasible=two_sided_feasible, telemetry=dict(self.packet_status),
                          reentry=reentry)
        self._remember_command(command)
        return float(command)


_CONTROLLER = LookBehindController()


def student_control(leader_state, student_state, following_state, history=None) -> float:
    """Required TA callback; returns a desired speed command for the middle car."""
    result = _CONTROLLER.step(leader_state, student_state, following_state, history)
    student_control.last_debug = dict(_CONTROLLER.debug)
    return float(result)
