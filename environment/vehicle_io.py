# -*- coding: utf-8 -*-
"""Network clients for sandbox vehicles.

This refactored version keeps state reception in its own thread and adds an
asynchronous command sender. GUI code should use :class:`AsyncVehicleCommandClient`
so TCP delays never block the Qt event loop.
"""
from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class VehicleState:
    role: str
    host: str
    recv_time: float
    seq: int
    data: Dict[str, Any] = field(default_factory=dict)

    def get_float(self, key: str, default: float = 0.0) -> float:
        try:
            return float(self.data.get(key, default))
        except (TypeError, ValueError):
            return default

    @property
    def x(self) -> float:
        return self.get_float("x")

    @property
    def y(self) -> float:
        return self.get_float("y")

    @property
    def heading(self) -> float:
        return self.get_float("heading")

    @property
    def velocity(self) -> float:
        return self.get_float("velocity")


class VehicleCommandClient:
    """Synchronous low-level client.

    Prefer ``AsyncVehicleCommandClient`` in any GUI/control loop.  The short
    timeout here is only a final bound for the worker thread; it should not be
    used to drive a Qt timer directly.
    """

    def __init__(self, host: str, port: int = 10100, timeout: float = 0.25):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def close(self) -> None:
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def _connect_locked(self) -> None:
        if self._sock is None:
            self._sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock.settimeout(self.timeout)

    def send_json(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        data = (json.dumps(payload) + "\n").encode("utf-8")
        with self._lock:
            for attempt in range(2):
                try:
                    self._connect_locked()
                    assert self._sock is not None
                    self._sock.sendall(data)
                    try:
                        response = self._sock.recv(4096).decode("utf-8", errors="ignore").strip()
                        if response:
                            start = response.find("{")
                            end = response.rfind("}")
                            if start >= 0 and end >= start:
                                return json.loads(response[start:end + 1])
                    except (socket.timeout, json.JSONDecodeError):
                        # Many vehicle programs do not return a frame.  Treat a
                        # successful send as success after the bounded wait.
                        pass
                    return {"status": "sent"}
                except OSError:
                    if self._sock is not None:
                        try:
                            self._sock.close()
                        except OSError:
                            pass
                        self._sock = None
                    if attempt == 1:
                        raise
            return {"status": "failed"}

    def start_line1(self, velocity: int = 50) -> Dict[str, Any]:
        return self.send_json({"action": "start_line1", "velocity": int(velocity)})

    def start_line(self, line_index: int = 0, velocity: int = 50) -> Dict[str, Any]:
        return self.send_json({"action": "start_line1", "line_index": int(line_index), "velocity": int(velocity)})

    def update_target_line(self, line_index: int = 0) -> Dict[str, Any]:
        return self.send_json({"action": "update_target_line", "line_index": int(line_index)})

    def set_velocity(self, velocity: int) -> Dict[str, Any]:
        return self.send_json({"action": "set_velocity", "velocity": int(velocity)})

    def stop(self) -> Dict[str, Any]:
        return self.send_json({"action": "stop"})

    def direction_stop(self) -> Dict[str, Any]:
        return self.send_json({
            "action": "direction_control",
            "velocity": 0,
            "steering": 0,
            "blink": 0,
            "brake": 1,
            "emergence_flag": 0,
        })


class AsyncVehicleCommandClient(threading.Thread):
    """Non-blocking command facade for GUI/control code.

    Velocity commands are *coalesced*: if the control loop produces several
    commands while TCP is busy, only the newest velocity is retained.  This
    prevents stale commands from accumulating in a queue.

    Start/stop/route commands are kept as ordered discrete commands. Emergency
    stop commands have priority over everything else.
    """

    def __init__(self, host: str, port: int = 10100, timeout: float = 0.25):
        super().__init__(daemon=True)
        self.host = host
        self.port = int(port)
        self.client = VehicleCommandClient(host, port, timeout=timeout)
        self._cv = threading.Condition()
        self._running = True
        self._closing = False
        self._urgent: list[Dict[str, Any]] = []
        self._discrete: list[Dict[str, Any]] = []
        self._latest_velocity: Optional[int] = None
        self.status = "created"
        self.last_error = ""
        self.last_response: Dict[str, Any] = {}
        self.last_sent_time = 0.0
        self.start()

    def _enqueue_discrete(self, payload: Dict[str, Any]) -> None:
        with self._cv:
            self._discrete.append(payload)
            self._cv.notify()

    def start_line1(self, velocity: int = 50) -> None:
        self._enqueue_discrete({"action": "start_line1", "velocity": int(velocity)})

    def start_line(self, line_index: int = 0, velocity: int = 50) -> None:
        self._enqueue_discrete({"action": "start_line1", "line_index": int(line_index), "velocity": int(velocity)})

    def update_target_line(self, line_index: int = 0) -> None:
        self._enqueue_discrete({"action": "update_target_line", "line_index": int(line_index)})

    def set_velocity(self, velocity: int) -> None:
        with self._cv:
            self._latest_velocity = int(velocity)
            self._cv.notify()

    def stop_vehicle(self) -> None:
        with self._cv:
            self._latest_velocity = None
            self._discrete.clear()  # cancel a not-yet-sent start/route command
            self._urgent.append({"action": "stop"})
            self._cv.notify()

    # Keep the old call spelling convenient for GUI code.
    def stop(self) -> None:  # type: ignore[override]
        self.stop_vehicle()

    def emergency_stop(self) -> None:
        with self._cv:
            self._latest_velocity = None
            self._discrete.clear()  # emergency stop must dominate pending starts
            self._urgent = [
                {
                    "action": "direction_control",
                    "velocity": 0,
                    "steering": 0,
                    "blink": 0,
                    "brake": 1,
                    "emergence_flag": 0,
                },
                {"action": "stop"},
            ] + self._urgent
            self._cv.notify()

    def close(self) -> None:
        # At shutdown, discard stale velocity commands but give an already queued
        # stop/emergency command a brief chance to leave the worker before the
        # socket is torn down. This wait occurs only while closing the app, never
        # in the 10 Hz control loop.
        with self._cv:
            self._latest_velocity = None
            self._discrete.clear()
            self._closing = True
            if not any(p.get("action") in {"stop", "direction_control"} for p in self._urgent):
                self._urgent.append({"action": "stop"})
            self._cv.notify_all()
        if threading.current_thread() is not self:
            self.join(timeout=1.0)
        with self._cv:
            self._running = False
            self._cv.notify_all()
        self.client.close()

    def _take_next(self) -> Optional[Dict[str, Any]]:
        with self._cv:
            while self._running and not self._urgent and not self._discrete and self._latest_velocity is None and not self._closing:
                self._cv.wait(timeout=0.5)
            if not self._running:
                return None
            if self._closing and not self._urgent and not self._discrete:
                return None
            if self._urgent:
                return self._urgent.pop(0)
            if self._discrete:
                return self._discrete.pop(0)
            if self._latest_velocity is not None:
                velocity = self._latest_velocity
                self._latest_velocity = None
                return {"action": "set_velocity", "velocity": int(velocity)}
            return None

    def run(self) -> None:
        self.status = "running"
        while True:
            payload = self._take_next()
            if payload is None:
                break
            try:
                self.status = f"sending {payload.get('action', '?')}"
                self.last_response = self.client.send_json(payload)
                self.last_sent_time = time.time()
                self.last_error = ""
                self.status = "ready"
            except Exception as exc:  # network worker must never kill GUI
                self.last_error = str(exc)
                self.status = "error"
        self.status = "closed"


class VehicleStateClient(threading.Thread):
    def __init__(self, role: str, host: str, port: int = 10200, reconnect_delay: float = 1.0):
        super().__init__(daemon=True)
        self.role = role
        self.host = host
        self.port = int(port)
        self.reconnect_delay = float(reconnect_delay)
        self._running = threading.Event()
        self._running.set()
        self._lock = threading.Lock()
        self._latest: Optional[VehicleState] = None
        self._seq = 0
        self.status = "created"
        self.last_error = ""
        self._sock: Optional[socket.socket] = None

    def stop(self) -> None:
        self._running.clear()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass

    def latest(self) -> Optional[VehicleState]:
        with self._lock:
            return self._latest

    def run(self) -> None:
        while self._running.is_set():
            try:
                self.status = f"connecting {self.host}:{self.port}"
                self._sock = socket.create_connection((self.host, self.port), timeout=3.0)
                self._sock.settimeout(1.0)
                self.status = "connected"
                self._listen()
            except OSError as exc:
                self.status = "disconnected"
                self.last_error = str(exc)
                time.sleep(self.reconnect_delay)
            finally:
                if self._sock is not None:
                    try:
                        self._sock.close()
                    except OSError:
                        pass
                    self._sock = None

    def _listen(self) -> None:
        assert self._sock is not None
        buffer = ""
        while self._running.is_set():
            try:
                data = self._sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise OSError("remote closed")
            buffer += data.decode("utf-8", errors="ignore")
            frames, buffer = pop_json_frames(buffer)
            for raw in frames:
                try:
                    payload = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                self._seq += 1
                state = VehicleState(
                    role=self.role,
                    host=self.host,
                    recv_time=time.time(),
                    seq=self._seq,
                    data=payload,
                )
                with self._lock:
                    self._latest = state


def pop_json_frames(buffer: str) -> tuple[list[str], str]:
    frames: list[str] = []
    while buffer:
        newline = buffer.find("\n")
        if newline >= 0:
            frame = buffer[:newline].strip()
            buffer = buffer[newline + 1:]
            if frame:
                frames.append(frame)
            continue
        start, end = find_json_frame(buffer)
        if start < 0:
            return frames, ""
        if end < 0:
            return frames, buffer[start:]
        frames.append(buffer[start:end])
        buffer = buffer[end:]
    return frames, buffer


def find_json_frame(buffer: str) -> tuple[int, int]:
    start = -1
    level = 0
    for i, ch in enumerate(buffer):
        if ch == "{":
            if level == 0:
                start = i
            level += 1
        elif ch == "}" and level > 0:
            level -= 1
            if level == 0:
                return start, i + 1
    return start, -1
