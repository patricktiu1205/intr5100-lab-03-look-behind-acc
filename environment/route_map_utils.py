# -*- coding: utf-8 -*-
"""
Route and corner-zone utilities for the INTR5100 sandbox experiments.

This module reads the sandbox map YAML, extracts a named route such as line1,
computes cumulative path coordinate s, detects map-based corner zones, projects
vehicle x/y to the route, and visualizes the result.

Typical use:
    python route_map_utils.py --map 6e68_map.yaml --line line1 --visualize

Generated files:
    corner_zones_line1.json
    route_zones_line1.png
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import yaml


@dataclass
class RoutePoint:
    path_i: int
    point_id: int
    x: float
    y: float
    s: float = 0.0


@dataclass
class CornerZone:
    name: str
    corner_id: int
    center_path_i: int
    center_point_id: int
    center_s: float
    score_deg: float
    approach_start_s: float
    corner_start_s: float
    corner_end_s: float
    exit_end_s: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ProjectionResult:
    s: float
    segment_i: int
    ratio: float
    proj_x: float
    proj_y: float
    lateral_error: float
    distance: float
    zone_type: str
    corner_id: int
    corner_name: str


class RouteAnalyzer:
    def __init__(
        self,
        route_points: Sequence[RoutePoint],
        corner_zones: Optional[Sequence[CornerZone]] = None,
    ):
        if len(route_points) < 3:
            raise ValueError("Route must contain at least 3 points")
        self.points: List[RoutePoint] = [RoutePoint(**asdict(p)) if isinstance(p, RoutePoint) else RoutePoint(**p) for p in route_points]
        self._compute_geometry()
        self.corner_zones: List[CornerZone] = list(corner_zones or [])

    @classmethod
    def from_map(
        cls,
        map_path: str | Path,
        line_name: str = "line1",
        line_index: Optional[int] = None,
        detect_zones: bool = True,
        **corner_kwargs: Any,
    ) -> "RouteAnalyzer":
        route_points = load_route_from_map(map_path, line_name=line_name, line_index=line_index)
        analyzer = cls(route_points)
        if detect_zones:
            analyzer.corner_zones = detect_corner_zones(analyzer, **corner_kwargs)
        return analyzer

    def _compute_geometry(self) -> None:
        n = len(self.points)
        cumulative = 0.0
        for i, p in enumerate(self.points):
            p.s = cumulative
            q = self.points[(i + 1) % n]
            if i < n - 1:
                cumulative += distance_xy(p.x, p.y, q.x, q.y)
        # include closing segment for loop length
        self.loop_length = cumulative + distance_xy(self.points[-1].x, self.points[-1].y, self.points[0].x, self.points[0].y)

    def segment_length(self, i: int) -> float:
        p = self.points[i % len(self.points)]
        q = self.points[(i + 1) % len(self.points)]
        return distance_xy(p.x, p.y, q.x, q.y)

    def wrap_s(self, s: float) -> float:
        if self.loop_length <= 0:
            return float(s)
        return float(s) % self.loop_length

    def forward_gap(self, s_front: float, s_back: float) -> float:
        return (self.wrap_s(s_front) - self.wrap_s(s_back)) % self.loop_length

    def zone_at_s(self, s: float) -> Tuple[str, int, str]:
        s = self.wrap_s(s)
        # priority: corner > approach > exit > straight
        for zone_type in ("corner", "approach", "exit"):
            for zone in self.corner_zones:
                if zone_type == "corner":
                    inside = interval_contains(s, zone.corner_start_s, zone.corner_end_s, self.loop_length)
                elif zone_type == "approach":
                    inside = interval_contains(s, zone.approach_start_s, zone.corner_start_s, self.loop_length)
                else:
                    inside = interval_contains(s, zone.corner_end_s, zone.exit_end_s, self.loop_length)
                if inside:
                    return zone_type, zone.corner_id, zone.name
        return "straight", -1, "none"

    def project_xy(self, x: float, y: float, search_center_i: Optional[int] = None, search_window: int = 50) -> ProjectionResult:
        n = len(self.points)
        if search_center_i is None:
            indices = range(n)
        else:
            indices = [((search_center_i + offset) % n) for offset in range(-search_window, search_window + 1)]

        best = None
        for i in indices:
            p = self.points[i % n]
            q = self.points[(i + 1) % n]
            proj_x, proj_y, ratio, signed_error, dist = project_to_segment_xy(x, y, (p.x, p.y), (q.x, q.y))
            seg_len = self.segment_length(i)
            s = self.wrap_s(p.s + ratio * seg_len)
            candidate = (dist, i % n, ratio, proj_x, proj_y, signed_error, s)
            if best is None or candidate[0] < best[0]:
                best = candidate
        assert best is not None
        dist, seg_i, ratio, proj_x, proj_y, signed_error, s = best
        zone_type, corner_id, corner_name = self.zone_at_s(s)
        return ProjectionResult(
            s=s,
            segment_i=seg_i,
            ratio=ratio,
            proj_x=proj_x,
            proj_y=proj_y,
            lateral_error=signed_error,
            distance=dist,
            zone_type=zone_type,
            corner_id=corner_id,
            corner_name=corner_name,
        )

    def export_zones(self, path: str | Path, map_name: str = "", line_name: str = "line1") -> None:
        payload = {
            "map": map_name,
            "line_name": line_name,
            "loop_length": self.loop_length,
            "zones": [z.to_dict() for z in self.corner_zones],
        }
        Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def distance_xy(x1: float, y1: float, x2: float, y2: float) -> float:
    return math.hypot(float(x1) - float(x2), float(y1) - float(y2))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def load_route_from_map(
    map_path: str | Path,
    line_name: str = "line1",
    line_index: Optional[int] = None,
) -> List[RoutePoint]:
    map_path = Path(map_path)
    with map_path.open("r", encoding="utf-8") as f:
        map_data = yaml.safe_load(f)

    paths = map_data.get("path", [])
    selected = None
    if line_index is not None:
        selected = paths[int(line_index)]
    else:
        for p in paths:
            if str(p.get("name", "")) == line_name:
                selected = p
                break
    if selected is None:
        available = [p.get("name", f"path_{i}") for i, p in enumerate(paths)]
        raise ValueError(f"Cannot find route {line_name!r}; available routes: {available}")

    point_list = map_data["all_point"]["list"]
    point_by_id = {int(p.get("index", i)): p for i, p in enumerate(point_list)}
    route_points: List[RoutePoint] = []
    for path_i, point_id in enumerate(selected["index"]):
        p = point_by_id[int(point_id)]
        route_points.append(RoutePoint(path_i=path_i, point_id=int(point_id), x=float(p["x"]), y=float(p["y"])))
    return route_points


def project_to_segment_xy(
    px: float,
    py: float,
    start: Tuple[float, float],
    end: Tuple[float, float],
) -> Tuple[float, float, float, float, float]:
    sx, sy = float(start[0]), float(start[1])
    ex, ey = float(end[0]), float(end[1])
    vx, vy = ex - sx, ey - sy
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-9:
        dist = math.hypot(px - sx, py - sy)
        return sx, sy, 0.0, 0.0, dist
    ratio = ((px - sx) * vx + (py - sy) * vy) / length_sq
    ratio = clamp(ratio, 0.0, 1.0)
    proj_x = sx + ratio * vx
    proj_y = sy + ratio * vy
    length = math.sqrt(length_sq)
    signed_error = (vx * (py - proj_y) - vy * (px - proj_x)) / length
    dist = math.hypot(px - proj_x, py - proj_y)
    return proj_x, proj_y, ratio, signed_error, dist


def interval_contains(s: float, start: float, end: float, loop_length: float) -> bool:
    s = s % loop_length
    start = start % loop_length
    end = end % loop_length
    if start <= end:
        return start <= s <= end
    return s >= start or s <= end


def _local_turn_score(points: Sequence[RoutePoint], i: int, window_points: int) -> float:
    n = len(points)
    prev = points[(i - window_points) % n]
    mid = points[i]
    nxt = points[(i + window_points) % n]
    v1 = (mid.x - prev.x, mid.y - prev.y)
    v2 = (nxt.x - mid.x, nxt.y - mid.y)
    n1 = math.hypot(v1[0], v1[1])
    n2 = math.hypot(v2[0], v2[1])
    if n1 <= 1e-9 or n2 <= 1e-9:
        return 0.0
    dot = (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)
    dot = clamp(dot, -1.0, 1.0)
    return math.degrees(math.acos(dot))


def detect_corner_zones(
    analyzer: RouteAnalyzer,
    threshold_deg: float = 30.0,
    window_points: int = 8,
    max_corners: int = 8,
    min_corner_gap: float = 500.0,
    corner_half_length: float = 260.0,
    approach_length: float = 280.0,
    exit_length: float = 220.0,
) -> List[CornerZone]:
    """Detect corner centers and expand them into approach/corner/exit zones.

    The algorithm is intentionally map-based and deterministic. It scores each
    route point by the heading change between a short incoming segment and a
    short outgoing segment, then selects the strongest peaks separated by at
    least `min_corner_gap` along the closed loop.
    """
    scores = [_local_turn_score(analyzer.points, i, window_points) for i in range(len(analyzer.points))]
    candidates = [i for i, score in enumerate(scores) if score >= threshold_deg]
    candidates.sort(key=lambda i: scores[i], reverse=True)

    selected: List[int] = []
    for i in candidates:
        s_i = analyzer.points[i].s
        too_close = False
        for j in selected:
            gap = min(analyzer.forward_gap(s_i, analyzer.points[j].s), analyzer.forward_gap(analyzer.points[j].s, s_i))
            if gap < min_corner_gap:
                too_close = True
                break
        if not too_close:
            selected.append(i)
        if len(selected) >= max_corners:
            break

    selected.sort(key=lambda i: analyzer.points[i].s)
    zones: List[CornerZone] = []
    for idx, i in enumerate(selected, start=1):
        p = analyzer.points[i]
        center_s = analyzer.wrap_s(p.s)
        zone = CornerZone(
            name=f"corner_{idx}",
            corner_id=idx,
            center_path_i=i,
            center_point_id=p.point_id,
            center_s=center_s,
            score_deg=float(scores[i]),
            approach_start_s=analyzer.wrap_s(center_s - corner_half_length - approach_length),
            corner_start_s=analyzer.wrap_s(center_s - corner_half_length),
            corner_end_s=analyzer.wrap_s(center_s + corner_half_length),
            exit_end_s=analyzer.wrap_s(center_s + corner_half_length + exit_length),
        )
        zones.append(zone)
    return zones


def visualize_route_zones(
    analyzer: RouteAnalyzer,
    output_path: str | Path,
    title: str = "Map-based route zones",
    show_point_labels: bool = False,
) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [p.x for p in analyzer.points] + [analyzer.points[0].x]
    ys = [p.y for p in analyzer.points] + [analyzer.points[0].y]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(xs, ys, linewidth=1.5, label="route")

    zone_styles = {
        "approach": {"marker": "o", "s": 14, "label": "approach"},
        "corner": {"marker": "s", "s": 20, "label": "corner"},
        "exit": {"marker": "^", "s": 14, "label": "exit"},
        "straight": {"marker": ".", "s": 4, "label": "straight"},
    }
    plotted_labels = set()
    zone_to_points: Dict[str, List[Tuple[float, float]]] = {k: [] for k in zone_styles}
    for p in analyzer.points:
        zt, _, _ = analyzer.zone_at_s(p.s)
        zone_to_points[zt].append((p.x, p.y))

    for zt, pts in zone_to_points.items():
        if not pts:
            continue
        xz = [p[0] for p in pts]
        yz = [p[1] for p in pts]
        style = zone_styles[zt]
        label = style["label"] if style["label"] not in plotted_labels else None
        plotted_labels.add(style["label"])
        ax.scatter(xz, yz, marker=style["marker"], s=style["s"], label=label)

    for z in analyzer.corner_zones:
        p = analyzer.points[z.center_path_i]
        ax.text(p.x, p.y, f"C{z.corner_id}\n{s_round(z.center_s)}", fontsize=8, ha="center", va="bottom")

    if show_point_labels:
        for p in analyzer.points[:: max(1, len(analyzer.points) // 40)]:
            ax.text(p.x, p.y, str(p.path_i), fontsize=6)

    ax.set_title(title)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.axis("equal")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def s_round(value: float) -> str:
    return str(int(round(value)))


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect and visualize map-based corner zones.")
    parser.add_argument("--map", default="6e68_map.yaml", help="Path to sandbox map YAML")
    parser.add_argument("--line", default="line1", help="Route name in map YAML")
    parser.add_argument("--line-index", type=int, default=None, help="Optional route index; overrides --line")
    parser.add_argument("--threshold-deg", type=float, default=30.0)
    parser.add_argument("--window-points", type=int, default=8)
    parser.add_argument("--max-corners", type=int, default=8)
    parser.add_argument("--min-corner-gap", type=float, default=500.0)
    parser.add_argument("--corner-half-length", type=float, default=260.0)
    parser.add_argument("--approach-length", type=float, default=280.0)
    parser.add_argument("--exit-length", type=float, default=220.0)
    parser.add_argument("--output-json", default=None)
    parser.add_argument("--output-png", default=None)
    parser.add_argument("--visualize", action="store_true")
    args = parser.parse_args()

    analyzer = RouteAnalyzer.from_map(
        args.map,
        line_name=args.line,
        line_index=args.line_index,
        threshold_deg=args.threshold_deg,
        window_points=args.window_points,
        max_corners=args.max_corners,
        min_corner_gap=args.min_corner_gap,
        corner_half_length=args.corner_half_length,
        approach_length=args.approach_length,
        exit_length=args.exit_length,
    )

    map_stem = Path(args.map).stem
    output_json = Path(args.output_json or f"corner_zones_{args.line}.json")
    output_png = Path(args.output_png or f"route_zones_{args.line}.png")
    analyzer.export_zones(output_json, map_name=map_stem, line_name=args.line)

    print(f"Route length: {analyzer.loop_length:.1f}")
    print(f"Detected {len(analyzer.corner_zones)} corner zone(s):")
    for z in analyzer.corner_zones:
        print(
            f"  {z.name}: center_i={z.center_path_i}, point_id={z.center_point_id}, "
            f"center_s={z.center_s:.1f}, score={z.score_deg:.1f} deg"
        )
    print(f"Saved zones: {output_json}")

    if args.visualize:
        visualize_route_zones(analyzer, output_png, title=f"{map_stem} {args.line} zones")
        print(f"Saved visualization: {output_png}")


if __name__ == "__main__":
    main()

# ---------------------------------------------------------------------------
# Robust circular straight-section utilities used by the refactored GUIs.
# ---------------------------------------------------------------------------

@dataclass
class RouteSection:
    name: str
    start_s: float
    end_s: float
    length: float
    wraps: bool = False

    def contains(self, s: float, loop_length: float) -> bool:
        return interval_contains(float(s), self.start_s, self.end_s, loop_length)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def circular_straight_sections(analyzer: RouteAnalyzer) -> List[RouteSection]:
    """Return contiguous straight sections on a *closed* route.

    Unlike a linear scan, this deliberately merges the straight run that may
    straddle ``s=0``.  The section boundaries are route sample points, matching
    the previous GUI behavior, but the topology is now circular.
    """
    points = analyzer.points
    n = len(points)
    if n == 0:
        return []

    straight = [analyzer.zone_at_s(p.s)[0] == "straight" for p in points]
    if not any(straight):
        return []
    if all(straight):
        return [RouteSection("straight_1", 0.0, analyzer.loop_length - 1e-9, analyzer.loop_length, False)]

    # Start immediately after a non-straight point. Walking exactly n samples
    # from there guarantees that a wrap-around straight is represented once.
    nonstraight_i = next(i for i, flag in enumerate(straight) if not flag)
    start_i = (nonstraight_i + 1) % n

    runs: List[List[int]] = []
    current: List[int] = []
    for k in range(n):
        i = (start_i + k) % n
        if straight[i]:
            current.append(i)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)

    sections: List[RouteSection] = []
    for idx, run in enumerate(runs, start=1):
        a = analyzer.wrap_s(points[run[0]].s)
        b = analyzer.wrap_s(points[run[-1]].s)
        length = (b - a) % analyzer.loop_length
        sections.append(
            RouteSection(
                name=f"straight_{idx}",
                start_s=a,
                end_s=b,
                length=length,
                wraps=a > b,
            )
        )
    return sections


def longest_straight_sections(analyzer: RouteAnalyzer, count: int = 3) -> List[RouteSection]:
    sections = circular_straight_sections(analyzer)
    selected = sorted(sections, key=lambda s: s.length, reverse=True)[: max(0, int(count))]
    # Stable route-order presentation is easier for logs/leader events.
    return sorted(selected, key=lambda s: s.start_s)


def control_section_index(sections: Sequence[RouteSection], s: float, loop_length: float) -> int:
    for i, section in enumerate(sections):
        if section.contains(s, loop_length):
            return i
    return -1


def validate_route_for_experiments(
    analyzer: RouteAnalyzer,
    *,
    expected_corners: int = 0,
    detector_max_corners: Optional[int] = None,
    required_control_sections: int = 3,
) -> Tuple[bool, List[str], List[RouteSection]]:
    """Validate route zoning before hardware can be started.

    ``expected_corners=0`` disables exact-count checking, but zero detected
    corners is always an error.  Hitting ``detector_max_corners`` is also an
    error because it usually means peak detection saturated and the TA should
    tune the detector rather than silently accepting the result.
    """
    messages: List[str] = []
    ok = True
    n_corners = len(analyzer.corner_zones)
    if n_corners == 0:
        ok = False
        messages.append("ERROR: no corners detected; route would be classified as straight everywhere.")
    if expected_corners > 0 and n_corners != expected_corners:
        ok = False
        messages.append(f"ERROR: detected {n_corners} corners, expected {expected_corners}.")
    if detector_max_corners and n_corners >= detector_max_corners:
        ok = False
        messages.append(
            f"ERROR: detector returned {n_corners} corners and hit max_corners={detector_max_corners}; "
            "increase/tune the detector and re-check the visualization."
        )

    sections = longest_straight_sections(analyzer, required_control_sections)
    if len(sections) < required_control_sections:
        ok = False
        messages.append(
            f"ERROR: only {len(sections)} usable straight section(s); "
            f"{required_control_sections} are required."
        )
    if ok:
        messages.append(
            f"Route validation OK: {n_corners} corner zone(s), "
            f"{len(sections)} selected control straight(s)."
        )
    return ok, messages, sections
