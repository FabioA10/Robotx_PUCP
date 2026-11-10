"""UAV Geofence derivation, QGroundControl .plan loader, and RoboCommand validator."""

from __future__ import annotations

import json
import math
from typing import List, Sequence, Tuple

LatLngTuple = Tuple[float, float]

EARTH_RADIUS_M = 6371000.0
DEFAULT_MAX_UAV_ALTITUDE_HAE_M = 50.0
GEOFENCE_TOLERANCE_M = 1.0
EDGE_STEP_M = 0.5


def ensure_closed_ring(points: Sequence[LatLngTuple]) -> List[LatLngTuple]:
    """Ensure the polygon is a closed ring whose first and last (lat, lng) match."""
    pts = [(float(lat), float(lng)) for lat, lng in points]
    if not pts:
        return []
    if pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def load_geofence_from_qgc_plan(plan_path: str) -> List[LatLngTuple]:
    """Load the first polygon fence from a QGroundControl .plan or .fence JSON file.

    QGC stores polygon vertices in `geoFence.polygons[0].polygon` as `[lat, lon]`.
    Returns a closed list of `(lat, lng)` tuples.
    """
    with open(plan_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    geo_fence = data.get("geoFence", data)
    polygons = geo_fence.get("polygons", [])
    if not polygons:
        raise ValueError(f"No geoFence.polygons found in QGC plan file: {plan_path}")

    raw_poly = polygons[0].get("polygon", [])
    if len(raw_poly) < 3:
        raise ValueError(
            f"QGC polygon in {plan_path} has {len(raw_poly)} vertices; at least 3 are required"
        )

    pts = [(float(vertex[0]), float(vertex[1])) for vertex in raw_poly]
    return ensure_closed_ring(pts)


def _to_xy(p: LatLngTuple, origin: LatLngTuple) -> Tuple[float, float]:
    """Convert (lat, lng) to local (east_m, north_m) around origin."""
    return (
        math.radians(p[1] - origin[1]) * EARTH_RADIUS_M * math.cos(math.radians(origin[0])),
        math.radians(p[0] - origin[0]) * EARTH_RADIUS_M,
    )


def _to_latlng(xy: Tuple[float, float], origin: LatLngTuple) -> LatLngTuple:
    """Convert local (east_m, north_m) back to (lat, lng) around origin."""
    east_m, north_m = xy
    lat = origin[0] + math.degrees(north_m / EARTH_RADIUS_M)
    lng = origin[1] + math.degrees(
        east_m / (EARTH_RADIUS_M * math.cos(math.radians(origin[0])))
    )
    return (round(lat, 9), round(lng, 9))


def _pts_close(a: LatLngTuple, b: LatLngTuple, tol_deg: float = 1e-7) -> bool:
    return abs(a[0] - b[0]) <= tol_deg and abs(a[1] - b[1]) <= tol_deg


def merge_course_and_transit_rings(
    course_ring: Sequence[LatLngTuple],
    transit_ring: Sequence[LatLngTuple],
) -> List[LatLngTuple]:
    """Merge `RxCourse.corners` and `RxCourse.uav_transit_area` if they share an edge.

    If they do not share an edge or `transit_ring` is empty, returns `course_ring` closed.
    """
    c_closed = ensure_closed_ring(course_ring)
    t_closed = ensure_closed_ring(transit_ring)
    if len(c_closed) < 4 or len(t_closed) < 4:
        return c_closed

    c_verts = c_closed[:-1]
    t_verts = t_closed[:-1]
    n_c = len(c_verts)
    n_t = len(t_verts)

    for i in range(n_c):
        c_a = c_verts[i]
        c_b = c_verts[(i + 1) % n_c]
        for j in range(n_t):
            t_a = t_verts[j]
            t_b = t_verts[(j + 1) % n_t]
            # Check if edge (c_a -> c_b) matches (t_b -> t_a) or (t_a -> t_b)
            if _pts_close(c_a, t_b) and _pts_close(c_b, t_a):
                # Traverse c_verts up to i, then t_verts from j+1 wrapping around to j, then rest of c_verts
                transit_chain = [t_verts[(j + 1 + k) % n_t] for k in range(n_t - 1)]
                merged = c_verts[: i + 1] + transit_chain + c_verts[i + 1 :]
                return ensure_closed_ring(merged)
            if _pts_close(c_a, t_a) and _pts_close(c_b, t_b):
                transit_chain = [t_verts[(j - k) % n_t] for k in range(n_t - 1)]
                merged = c_verts[: i + 1] + transit_chain + c_verts[i + 1 :]
                return ensure_closed_ring(merged)

    return c_closed


def inset_polygon_ring(
    closed_ring: Sequence[LatLngTuple],
    inset_m: float,
) -> List[LatLngTuple]:
    """Inset a closed polygon ring toward its centroid by `inset_m` meters."""
    ring = ensure_closed_ring(closed_ring)
    if inset_m <= 0.0 or len(ring) < 4:
        return ring

    origin = ring[0]
    verts_xy = [_to_xy(p, origin) for p in ring[:-1]]
    cx = sum(v[0] for v in verts_xy) / len(verts_xy)
    cy = sum(v[1] for v in verts_xy) / len(verts_xy)

    inset_verts: List[LatLngTuple] = []
    for vx, vy in verts_xy:
        dx, dy = vx - cx, vy - cy
        dist = math.hypot(dx, dy)
        if dist > inset_m and dist > 1e-6:
            scale = (dist - inset_m) / dist
            nx, ny = cx + dx * scale, cy + dy * scale
        else:
            nx, ny = vx, vy
        inset_verts.append(_to_latlng((nx, ny), origin))

    return ensure_closed_ring(inset_verts)


def derive_geofence_from_course(
    course_corners: Sequence[LatLngTuple],
    transit_corners: Sequence[LatLngTuple] = (),
    include_transit: bool = True,
    inset_m: float = 0.0,
) -> List[LatLngTuple]:
    """Derive a valid closed UAV geofence polygon from `RxCourse`."""
    if include_transit and transit_corners:
        base_ring = merge_course_and_transit_rings(course_corners, transit_corners)
    else:
        base_ring = ensure_closed_ring(course_corners)
    return inset_polygon_ring(base_ring, inset_m=inset_m)


def _inside(p: Tuple[float, float], ring: Sequence[Tuple[float, float]]) -> bool:
    x, y = p
    inside = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _outside_by(p: Tuple[float, float], ring: Sequence[Tuple[float, float]]) -> float:
    if _inside(p, ring):
        return 0.0
    best = math.inf
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        dx, dy = bx - ax, by - ay
        t = (
            0.0
            if dx == dy == 0
            else max(
                0.0,
                min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)),
            )
        )
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def validate_uav_geofence(
    geofence: Sequence[LatLngTuple],
    max_altitude_hae_m: float,
    course_corners: Sequence[LatLngTuple] = (),
    transit_corners: Sequence[LatLngTuple] = (),
    max_allowed_altitude_hae_m: float = DEFAULT_MAX_UAV_ALTITUDE_HAE_M,
) -> List[str]:
    """Validate the UAV geofence using RoboCommand's exact acceptance criteria.

    Returns a list of human-readable problem descriptions (empty list means valid).
    """
    fence = [(float(lat), float(lng)) for lat, lng in geofence]
    problems: List[str] = []

    if len(fence) < 4:
        problems.append(f"uav_geofence has {len(fence)} point(s), needs at least 4")
    elif fence[0] != fence[-1]:
        problems.append(
            "uav_geofence isn't closed: its first and last points must be the same"
        )
    elif course_corners and len(course_corners) >= 4:
        origin = course_corners[0]
        raw_rings: List[Sequence[LatLngTuple]] = [ensure_closed_ring(course_corners)]
        if transit_corners and len(transit_corners) >= 4:
            raw_rings.append(ensure_closed_ring(transit_corners))

        areas = [[_to_xy(p, origin) for p in ring] for ring in raw_rings]
        pts = [_to_xy(p, origin) for p in fence]
        worst = 0.0
        where = None
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            steps = max(1, math.ceil(math.dist(a, b) / EDGE_STEP_M))
            for k in range(steps + 1):
                q = (
                    a[0] + (b[0] - a[0]) * k / steps,
                    a[1] + (b[1] - a[1]) * k / steps,
                )
                out = min(_outside_by(q, area) for area in areas)
                if out > worst:
                    worst = out
                    where = (
                        f"point {i + 1}"
                        if k == 0
                        else f"point {i + 2}"
                        if k == steps
                        else f"edge {i + 1}-{i + 2}"
                    )
        if worst > GEOFENCE_TOLERANCE_M:
            problems.append(
                f"uav_geofence leaves the course and transit area by {worst:.1f} m at {where}"
            )

    if not (0.0 < max_altitude_hae_m <= max_allowed_altitude_hae_m):
        problems.append(
            f"uav_geofence_max_altitude_hae_m is {max_altitude_hae_m:g}: "
            f"it must be above 0 and at most {max_allowed_altitude_hae_m:g}"
        )

    return problems
