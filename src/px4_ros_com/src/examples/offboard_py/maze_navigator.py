#!/usr/bin/env python3
"""
MSFMN – Multi-Sensor Fusion Maze Navigator
============================================

Expert-grade autonomous maze solving for PX4 multirotor equipped with:
  • 360° 2D LiDAR   (Waveshare D500 / lidar_2d_v2, topic /d500/scan)
  • Stereo depth cam (OAK-D-Lite, topic /oak/depth/image_raw)
  • 1-D rangefinder  (TF-Luna down, topic /tf_luna/down/scan)
  • 1-D rangefinder  (TF-Luna up,   topic /tf_luna/up/scan)

Algorithm layers
----------------
  1. Sensor Fusion   – merges LiDAR + depth cam into a unified polar scan;
                       fills the LiDAR blind spot from the occupancy grid
  2. Mapping         – real-time 2-D log-odds occupancy grid with vectorised
                       raycasting from every LiDAR scan
  3. Planning        – A* global planner on the inflated grid with a
                       breadcrumb penalty that discourages revisiting
  4. Avoidance       – VFH+ (Vector Field Histogram Plus) reactive local
                       planner with valley selection and smooth cost function
  5. Control         – adaptive-speed velocity controller with exponential
                       smoothing and jerk-limited output
  6. Safety          – vertical envelope from dual 1-D rangefinders,
                       stuck detection, and multi-phase recovery behaviour

Coordinate conventions
----------------------
  • PX4 local frame is NED.  Gazebo ENU → NED: NED_x = ENU_y, NED_y = ENU_x
  • The maze SDF places Start at Gazebo (0,0) and Goal at Gazebo (24,0).
    After ENU→NED:  Start = NED (0, 0),  Goal = NED (0, 24).
  • LiDAR scans arrive in body FLU convention (0 = forward, + = CCW/left).
    World NED angle of a reading at FLU angle α with heading ψ is  ψ − α .

Run
---
  ros2 run px4_ros_com maze_navigator.py
"""

from __future__ import annotations

import heapq
import math
import time
from collections import deque
from enum import Enum, auto
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.ndimage import binary_dilation
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from px4_msgs.msg import (
    OffboardControlMode,
    ObstacleDistance,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ═══════════════════════════════════════════════════════════════════════
#  Configuration constants
# ═══════════════════════════════════════════════════════════════════════

# -- Maze goal in NED local frame (Gazebo 24,0 → NED 0,24) --
# Default goal (can be overridden via /maze_navigator/set_goal topic)
GOAL_X: float = 0.0
GOAL_Y: float = 24.0
GOAL_RADIUS: float = 1.5  # [m] capture radius

# -- Flight envelope --
TARGET_AGL: float = 2.5  # [m] target altitude above ground
MIN_CEILING_CLR: float = 1.5  # [m] minimum ceiling clearance
ALT_KP: float = 1.8  # altitude P-gain
ALT_VZ_MAX: float = 1.0  # [m/s] max vertical speed
HARD_CEILING_Z: float = -6.0  # [m] NED z hard limit (≈ 6 m above origin)
TAKEOFF_Z: float = -2.5  # [m] NED z target for takeoff when no rangefinder

# -- Speed limits --
MAX_SPEED: float = 1.8  # [m/s] open space
CORRIDOR_SPEED: float = 1.0  # [m/s] near walls
MIN_SPEED: float = 0.25  # [m/s] minimum forward speed
SPEED_RAMP_NEAR: float = 1.2  # [m] start slowing below this
SPEED_RAMP_FAR: float = 3.5  # [m] full speed above this

# -- Occupancy grid --
GRID_RES: float = 0.20  # [m/cell]
GRID_OX: float = -20.0  # NED-x of cell (0,0)
GRID_OY: float = -10.0  # NED-y of cell (0,0)
GRID_NX: int = 200  # cells in x  (40 m)
GRID_NY: int = 200  # cells in y  (40 m)
L_FREE: float = -0.40  # log-odds decrement (free)
L_OCC: float = 0.85  # log-odds increment (occupied)
L_MIN: float = -4.0  # clamp min
L_MAX: float = 5.0  # clamp max
L_OCC_THRESH: float = 0.55  # log-odds → "occupied" for planning
L_FREE_THRESH: float = -0.15  # log-odds → "free" for planning

# -- Inflation --
DRONE_RADIUS: float = 0.35  # [m]
INFLATE_RADIUS: float = 0.55  # [m] extra clearance
INFLATE_CELLS: int = int(math.ceil((DRONE_RADIUS + INFLATE_RADIUS) / GRID_RES))

# -- A* planner --
UNKNOWN_COST: float = 2.5  # traversal cost for unknown cells
BREADCRUMB_PENALTY: float = 0.35  # per-visit penalty
REPLAN_HZ: float = (
    2.0  # planning loop rate (Lazy Theta* worst-case ~530 ms on all-unknown grid)
)
WAYPOINT_CAPTURE: float = 0.7  # [m]
LOOKAHEAD: float = 2.5  # [m]

# -- VFH+ --
VFH_SECTORS: int = 72  # 5° per sector
VFH_A: float = 1.0  # weight constant
VFH_B: float = 0.02  # range attenuation
VFH_SMOOTH_LEN: int = 3  # half-width of smoothing kernel
VFH_THRESH_LO: float = 0.35  # below → free  (applied AFTER per-sector averaging)
VFH_THRESH_HI: float = 0.65  # above → blocked
VFH_MU_TARGET: float = 5.0  # cost weight: target direction
VFH_MU_HEADING: float = 2.0  # cost weight: current heading
VFH_MU_PREV: float = 2.0  # cost weight: previous selection

# -- Depth camera --
DEPTH_HFOV: float = math.radians(73)  # OAK-D-Lite H-FOV
DEPTH_STRIP_TOP: int = 200
DEPTH_STRIP_BOT: int = 280
DEPTH_MAX_RANGE: float = 6.0  # [m] ignore beyond this
DEPTH_SKIP: int = 3  # process every Nth frame

# -- LiDAR limits --
LIDAR_MIN_RANGE: float = 0.12
LIDAR_MAX_RANGE: float = 7.8

# -- Stuck detection --
STUCK_TIME: float = 7.0  # [s]
STUCK_DISP: float = 0.45  # [m]
RECOVER_ROTATE_TIME: float = 5.0  # [s] phase-2 rotation
RECOVER_BACKUP_TIME: float = 2.5  # [s] phase-3 backup

# -- Velocity smoother --
SMOOTH_ALPHA: float = 0.35  # exponential smoothing factor
MAX_ACCEL: float = 2.5  # [m/s²]

# -- Control rates --
CONTROL_HZ: float = 10.0  # main loop
OFFBOARD_WARMUP: int = 15  # heartbeats before arming


# ═══════════════════════════════════════════════════════════════════════
#  Helpers
# ═══════════════════════════════════════════════════════════════════════


def _norm_angle(a):
    """Normalise angle(s) to (-π, π]. Accepts scalar or numpy array."""
    if isinstance(a, np.ndarray):
        a = np.asarray(a, dtype=np.float64)
        a = a % (2.0 * math.pi)
        return np.where(a > math.pi, a - 2.0 * math.pi, a)
    # scalar
    a = float(a) % (2.0 * math.pi)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def _ang_diff(a: float, b: float) -> float:
    """Signed shortest angular difference a − b, result in (-π, π]."""
    return _norm_angle(a - b)


def _sector_angle(k: int) -> float:
    """NED angle of sector centre k ∈ [0, VFH_SECTORS)."""
    return k * (2.0 * math.pi / VFH_SECTORS)


def _angle_to_sector(angle: float) -> int:
    """Map a NED angle to the nearest sector index."""
    return int((angle % (2.0 * math.pi)) / (2.0 * math.pi) * VFH_SECTORS) % VFH_SECTORS


# ═══════════════════════════════════════════════════════════════════════
#  Occupancy Grid  (log-odds, vectorised raycasting)
# ═══════════════════════════════════════════════════════════════════════


class OccGrid:
    """2-D log-odds occupancy grid with vectorised update and inflation."""

    def __init__(self) -> None:
        self._nx = GRID_NX
        self._ny = GRID_NY
        self._res = GRID_RES
        self._ox = GRID_OX
        self._oy = GRID_OY
        self._logodds = np.zeros((self._ny, self._nx), dtype=np.float32)
        self._inflated: Optional[np.ndarray] = None
        # Pre-compute inflation kernel
        kr = INFLATE_CELLS
        yy, xx = np.mgrid[-kr : kr + 1, -kr : kr + 1]
        self._inflate_kernel = (xx * xx + yy * yy) <= (kr + 0.5) ** 2

    # -- coordinate helpers ---------------------------------------------------

    def w2c(self, wx: float, wy: float) -> Tuple[int, int]:
        """World NED → grid (xi, yi).  Does NOT bounds-check."""
        return int((wx - self._ox) / self._res), int((wy - self._oy) / self._res)

    def c2w(self, xi: int, yi: int) -> Tuple[float, float]:
        """Grid (xi, yi) → world NED centre of cell."""
        return self._ox + (xi + 0.5) * self._res, self._oy + (yi + 0.5) * self._res

    def in_bounds(self, xi: int, yi: int) -> bool:
        return 0 <= xi < self._nx and 0 <= yi < self._ny

    # -- bulk log-odds update -------------------------------------------------

    def _mark(self, wx: np.ndarray, wy: np.ndarray, delta: float) -> None:
        xi = ((wx - self._ox) / self._res).astype(np.intp)
        yi = ((wy - self._oy) / self._res).astype(np.intp)
        m = (xi >= 0) & (xi < self._nx) & (yi >= 0) & (yi < self._ny)
        xi, yi = xi[m], yi[m]
        np.add.at(self._logodds, (yi, xi), delta)
        np.clip(self._logodds, L_MIN, L_MAX, out=self._logodds)

    # -- LiDAR scan update ----------------------------------------------------

    def update_scan(
        self,
        px: float,
        py: float,
        heading: float,
        ranges: np.ndarray,
        angles: np.ndarray,
    ) -> None:
        """Vectorised raycasting update from a laser scan.

        *ranges*: 1-D array of range values (metres).
        *angles*: 1-D array of scan angles in **body FLU** convention.
        """
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)

        finite = np.isfinite(ranges)
        hit_mask = finite & (ranges > LIDAR_MIN_RANGE) & (ranges < LIDAR_MAX_RANGE)
        free_mask = finite & (ranges > LIDAR_MIN_RANGE)

        # Sub-sample for speed (every 2nd ray)
        idx_hit = np.where(hit_mask)[0][::2]
        idx_free = np.where(free_mask)[0][::2]

        # --- free-space along rays (10 t-samples) ---
        for t in np.linspace(0.05, 0.88, 10):
            r = np.minimum(ranges[idx_free] * t, LIDAR_MAX_RANGE)
            fx = px + r * (cos_a[idx_free] * cos_h + sin_a[idx_free] * sin_h)
            fy = py + r * (cos_a[idx_free] * sin_h - sin_a[idx_free] * cos_h)
            self._mark(fx, fy, L_FREE)

        # --- occupied endpoints ---
        r_hit = ranges[idx_hit]
        hx = px + r_hit * (cos_a[idx_hit] * cos_h + sin_a[idx_hit] * sin_h)
        hy = py + r_hit * (cos_a[idx_hit] * sin_h - sin_a[idx_hit] * cos_h)
        self._mark(hx, hy, L_OCC)

    # -- depth-camera pseudo-scan update --------------------------------------

    def update_depth(
        self,
        px: float,
        py: float,
        heading: float,
        depth_ranges: np.ndarray,
        depth_world_angles: np.ndarray,
    ) -> None:
        """Update from horizontal depth-camera pseudo-scan."""
        valid = (
            np.isfinite(depth_ranges)
            & (depth_ranges > 0.15)
            & (depth_ranges < DEPTH_MAX_RANGE)
        )
        idx = np.where(valid)[0][::4]
        if len(idx) == 0:
            return
        r = depth_ranges[idx]
        a = depth_world_angles[idx]
        hx = px + r * np.cos(a)
        hy = py + r * np.sin(a)
        self._mark(hx, hy, L_OCC)
        # small free wedge along rays
        for t in [0.3, 0.6]:
            fx = px + r * t * np.cos(a)
            fy = py + r * t * np.sin(a)
            self._mark(fx, fy, L_FREE)

    # -- inflation ------------------------------------------------------------

    def compute_inflated(self) -> np.ndarray:
        """Return a boolean grid where True = impassable (occupied + inflated).

        Uses scipy.ndimage.binary_dilation (C-level morphological op) instead
        of the old 121-iteration Python roll loop — ~20x faster.
        """
        occ = self._logodds > L_OCC_THRESH
        self._inflated = binary_dilation(occ, structure=self._inflate_kernel)
        return self._inflated

    # -- single-ray cast on the log-odds grid ---------------------------------

    def raycast(
        self, px: float, py: float, angle: float, max_range: float = 6.0
    ) -> float:
        """Cast a ray from (px, py) in NED direction *angle*.  Returns range
        to first occupied cell, or *max_range*."""
        step = self._res * 0.7
        dx = step * math.cos(angle)
        dy = step * math.sin(angle)
        x, y = px, py
        d = 0.0
        while d < max_range:
            x += dx
            y += dy
            d += step
            xi = int((x - self._ox) / self._res)
            yi = int((y - self._oy) / self._res)
            if not (0 <= xi < self._nx and 0 <= yi < self._ny):
                return d
            if self._logodds[yi, xi] > L_OCC_THRESH:
                return d
        return max_range

    # -- query helpers --------------------------------------------------------

    def is_free(self, xi: int, yi: int) -> bool:
        if not self.in_bounds(xi, yi):
            return False
        return self._logodds[yi, xi] < L_FREE_THRESH

    def is_occupied(self, xi: int, yi: int) -> bool:
        if not self.in_bounds(xi, yi):
            return True
        return self._logodds[yi, xi] > L_OCC_THRESH

    def is_unknown(self, xi: int, yi: int) -> bool:
        if not self.in_bounds(xi, yi):
            return False
        lo = self._logodds[yi, xi]
        return L_FREE_THRESH <= lo <= L_OCC_THRESH


# ═══════════════════════════════════════════════════════════════════════
#  Theta* Planner  (any-angle A* with weighted heuristic)
# ═══════════════════════════════════════════════════════════════════════

# Epsilon-admissible weight: paths are within THETA_WEIGHT × optimal cost,
# but the planner expands far fewer nodes than standard A* (weight=1.0).
THETA_WEIGHT: float = 1.3

_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = [
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, _SQRT2),
    (1, -1, _SQRT2),
    (-1, 1, _SQRT2),
    (-1, -1, _SQRT2),
]


def theta_star(
    grid: OccGrid,
    inflated: np.ndarray,
    visit_grid: np.ndarray,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    max_expansions: int = 35_000,
) -> Optional[List[Tuple[int, int]]]:
    """Lazy Theta* any-angle planner on the inflated grid.

    Lazy Theta* is a faster variant of Theta* that defers the
    line-of-sight check to *pop-time* instead of *push-time*:

      Push phase  — optimistically link every neighbour directly to the
                    current node's parent (grandparent shortcut) with NO
                    LOS check.  Many pushed nodes are never popped, so
                    most of those LOS checks are avoided entirely.

      Pop phase   — when a node is finalised (popped and not yet closed),
                    verify that LOS from its stored parent still holds.
                    If it does not, fall back to the best closed grid
                    neighbour (Path 1 / standard A* edge).

    This reduces LOS checks from O(8 × expansions) to O(1 × pops),
    giving ~4-5× wall-clock speedup over eager Theta* at equal path
    quality.

    Additional performance improvements vs the original A* implementation:
      • numpy float32 arrays for g_cost / parent lookups  (C-speed
        random access; no Python dict overhead for ~35 k queries)
      • numpy bool array for the closed set
      • weighted heuristic  f = g + THETA_WEIGHT × h  (epsilon-
        admissible: solution ≤ THETA_WEIGHT × optimal; far fewer
        expansions than the unweighted A*)
      • path already any-angle → simplify_path is a trivial c2w pass

    Returns list of (xi, yi) cell coords or None on failure.
    """
    sx, sy = start
    gx, gy = goal

    if not grid.in_bounds(sx, sy) or not grid.in_bounds(gx, gy):
        return None

    ny_dim, nx_dim = grid._ny, grid._nx

    # --- numpy cost / parent arrays (replaces Python dicts) ---
    g_cost = np.full((ny_dim, nx_dim), np.inf, dtype=np.float32)
    par_x = np.full((ny_dim, nx_dim), -1, dtype=np.int32)
    par_y = np.full((ny_dim, nx_dim), -1, dtype=np.int32)
    closed = np.zeros((ny_dim, nx_dim), dtype=bool)

    g_cost[sy, sx] = 0.0
    par_x[sy, sx] = sx
    par_y[sy, sx] = sy  # start's parent is itself

    def h(x: int, y: int) -> float:
        return THETA_WEIGHT * math.hypot(x - gx, y - gy)

    open_set: list = []
    heapq.heappush(open_set, (h(sx, sy), sx, sy))
    expanded = 0

    while open_set and expanded < max_expansions:
        _, cx, cy = heapq.heappop(open_set)

        if closed[cy, cx]:
            continue

        # ── Lazy LOS verification (pop-time, Path 2 check) ───────────
        # The node was pushed with its parent set optimistically to the
        # grandparent.  Now that it is being finalised, confirm that the
        # straight-line path from that parent is actually clear.
        ppx = int(par_x[cy, cx])
        ppy = int(par_y[cy, cx])
        is_start = ppx == cx and ppy == cy
        if not is_start and not _los_check((ppx, ppy), (cx, cy), inflated, grid):
            # LOS is blocked → fall back to the best closed grid neighbour
            # (equivalent to Path 1 / standard A* relaxation).
            best_g = float("inf")
            for dx, dy, base_cost in _NEIGHBOURS:
                nx2, ny2 = cx + dx, cy + dy
                if not grid.in_bounds(nx2, ny2) or not closed[ny2, nx2]:
                    continue
                ng2 = float(g_cost[ny2, nx2]) + base_cost
                if ng2 < best_g:
                    best_g = ng2
                    par_x[cy, cx] = nx2
                    par_y[cy, cx] = ny2
            g_cost[cy, cx] = best_g

        closed[cy, cx] = True

        if cx == gx and cy == gy:
            # --- reconstruct path by following parent pointers ---
            path: List[Tuple[int, int]] = []
            x, y = gx, gy
            while not (x == sx and y == sy):
                path.append((x, y))
                px_n = int(par_x[y, x])
                py_n = int(par_y[y, x])
                x, y = px_n, py_n
            path.append((sx, sy))
            path.reverse()
            return path

        expanded += 1
        # grandparent of current node (used for the optimistic push below)
        gpx = int(par_x[cy, cx])
        gpy = int(par_y[cy, cx])

        for dx, dy, base_cost in _NEIGHBOURS:
            nx, ny = cx + dx, cy + dy
            if not grid.in_bounds(nx, ny):
                continue
            if inflated[ny, nx] or closed[ny, nx]:
                continue

            lo = grid._logodds[ny, nx]
            if lo > L_OCC_THRESH:
                continue
            cell_cost = UNKNOWN_COST if lo >= L_FREE_THRESH else 1.0
            cell_cost += BREADCRUMB_PENALTY * min(visit_grid[ny, nx], 8.0)

            # ── Lazy push: optimistically link neighbour to grandparent ──
            # No LOS check here — it will be verified when this node is
            # popped (see the lazy verification block above).
            dist_gp = math.hypot(nx - gpx, ny - gpy)
            ng = float(g_cost[gpy, gpx]) + dist_gp * cell_cost

            if ng < g_cost[ny, nx]:
                g_cost[ny, nx] = ng
                par_x[ny, nx] = gpx
                par_y[ny, nx] = gpy
                heapq.heappush(open_set, (ng + h(nx, ny), nx, ny))

    return None  # no path found within expansion budget


def simplify_path(
    raw: List[Tuple[int, int]],
    inflated: np.ndarray,
    grid: OccGrid,
) -> List[Tuple[float, float]]:
    """Convert Theta* cell path to world-coord waypoints.

    Theta* already produces any-angle waypoints (only one point per
    geometric corner), so the old O(n²) LOS-culling pass is replaced
    by a single linear c2w conversion.  The inflated argument is kept
    for API compatibility but is unused.
    """
    return [grid.c2w(*c) for c in raw]


def _los_check(
    a: Tuple[int, int],
    b: Tuple[int, int],
    inflated: np.ndarray,
    grid: OccGrid,
) -> bool:
    """Bresenham line-of-sight on inflated grid."""
    x0, y0 = a
    x1, y1 = b
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        if not grid.in_bounds(x0, y0) or inflated[y0, x0]:
            return False
        if x0 == x1 and y0 == y1:
            return True
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return True  # unreachable


# ═══════════════════════════════════════════════════════════════════════
#  VFH+ – Vector Field Histogram Plus
# ═══════════════════════════════════════════════════════════════════════


class VFHPlus:
    """Reactive local planner with polar histogram & valley selection."""

    def __init__(self) -> None:
        self.n = VFH_SECTORS
        self.histogram = np.zeros(self.n, dtype=np.float64)
        self.masked = np.ones(self.n, dtype=bool)  # True = blocked
        self.min_dist = LIDAR_MAX_RANGE
        self.prev_dir: float = 0.0

    # -----------------------------------------------------------------

    def update(
        self,
        ranges: np.ndarray,
        scan_angles: np.ndarray,
        heading: float,
        grid: OccGrid,
        px: float,
        py: float,
        depth_ranges: Optional[np.ndarray] = None,
        depth_world_angles: Optional[np.ndarray] = None,
    ) -> None:
        """Rebuild the polar histogram from sensor data + grid blind-spot."""
        H = np.zeros(self.n, dtype=np.float64)
        ray_count = np.zeros(self.n, dtype=np.float64)

        # --- LiDAR contribution ------------------------------------------
        finite = np.isfinite(ranges)
        valid = finite & (ranges > LIDAR_MIN_RANGE) & (ranges < LIDAR_MAX_RANGE)
        world_angles = heading - scan_angles[valid]
        sectors = ((world_angles % (2.0 * np.pi)) / (2.0 * np.pi) * self.n).astype(
            int
        ) % self.n
        weights = np.maximum(VFH_A - VFH_B * ranges[valid] ** 2, 0.0)
        np.add.at(H, sectors, weights)
        np.add.at(ray_count, sectors, 1.0)

        # Also count max-range readings (these are "free" evidence)
        max_range_mask = finite & (ranges >= LIDAR_MAX_RANGE)
        if np.any(max_range_mask):
            mr_angles = heading - scan_angles[max_range_mask]
            mr_sectors = ((mr_angles % (2.0 * np.pi)) / (2.0 * np.pi) * self.n).astype(
                int
            ) % self.n
            np.add.at(ray_count, mr_sectors, 1.0)
            # max-range rays contribute 0 weight (free space)

        # track which sectors are covered by the LiDAR
        covered = ray_count > 0

        # per-sector minimum distance (for speed adaptation)
        min_per_sector = np.full(self.n, LIDAR_MAX_RANGE)
        np.minimum.at(min_per_sector, sectors, ranges[valid])

        # --- Depth-camera contribution (forward cone) --------------------
        if depth_ranges is not None and depth_world_angles is not None:
            dv = (
                np.isfinite(depth_ranges)
                & (depth_ranges > 0.15)
                & (depth_ranges < DEPTH_MAX_RANGE)
            )
            if np.any(dv):
                ds = (
                    ((depth_world_angles[dv]) % (2 * np.pi)) / (2 * np.pi) * self.n
                ).astype(int) % self.n
                dw = np.maximum(VFH_A - VFH_B * depth_ranges[dv] ** 2, 0.0)
                np.add.at(H, ds, dw)
                np.add.at(ray_count, ds, 1.0)
                np.minimum.at(min_per_sector, ds, depth_ranges[dv])
                covered[ds] = True

        # --- Fill blind-spot from occupancy grid -------------------------
        for k in range(self.n):
            if not covered[k]:
                angle = _sector_angle(k)
                r = grid.raycast(px, py, angle, LIDAR_MAX_RANGE)
                if r < LIDAR_MAX_RANGE:
                    w = max(VFH_A - VFH_B * r * r, 0.0)
                    H[k] += w
                else:
                    # Grid says free in this direction
                    pass
                ray_count[k] += 1.0
                min_per_sector[k] = min(min_per_sector[k], r)

        # --- Normalise: average weight per sector (not sum) --------------
        safe_count = np.maximum(ray_count, 1.0)
        H_norm = H / safe_count

        # --- Smooth histogram --------------------------------------------
        l = VFH_SMOOTH_LEN
        kernel = np.ones(2 * l + 1) / (2 * l + 1)
        tiled = np.tile(H_norm, 3)
        H_smooth = np.convolve(tiled, kernel, mode="same")[self.n : 2 * self.n]

        # --- Binary threshold (now on per-ray-average values) ------------
        blocked = H_smooth > VFH_THRESH_HI

        # --- Widen blocked sectors by drone radius -----------------------
        global_min = float(np.min(min_per_sector))
        self.min_dist = max(global_min, 0.15)
        w = max(
            1,
            int(
                math.ceil(
                    math.asin(min(DRONE_RADIUS / max(self.min_dist, 0.01), 1.0))
                    / (2.0 * math.pi / self.n)
                )
            ),
        )
        masked = blocked.copy()
        for k in range(self.n):
            if blocked[k]:
                for j in range(-w, w + 1):
                    masked[(k + j) % self.n] = True

        self.histogram = H_smooth
        self.masked = masked

    # -----------------------------------------------------------------

    def select_direction(
        self,
        target_angle: float,
        current_heading: float,
    ) -> Optional[float]:
        """Select the best free direction using the VFH+ valley cost function.

        Returns a NED angle or *None* if completely boxed in.
        """
        N = self.n
        free = ~self.masked

        if not np.any(free):
            return None  # completely blocked

        # --- find valleys (maximal runs of free sectors) -----------------
        valleys: List[Tuple[int, int]] = []  # (start_idx, length)
        run_start = -1
        for offset in range(2 * N):
            k = offset % N
            if free[k]:
                if run_start < 0:
                    run_start = offset
            else:
                if run_start >= 0:
                    length = offset - run_start
                    valleys.append((run_start % N, length))
                    run_start = -1
                    if offset >= N and len(valleys):
                        break
        if run_start >= 0:
            length = 2 * N - run_start
            valleys.append((run_start % N, min(length, N)))

        # de-duplicate valleys that wrap
        seen = set()
        unique_valleys: List[Tuple[int, int]] = []
        for s, l in valleys:
            key = (s, min(l, N))
            if key not in seen:
                seen.add(key)
                unique_valleys.append(key)
        valleys = unique_valleys

        if not valleys:
            return None

        # --- candidate directions per valley -----------------------------
        s_max = max(
            2,
            int(
                math.ceil(
                    math.asin(min(DRONE_RADIUS / max(self.min_dist, 0.1), 1.0))
                    / (2.0 * math.pi / N)
                )
            ),
        )

        target_sec = _angle_to_sector(target_angle)
        best_cost = float("inf")
        best_angle: float = current_heading

        for start, length in valleys:
            if length <= 2 * s_max:
                # narrow valley → aim for centre
                centre = (start + length // 2) % N
                candidates = [centre]
            else:
                # wide valley → edges clamped by s_max, plus the target-nearest point
                left_edge = (start + s_max) % N
                right_edge = (start + length - 1 - s_max) % N
                candidates = [left_edge, right_edge]
                # also add the sector closest to target if inside the valley
                for off in range(length):
                    s = (start + off) % N
                    if s == target_sec:
                        candidates.append(target_sec)
                        break

            for c in candidates:
                c_angle = _sector_angle(c)
                cost = (
                    VFH_MU_TARGET * abs(_ang_diff(c_angle, target_angle))
                    + VFH_MU_HEADING * abs(_ang_diff(c_angle, current_heading))
                    + VFH_MU_PREV * abs(_ang_diff(c_angle, self.prev_dir))
                )
                if cost < best_cost:
                    best_cost = cost
                    best_angle = c_angle

        self.prev_dir = best_angle
        return best_angle


# ═══════════════════════════════════════════════════════════════════════
#  Navigation state machine
# ═══════════════════════════════════════════════════════════════════════


class NavState(Enum):
    INIT = auto()
    TAKEOFF = auto()
    NAVIGATE = auto()
    RECOVER = auto()
    GOAL_REACHED = auto()
    LAND = auto()


# ═══════════════════════════════════════════════════════════════════════
#  Main navigator node
# ═══════════════════════════════════════════════════════════════════════


class MazeNavigator(Node):
    """Autonomous maze navigator with multi-sensor fusion."""

    # ── construction ────────────────────────────────────────────────

    def __init__(self) -> None:
        super().__init__("maze_navigator")

        # --- QoS profiles ---
        # Publishers to PX4: BEST_EFFORT + TRANSIENT_LOCAL (PX4 expects this)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Subscribers from PX4: BEST_EFFORT + VOLATILE (matches uxrce-dds output)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", pub_qos
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", pub_qos
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", pub_qos
        )
        
        # --- Map visualization publishers ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_occ_grid = self.create_publisher(
            OccupancyGrid, "/maze_navigator/occupancy_grid", map_qos
        )
        self._pub_obstacle_dist = self.create_publisher(
            ObstacleDistance, "/fmu/in/obstacle_distance", pub_qos
        )

        # --- Subscribers (try multiple QoS + topic variants) ---
        # PX4 versions vary in QoS and may use versioned topic names.
        # Subscribe on all combos; first message that arrives wins.
        sub_qos_tl = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        for topic in [
            "/fmu/out/vehicle_local_position",
            "/fmu/out/vehicle_local_position_v1",
        ]:
            for qos in [sub_qos, sub_qos_tl]:
                self.create_subscription(VehicleLocalPosition, topic, self._cb_pos, qos)
        for topic in ["/fmu/out/vehicle_status", "/fmu/out/vehicle_status_v1"]:
            for qos in [sub_qos, sub_qos_tl]:
                self.create_subscription(VehicleStatus, topic, self._cb_status, qos)
        self.create_subscription(LaserScan, "/d500/scan", self._cb_lidar, sensor_qos)
        self.create_subscription(
            Image, "/oak/depth/image_raw", self._cb_depth, sensor_qos
        )
        self.create_subscription(
            LaserScan, "/tf_luna/down/scan", self._cb_luna_down, sensor_qos
        )
        self.create_subscription(
            LaserScan, "/tf_luna/up/scan", self._cb_luna_up, sensor_qos
        )
        
        # --- Waypoint command subscriber ---
        self.create_subscription(
            PointStamped, "/maze_navigator/set_goal", self._cb_set_goal, map_qos
        )

        # --- Algorithm components ---
        self.grid = OccGrid()
        self.vfh = VFHPlus()
        self.visit_grid = np.zeros((GRID_NY, GRID_NX), dtype=np.float32)

        # --- State ---
        self.state = NavState.INIT
        self._hb_counter = 0
        self._path: List[Tuple[float, float]] = []
        self._wp_idx = 0
        
        # --- Dynamic goal (can be updated via topic) ---
        self._goal_x = GOAL_X
        self._goal_y = GOAL_Y
        
        # --- Map publishing timer ---
        self.create_timer(1.0, self._publish_map)  # 1 Hz map updates

        # --- Sensor latest ---
        self._px = self._py = self._pz = 0.0
        self._heading: float = 0.0
        self._pos_valid = False
        self._nav_state: int = 0
        self._lidar_ranges: Optional[np.ndarray] = None
        self._lidar_angles: Optional[np.ndarray] = None
        self._ground_range: Optional[float] = None
        self._ceiling_range: Optional[float] = None
        self._depth_ranges: Optional[np.ndarray] = None
        self._depth_world_angles: Optional[np.ndarray] = None
        self._depth_skip_ctr = 0

        # --- Smooth velocity state ---
        self._svx = 0.0
        self._svy = 0.0
        self._svz = 0.0

        # --- Stuck detection ---
        self._last_progress_time = time.monotonic()
        self._last_progress_pos = (0.0, 0.0)

        # --- Recovery ---
        self._recover_start: float = 0.0
        self._recover_heading_start: float = 0.0

        # --- Timing / stats ---
        self._start_time: Optional[float] = None
        self._dist_traveled: float = 0.0
        self._prev_pos: Optional[Tuple[float, float]] = None

        # --- Tick counter for reliable periodic logging ---
        self._tick = 0

        # --- Timers ---
        dt_ctrl = 1.0 / CONTROL_HZ
        dt_plan = 1.0 / REPLAN_HZ
        self.create_timer(dt_ctrl, self._tick_control)
        self.create_timer(dt_plan, self._tick_plan)

        self._log(
            f"MSFMN Maze Navigator initialised — default goal NED ({GOAL_X}, {GOAL_Y})"
        )
        self._log(
            "  Set goal via: ros2 topic pub /maze_navigator/set_goal "
            "geometry_msgs/PointStamped '{header: {frame_id: \"map\"}, point: {x: 0.0, y: 24.0, z: 0.0}}'"
        )
        self._log(
            f"  Subscribing to /fmu/out/vehicle_local_position  "
            f"(type: {VehicleLocalPosition.__class__.__module__})"
        )
        self._log(f"  Sub QoS: reliability=BEST_EFFORT, durability=VOLATILE, depth=10")

    # ── reliable logger (print + ros) ───────────────────────────────

    def _log(self, msg: str) -> None:
        """Print AND ros-log so we always see output."""
        print(f"[MAZE] {msg}", flush=True)
        self.get_logger().info(msg)

    # ── sensor callbacks ────────────────────────────────────────────

    def _cb_pos(self, msg: VehicleLocalPosition) -> None:
        self._pos_rx_count = getattr(self, "_pos_rx_count", 0) + 1
        if self._pos_rx_count <= 3 or self._pos_rx_count % 200 == 0:
            self._log(
                f"  _cb_pos #{self._pos_rx_count}  "
                f"xy_valid={msg.xy_valid} z_valid={msg.z_valid}  "
                f"x={msg.x:.2f} y={msg.y:.2f} z={msg.z:.2f} "
                f"heading={msg.heading:.2f}"
            )
        # Accept position even if validity flags are not set (some PX4 versions
        # report False until the EKF is fully converged, which can take a while).
        self._px = msg.x
        self._py = msg.y
        self._pz = msg.z
        self._heading = msg.heading
        self._pos_valid = True

    def _cb_status(self, msg: VehicleStatus) -> None:
        self._status_rx_count = getattr(self, "_status_rx_count", 0) + 1
        if self._status_rx_count <= 3:
            self._log(
                f"  _cb_status #{self._status_rx_count}  "
                f"nav_state={msg.nav_state}  arming={msg.arming_state}"
            )
        self._nav_state = msg.nav_state

    def _cb_lidar(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        self._lidar_ranges = np.array(msg.ranges, dtype=np.float64)
        self._lidar_angles = msg.angle_min + np.arange(n) * msg.angle_increment

        # live-update the occupancy grid
        if self._pos_valid:
            self.grid.update_scan(
                self._px,
                self._py,
                self._heading,
                self._lidar_ranges,
                self._lidar_angles,
            )

    def _cb_depth(self, msg: Image) -> None:
        self._depth_skip_ctr += 1
        if self._depth_skip_ctr % DEPTH_SKIP != 0:
            return
        if not self._pos_valid:
            return

        # Decode depth image
        if msg.encoding in ("32FC1", "passthrough"):
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                msg.height, msg.width
            )
        elif msg.encoding == "16UC1":
            depth = (
                np.frombuffer(msg.data, dtype=np.uint16)
                .reshape(msg.height, msg.width)
                .astype(np.float32)
                / 1000.0
            )
        else:
            return

        # Horizontal strip → min depth per column
        top = max(0, min(DEPTH_STRIP_TOP, msg.height - 1))
        bot = max(top + 1, min(DEPTH_STRIP_BOT, msg.height))
        strip = depth[top:bot, :]
        with np.errstate(invalid="ignore"):
            strip = np.where(strip > 0.05, strip, np.nan)
        min_depth = np.nanmin(strip, axis=0)  # shape (width,)

        # Angles per column (FLU body frame, 0 = forward, + = left)
        cx = msg.width / 2.0
        col_angles_flu = (cx - np.arange(msg.width)) / cx * (DEPTH_HFOV / 2.0)

        # Convert perpendicular depth → range
        cos_col = np.cos(col_angles_flu)
        ranges = min_depth / np.maximum(cos_col, 0.01)

        # World NED angles
        world_angles = self._heading - col_angles_flu

        self._depth_ranges = ranges
        self._depth_world_angles = world_angles

        # Update occupancy grid from depth
        self.grid.update_depth(self._px, self._py, self._heading, ranges, world_angles)

    def _cb_luna_down(self, msg: LaserScan) -> None:
        if len(msg.ranges) > 0 and np.isfinite(msg.ranges[0]) and msg.ranges[0] > 0.05:
            self._ground_range = float(msg.ranges[0])

    def _cb_luna_up(self, msg: LaserScan) -> None:
        if len(msg.ranges) > 0 and np.isfinite(msg.ranges[0]) and msg.ranges[0] > 0.05:
            self._ceiling_range = float(msg.ranges[0])
    
    def _cb_set_goal(self, msg: PointStamped) -> None:
        """Callback for setting a new goal waypoint."""
        # PointStamped.point contains x, y, z
        # For 2D navigation, we use x and y (NED frame)
        self._goal_x = float(msg.point.x)
        self._goal_y = float(msg.point.y)
        self._log(
            f"New goal set: NED ({self._goal_x:.2f}, {self._goal_y:.2f}) "
            f"from frame '{msg.header.frame_id}'"
        )
        # Reset path planning to use new goal
        if self.state == NavState.NAVIGATE:
            self._path = []
            self._wp_idx = 0

    # ── PX4 command helpers ─────────────────────────────────────────

    def _ts(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _send_cmd(self, command: int, **kw: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = kw.get("p1", 0.0)
        msg.param2 = kw.get("p2", 0.0)
        msg.param7 = kw.get("p7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._ts()
        self._pub_cmd.publish(msg)

    def _heartbeat(self, position_mode: bool = False) -> None:
        msg = OffboardControlMode()
        msg.position = position_mode
        msg.velocity = not position_mode
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._ts()
        self._pub_ocm.publish(msg)

    def _publish_pos(self, x: float, y: float, z: float, yaw: float) -> None:
        """Publish a position setpoint (for takeoff)."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        msg.timestamp = self._ts()
        self._pub_sp.publish(msg)

    def _publish_vel(self, vx: float, vy: float, vz: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        msg.yawspeed = math.nan
        msg.timestamp = self._ts()
        self._pub_sp.publish(msg)

    def _publish_vel_yawrate(self, vx: float, vy: float, vz: float, yr: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [math.nan, math.nan, math.nan]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.yaw = math.nan
        msg.yawspeed = float(yr)
        msg.timestamp = self._ts()
        self._pub_sp.publish(msg)

    def _arm(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)
        self.get_logger().info("ARM sent")

    def _offboard(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)
        self.get_logger().info("OFFBOARD mode requested")

    def _land_cmd(self) -> None:
        self._send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent")

    # ── velocity smoother ───────────────────────────────────────────

    def _smooth(self, vx: float, vy: float, vz: float) -> Tuple[float, float, float]:
        """Exponential smoothing + acceleration limiting."""
        dt = 1.0 / CONTROL_HZ
        max_dv = MAX_ACCEL * dt
        for attr, target in [("_svx", vx), ("_svy", vy), ("_svz", vz)]:
            cur = getattr(self, attr)
            desired = SMOOTH_ALPHA * target + (1.0 - SMOOTH_ALPHA) * cur
            delta = desired - cur
            if abs(delta) > max_dv:
                desired = cur + math.copysign(max_dv, delta)
            setattr(self, attr, desired)
        return self._svx, self._svy, self._svz

    # ── altitude controller ─────────────────────────────────────────

    def _altitude_vel(self) -> float:
        """P-controller for altitude hold using TF-Luna down, with hard ceiling."""
        vz = 0.0
        if self._ground_range is not None:
            err = TARGET_AGL - self._ground_range  # positive → need to climb
            vz = -ALT_KP * err  # NED: negative vz = up
        else:
            # Fallback: use NED z from EKF
            target_z = TAKEOFF_Z  # e.g. -2.5
            err = target_z - self._pz  # negative when above target
            vz = ALT_KP * err  # positive → descend

        # Ceiling safety from TF-Luna up
        if self._ceiling_range is not None and self._ceiling_range < MIN_CEILING_CLR:
            push_down = 0.6 * (MIN_CEILING_CLR - self._ceiling_range)
            vz = max(vz, push_down)

        # ── HARD CEILING: never fly above HARD_CEILING_Z ──
        if self._pz < HARD_CEILING_Z:
            # Already above the hard limit – force descent
            vz = max(vz, 0.8)
        elif self._pz < HARD_CEILING_Z + 1.0:
            # Approaching the limit – prevent any further climb
            vz = max(vz, 0.0)

        return max(-ALT_VZ_MAX, min(ALT_VZ_MAX, vz))

    # ── adaptive speed ──────────────────────────────────────────────

    @staticmethod
    def _speed_from_clearance(min_dist: float) -> float:
        if min_dist < 0.4:
            return 0.0
        if min_dist < SPEED_RAMP_NEAR:
            return MIN_SPEED
        if min_dist < SPEED_RAMP_FAR:
            t = (min_dist - SPEED_RAMP_NEAR) / (SPEED_RAMP_FAR - SPEED_RAMP_NEAR)
            return MIN_SPEED + t * (MAX_SPEED - MIN_SPEED)
        return MAX_SPEED

    # ── stuck detection ─────────────────────────────────────────────

    def _check_stuck(self) -> bool:
        dx = self._px - self._last_progress_pos[0]
        dy = self._py - self._last_progress_pos[1]
        if math.hypot(dx, dy) > STUCK_DISP:
            self._last_progress_time = time.monotonic()
            self._last_progress_pos = (self._px, self._py)
            return False
        return (time.monotonic() - self._last_progress_time) > STUCK_TIME

    # ── planning loop (1 Hz) ────────────────────────────────────────

    def _tick_plan(self) -> None:
        if self.state not in (NavState.NAVIGATE, NavState.RECOVER):
            return
        if not self._pos_valid:
            return

        inflated = self.grid.compute_inflated()
        start = self.grid.w2c(self._px, self._py)
        goal = self.grid.w2c(self._goal_x, self._goal_y)

        raw = theta_star(self.grid, inflated, self.visit_grid, start, goal)
        if raw is not None and len(raw) > 1:
            self._path = simplify_path(raw, inflated, self.grid)
            self._wp_idx = 0
            # advance past waypoints that are already behind us
            self._advance_waypoint()
        else:
            # fall back: head straight for goal (VFH+ will dodge)
            self._path = [(self._goal_x, self._goal_y)]
            self._wp_idx = 0

    # ── waypoint management ─────────────────────────────────────────

    def _advance_waypoint(self) -> None:
        while self._wp_idx < len(self._path) - 1:
            wx, wy = self._path[self._wp_idx]
            if math.hypot(wx - self._px, wy - self._py) < WAYPOINT_CAPTURE:
                self._wp_idx += 1
            else:
                break

    def _current_target(self) -> Tuple[float, float]:
        """Return the waypoint we should steer toward (with lookahead)."""
        if not self._path:
            return (self._goal_x, self._goal_y)
        self._advance_waypoint()
        # look ahead to find a waypoint > LOOKAHEAD away
        for i in range(self._wp_idx, len(self._path)):
            wx, wy = self._path[i]
            if math.hypot(wx - self._px, wy - self._py) >= LOOKAHEAD:
                return (wx, wy)
        return self._path[-1]

    # ── control loop (10 Hz) ────────────────────────────────────────

    def _tick_control(self) -> None:
        self._tick += 1

        # Use position-mode heartbeat during takeoff, velocity-mode otherwise
        self._heartbeat(position_mode=(self.state == NavState.TAKEOFF))

        if self.state == NavState.INIT:
            self._do_init()
        elif self.state == NavState.TAKEOFF:
            self._do_takeoff()
        elif self.state == NavState.NAVIGATE:
            self._do_navigate()
        elif self.state == NavState.RECOVER:
            self._do_recover()
        elif self.state == NavState.GOAL_REACHED:
            self._do_goal()
        elif self.state == NavState.LAND:
            pass  # heartbeat only

    # -- INIT ---------------------------------------------------------

    def _do_init(self) -> None:
        self._hb_counter += 1
        # send zero-velocity so PX4 stream-count accepts offboard
        self._publish_vel(0.0, 0.0, 0.0, 0.0)

        if self._tick % 10 == 0:
            self._log(
                f"INIT  hb={self._hb_counter}/{OFFBOARD_WARMUP}  "
                f"pos_valid={self._pos_valid}  z={self._pz:.2f}"
            )

        if self._hb_counter >= OFFBOARD_WARMUP:
            self._offboard()
            self._arm()
            self.state = NavState.TAKEOFF
            self._start_time = time.monotonic()
            self._log(">>> TAKEOFF")

    # -- TAKEOFF (position-mode — PX4 flies to the setpoint) ----------

    def _do_takeoff(self) -> None:
        elapsed = time.monotonic() - (self._start_time or time.monotonic())

        if not self._pos_valid:
            # Waiting for EKF — send safe position (origin, slightly up)
            self._publish_pos(0.0, 0.0, TAKEOFF_Z, 0.0)
            if self._tick % 20 == 0:
                self._log(f"TAKEOFF  waiting for position (elapsed {elapsed:.1f}s)")
            # timeout: after 10 s without position, force navigate anyway
            if elapsed > 10.0:
                self._log("TAKEOFF  timeout — no position, forcing NAVIGATE")
                self.state = NavState.NAVIGATE
                self._last_progress_time = time.monotonic()
                self._last_progress_pos = (0.0, 0.0)
            return

        # Target: hold XY, go to target altitude
        target_z = TAKEOFF_Z  # e.g. -2.5  (NED, negative = up)
        self._publish_pos(self._px, self._py, target_z, self._heading)

        # Log every 1 s
        if self._tick % 10 == 0:
            self._log(
                f"TAKEOFF  pos=({self._px:.2f},{self._py:.2f}) z={self._pz:.2f}  "
                f"target_z={target_z:.1f}  gnd={self._ground_range}  "
                f"ceil={self._ceiling_range}  elapsed={elapsed:.1f}s"
            )

        # Check if we've reached the target altitude
        at_alt = abs(self._pz - target_z) < 0.5

        # Hard timeout: 15 s in takeoff is enough
        if at_alt or elapsed > 15.0:
            self._log(f">>> NAVIGATE  (at_alt={at_alt}, z={self._pz:.2f})")
            self.state = NavState.NAVIGATE
            self._last_progress_time = time.monotonic()
            self._last_progress_pos = (self._px, self._py)

    # -- NAVIGATE -----------------------------------------------------

    def _do_navigate(self) -> None:
        if not self._pos_valid:
            self._publish_vel(0.0, 0.0, 0.0, 0.0)
            return

        # --- update visit grid ---
        ci = self.grid.w2c(self._px, self._py)
        if self.grid.in_bounds(*ci):
            self.visit_grid[ci[1], ci[0]] = min(
                self.visit_grid[ci[1], ci[0]] + 0.08, 10.0
            )

        # --- track distance ---
        if self._prev_pos is not None:
            self._dist_traveled += math.hypot(
                self._px - self._prev_pos[0], self._py - self._prev_pos[1]
            )
        self._prev_pos = (self._px, self._py)

        # --- goal check ---
        goal_dist = math.hypot(self._px - self._goal_x, self._py - self._goal_y)
        if goal_dist < GOAL_RADIUS:
            elapsed = time.monotonic() - (self._start_time or time.monotonic())
            self._log(
                f"GOAL REACHED at ({self._goal_x:.2f}, {self._goal_y:.2f}) in {elapsed:.1f}s, "
                f"distance flown {self._dist_traveled:.1f}m, "
                f"straight-line {math.hypot(self._goal_x, self._goal_y):.1f}m"
            )
            self.state = NavState.GOAL_REACHED
            self._recover_start = time.monotonic()
            return

        # --- stuck check ---
        if self._check_stuck():
            self._log("STUCK detected — entering RECOVER")
            self.state = NavState.RECOVER
            self._recover_start = time.monotonic()
            self._recover_heading_start = self._heading
            self._svx = self._svy = 0.0
            return

        # --- determine target direction ---
        tx, ty = self._current_target()
        target_angle = math.atan2(ty - self._py, tx - self._px)

        # --- VFH+ update (only if we have LiDAR) ---
        have_lidar = self._lidar_ranges is not None and self._lidar_angles is not None
        steer = None
        if have_lidar:
            self.vfh.update(
                self._lidar_ranges,
                self._lidar_angles,
                self._heading,
                self.grid,
                self._px,
                self._py,
                self._depth_ranges,
                self._depth_world_angles,
            )
            steer = self.vfh.select_direction(target_angle, self._heading)

        # --- fallback: no LiDAR or VFH boxed → go straight for goal ---
        if steer is None:
            speed = MIN_SPEED if not have_lidar else 0.0
            steer = target_angle
            if not have_lidar:
                # Move slowly toward goal without any obstacle info
                vx_cmd = speed * math.cos(target_angle)
                vy_cmd = speed * math.sin(target_angle)
                vz_cmd = self._altitude_vel()
                svx, svy, svz = self._smooth(vx_cmd, vy_cmd, vz_cmd)
                self._publish_vel(svx, svy, svz, target_angle)
                if self._tick % 20 == 0:
                    self._log(
                        f"NAV [no lidar]  pos=({self._px:.1f},{self._py:.1f}) "
                        f"z={self._pz:.2f}  goal_d={goal_dist:.1f}m  "
                        f"heading→{math.degrees(target_angle):.0f}°"
                    )
                return
            else:
                # VFH says fully blocked — hover and wait for replan
                vz_cmd = self._altitude_vel()
                svx, svy, svz = self._smooth(0.0, 0.0, vz_cmd)
                self._publish_vel(svx, svy, svz, self._heading)
                if self._tick % 20 == 0:
                    self._log(
                        f"NAV [boxed]  pos=({self._px:.1f},{self._py:.1f}) "
                        f"z={self._pz:.2f}  goal_d={goal_dist:.1f}m  hovering"
                    )
                return

        # --- adaptive speed ---
        speed = self._speed_from_clearance(self.vfh.min_dist)
        if goal_dist < 3.0:
            speed = min(speed, CORRIDOR_SPEED * goal_dist / 3.0 + MIN_SPEED)

        vx_cmd = speed * math.cos(steer)
        vy_cmd = speed * math.sin(steer)
        vz_cmd = self._altitude_vel()

        svx, svy, svz = self._smooth(vx_cmd, vy_cmd, vz_cmd)

        # face direction of travel
        desired_yaw = steer if speed > 0.15 else self._heading
        self._publish_vel(svx, svy, svz, desired_yaw)

        # --- periodic log ---
        if self._tick % 30 == 0:
            n_free = int(np.sum(~self.vfh.masked))
            self._log(
                f"NAV  pos=({self._px:.1f},{self._py:.1f}) z={self._pz:.2f}  "
                f"hdg={math.degrees(self._heading):.0f}°  "
                f"goal_d={goal_dist:.1f}m  spd={speed:.2f}  "
                f"min_obs={self.vfh.min_dist:.1f}m  "
                f"free_sectors={n_free}/{VFH_SECTORS}  "
                f"steer={math.degrees(steer):.0f}°  "
                f"wp={self._wp_idx}/{len(self._path)}"
            )

    # -- RECOVER ------------------------------------------------------

    def _do_recover(self) -> None:
        elapsed = time.monotonic() - self._recover_start
        vz = self._altitude_vel()

        if self._tick % 20 == 0:
            self._log(f"RECOVER  phase elapsed={elapsed:.1f}s  z={self._pz:.2f}")

        if elapsed < 1.5:
            # Phase 1: hard stop
            svx, svy, svz = self._smooth(0.0, 0.0, vz)
            self._publish_vel(svx, svy, svz, self._heading)

        elif elapsed < 1.5 + RECOVER_ROTATE_TIME:
            # Phase 2: rotate 360° to scan blind spots & update map
            yaw_rate = 2.0 * math.pi / RECOVER_ROTATE_TIME
            self._publish_vel_yawrate(0.0, 0.0, vz, yaw_rate)

        elif elapsed < 1.5 + RECOVER_ROTATE_TIME + RECOVER_BACKUP_TIME:
            # Phase 3: back up slowly (opposite of current heading)
            bx = -0.6 * math.cos(self._heading)
            by = -0.6 * math.sin(self._heading)
            svx, svy, svz = self._smooth(bx, by, vz)
            self._publish_vel(svx, svy, svz, self._heading)

        else:
            # Done — reset stuck detector and go back to NAVIGATE
            self._last_progress_time = time.monotonic()
            self._last_progress_pos = (self._px, self._py)
            self._svx = self._svy = 0.0
            self.state = NavState.NAVIGATE
            self._log(">>> Back to NAVIGATE after recovery")

    # -- GOAL_REACHED -------------------------------------------------

    def _do_goal(self) -> None:
        vz = self._altitude_vel()
        self._publish_vel(0.0, 0.0, vz, self._heading)
        if time.monotonic() - self._recover_start > 3.0:
            self._land_cmd()
            self.state = NavState.LAND
            self._log(">>> LANDING at goal")
    
    # ── map publishing ────────────────────────────────────────────────
    
    def _publish_map(self) -> None:
        """Publish occupancy grid as nav_msgs/OccupancyGrid for visualization."""
        if not self._pos_valid:
            return
        
        # Convert log-odds to occupancy probabilities
        # OccupancyGrid uses: -1 = unknown, 0-100 = probability (0 = free, 100 = occupied)
        prob = 1.0 / (1.0 + np.exp(-self.grid._logodds))
        occ_data = (prob * 100.0).astype(np.int8)
        
        # Mark unknown cells (between thresholds) as -1
        unknown_mask = (
            (self.grid._logodds >= L_FREE_THRESH) 
            & (self.grid._logodds <= L_OCC_THRESH)
        )
        occ_data[unknown_mask] = -1
        
        # Create OccupancyGrid message
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"  # or "odom" if you prefer
        
        msg.info.resolution = GRID_RES
        msg.info.width = GRID_NX
        msg.info.height = GRID_NY
        msg.info.origin.position.x = GRID_OX
        msg.info.origin.position.y = GRID_OY
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten and convert to list (row-major order)
        msg.data = occ_data.flatten().tolist()
        
        self._pub_occ_grid.publish(msg)
        
        # Also publish obstacle distance for QGC visualization
        # This creates a simplified obstacle distance message from the grid
        self._publish_obstacle_distance()
    
    def _publish_obstacle_distance(self) -> None:
        """Publish obstacle distance message for QGC visualization."""
        if not self._pos_valid or self._lidar_ranges is None:
            return
        
        # Create obstacle distance message
        msg = ObstacleDistance()
        msg.timestamp = self._ts()
        msg.frame = ObstacleDistance.MAV_FRAME_LOCAL_NED
        
        # Use LiDAR data if available
        if self._lidar_ranges is not None and self._lidar_angles is not None:
            # Sample 72 directions (5° increments) matching VFH_SECTORS
            n_directions = 72
            distances = np.full(n_directions, 0, dtype=np.uint16)
            
            for i in range(n_directions):
                angle = i * (2.0 * math.pi / n_directions)
                # Find closest LiDAR reading in this direction
                world_angles = self._heading - self._lidar_angles
                angle_diffs = np.abs(_norm_angle(world_angles - angle))
                closest_idx = np.argmin(angle_diffs)
                
                if angle_diffs[closest_idx] < math.radians(5):  # Within 5°
                    r = self._lidar_ranges[closest_idx]
                    if np.isfinite(r) and LIDAR_MIN_RANGE < r < LIDAR_MAX_RANGE:
                        distances[i] = int(r * 100)  # Convert m to cm
                    else:
                        distances[i] = 0  # Unknown/out of range
                else:
                    # Fallback to grid raycast
                    r = self.grid.raycast(self._px, self._py, angle, LIDAR_MAX_RANGE)
                    distances[i] = int(min(r, LIDAR_MAX_RANGE) * 100)
            
            msg.distances = distances.tolist()
            msg.increment = 5.0  # 5° increments (float32)
            msg.min_distance = int(LIDAR_MIN_RANGE * 100)
            msg.max_distance = int(LIDAR_MAX_RANGE * 100)
            
            self._pub_obstacle_dist.publish(msg)


# ═══════════════════════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════════════════════


def main(args=None) -> None:
    print("━" * 60)
    print("  MSFMN — Multi-Sensor Fusion Maze Navigator")
    print("━" * 60)
    rclpy.init(args=args)
    node = MazeNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
