"""The map behind frontier_explorer.py: what the lidar has seen, and where to
look next. No ROS in here (test/test_frontier_map.py runs it on its own).

A 2D grid of the floor. Every cell is one of

* obstacle: by the terrain filter's rule (terrain_filter.py): the returns in
  the cell span more than ``max_step``, or its ground is a ledge;
* free: a lidar ray at or below the sensor's height passed over it or ended
  in it, and it is not an obstacle;
* unknown: neither.

The robot can stand on the free cells further than ``clearance`` from every
obstacle. Those it can walk to from where it is, through such cells only, are
*reachable*. A *frontier* is a reachable cell next to an unknown one: standing
there, the robot sees something new. Frontier cells that touch form a
cluster; the explorer walks to the nearest cluster, and the map is complete
when no cluster is left.
"""

import heapq
from collections import deque

import numpy as np

UNKNOWN, FREE, OBSTACLE = -1, 0, 100

_NEIGHBOURS8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def _shift_any(mask, offsets):
    """True where ``mask`` is true at any of the offsets from the cell."""
    out = np.zeros_like(mask)
    pad = np.pad(mask, 1, constant_values=False)
    n, m = mask.shape
    for dx, dy in offsets:
        out |= pad[1 + dx:1 + dx + n, 1 + dy:1 + dy + m]
    return out


def _count_neighbours4(mask):
    pad = np.pad(mask, 1, constant_values=False).astype(np.int8)
    n, m = mask.shape
    return (pad[0:n, 1:m + 1] + pad[2:n + 2, 1:m + 1] + pad[1:n + 1, 0:m] + pad[1:n + 1, 2:m + 2])


def dilate(mask, cells):
    """``mask`` grown by a disc of ``cells`` cells."""
    if cells <= 0:
        return mask.copy()
    r = int(np.ceil(cells))
    offsets = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
               if dx * dx + dy * dy <= cells * cells]
    out = np.zeros_like(mask)
    pad = np.pad(mask, r, constant_values=False)
    n, m = mask.shape
    for dx, dy in offsets:
        out |= pad[r + dx:r + dx + n, r + dy:r + dy + m]
    return out


class Cluster:
    """Frontier cells that touch. ``goal`` is the cell to walk to (the one
    nearest their centre), ``cost`` the length of the way there (m)."""

    def __init__(self, cells, goal, cost):
        self.cells = cells
        self.goal = goal
        self.cost = cost

    def __len__(self):
        return len(self.cells)


class FrontierMap:

    def __init__(self, size=(30.0, 30.0), centre=(0.0, 0.0), resolution=0.1,
                 robot_height=0.55, max_step=0.09, clearance=0.4,
                 max_range=8.0, max_rays=4000, min_cluster=5, bounds=None):
        self.res = float(resolution)
        self.origin = np.array([centre[0] - size[0] / 2.0, centre[1] - size[1] / 2.0])
        self.shape = (int(np.ceil(size[0] / self.res)), int(np.ceil(size[1] / self.res)))
        self.robot_height = robot_height
        self.max_step = max_step
        self.clearance = clearance
        self.max_range = max_range
        self.max_rays = max_rays
        self.min_cluster = min_cluster
        self.ground = np.full(self.shape, np.inf, np.float32)
        self.top = np.full(self.shape, -np.inf, np.float32)
        self.seen = np.zeros(self.shape, bool)
        # Cells the explorer gave up on: not frontiers any more.
        self.given_up = np.zeros(self.shape, bool)
        # Frontiers only inside (xmin, ymin, xmax, ymax), if given.
        self.inside = np.ones(self.shape, bool)
        if bounds is not None:
            x = self.origin[0] + (np.arange(self.shape[0]) + 0.5) * self.res
            y = self.origin[1] + (np.arange(self.shape[1]) + 0.5) * self.res
            self.inside = np.outer((x >= bounds[0]) & (x <= bounds[2]),
                                   (y >= bounds[1]) & (y <= bounds[3]))
        self._rng = np.random.default_rng(0)

    # -- coordinates -----------------------------------------------------

    def cell(self, xy):
        """Cell of a point, clipped to the grid."""
        ij = np.floor((np.asarray(xy, float)[:2] - self.origin) / self.res).astype(int)
        return (int(np.clip(ij[0], 0, self.shape[0] - 1)), int(np.clip(ij[1], 0, self.shape[1] - 1)))

    def point(self, cell):
        """Centre of a cell."""
        return self.origin + (np.asarray(cell, float) + 0.5) * self.res

    def _cells(self, xy):
        ij = np.floor((xy - self.origin) / self.res).astype(np.int64)
        ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.shape[1])
        return ij, ok

    # -- sensor input ----------------------------------------------------

    def insert(self, sensor, pts):
        """One lidar scan: ``sensor`` (x, y, z) and its returns (n, 3), world frame."""
        sensor = np.asarray(sensor, float)
        pts = np.asarray(pts, float)
        if not len(pts):
            return
        # Heights, as in the terrain filter.
        ij, ok = self._cells(pts[:, :2])
        flat = ij[ok, 0] * self.shape[1] + ij[ok, 1]
        z = pts[ok, 2].astype(np.float32)
        ground = self.ground.ravel()
        np.minimum.at(ground, flat, z)
        band = z <= ground[flat] + self.robot_height
        np.maximum.at(self.top.ravel(), flat[band], z[band])

        # Seen: under the rays that do not climb. A ray that ends above the
        # sensor (the top of a wall, a table top) says nothing about the
        # floor below it.
        d = pts[:, :2] - sensor[:2]
        dist = np.hypot(d[:, 0], d[:, 1])
        rays = np.flatnonzero((pts[:, 2] <= sensor[2] + 0.02) & (dist > 1e-3) & (dist <= self.max_range))
        if len(rays) > self.max_rays:
            rays = self._rng.choice(rays, self.max_rays, replace=False)
        if not len(rays):
            return
        steps = int(np.ceil(dist[rays].max() / (0.5 * self.res))) + 1
        t = np.linspace(0.0, 1.0, steps)
        xy = sensor[:2] + d[rays, None, :] * t[None, :, None]
        ij, ok = self._cells(xy.reshape(-1, 2))
        self.seen[ij[ok, 0], ij[ok, 1]] = True

    def mark_seen(self, xy, radius):
        """The floor within ``radius`` of ``xy`` (under the robot, where the
        lidar does not look)."""
        ci, cj = self.cell(xy)
        r = int(np.ceil(radius / self.res))
        i0, i1 = max(ci - r, 0), min(ci + r + 1, self.shape[0])
        j0, j1 = max(cj - r, 0), min(cj + r + 1, self.shape[1])
        ii, jj = np.mgrid[i0:i1, j0:j1]
        near = (ii - ci) ** 2 + (jj - cj) ** 2 <= (radius / self.res) ** 2
        self.seen[i0:i1, j0:j1] |= near

    def give_up(self, xy, radius):
        """No frontiers within ``radius`` of ``xy`` from now on."""
        ci, cj = self.cell(xy)
        r = int(np.ceil(radius / self.res))
        i0, i1 = max(ci - r, 0), min(ci + r + 1, self.shape[0])
        j0, j1 = max(cj - r, 0), min(cj + r + 1, self.shape[1])
        ii, jj = np.mgrid[i0:i1, j0:j1]
        near = (ii - ci) ** 2 + (jj - cj) ** 2 <= (radius / self.res) ** 2
        self.given_up[i0:i1, j0:j1] |= near

    # -- the map ---------------------------------------------------------

    def obstacles(self):
        g, t = self.ground, self.top
        observed = np.isfinite(g)
        tall = observed & (t - g > self.max_step)
        pad = np.pad(g, 1, constant_values=np.inf)
        lowest = np.full_like(g, np.inf)
        n, m = g.shape
        for dx, dy in _NEIGHBOURS8:
            lowest = np.minimum(lowest, pad[1 + dx:1 + dx + n, 1 + dy:1 + dy + m])
        with np.errstate(invalid="ignore"):
            rise = g - lowest
            ledge = observed & (rise > self.max_step) & (rise < self.robot_height)
        return tall | ledge

    def grid(self):
        """UNKNOWN, FREE or OBSTACLE per cell (int8)."""
        obstacle = self.obstacles()
        known = self.seen | obstacle
        # The rays thin out with distance: a lone unknown cell among known
        # ones is a gap between two rays, not something left to look at.
        for _ in range(2):
            known |= ~known & (_count_neighbours4(known) >= 3)
        out = np.full(self.shape, UNKNOWN, np.int8)
        out[known] = FREE
        out[obstacle] = OBSTACLE
        return out

    def coverage(self, bounds):
        """Fraction of the cells inside (xmin, ymin, xmax, ymax) that are known."""
        i0, j0 = self.cell(bounds[:2])
        i1, j1 = self.cell(bounds[2:])
        part = self.grid()[i0:i1 + 1, j0:j1 + 1]
        return float((part != UNKNOWN).mean()) if part.size else 0.0

    # -- frontiers -------------------------------------------------------

    def plan(self, robot_xy, grid=None):
        """Frontier clusters the robot can walk to, nearest first.

        Returns (clusters, dist): ``dist`` is the walking distance (m) from
        the robot to every reachable cell, inf elsewhere."""
        if grid is None:
            grid = self.grid()
        standable = (grid == FREE) & ~dilate(grid == OBSTACLE, self.clearance / self.res)
        start = self._nearest(standable, self.cell(robot_xy), 0.6)
        dist = np.full(self.shape, np.inf)
        if start is None:
            return [], dist
        dist = self._distances(standable, start)
        reachable = np.isfinite(dist)
        frontier = (reachable & _shift_any(grid == UNKNOWN, [(-1, 0), (1, 0), (0, -1), (0, 1)])
                    & self.inside & ~self.given_up)
        clusters = []
        for cells in self._components(frontier):
            if len(cells) < self.min_cluster:
                continue
            arr = np.array(cells)
            centre = arr.mean(0)
            goal = cells[int(np.argmin(((arr - centre) ** 2).sum(1)))]
            clusters.append(Cluster(cells, goal, float(dist[goal]) * self.res))
        clusters.sort(key=lambda c: c.cost)
        return clusters, dist * self.res

    def path(self, dist, cell):
        """Cells from the robot to ``cell``, down the distances of plan()."""
        if not np.isfinite(dist[cell]):
            return []
        out = [cell]
        n, m = self.shape
        while dist[cell] > 0:
            best = cell
            for dx, dy in _NEIGHBOURS8:
                c = (cell[0] + dx, cell[1] + dy)
                if 0 <= c[0] < n and 0 <= c[1] < m and dist[c] < dist[best]:
                    best = c
            if best == cell:
                break
            cell = best
            out.append(cell)
        return out[::-1]

    def _nearest(self, mask, cell, radius):
        """``cell`` if ``mask`` is true there, else the nearest such cell
        within ``radius``, else None."""
        if mask[cell]:
            return cell
        r = int(np.ceil(radius / self.res))
        i0, i1 = max(cell[0] - r, 0), min(cell[0] + r + 1, self.shape[0])
        j0, j1 = max(cell[1] - r, 0), min(cell[1] + r + 1, self.shape[1])
        cand = np.argwhere(mask[i0:i1, j0:j1])
        if not len(cand):
            return None
        cand += (i0, j0)
        d = ((cand - cell) ** 2).sum(1)
        best = cand[np.argmin(d)]
        return (int(best[0]), int(best[1])) if d.min() <= r * r else None

    def _distances(self, mask, start):
        """Shortest way (in cells, diagonal steps sqrt 2) from ``start`` to
        every cell of ``mask`` connected to it."""
        n, m = mask.shape
        dist = np.full(mask.shape, np.inf)
        dist[start] = 0.0
        heap = [(0.0, start)]
        steps = [(dx, dy, float(np.hypot(dx, dy))) for dx, dy in _NEIGHBOURS8]
        while heap:
            d, (i, j) = heapq.heappop(heap)
            if d > dist[i, j]:
                continue
            for dx, dy, w in steps:
                a, b = i + dx, j + dy
                if 0 <= a < n and 0 <= b < m and mask[a, b] and d + w < dist[a, b]:
                    # No cutting a corner between two blocked cells.
                    if dx and dy and not (mask[i + dx, j] and mask[i, j + dy]):
                        continue
                    dist[a, b] = d + w
                    heapq.heappush(heap, (d + w, (a, b)))
        return dist

    def _components(self, mask):
        """Lists of the cells of ``mask`` that touch (also diagonally)."""
        todo = set(map(tuple, np.argwhere(mask).tolist()))
        out = []
        while todo:
            first = todo.pop()
            cells, queue = [first], deque([first])
            while queue:
                i, j = queue.popleft()
                for dx, dy in _NEIGHBOURS8:
                    c = (i + dx, j + dy)
                    if c in todo:
                        todo.remove(c)
                        cells.append(c)
                        queue.append(c)
            out.append(cells)
        return out
