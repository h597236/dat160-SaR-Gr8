import math
from collections import deque

import numpy as np


class Coverage:

    def __init__(self, planner, start_xy,
                 min_range=1.0, max_range=2.8, max_view_angle=65.0, half_fov=28.0,
                 candidate_spacing=0.3, candidate_clearance=0.4, goal_coverage=0.95):
        self.p = planner
        self.min_range = min_range
        self.max_range = max_range
        self.max_view = math.radians(max_view_angle)
        self.half_fov = math.radians(half_fov)
        self.goal_coverage = goal_coverage

        start = planner.nearest_free(planner.to_cell(*start_xy), max_dist=0.5)
        self.reachable = self.flood(start, ~planner.blocked)
        interior = self.flood(start, ~planner.walls)

        w = planner.walls
        next_to_wall = np.zeros_like(w)
        next_to_wall[1:, :] |= w[:-1, :]
        next_to_wall[:-1, :] |= w[1:, :]
        next_to_wall[:, 1:] |= w[:, :-1]
        next_to_wall[:, :-1] |= w[:, 1:]
        cells = np.argwhere(interior & next_to_wall & ~w)
        self.target_cells = cells
        self.target_xy = np.array([planner.to_world(r, c) for r, c in cells])
        self.normals = self.wall_normals(cells)
        self.seen = np.zeros(len(cells), dtype=bool)
        self.not_white = np.zeros(len(cells), dtype=bool)

        step = max(1, int(round(candidate_spacing / planner.res)))
        lattice = np.zeros_like(w)
        lattice[::step, ::step] = True
        clear = planner.dist >= int(math.ceil(candidate_clearance / planner.res))
        cand = np.argwhere(self.reachable & lattice & clear)
        self.candidate_xy = np.array([planner.to_world(r, c) for r, c in cand])

    def flood(self, start, passable):
        out = np.zeros_like(passable)
        out[start] = True
        queue = deque([start])
        h, w = passable.shape
        while queue:
            r, c = queue.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (r + dr, c + dc)
                if 0 <= n[0] < h and 0 <= n[1] < w and passable[n] and not out[n]:
                    out[n] = True
                    queue.append(n)
        return out

    def wall_normals(self, cells):
        w = self.p.walls
        normals = np.zeros((len(cells), 2))
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                if dr == 0 and dc == 0:
                    continue
                r = np.clip(cells[:, 0] + dr, 0, w.shape[0] - 1)
                c = np.clip(cells[:, 1] + dc, 0, w.shape[1] - 1)
                hit = w[r, c]
                normals[hit, 0] -= dc
                normals[hit, 1] -= dr
        length = np.linalg.norm(normals, axis=1, keepdims=True)
        return normals / np.maximum(length, 1e-9)

    def visible(self, x, y, heading=None, subset=None):
        idx = np.arange(len(self.target_xy)) if subset is None else subset
        if len(idx) == 0:
            return idx
        d = self.target_xy[idx] - np.array([x, y])
        dist = np.hypot(d[:, 0], d[:, 1])
        ok = (dist >= self.min_range) & (dist <= self.max_range)

        cos_view = -(d[:, 0] * self.normals[idx, 0] + d[:, 1] * self.normals[idx, 1]) / np.maximum(dist, 1e-9)
        ok &= cos_view >= math.cos(self.max_view)

        if heading is not None:
            bearing = np.arctan2(d[:, 1], d[:, 0]) - heading
            ok &= np.abs(np.arctan2(np.sin(bearing), np.cos(bearing))) <= self.half_fov

        idx, d, dist = idx[ok], d[ok], dist[ok]
        if len(idx) == 0:
            return idx

        n = int(math.ceil(self.max_range / (self.p.res * 0.5)))
        t = np.linspace(0.0, 1.0, n)[None, :] * np.minimum(1.0, (dist[:, None] - self.p.res) / np.maximum(dist[:, None], 1e-9))
        px = x + d[:, 0:1] * t
        py = y + d[:, 1:2] * t
        r = ((py - self.p.oy) / self.p.res).astype(int).clip(0, self.p.h - 1)
        c = ((px - self.p.ox) / self.p.res).astype(int).clip(0, self.p.w - 1)
        clear = ~self.p.walls[r, c].any(axis=1)
        return idx[clear]

    def plan_viewpoints(self):
        sees = [self.visible(x, y) for x, y in self.candidate_xy]
        coverable = np.zeros(len(self.target_xy), dtype=bool)
        for s in sees:
            coverable[s] = True

        covered = self.seen.copy()
        chosen = []
        need = self.goal_coverage * coverable.sum()
        while (covered & coverable).sum() < need:
            gains = [np.count_nonzero(~covered[s]) for s in sees]
            best = int(np.argmax(gains))
            if gains[best] == 0:
                break
            chosen.append(best)
            covered[sees[best]] = True

        self.coverable = coverable
        self.viewpoints = [tuple(self.candidate_xy[i]) for i in chosen]
        self.viewpoint_targets = [sees[i] for i in chosen]
        return self.viewpoints

    def mark_seen(self, x, y, heading):
        near = np.flatnonzero(~self.seen & (np.abs(self.target_xy[:, 0] - x) <= self.max_range)
                              & (np.abs(self.target_xy[:, 1] - y) <= self.max_range))
        self.seen[self.visible(x, y, heading, near)] = True

    def mark_not_white(self, points, radius=0.12):
        for x, y in points:
            near = np.hypot(self.target_xy[:, 0] - x, self.target_xy[:, 1] - y) <= radius
            self.seen[near] = True
            self.not_white[near] = True

    def scan_targets(self, i, everything):
        t = self.viewpoint_targets[i]
        return t[~self.not_white[t]] if everything else t[~self.seen[t]]

    def remaining(self, i):
        t = self.viewpoint_targets[i]
        return t[~self.seen[t]]

    def headings_for(self, x, y, targets, width=None):
        if len(targets) == 0:
            return []
        width = width or 2 * self.half_fov * 0.7
        d = self.target_xy[targets] - np.array([x, y])
        angles = np.sort(np.arctan2(d[:, 1], d[:, 0]))
        gaps = np.diff(np.concatenate([angles, angles[:1] + 2 * math.pi]))
        start = (int(np.argmax(gaps)) + 1) % len(angles)
        ordered = np.concatenate([angles[start:], angles[:start] + 2 * math.pi])
        headings = []
        i = 0
        while i < len(ordered):
            first = ordered[i]
            while i < len(ordered) and ordered[i] - first <= width:
                i += 1
            mid = (first + ordered[i - 1]) / 2
            headings.append(math.atan2(math.sin(mid), math.cos(mid)))
        return headings

    def coverage_fraction(self):
        return (self.seen & self.coverable).sum() / max(1, self.coverable.sum())
