"""Test utforskinga på verd 1: kva viewpoint vel tb3_0 først?"""
import sys, os, time
import numpy as np
from PIL import Image
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'multi_robot_challenge_23'))
import rescue_core as rc

pkg = os.path.join(os.path.dirname(__file__), '..')
im = np.array(Image.open(os.path.join(pkg, 'maps', 'map_dat160_w1.png')).convert('L'))
data = np.where(im[::-1] < 128, 100, 0)
mapper = rc.OccupancyMapper(data, 0.01, -10.0, -10.0)
occ = mapper.occ

# innsida av labyrinten og veggflatene
seeds = [tuple(int(v) for v in mapper.spec.to_cell(0.0, -1.0))]
interior = rc.flood_fill(~occ, seeds)
cov = rc.Coverage(mapper.spec)
cov.set_map(occ, interior)
print(f'Veggflate-celler å sjå: {cov.remaining()}')

pg = rc.PlanningGrid(occ, mapper.spec)
t0 = time.time()
dist, parent = rc.dijkstra(pg, pg.nearest_free(0.0, -1.0))
t1 = time.time()
ex = rc.Explorer(cov)
choice = ex.choose(pg, dist, parent)
t2 = time.time()
print(f'Dijkstra: {t1 - t0:.2f} s, utforsking: {t2 - t1:.2f} s')
print(f"Første mål: ({choice['x']:.1f}, {choice['y']:.1f}), retning {np.degrees(choice['yaw']):.0f} grader, "
      f"gevinst {choice['gain']} strålar, reisekostnad {choice['cost']:.1f}")




# ============================================================================ styring
class PathFollower:
    """Følgjer ei liste med waypoints. step() returnerer (v, w, status).
    status: 'idle' (inga rute), 'active', 'reached', 'blocked' (hinder framfor), 'failed' (står fast)."""

    def __init__(self, v_max=0.30, w_max=1.2, reach_tol=0.18, yaw_tol=0.12,
                 safety_dist=0.28, slow_dist=0.6, blocked_timeout=4.0, stuck_timeout=12.0):
        self.v_max, self.w_max = v_max, w_max                     # m/s, rad/s
        self.reach_tol, self.yaw_tol = reach_tol, yaw_tol         # m, rad – "framme" og "rett retning"
        self.safety_dist, self.slow_dist = safety_dist, slow_dist  # m – stopp / sakk farten
        self.blocked_timeout, self.stuck_timeout = blocked_timeout, stuck_timeout   # s
        self.set_path([], None, 0.0)

    def set_path(self, waypoints, final_yaw, now):
        """Ny rute. final_yaw = retninga roboten skal snu seg til ved målet (None = same kva)."""
        self.wps = list(waypoints)
        self.final_yaw = final_yaw
        self.idx = 1 if len(self.wps) > 1 else 0                 # wps[0] er der roboten står
        self.status = 'active' if self.wps else 'idle'
        self.blocked_since = None
        self.best_d = np.inf
        self.progress_t = now

    def step(self, x, y, yaw, front_min, now):
        """Éin kontrollsyklus. front_min = kortaste lidar-avstand framfor roboten."""
        if self.status != 'active':
            return 0.0, 0.0, self.status

        # gå vidare til neste waypoint når vi er innanfor 0.3 m
        while self.idx < len(self.wps) - 1 and math.hypot(self.wps[self.idx][0] - x, self.wps[self.idx][1] - y) < 0.3:
            self.idx += 1
            self.best_d = np.inf
        tx, ty = self.wps[self.idx]
        d = math.hypot(tx - x, ty - y)
        last = self.idx == len(self.wps) - 1

        # står vi fast? (ingen framgang på stuck_timeout sekund)
        if d < self.best_d - 0.05:
            self.best_d = d
            self.progress_t = now
        if now - self.progress_t > self.stuck_timeout:
            self.status = 'failed'
            return 0.0, 0.0, self.status

        # framme: snu mot sluttretninga (der kameraet ser mest usett vegg)
        if last and d < self.reach_tol:
            if self.final_yaw is None:
                self.status = 'reached'
                return 0.0, 0.0, self.status
            e = wrap(self.final_yaw - yaw)
            if abs(e) < self.yaw_tol:
                self.status = 'reached'
                return 0.0, 0.0, self.status
            self.progress_t = now
            return 0.0, float(np.clip(2.0 * e, -self.w_max, self.w_max)), self.status

        # køyr mot waypointet
        e = wrap(math.atan2(ty - y, tx - x) - yaw)                 # vinkelfeil
        w = float(np.clip(2.2 * e, -self.w_max, self.w_max))
        if abs(e) > 0.7:
            v = 0.0                                                # meir enn 40 grader feil: snu på staden
        else:
            v = self.v_max * math.cos(e) ** 2                      # sakk farten når vi peikar litt feil
            if last:
                v = min(v, 0.6 * d + 0.05)                         # brems inn mot målet

        # tryggleik med lidar
        if front_min < self.safety_dist and v > 0.0:
            v = 0.0
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > self.blocked_timeout:
                self.status = 'blocked'                            # be om ny rute
                return 0.0, 0.0, self.status
        else:
            self.blocked_since = None
            if front_min < self.slow_dist:
                v *= max(0.3, (front_min - self.safety_dist) / (self.slow_dist - self.safety_dist))
        return float(v), w, self.status