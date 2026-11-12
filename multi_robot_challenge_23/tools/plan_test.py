"""Test A* på kartet til verd 1: finn veg frå startsona til kvar markør."""
import sys, os, re
import numpy as np
from PIL import Image
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'multi_robot_challenge_23'))
import rescue_core as rc

pkg = os.path.join(os.path.dirname(__file__), '..')
im = np.array(Image.open(os.path.join(pkg, 'maps', 'map_dat160_w1.png')).convert('L'))
data = np.where(im[::-1] < 128, 100, 0)                 # rad 0 = botn, som /map
mapper = rc.OccupancyMapper(data, 0.01, -10.0, -10.0)
pg = rc.PlanningGrid(mapper.occ, mapper.spec)

start = pg.nearest_free(0.0, -1.0)                      # tb3_0 sin startposisjon
goals = {'markør 0': (-1.0, 4.5), 'markør 3': (-0.65, -3.6), 'stor brann': (4.3, -0.4)}
for name, (gx, gy) in goals.items():
    goal = pg.nearest_free(gx, gy)
    path = rc.astar(pg, start, goal)
    if path is None:
        print(f'{name}: INGEN VEG')
        continue
    wps = rc.simplify_path(pg, path)
    xs, ys = pg.spec.to_world(np.array([c[0] for c in wps]), np.array([c[1] for c in wps]))
    length = sum(np.hypot(np.diff(xs), np.diff(ys)))
    print(f'{name}: {len(path)} celler -> {len(wps)} waypoints, {length:.1f} m')