"""rescue_core – algoritmane til search & rescue-systemet, utan ROS-avhengigheiter.

Alt her er rein Python + numpy slik at det kan testast offline (sjå tools/offline_sim.py)
og brukast likt frå ROS-nodane.

Innhald
  GridSpec          koordinatar <-> celler
  OccupancyMapper   blueprint-kart + lidar (log-odds) -> occupancy grid (0.05 m)
  Coverage          kva veggflater kameraet har sett ("sett"-laget)
  PlanningGrid      grovt planleggingsgitter (0.1 m) med clearance, kostnad og blokkering
  dijkstra / astar  eigne søkealgoritmar på planleggingsgitteret
  simplify_path     line-of-sight-forenkling av ruta til få waypoints
  Explorer          vel utforskingsmål (viewpoints) med best gevinst/kostnad
  PathFollower      styring av roboten langs ruta (v, w) med lidar-tryggleik
  Coordinator       fordeler mål mellom robotane, stor brann, avslutning
"""
import heapq
import math
from collections import deque

import numpy as np

SQRT2 = math.sqrt(2.0)
NEIGH8 = [(-1, -1, SQRT2), (-1, 0, 1.0), (-1, 1, SQRT2), (0, -1, 1.0),
          (0, 1, 1.0), (1, -1, SQRT2), (1, 0, 1.0), (1, 1, SQRT2)]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ============================================================================ grid
class GridSpec:
    """Gitter med origo (ox, oy) i nedre venstre hjørne, rad = y, kolonne = x."""

    def __init__(self, res, ox, oy, h, w):
        self.res, self.ox, self.oy, self.h, self.w = float(res), float(ox), float(oy), int(h), int(w)

    def to_cell(self, x, y):
        ix = np.floor((np.asarray(x) - self.ox) / self.res).astype(int)
        iy = np.floor((np.asarray(y) - self.oy) / self.res).astype(int)
        return iy, ix

    def to_world(self, iy, ix):
        return (self.ox + (np.asarray(ix) + 0.5) * self.res,
                self.oy + (np.asarray(iy) + 0.5) * self.res)

    def inside(self, iy, ix):
        return (iy >= 0) & (iy < self.h) & (ix >= 0) & (ix < self.w)


def block_any(mask, f):
    """Nedskaler bool-maske med faktor f (True dersom nokon celle i blokka er True)."""
    h, w = mask.shape
    H, W = h // f, w // f
    return mask[:H * f, :W * f].reshape(H, f, W, f).any(axis=(1, 3))


def shift(a, dy, dx, fill):
    """Forskyv 2D-array (dy, dx) og fyll inn kantane."""
    out = np.full_like(a, fill)
    h, w = a.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = a[ys, xs]
    return out


def clearance_map(occ, res, max_dist):
    """Avstand (m) til næraste okkuperte celle, avgrensa til max_dist. Brute force over disk-offsets."""
    r = int(math.ceil(max_dist / res))
    dist = np.full(occ.shape, max_dist, dtype=np.float32)
    offs = sorted(((dy, dx) for dy in range(-r, r + 1) for dx in range(-r, r + 1)
                   if dy * dy + dx * dx <= r * r), key=lambda o: o[0] ** 2 + o[1] ** 2)
    for dy, dx in offs:
        d = math.hypot(dy, dx) * res
        if d >= max_dist:
            break
        hit = shift(occ, dy, dx, False)
        np.minimum(dist, np.where(hit, d, max_dist).astype(np.float32), out=dist)
    return dist


def flood_fill(free, seeds):
    """BFS (4-naboar) i free frå seeds -> bool-maske over samanhengande område."""
    h, w = free.shape
    out = np.zeros_like(free, dtype=bool)
    q = deque()
    for (iy, ix) in seeds:
        if 0 <= iy < h and 0 <= ix < w and free[iy, ix] and not out[iy, ix]:
            out[iy, ix] = True
            q.append((iy, ix))
    while q:
        y, x = q.popleft()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True
                q.append((ny, nx))
    return out


# ============================================================================ mapping
class OccupancyMapper:
    """Occupancy grid på 0.05 m: blueprint (frå /map) kombinert med lidar log-odds.

    Blueprint-veggane er alltid okkuperte. Lidar kan leggje til hindringar som ikkje
    står i blueprintet (t.d. Baby/fake_fire-modellar), men andre robotar blir filtrerte bort.
    """
    L_HIT, L_MISS, L_MIN, L_MAX, L_OCC = 0.9, -0.4, -2.0, 3.5, 1.5

    def __init__(self, bp_data, bp_res, bp_ox, bp_oy, res=0.05):
        bp = np.asarray(bp_data, dtype=np.int16)
        f = max(1, int(round(res / bp_res)))
        occ_bp = (bp >= 65) | (bp < 0)
        self.blueprint = block_any(occ_bp, f)
        h, w = self.blueprint.shape
        self.spec = GridSpec(bp_res * f, bp_ox, bp_oy, h, w)
        self.logodds = np.zeros((h, w), dtype=np.float32)
        self.version = 0
        # Lidar får berre leggje til hindringar som ligg minst 2 celler frå blueprint-veggane.
        # Elles gjer diskretiseringa veggane tjukkare og nye "usette" veggflater dukkar stadig opp.
        near = self.blueprint.copy()
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                near |= shift(self.blueprint, dy, dx, False)
        self.lidar_allowed = ~near

    @property
    def occ(self):
        return self.blueprint | ((self.logodds > self.L_OCC) & self.lidar_allowed)

    def update_scan(self, x, y, yaw, ranges, angle_min, angle_inc, range_max,
                    others=(), beam_step=2, range_min=0.12):
        """Integrer éin lidar-scan. others = posisjonar til andre robotar (blir ikkje kartlagde)."""
        r = np.asarray(ranges, dtype=np.float32)[::beam_step]
        ang = yaw + angle_min + np.arange(0, len(ranges), beam_step)[:len(r)] * angle_inc
        valid = np.isfinite(r) & (r > range_min)
        hit = valid & (r < range_max * 0.98)
        r = np.where(valid, np.minimum(r, range_max), 0.0)
        r = np.minimum(r, 3.5)  # stol berre på nære målingar
        res = self.spec.res
        steps = np.arange(0.0, 3.5, res * 0.7, dtype=np.float32)
        d = steps[None, :]
        free_m = d < (r[:, None] - res)
        px = x + np.cos(ang)[:, None] * d
        py = y + np.sin(ang)[:, None] * d
        iy, ix = self.spec.to_cell(px[free_m], py[free_m])
        ok = self.spec.inside(iy, ix)
        free_idx = np.unique(iy[ok] * self.spec.w + ix[ok])
        hx = x + np.cos(ang[hit]) * r[hit]
        hy = y + np.sin(ang[hit]) * r[hit]
        keep = np.ones(hx.shape, dtype=bool)
        for (ox, oy) in others:
            keep &= np.hypot(hx - ox, hy - oy) > 0.35
        iy, ix = self.spec.to_cell(hx[keep], hy[keep])
        ok = self.spec.inside(iy, ix)
        hit_idx = np.unique(iy[ok] * self.spec.w + ix[ok])
        free_idx = np.setdiff1d(free_idx, hit_idx, assume_unique=True)
        lo = self.logodds.reshape(-1)
        cells = np.concatenate([free_idx, hit_idx])
        cells = cells[self.lidar_allowed.reshape(-1)[cells]]   # berre celler lidar kan endre
        b0 = lo[cells] > self.L_OCC
        lo[free_idx] = np.maximum(lo[free_idx] + self.L_MISS, self.L_MIN)
        lo[hit_idx] = np.minimum(lo[hit_idx] + self.L_HIT, self.L_MAX)
        if np.any((lo[cells] > self.L_OCC) != b0):
            self.version += 1   # det kombinerte kartet har endra seg


def wall_faces(occ, interior):
    """Veggflate-celler (okkuperte med fri interiør-nabo) og normalvektorar (peikar ut i rommet).

    Der normalen ikkje er veldefinert (tynne veggar, veggendar) blir han (0, 0), og
    innfallsvinkel-kravet blir då ikkje brukt for den cella.
    """
    ny = np.zeros(occ.shape, dtype=np.float32)
    nx = np.zeros(occ.shape, dtype=np.float32)
    cnt = np.zeros(occ.shape, dtype=np.int16)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            nb = shift(interior, -dy, -dx, False)  # nb[y,x] = interior[y+dy, x+dx]
            n = math.hypot(dy, dx)
            ny += nb * (dy / n)
            nx += nb * (dx / n)
            cnt += nb
    face = occ & (cnt > 0)
    norm = np.hypot(nx, ny)
    weak = norm < 0.5
    norm[weak] = 1.0
    nx, ny = nx / norm, ny / norm
    nx[weak] = 0.0
    ny[weak] = 0.0
    return face, nx, ny


def raycast(occ, spec, x, y, angles, max_range, step_frac=0.5):
    """Kast strålar. x,y: (N,), angles: (N,R). Returnerer flat indeks til første treff (-1 = ingen) og avstand."""
    x = np.asarray(x, dtype=np.float32)[:, None, None]
    y = np.asarray(y, dtype=np.float32)[:, None, None]
    a = np.asarray(angles, dtype=np.float32)[:, :, None]
    step = spec.res * step_frac
    d = np.arange(step, max_range + step, step, dtype=np.float32)[None, None, :]
    px = x + np.cos(a) * d
    py = y + np.sin(a) * d
    ix = np.floor((px - spec.ox) / spec.res).astype(np.int32)
    iy = np.floor((py - spec.oy) / spec.res).astype(np.int32)
    inside = (ix >= 0) & (ix < spec.w) & (iy >= 0) & (iy < spec.h)
    ixc = np.clip(ix, 0, spec.w - 1)
    iyc = np.clip(iy, 0, spec.h - 1)
    blocked = occ[iyc, ixc] | ~inside
    any_hit = blocked.any(axis=2)
    first = blocked.argmax(axis=2)
    flat = np.take_along_axis(iyc * spec.w + ixc, first[:, :, None], axis=2)[:, :, 0]
    flat = np.where(any_hit & np.take_along_axis(inside, first[:, :, None], axis=2)[:, :, 0], flat, -1)
    dist = np.where(any_hit, d[0, 0, first], np.inf)
    return flat, dist


# ============================================================================ coverage
def wall_segments(faces, nx, ny, spec, dot_thr=0.97, off_tol=0.075):
    """Del veggflatene opp i rette veggar: samanhengande flateceller med same normal (dot > dot_thr)
    som ligg på same linje (avstand langs normalen innanfor off_tol). Returnerer etikett per celle
    (-1 = ikkje flate / ingen normal) og talet på veggar."""
    h, w = faces.shape
    lab = -np.ones((h, w), dtype=np.int32)
    has_n = faces & ((nx != 0) | (ny != 0))
    k = 0
    for y0, x0 in zip(*np.nonzero(has_n)):
        if lab[y0, x0] >= 0:
            continue
        n0x, n0y = float(nx[y0, x0]), float(ny[y0, x0])
        off0 = (x0 * n0x + y0 * n0y) * spec.res
        lab[y0, x0] = k
        q = deque([(y0, x0)])
        while q:
            y, x = q.popleft()
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < h and 0 <= xx < w and lab[yy, xx] < 0 and has_n[yy, xx] \
                            and n0x * nx[yy, xx] + n0y * ny[yy, xx] > dot_thr \
                            and abs((xx * n0x + yy * n0y) * spec.res - off0) < off_tol:
                        lab[yy, xx] = k
                        q.append((yy, xx))
        k += 1
    return lab, k


def image_to_rgb(data, height, width, step, encoding):
    """sensor_msgs/Image-data -> (H, W, 3) uint8 RGB. None for koding vi ikkje støttar."""
    enc = str(encoding).lower()
    chans = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    if enc not in chans:
        return None
    ch = chans[enc]
    try:
        buf = np.frombuffer(data, dtype=np.uint8)
    except (TypeError, ValueError):
        buf = np.asarray(data, dtype=np.uint8)
    h, w = int(height), int(width)
    step = int(step) if step else w * ch
    if h <= 0 or w <= 0 or step < w * ch or buf.size < h * step:
        return None
    img = buf[:h * step].reshape(h, step)[:, :w * ch].reshape(h, w, ch)
    if enc.startswith('bgr'):
        img = img[:, :, 2::-1]
    else:
        img = img[:, :, :3]
    return img


class WallColorClassifier:
    """Klassifiserer veggfarge i kamerabiletet, kolonne for kolonne.

    For kvar av n_cols biletkolonner: vinkel i kartet frå pinhole-modellen
    (angle = yaw - atan((u - cx) / f)), avstand til veggen frå raycast i occupancy-kartet,
    og ein liten patch på veggen i høgda sample_h (m over golvet). Patchen ligg lågt
    (markørane er 0–0.5 m, brann-/babytavlene heng over markørane frå ca. 0.75 m og er
    sterkt farga) og over kamerahøgda, så han treffer aldri golvet.

    Resultat per kolonne: +1 = kvit/grå (låg metning), -1 = farga, 0 = usikker/ingen vegg.
    """

    def __init__(self, n_cols=32, hfov=1.085595, cam_height=0.116, sample_h=(0.18, 0.42),
                 sat_plain=0.12, sat_color=0.22, min_value=35.0, col_half=4, edge_margin=8,
                 depth_tol=0.12, angle_margin=0.06):
        self.n_cols = int(n_cols)
        self.hfov = float(hfov)
        self.cam_height = float(cam_height)
        self.sample_h = (float(sample_h[0]), float(sample_h[1]))
        self.sat_plain, self.sat_color = float(sat_plain), float(sat_color)
        self.min_value = float(min_value)
        self.col_half, self.edge_margin = int(col_half), int(edge_margin)
        self.depth_tol = float(depth_tol)
        self.angle_margin = float(angle_margin)

    def intrinsics(self, width, height):
        f = (width / 2.0) / math.tan(self.hfov / 2.0)
        return f, (width - 1) / 2.0, (height - 1) / 2.0

    def columns(self, width):
        lo = self.edge_margin + self.col_half
        hi = width - 1 - self.edge_margin - self.col_half
        return np.rint(np.linspace(lo, hi, self.n_cols)).astype(int)

    def column_angles(self, yaw, width, height):
        """Biletkolonner og vinklane deira i kartet (u aukar mot høgre = negativ vinkel)."""
        f, cx, _ = self.intrinsics(width, height)
        u = self.columns(width)
        return u, yaw - np.arctan((u - cx) / f)

    def sample_rows(self, dist, width, height):
        """Øvste og nedste biletrad (inkl.) for patchen, gitt veggavstand. -1 der det ikkje går."""
        f, _, cy = self.intrinsics(width, height)
        d = np.maximum(np.asarray(dist, dtype=np.float64), 1e-3)
        h0, h1 = self.sample_h
        v_top = cy - f * (h1 - self.cam_height) / d
        v_bot = cy - f * (h0 - self.cam_height) / d
        top = np.clip(np.floor(v_top), 1, height - 2).astype(int)
        bot = np.clip(np.ceil(v_bot), 1, height - 2).astype(int)
        bad = ~np.isfinite(np.asarray(dist, dtype=np.float64)) | (bot - top < 1)
        return np.where(bad, -1, top), np.where(bad, -1, bot)

    def classify_patches(self, img, u, top, bot):
        """img (H, W, 3) uint8. Returnerer label (+1/-1/0), gjennomsnittleg metning og lysstyrke."""
        n = len(u)
        lab = np.zeros(n, dtype=np.int8)
        sat = np.full(n, np.nan)
        val = np.full(n, np.nan)
        H, W = img.shape[:2]
        for k in range(n):
            if top[k] < 0:
                continue
            u0, u1 = max(0, u[k] - self.col_half), min(W, u[k] + self.col_half + 1)
            p = img[top[k]:bot[k] + 1, u0:u1].reshape(-1, 3).astype(np.float32)
            if p.shape[0] < 4:
                continue
            mean = p.mean(axis=0)
            v = float(mean.max())
            s = float(mean.max() - mean.min()) / max(v, 1.0)
            px_max = p.max(axis=1)
            px_sat = (px_max - p.min(axis=1)) / np.maximum(px_max, 1.0)
            frac = float(((px_sat > self.sat_color) & (px_max > self.min_value)).mean())
            sat[k], val[k] = s, v
            if v < self.min_value:
                continue             # for mørkt (skugge, svart markørfelt, robot) – usikker
            if s < self.sat_plain and frac < 0.15:
                lab[k] = 1
            elif s > self.sat_color and frac > 0.5:
                lab[k] = -1
        return lab, sat, val

    def depth_ok(self, occ, spec, x, y, ang, width, max_range=4.0):
        """Djupnesjekk per kolonne: strålar ved ±(patchbreidd + angle_margin) må treffe om lag same
        avstand som midtstrålen. Fjernar kolonner nær veggkantar/hjørne der ein liten posefeil
        (yaw, tidsforseinking) kunne gjeve fargen til feil vegg. Returnerer (ok, avstand)."""
        f = (width / 2.0) / math.tan(self.hfov / 2.0)
        da = math.atan(self.col_half / f) + self.angle_margin
        ang = np.asarray(ang, dtype=np.float64)
        rays = np.stack([ang - da, ang - 0.5 * da, ang, ang + 0.5 * da, ang + da], axis=1).reshape(1, -1)
        _, dist = raycast(occ, spec, [x], [y], rays, max_range)
        dist = dist.reshape(-1, 5)
        dc = dist[:, 2]
        ok = np.isfinite(dist).all(axis=1)
        dd = np.where(np.isfinite(dist), dist, 1e3)
        ok &= (dd.max(axis=1) - dd.min(axis=1)) <= self.depth_tol + 0.15 * np.minimum(dc, 1e3)
        return ok, dc

    def classify(self, img, x, y, yaw, occ, spec, max_range=4.0):
        """Heile kjeda for eitt bilete. x, y = kameraposisjon i kartet.

        Returnerer (vinklar, label) for kolonnene, label som i classify_patches. Kolonner der
        djupna hoppar (veggkant/hjørne innanfor patchen eller posefeil) blir usikre.
        """
        H, W = img.shape[:2]
        u, ang = self.column_angles(yaw, W, H)
        ok, dc = self.depth_ok(occ, spec, x, y, ang, W, max_range)
        top, bot = self.sample_rows(np.where(ok, dc, np.inf), W, H)
        lab, _, _ = self.classify_patches(img, u, top, bot)
        return ang, lab


class Coverage:
    """"Sett"-laget: kva veggflater kameraet har sett frå godt nok nært hald og vinkel."""

    def __init__(self, spec, seen_range=2.5, min_range=0.8, fov=0.90, max_incidence_deg=60.0, n_rays=48):
        self.spec = spec
        self.seen_range, self.min_range, self.fov = seen_range, min_range, fov
        self.cos_inc = math.cos(math.radians(max_incidence_deg))
        self.n_rays = n_rays
        self.seen = np.zeros((spec.h, spec.w), dtype=bool)
        self.targets = np.zeros((spec.h, spec.w), dtype=bool)
        self.occ = None
        self.nx = self.ny = None
        # fargefilter: alle markørane heng på kvite/grå veggar. Ein heil rett vegg som kameraet
        # har sett er farga (murstein/tre), blir registrert som skanna.
        self.seg = None
        self.n_seg = 0
        self.col_cnt = np.zeros((spec.h, spec.w), dtype=np.int16)   # bilete som såg cella som farga
        self.pl_cnt = np.zeros((spec.h, spec.w), dtype=np.int16)    # bilete som såg cella som kvit/grå
        self.excluded = np.zeros((spec.h, spec.w), dtype=bool)
        self.min_color_votes = 6      # sum av farga stemmer på veggen før han blir droppa
        self.min_color_cells = 3      # ... frå minst så mange ulike celler

    def set_map(self, occ, interior):
        self.occ = occ
        face, self.nx, self.ny = wall_faces(occ, interior)
        self.targets = face
        self.seg, self.n_seg = wall_segments(face, self.nx, self.ny, self.spec)
        self.excluded &= face
        self._update_excluded()

    def mark_colors(self, x, y, angles, plain, max_range=4.0):
        """Fargeresultat frå eitt kamerabilete: angles (R,) strålevinklar i kartet,
        plain (R,) True = kvit/grå vegg, False = farga. Kvar celle får maks éi stemme per bilete."""
        if self.occ is None or len(angles) == 0:
            return 0
        ang = np.asarray(angles, dtype=np.float32).reshape(1, -1)
        plain = np.asarray(plain, dtype=bool).reshape(-1)
        flat, dist = raycast(self.occ, self.spec, [x], [y], ang, max_range)
        ok = (flat[0] >= 0) & self.targets.reshape(-1)[np.maximum(flat[0], 0)]
        for want, arr in ((False, self.col_cnt), (True, self.pl_cnt)):
            idx = np.unique(flat[0][ok & (plain == want)])
            arr.reshape(-1)[idx] += 1
        return self._update_excluded()

    def _update_excluded(self):
        """Ein rett vegg blir droppa (og registrert som skanna) når han har nok farga stemmer
        frå fleire celler og ingen kvite/grå stemmer i det heile."""
        if self.seg is None or self.n_seg == 0:
            return 0
        m = self.seg >= 0
        segs = self.seg[m]
        col = np.bincount(segs, weights=self.col_cnt[m], minlength=self.n_seg)
        pl = np.bincount(segs, weights=self.pl_cnt[m], minlength=self.n_seg)
        ncells = np.bincount(segs, weights=(self.col_cnt[m] > 0), minlength=self.n_seg)
        bad = (col >= self.min_color_votes) & (ncells >= self.min_color_cells) & (pl == 0)
        new = m.copy()
        new[m] = bad[segs]
        new &= ~self.excluded
        n_new = int(new.sum())
        if n_new:
            self.excluded |= new
            self.seen |= new
        return n_new

    def valid_hits(self, flat, dist, angles):
        """Kva for treff tel som 'sett' (rekkjevidd + innfallsvinkel)."""
        ok = (flat >= 0) & (dist <= self.seen_range) & (dist >= self.min_range)
        f = np.where(ok, flat, 0)
        nxv = self.nx.reshape(-1)[f]
        nyv = self.ny.reshape(-1)[f]
        cos_inc = -(np.cos(angles) * nxv + np.sin(angles) * nyv)
        no_normal = (nxv == 0.0) & (nyv == 0.0)
        ok &= (cos_inc >= self.cos_inc) | no_normal
        ok &= self.targets.reshape(-1)[f]
        return ok

    def mark_camera(self, x, y, yaw):
        if self.occ is None:
            return 0
        ang = yaw + np.linspace(-self.fov / 2, self.fov / 2, self.n_rays, dtype=np.float32)[None, :]
        flat, dist = raycast(self.occ, self.spec, [x], [y], ang, self.seen_range)
        ok = self.valid_hits(flat, dist, ang)
        idx = flat[ok]
        if idx.size == 0:
            return 0
        # merk treffcella + naboflater (strålane har litt mellomrom)
        iy, ix = np.divmod(idx, self.spec.w)
        new = 0
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy = np.clip(iy + dy, 0, self.spec.h - 1)
                xx = np.clip(ix + dx, 0, self.spec.w - 1)
                m = self.targets[yy, xx] & ~self.seen[yy, xx]
                new += int(m.sum())
                self.seen[yy[m], xx[m]] = True
        return new

    def remaining(self):
        return int((self.targets & ~self.seen).sum())

    def reset(self, seen_range=None):
        self.seen[:] = self.excluded          # farga veggar er framleis "skanna"
        if seen_range is not None:
            self.seen_range = seen_range


# ============================================================================ planning
class PlanningGrid:
    """Grovt gitter (standard 0.1 m) for Dijkstra/A*, med clearance-basert kostnad."""

    def __init__(self, occ_fine, fine_spec, factor=2, robot_radius=0.24, soft_radius=0.55, wall_cost=6.0):
        self.occ = block_any(occ_fine, factor)
        h, w = self.occ.shape
        self.spec = GridSpec(fine_spec.res * factor, fine_spec.ox, fine_spec.oy, h, w)
        self.robot_radius = robot_radius
        self.clear = clearance_map(self.occ, self.spec.res, soft_radius + 0.05)
        # occ er utvida med opptil éi fin celle -> legg den tilbake i clearance
        self.clear = self.clear + fine_spec.res * 0.5
        self.base_blocked = self.clear < robot_radius
        pen = np.clip((soft_radius - self.clear) / (soft_radius - robot_radius), 0.0, 1.0)
        self.cost = (1.0 + wall_cost * pen ** 2).astype(np.float32)
        self.blocked = self.base_blocked.copy()

    def with_obstacles(self, points, radius):
        """Blokker disk rundt punkt (t.d. den andre roboten)."""
        self.blocked = self.base_blocked.copy()
        for (x, y) in points:
            iy, ix = self.spec.to_cell(x, y)
            r = int(math.ceil(radius / self.spec.res))
            y0, y1 = max(0, iy - r), min(self.spec.h, iy + r + 1)
            x0, x1 = max(0, ix - r), min(self.spec.w, ix + r + 1)
            yy, xx = np.mgrid[y0:y1, x0:x1]
            m = (yy - iy) ** 2 + (xx - ix) ** 2 <= r * r
            self.blocked[yy[m], xx[m]] = True

    def nearest_free(self, x, y, max_r=0.8):
        iy, ix = self.spec.to_cell(x, y)
        iy, ix = int(iy), int(ix)
        if self.spec.inside(iy, ix) and not self.blocked[iy, ix]:
            return iy, ix
        r = int(math.ceil(max_r / self.spec.res))
        best = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                yy, xx = iy + dy, ix + dx
                if 0 <= yy < self.spec.h and 0 <= xx < self.spec.w and not self.blocked[yy, xx]:
                    d = dy * dy + dx * dx
                    if best is None or d < best[0]:
                        best = (d, yy, xx)
        return (best[1], best[2]) if best else None


def dijkstra(pg, start, max_cost=np.inf):
    """Dijkstra frå start (iy, ix) over pg. Returnerer kostnad (m-ekvivalent) og forelder-indeks (flat)."""
    h, w = pg.blocked.shape
    dist = np.full(h * w, np.inf, dtype=np.float64)
    parent = np.full(h * w, -1, dtype=np.int64)
    s = start[0] * w + start[1]
    dist[s] = 0.0
    blocked = pg.blocked.reshape(-1)
    cost = pg.cost.reshape(-1)
    res = pg.spec.res
    pq = [(0.0, s)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u] or d > max_cost:
            continue
        uy, ux = divmod(u, w)
        for dy, dx, step in NEIGH8:
            vy, vx = uy + dy, ux + dx
            if vy < 0 or vy >= h or vx < 0 or vx >= w:
                continue
            v = vy * w + vx
            if blocked[v]:
                continue
            if dy and dx and (blocked[uy * w + vx] or blocked[vy * w + ux]):
                continue  # ikkje kutt hjørne
            nd = d + step * res * 0.5 * (cost[u] + cost[v])
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))
    return dist.reshape(h, w), parent


def astar(pg, start, goal):
    """A* frå start til goal (celler). Returnerer liste av celler eller None."""
    h, w = pg.blocked.shape
    blocked = pg.blocked.reshape(-1)
    cost = pg.cost.reshape(-1)
    res = pg.spec.res
    s, g = start[0] * w + start[1], goal[0] * w + goal[1]
    gy, gx = goal

    def heur(u):
        uy, ux = divmod(u, w)
        dy, dx = abs(uy - gy), abs(ux - gx)
        return res * (max(dy, dx) + (SQRT2 - 1) * min(dy, dx))

    gcost = {s: 0.0}
    parent = {s: -1}
    pq = [(heur(s), 0.0, s)]
    closed = set()
    while pq:
        _, d, u = heapq.heappop(pq)
        if u in closed:
            continue
        if u == g:
            path = []
            while u != -1:
                path.append(divmod(u, w))
                u = parent[u]
            return path[::-1]
        closed.add(u)
        uy, ux = divmod(u, w)
        for dy, dx, step in NEIGH8:
            vy, vx = uy + dy, ux + dx
            if vy < 0 or vy >= h or vx < 0 or vx >= w:
                continue
            v = vy * w + vx
            if blocked[v] or v in closed:
                continue
            if dy and dx and (blocked[uy * w + vx] or blocked[vy * w + ux]):
                continue
            nd = d + step * res * 0.5 * (cost[u] + cost[v])
            if nd < gcost.get(v, np.inf):
                gcost[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd + heur(v), nd, v))
    return None


def path_from_parent(parent, w, goal):
    path = []
    u = goal[0] * w + goal[1]
    while u != -1:
        path.append(divmod(int(u), w))
        u = parent[u]
    return path[::-1]


def line_free(pg, a, b, max_cost=1.6):
    """Er linja a->b (celler) fri og langt nok frå veggar?"""
    n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2) + 1
    ys = np.rint(np.linspace(a[0], b[0], n + 1)).astype(int)
    xs = np.rint(np.linspace(a[1], b[1], n + 1)).astype(int)
    return not (pg.blocked[ys, xs].any() or (pg.cost[ys, xs] > max_cost).any())


def simplify_path(pg, cells, max_seg=12):
    """Line-of-sight 'string pulling': behald berre naudsynte knekkpunkt (maks seg-lengd i celler)."""
    if len(cells) <= 2:
        return list(cells)
    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        j = min(len(cells) - 1, i + max_seg)
        while j > i + 1 and not line_free(pg, cells[i], cells[j]):
            j -= 1
        out.append(cells[j])
        i = j
    return out


# ============================================================================ exploration
class Explorer:
    """Vel viewpoints: posisjon + retning der kameraet ser flest usette veggflater, minus reisekostnad."""

    def __init__(self, coverage, n_rays=72, stride=4, dist_weight=1.2, min_gain=3):
        self.cov = coverage
        self.n_rays = n_rays
        self.stride = stride          # i planleggingsceller (0.4 m)
        self.dist_weight = dist_weight
        self.min_gain = min_gain

    def view_gains(self, xs, ys, exclude=None):
        """For kvar kandidat: (beste gevinst, beste yaw, treff-indeksar i beste vindauge)."""
        cov = self.cov
        R = self.n_rays
        base = np.linspace(-math.pi, math.pi, R, endpoint=False, dtype=np.float32)
        win = max(1, int(round(cov.fov / (2 * math.pi / R))))
        gains = np.zeros(len(xs), dtype=np.int32)
        yaws = np.zeros(len(xs), dtype=np.float32)
        hits = [None] * len(xs)
        for c0 in range(0, len(xs), 64):
            sl = slice(c0, c0 + 64)
            n = len(xs[sl])
            ang = np.broadcast_to(base, (n, R))
            flat, dist = raycast(cov.occ, cov.spec, xs[sl], ys[sl], ang, cov.seen_range)
            ok = cov.valid_hits(flat, dist, ang)
            f = np.where(ok, flat, 0)
            ok &= ~cov.seen.reshape(-1)[f]
            if exclude is not None:
                ok &= ~exclude.reshape(-1)[f]
            okf = ok.astype(np.int32)
            ext = np.concatenate([okf, okf[:, :win]], axis=1)
            cs = np.concatenate([np.zeros((n, 1), np.int32), np.cumsum(ext, axis=1)], axis=1)
            wsum = cs[:, win:win + R] - cs[:, :R]       # vindauge som startar på stråle k
            best = wsum.argmax(axis=1)
            gains[sl] = wsum[np.arange(n), best]
            yaws[sl] = base[(best + win // 2) % R] + (0.0 if win % 2 else math.pi / R)
            for k in range(n):
                idx = (best[k] + np.arange(win)) % R
                m = ok[k, idx]
                hits[c0 + k] = flat[k, idx][m]
        return gains, yaws, hits

    def choose(self, pg, dist, parent, exclude=None, avoid=(), avoid_r=2.0, others_dist=(), territory_w=1.0):
        """Vel beste viewpoint gitt Dijkstra-kostnader frå roboten. Returnerer dict eller None.

        others_dist: Dijkstra-kart frå dei andre robotane. Kandidatar som ein annan robot
        når klart raskare, får straff (enkel "territorie"-deling mellom robotane).
        """
        s = self.stride
        dsub = dist[::s, ::s]
        iy, ix = np.nonzero(np.isfinite(dsub))
        if iy.size == 0:
            return None
        iy, ix = iy * s, ix * s
        xs, ys = pg.spec.to_world(iy, ix)
        gains, yaws, hits = self.view_gains(xs, ys, exclude)
        d = dist[iy, ix]
        util = gains.astype(np.float64) - self.dist_weight * d
        for od in others_dist:
            do = od[iy, ix]
            adv = np.where(np.isfinite(do), d - do, 0.0)
            util -= territory_w * np.clip(adv, 0.0, 6.0)
        for (ax, ay) in avoid:
            util -= 4.0 * np.clip(1.0 - np.hypot(xs - ax, ys - ay) / avoid_r, 0.0, 1.0)
        util[gains < self.min_gain] = -np.inf
        k = int(np.argmax(util))
        if not np.isfinite(util[k]):
            return None
        return {'cell': (int(iy[k]), int(ix[k])), 'x': float(xs[k]), 'y': float(ys[k]),
                'yaw': float(yaws[k]), 'gain': int(gains[k]), 'hits': hits[k], 'cost': float(d[k])}


# ============================================================================ control
class PathFollower:
    """Følgjer ei liste med waypoints. Returnerer (v, w, status)."""

    def __init__(self, v_max=0.40, w_max=0.6, reach_tol=0.18, yaw_tol=0.12,
                 safety_dist=0.28, slow_dist=0.6, blocked_timeout=4.0, stuck_timeout=12.0):
        self.v_max, self.w_max = v_max, w_max
        self.reach_tol, self.yaw_tol = reach_tol, yaw_tol
        self.safety_dist, self.slow_dist = safety_dist, slow_dist
        self.blocked_timeout, self.stuck_timeout = blocked_timeout, stuck_timeout
        self.set_path([], None, 0.0)

    def set_path(self, waypoints, final_yaw, now):
        self.wps = list(waypoints)
        self.final_yaw = final_yaw
        self.idx = 1 if len(self.wps) > 1 else 0
        self.status = 'active' if self.wps else 'idle'
        self.blocked_since = None
        self.best_d = np.inf
        self.progress_t = now

    def step(self, x, y, yaw, front_min, now):
        if self.status != 'active':
            return 0.0, 0.0, self.status
        # gå vidare til neste waypoint når vi er nær
        while self.idx < len(self.wps) - 1 and math.hypot(self.wps[self.idx][0] - x, self.wps[self.idx][1] - y) < 0.3:
            self.idx += 1
            self.best_d = np.inf
        tx, ty = self.wps[self.idx]
        d = math.hypot(tx - x, ty - y)
        last = self.idx == len(self.wps) - 1

        # framdriftsovervaking
        if d < self.best_d - 0.05:
            self.best_d = d
            self.progress_t = now
        if now - self.progress_t > self.stuck_timeout:
            self.status = 'failed'
            return 0.0, 0.0, self.status

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

        e = wrap(math.atan2(ty - y, tx - x) - yaw)
        w = float(np.clip(2.2 * e, -self.w_max, self.w_max))
        if abs(e) > 0.7:
            v = 0.0  # snu på staden
        else:
            v = self.v_max * math.cos(e) ** 2
            if last:
                v = min(v, 0.6 * d + 0.05)
        # tryggleik med lidar
        if front_min < self.safety_dist and v > 0.0:
            v = 0.0
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > self.blocked_timeout:
                self.status = 'blocked'
                return 0.0, 0.0, self.status
        else:
            self.blocked_since = None
            if front_min < self.slow_dist:
                v *= max(0.3, (front_min - self.safety_dist) / (self.slow_dist - self.safety_dist))
        return float(v), w, self.status


# ============================================================================ coordination
class Coordinator:
    """Sentral planleggjar. Kall update() jamleg; den returnerer nye ruter for robotar som treng det.

    Tilstandar: EXPLORE -> (ID 4 funne) FIRE -> EXPLORE -> DONE
    """

    def __init__(self, names, fire_radius=1.3, fire_hold=2.0, log=print):
        self.names = list(names)
        self.log = log
        self.mapper_occ = None
        self.interior = None
        self.cov = None
        self.pg = None
        self.explorer = None
        self.goals = {n: None for n in self.names}      # aktivt mål per robot
        self.status = {n: 'idle' for n in self.names}
        self.blacklist = []                              # (x, y, t_until)
        self.found = set()
        self.fire = None
        self.fire_done = False
        self.fire_close_since = None
        self.fire_radius = fire_radius
        self.fire_hold = fire_hold
        self.mode = 'EXPLORE'
        self.rounds = 0
        self.territory_w = 1.0
        self.reset_cb = None        # kallast med ny seen_range når dekningskartet skal nullstillast
        self.retry_at = {n: 0.0 for n in self.names}  # pause etter "ingen mål funne"
        self.max_plans_per_tick = 2
        self.fire_team = []         # robotane som skal til stor brann (dei to nærmaste)
        self._dcache = {}           # mellomlagra Dijkstra-kart (per startcelle) for territorium
        self._dcache_pg = None

    # -- kart
    def set_map(self, occ, spec, seeds_xy, coverage):
        """occ: fint occupancy-gitter. seeds_xy: startposisjonar (definerer interiøret)."""
        self.mapper_occ = occ
        seeds = [tuple(int(v) for v in spec.to_cell(x, y)) for (x, y) in seeds_xy]
        self.interior = flood_fill(~occ, seeds)
        self.cov = coverage
        self.cov.set_map(occ, self.interior)
        self.pg = PlanningGrid(occ, spec)
        if self.explorer is None:
            self.explorer = Explorer(self.cov)
        else:
            self.explorer.cov = self.cov

    # -- hendingar
    def on_marker(self, marker_id, x, y):
        self.found.add(int(marker_id))
        if int(marker_id) == 4 and self.fire is None:
            self.fire = (x, y)
            if not self.fire_done:
                self.mode = 'FIRE'
                self.log(f'Stor brann funnen @ ({x:.2f}, {y:.2f}) – samlar robotane')

    def on_status(self, name, status):
        self.status[name] = status

    # -- hjelparar
    def _plan_from(self, name, poses):
        others = [poses[o][:2] for o in self.names if o != name and o in poses]
        self.pg.with_obstacles(others, 0.45)
        x, y, _ = poses[name]
        start = self.pg.nearest_free(x, y)
        if start is None:
            self.pg.with_obstacles([], 0.0)
            start = self.pg.nearest_free(x, y, 1.0)
        return start

    def _route(self, parent, cell):
        cells = path_from_parent(parent, self.pg.spec.w, cell)
        cells = simplify_path(self.pg, cells)
        xs, ys = self.pg.spec.to_world(np.array([c[0] for c in cells]), np.array([c[1] for c in cells]))
        return [(float(a), float(b)) for a, b in zip(xs, ys)]

    def _claimed(self, name):
        ex = np.zeros(self.cov.seen.shape, dtype=bool)
        for o in self.names:
            g = self.goals.get(o)
            if o != name and g is not None and g.get('hits') is not None and len(g['hits']):
                iy, ix = np.divmod(g['hits'], self.cov.spec.w)
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ex[np.clip(iy + dy, 0, ex.shape[0] - 1), np.clip(ix + dx, 0, ex.shape[1] - 1)] = True
        return ex

    def _goal_still_useful(self, g):
        if g.get('kind') != 'explore':
            return True
        hits = g['hits']
        if hits is None or len(hits) == 0:
            return False
        remaining = int((~self.cov.seen.reshape(-1)[hits]).sum())
        return remaining >= max(2, self.explorer.min_gain - 1)

    def _dist_from(self, x, y):
        """Dijkstra-kart frå eit punkt (utan andre robotar som hinder), mellomlagra per celle."""
        self.pg.with_obstacles([], 0.0)
        c = self.pg.nearest_free(x, y)
        if c is None:
            return None
        if c not in self._dcache:
            self._dcache[c] = dijkstra(self.pg, c)[0]
        return self._dcache[c]

    # -- hovudløkka
    def update(self, poses, now):
        """poses: {name: (x, y, yaw)}. Returnerer {name: (waypoints, final_yaw)} – tom liste = stopp."""
        out = {}
        if self.pg is None or any(n not in poses for n in self.names):
            return out
        if self._dcache_pg is not self.pg:
            self._dcache = {}
            self._dcache_pg = self.pg
        self.blacklist = [b for b in self.blacklist if b[2] > now]

        if self.mode == 'DONE':
            return out

        explorers = list(self.names)
        if self.mode == 'FIRE':
            explorers = self._update_fire(poses, now, out)

        if self.mode == 'EXPLORE' and self.fire_done and len(self.found) >= 5:
            self.mode = 'DONE'
            self.log('Alle markørar funne og brannen er sløkt – ferdig!')
            return {n: ([], None) for n in self.names}

        # EXPLORE (alle robotar, eller dei som ikkje er på brannlaget).
        # Maks 2 nye planar per runde, så koordinatoren aldri blokkerer lenge med mange robotar.
        planned = 0
        for n in sorted(explorers, key=lambda r: self.goals[r] is not None):   # robotar utan mål først
            g = self.goals[n]
            st = self.status[n]
            need = g is None or st in ('reached', 'failed', 'blocked', 'idle') or not self._goal_still_useful(g)
            if not need or now < self.retry_at[n]:
                continue
            if planned >= self.max_plans_per_tick:
                break
            planned += 1
            if g is not None and st in ('failed', 'blocked'):
                self.blacklist.append((g['x'], g['y'], now + 30.0))
            start = self._plan_from(n, poses)
            if start is None:
                self.log(f'{n}: finn ikkje fri startcelle')
                continue
            dist, parent = dijkstra(self.pg, start)
            for (bx, by, _) in self.blacklist:
                iy, ix = self.pg.spec.to_cell(bx, by)
                dist[max(0, iy - 3):iy + 4, max(0, ix - 3):ix + 4] = np.inf
            avoid = []
            others_dist = []
            for o in self.names:
                if o == n:
                    continue
                go = self.goals[o]
                if go is not None and go.get('kind') == 'explore':
                    avoid.append((go['x'], go['y']))
                # territorium: rekn frå målet til den andre roboten (eller posisjonen om han ikkje har mål)
                if go is not None and go.get('kind') == 'fire':
                    continue
                ox, oy = (go['x'], go['y']) if go is not None else poses[o][:2]
                od = self._dist_from(ox, oy)
                if od is not None:
                    others_dist.append(od)
            choice = self.explorer.choose(self.pg, dist, parent, exclude=self._claimed(n), avoid=avoid,
                                          others_dist=others_dist, territory_w=self.territory_w)
            if choice is None:
                self.goals[n] = None
                self.retry_at[n] = now + 2.0
                self._maybe_new_round(n, poses)
                out[n] = ([], None)
                continue
            choice['kind'] = 'explore'
            self.goals[n] = choice
            self.status[n] = 'active'
            out[n] = (self._route(parent, choice['cell']), choice['yaw'])
        return out

    def _update_fire(self, poses, now, out):
        """Stor brann: dei to robotane som er nærast (langs vegen) køyrer dit. Returnerer resten."""
        fx, fy = self.fire
        if not self.fire_team:
            dfire = self._dist_from(fx, fy)
            def cost(n):
                iy, ix = self.pg.spec.to_cell(poses[n][0], poses[n][1])
                v = dfire[iy, ix] if dfire is not None and self.pg.spec.inside(iy, ix) else np.inf
                return v if np.isfinite(v) else 1e6 + math.hypot(poses[n][0] - fx, poses[n][1] - fy)
            self.fire_team = sorted(self.names, key=cost)[:2]
            self.log(f'Brannlag: {self.fire_team}')
            for n in self.fire_team:
                self.goals[n] = None
        team = self.fire_team
        close = [math.hypot(poses[n][0] - fx, poses[n][1] - fy) < 1.8 for n in self.names]
        if sum(close) >= 2:
            self.fire_close_since = self.fire_close_since or now
            if now - self.fire_close_since >= self.fire_hold:
                self.fire_done = True
                self.mode = 'EXPLORE'
                self.log('Stor brann sløkt (+300) – held fram med utforsking')
                for n in team:
                    self.goals[n] = None
                    self.status[n] = 'idle'
                return list(self.names)
        else:
            self.fire_close_since = None
        taken = []
        for n in team:
            st = self.status[n]
            g = self.goals[n]
            if g is not None and g.get('kind') == 'fire' and st in ('active', 'reached'):
                taken.append((g['x'], g['y']))
                continue
            start = self._plan_from(n, poses)
            if start is None:
                continue
            dist, parent = dijkstra(self.pg, start)
            cand = self._fire_cell(dist, taken)
            if cand is None:
                continue
            cx, cy = self.pg.spec.to_world(*cand)
            yaw = math.atan2(fy - cy, fx - cx)
            self.goals[n] = {'kind': 'fire', 'x': float(cx), 'y': float(cy), 'hits': None}
            taken.append((float(cx), float(cy)))
            self.status[n] = 'active'
            out[n] = (self._route(parent, cand), yaw)
        return [n for n in self.names if n not in team]

    def _maybe_new_round(self, name, poses):
        """Ingen meir å sjå, men markørar manglar -> nullstill og sjå nærare."""
        busy = any(self.goals[o] is not None and self.goals[o].get('kind') == 'explore'
                   for o in self.names if o != name)
        if busy or len(self.found) >= 5:
            return
        self.rounds += 1
        new_range = max(1.5, self.cov.seen_range - 0.5)
        self.log(f'Alt er dekt men berre {len(self.found)}/5 funne – ny runde (seen_range {new_range:.1f} m)')
        self.cov.reset(new_range)
        if self.reset_cb is not None:
            self.reset_cb(new_range)

    def _fire_cell(self, dist, taken):
        fx, fy = self.fire
        h, w = dist.shape
        iy, ix = np.nonzero(np.isfinite(dist))
        xs, ys = self.pg.spec.to_world(iy, ix)
        dfire = np.hypot(xs - fx, ys - fy)
        ok = (dfire < self.fire_radius) & (dfire > 0.45)
        for (tx, ty) in taken:
            ok &= np.hypot(xs - tx, ys - ty) > 0.7
        if not ok.any():
            ok = (dfire < 1.7)
            if not ok.any():
                return None
        k = np.argmin(np.where(ok, dist[iy, ix], np.inf))
        return int(iy[k]), int(ix[k])