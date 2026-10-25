"""Géométrie des chemins (ligne brisée) et projection respectant la progression.
Jumeau Python de matlab/chemins/*.m."""
import math
from dataclasses import dataclass
import numpy as np


@dataclass
class Path:
    P: np.ndarray        # (N,2) points, premier point NON dupliqué si fermé
    closed: bool
    seg_len: np.ndarray  # (nseg,)
    s_start: np.ndarray  # (nseg,) abscisse au début de chaque segment
    seg_psi: np.ndarray  # (nseg,) cap de chaque segment
    length: float        # longueur totale (= S0 si P est la référence source)

    @property
    def nseg(self):
        return len(self.seg_len)

    def seg_end(self, i):
        return self.P[(i + 1) % len(self.P)]


def make_path(P, closed):
    P = np.asarray(P, dtype=float)
    Q = np.vstack([P, P[:1]]) if closed else P
    d = np.diff(Q, axis=0)
    seg_len = np.hypot(d[:, 0], d[:, 1])
    if not np.all(np.isfinite(P)):
        raise ValueError("valeurs non finies dans le chemin")
    if np.any(seg_len <= 1e-12):
        raise ValueError("segment de longueur nulle (point dupliqué ?)")
    s_start = np.concatenate([[0.0], np.cumsum(seg_len)[:-1]])
    return Path(P, closed, seg_len, s_start, np.arctan2(d[:, 1], d[:, 0]),
                float(seg_len.sum()))


# ---------- Références sources de l'annexe A.1 ----------
def source_reference(name, csv_dir=None):
    if name == "droite":
        j = np.arange(401)
        return make_path(np.c_[0.05 * j, np.zeros_like(j, float)], closed=False)
    if name in ("cercle", "huit"):
        u = 2 * np.pi * np.arange(10000) / 10000
        if name == "cercle":
            P = np.c_[5 * np.cos(u), 5 * np.sin(u)]
        else:
            P = np.c_[8 * np.sin(u), 4 * np.sin(2 * u)]
        return make_path(P, closed=True)
    if name in ("Spielberg", "Silverstone"):
        import os
        f = os.path.join(csv_dir, name, f"{name}_centerline.csv")
        data = np.loadtxt(f, delimiter=",", comments="#")
        return make_path(data[:, :2], closed=True)  # ordre du CSV conservé
    raise ValueError(name)


def initial_pose(name, ref: Path, recovery=False):
    """Pose nominale (tableau 1 / annexe A.3), éventuellement perturbée."""
    nominal = {"droite": (0.0, 0.0, 0.0),
               "cercle": (5.0, 0.0, math.pi / 2),
               "huit": (0.0, 0.0, math.pi / 4)}
    if name in nominal:
        x, y, psi = nominal[name]
    else:
        a0, a1 = ref.P[0], ref.P[1]
        x, y, psi = a0[0], a0[1], math.atan2(a1[1] - a0[1], a1[0] - a0[0])
    if recovery:
        x, y = x - 0.5 * math.sin(psi), y + 0.5 * math.cos(psi)
        psi = (psi + math.pi / 18 + math.pi) % (2 * math.pi) - math.pi
    return (x, y, psi)


# ---------- Projection avec progression ----------
@dataclass
class Progress:
    seg: int = 0        # segment courant
    s: float = 0.0      # abscisse dans [0, length[
    s_tot: float = 0.0  # abscisse déroulée (croît au-delà de length)


def _window_segments(path: Path, prog, back, fwd):
    """Indices des segments qui recouvrent [s-back, s+fwd] autour de la
    projection précédente (abscisses relatives ramenées dans [-L/2, L/2[
    pour un chemin fermé)."""
    rs = path.s_start - prog.s
    if path.closed:
        rs = (rs + path.length / 2) % path.length - path.length / 2
    re = rs + path.seg_len
    mask = (re >= -back) & (rs <= fwd)
    mask[prog.seg] = True
    return np.flatnonzero(mask)


def project(path: Path, p, prog: Progress, back=1.0, fwd=3.0):
    """Projette p sur les segments de la fenêtre. Retourne
    (nouvel état, point projeté, cap tangent, écart signé n^T(p - p_proj))."""
    idx = _window_segments(path, prog, back, fwd)
    A = path.P[idx]
    B = path.P[(idx + 1) % len(path.P)]
    AB = B - A
    t = np.clip(((p[0] - A[:, 0]) * AB[:, 0] + (p[1] - A[:, 1]) * AB[:, 1])
                / path.seg_len[idx] ** 2, 0.0, 1.0)
    C = A + t[:, None] * AB
    d2 = (p[0] - C[:, 0]) ** 2 + (p[1] - C[:, 1]) ** 2
    k = int(np.argmin(d2))
    j = int(idx[k])
    s_new = path.s_start[j] + t[k] * path.seg_len[j]
    ds = s_new - prog.s
    if path.closed:                          # passage par s = 0
        ds = (ds + path.length / 2) % path.length - path.length / 2
    new = Progress(j, s_new, prog.s_tot + ds)
    psi = path.seg_psi[j]
    e_lat = -math.sin(psi) * (p[0] - C[k, 0]) + math.cos(psi) * (p[1] - C[k, 1])
    return new, C[k], psi, e_lat


def point_at(path: Path, s_tot):
    """Point et cap à l'abscisse déroulée s_tot (pour les cibles de Pure Pursuit).
    Sur un chemin ouvert, s est borné à [0, length]."""
    if path.closed:
        s = s_tot % path.length
    else:
        s = min(max(s_tot, 0.0), path.length)
    j = int(np.searchsorted(path.s_start, s, side="right") - 1)
    j = min(max(j, 0), path.nseg - 1)
    t = (s - path.s_start[j]) / path.seg_len[j]
    A, B = path.P[j], path.seg_end(j)
    return A + t * (B - A), path.seg_psi[j], j
