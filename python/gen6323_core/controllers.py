"""Lois géométriques (section 2.3). Fonctions pures : aucune E/S."""
import math
from .path import project, point_at, Progress
from .vehicle import wrap


def pure_pursuit(q, v, path, prog, L, l0=0.6, tau=0.3):
    """Cible à l'abscisse s_arrière + l(v), l(v) = l0 + tau*v (avance le long
    du chemin, pas une distance droite). Retourne (delta_geom, prog, cible)."""
    x, y, psi = q
    prog, _, _, _ = project(path, (x, y), prog)
    tgt, _, _ = point_at(path, prog.s_tot + l0 + tau * v)
    dx, dy = tgt[0] - x, tgt[1] - y
    xb = math.cos(psi) * dx + math.sin(psi) * dy
    yb = -math.sin(psi) * dx + math.cos(psi) * dy
    d2 = xb * xb + yb * yb
    kappa = 0.0 if d2 < 1e-12 else 2.0 * yb / d2
    return math.atan(L * kappa), prog, tgt


def stanley(q, v, path, prog_f, L, ks=2.0, v0=1.0):
    """Erreurs au point avant (équation 3). prog_f : progression du point avant."""
    x, y, psi = q
    pf = (x + L * math.cos(psi), y + L * math.sin(psi))
    prog_f, pg, psi_g, ef = project(path, pf, prog_f)
    epsi = wrap(psi - psi_g)
    return -epsi - math.atan(ks * ef / (max(v, 0.0) + v0)), prog_f, pg
