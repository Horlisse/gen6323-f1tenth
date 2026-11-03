"""Étape 4 : import et préparation des circuits F1TENTH (jumeau de matlab/circuits/*.m)."""
import math
import os
import re

import numpy as np

from .path import make_path

L_WHEELBASE = 0.3302
DELTA_MAX = 0.4189
VEHICLE_WIDTH = 0.31          # largeur F1TENTH (paramètres « f1tenth » de Gym)
KAPPA_MAX = math.tan(DELTA_MAX) / L_WHEELBASE   # courbure maximale réalisable (~1,35 m^-1)

EXPECTED = {   # annexe A.1 et A.3 de l'énoncé
    'Spielberg': {'n': 864, 'S0': 343.323, 'psi0': -2.87898},
    'Silverstone': {'n': 1178, 'S0': 457.925, 'psi0': 0.94440},
}


# ---------------------------------------------------------------- lecture
def read_centerline(root, name):
    """Colonnes [x, y, w_droite, w_gauche] ; lignes commençant par # ignorées ; ordre conservé."""
    f = os.path.join(root, name, f'{name}_centerline.csv')
    return np.loadtxt(f, delimiter=',', comments='#', ndmin=2)


def read_map(root, name):
    """Carte au format ROS map_server : YAML (resolution, origin, seuils) + image."""
    ypath = os.path.join(root, name, f'{name}_map.yaml')
    text = open(ypath, encoding='utf-8').read()

    def val(key, default=None):
        m = re.search(rf'^\s*{key}\s*:\s*(.+?)\s*$', text, re.M)
        return m.group(1).strip().strip('\'"') if m else default

    origin = [float(v) for v in re.findall(r'[-+0-9.eE]+', val('origin'))]
    meta = {
        'image': val('image'), 'resolution': float(val('resolution')),
        'origin': origin, 'negate': int(float(val('negate', 0))),
        'occupied_thresh': float(val('occupied_thresh', 0.65)),
        'free_thresh': float(val('free_thresh', 0.196)),
    }
    from PIL import Image
    img = np.asarray(Image.open(os.path.join(os.path.dirname(ypath), meta['image'])).convert('L'), dtype=float)
    p = img / 255.0 if meta['negate'] else (255.0 - img) / 255.0     # probabilité d'occupation
    meta['free'] = p < meta['free_thresh']
    meta['H'], meta['W'] = img.shape
    return meta


def world_to_pixel(meta, x, y):
    """Ligne et colonne (indices 0) du pixel contenant (x, y). La ligne 0 est le HAUT de l'image."""
    res, (ox, oy) = meta['resolution'], meta['origin'][:2]
    col = np.floor((np.asarray(x) - ox) / res).astype(int)
    row = meta['H'] - 1 - np.floor((np.asarray(y) - oy) / res).astype(int)
    return row, col


def points_free(meta, x, y):
    row, col = world_to_pixel(meta, x, y)
    inside = (row >= 0) & (row < meta['H']) & (col >= 0) & (col < meta['W'])
    free = np.zeros(len(row), bool)
    free[inside] = meta['free'][row[inside], col[inside]]
    return free


# ---------------------------------------------------------------- analyse
def quality(P, closed=True):
    """Statistiques utiles pour décider d'un prétraitement."""
    Q = np.vstack([P, P[:1]]) if closed else P
    d = np.diff(Q, axis=0)
    seg = np.hypot(d[:, 0], d[:, 1])
    psi = np.arctan2(d[:, 1], d[:, 0])
    dpsi = (np.diff(np.r_[psi, psi[:1]] if closed else psi) + np.pi) % (2 * np.pi) - np.pi
    lmid = 0.5 * (seg + np.roll(seg, -1)) if closed else 0.5 * (seg[:-1] + seg[1:])
    kappa = dpsi / lmid
    area = 0.5 * np.sum(P[:, 0] * np.roll(P[:, 1], -1) - np.roll(P[:, 0], -1) * P[:, 1])
    return {
        'n': len(P), 'S0': float(seg.sum()),
        'seg_min': float(seg.min()), 'seg_mean': float(seg.mean()), 'seg_max': float(seg.max()),
        'dpsi_max': float(np.abs(dpsi).max()), 'kappa_max': float(np.abs(kappa).max()),
        'dkappa_max': float(np.abs(np.diff(kappa)).max()),
        'sens': 'antihoraire' if area > 0 else 'horaire',
        'psi_seg': psi, 'dpsi': dpsi, 'kappa': kappa, 'seg': seg,
    }


# ---------------------------------------------------------------- prétraitement
def resample_spline(P, ds=0.10):
    """Spline cubique PÉRIODIQUE paramétrée par l'abscisse curviligne (corde),
    rééchantillonnée à pas ~ds. Périodicité obtenue en répliquant un tour avant et après."""
    from scipy.interpolate import CubicSpline
    Q = np.vstack([P, P[:1]])
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(Q, axis=0).T))]
    S = s[-1]
    cs = CubicSpline(s, Q, bc_type='periodic')
    n = max(int(round(S / ds)), 3)
    u = np.linspace(0, S, n, endpoint=False)
    return cs(u)


def deviation(A, B, closed=True):
    """Distance de chaque point de A à la ligne brisée B (projection sur les segments)."""
    Bq = np.vstack([B, B[:1]]) if closed else B
    a, b = Bq[:-1], Bq[1:]
    ab = b - a
    l2 = (ab ** 2).sum(1)
    out = np.empty(len(A))
    for i, p in enumerate(A):
        t = np.clip(((p - a) * ab).sum(1) / l2, 0, 1)
        c = a + t[:, None] * ab
        out[i] = np.sqrt(((c - p) ** 2).sum(1)).min()
    return out


# ---------------------------------------------------------------- export (annexe A.4)
def export_path(P, closed, fname):
    """Colonnes s_m,x_m,y_m,psi_rad ; 17 chiffres significatifs ; cap par différence centrée."""
    n = len(P)
    if closed:
        nxt, prv = np.roll(P, -1, 0), np.roll(P, 1, 0)
    else:
        nxt = np.vstack([P[1:], P[-1:]])
        prv = np.vstack([P[:1], P[:-1]])
    psi = np.arctan2(nxt[:, 1] - prv[:, 1], nxt[:, 0] - prv[:, 0])
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    with open(fname, 'w', newline='\n') as f:
        f.write('s_m,x_m,y_m,psi_rad\n')
        for i in range(n):
            f.write(f'{s[i]:.17g},{P[i,0]:.17g},{P[i,1]:.17g},{psi[i]:.17g}\n')
    return make_path(P, closed)


def resample_linear(P, ds=0.10):
    """Rééchantillonnage à pas uniforme le long de la ligne brisée fermée (aucun lissage)."""
    Q = np.vstack([P, P[:1]])
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(Q, axis=0).T))]
    n = max(int(round(s[-1] / ds)), 3)
    u = np.linspace(0, s[-1], n, endpoint=False)
    return np.c_[np.interp(u, s, Q[:, 0]), np.interp(u, s, Q[:, 1])]


def smooth_gaussian(P, sigma=0.5, ds=0.10):
    """LISSAGE : rééchantillonnage uniforme puis filtre gaussien CIRCULAIRE (chemin fermé)
    d'écart type sigma [m] sur x et y. Atténue le bruit de numérisation de longueur d'onde
    << sigma ; déplace légèrement la ligne vers l'intérieur des virages (écart à quantifier)."""
    R = resample_linear(P, ds)
    n = len(R)
    half = int(np.ceil(4 * sigma / ds))
    k = np.exp(-0.5 * (np.arange(-half, half + 1) * ds / sigma) ** 2)
    k /= k.sum()
    out = np.empty_like(R)
    for j in range(2):
        ext = np.r_[R[-half:, j], R[:, j], R[:half, j]]        # bouclage
        out[:, j] = np.convolve(ext, k, mode='valid')
    return out
