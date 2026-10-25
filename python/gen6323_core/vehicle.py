"""Modèle cinématique Ackermann (point de référence : milieu de l'essieu arrière)
et saturations des consignes. Jumeau Python de matlab/vehicule/*.m.
Aucune dépendance ROS : ce module est réutilisé tel quel dans le nœud ROS 2."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Limits:
    L: float = 0.3302          # empattement [m]
    v_max: float = 5.0         # [m/s]
    delta_max: float = 0.4189  # [rad]
    a_max: float = 3.0         # |dv_c/dt| [m/s^2]
    ddelta_max: float = 3.2    # |ddelta_c/dt| [rad/s]
    dt: float = 0.01           # [s]


def wrap(a):
    """Ramène un angle dans [-pi, pi[."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def saturate(v_des, delta_des, v_prev, delta_prev, lim: Limits, dt=None):
    """Amplitude puis variation. Retourne (v_c, delta_c, flags) où flags indique
    quelle limite est active (pour la fraction de temps demandée en 3.3)."""
    dt = lim.dt if dt is None else dt
    v_a = _clip(v_des, 0.0, lim.v_max)
    d_a = _clip(delta_des, -lim.delta_max, lim.delta_max)
    dv = _clip(v_a - v_prev, -lim.a_max * dt, lim.a_max * dt)
    dd = _clip(d_a - delta_prev, -lim.ddelta_max * dt, lim.ddelta_max * dt)
    v_c, d_c = v_prev + dv, delta_prev + dd
    flags = {
        "v_amp": v_a != v_des,
        "d_amp": d_a != delta_des,
        "v_rate": dv != v_a - v_prev,
        "d_rate": dd != d_a - delta_prev,
    }
    return v_c, d_c, flags


def step(q, v, delta, lim: Limits, dt=None, method="exact"):
    """Un pas du modèle (1) à commande constante sur [t, t+dt].
    q = (x, y, psi). method = 'exact' (arc exact) ou 'euler'."""
    dt = lim.dt if dt is None else dt
    x, y, psi = q
    w = v * math.tan(delta) / lim.L
    if method == "euler":
        return (x + v * math.cos(psi) * dt,
                y + v * math.sin(psi) * dt,
                psi + w * dt)
    psi1 = psi + w * dt
    if abs(w * dt) < 1e-9:  # quasi rectiligne : développement limité au 2e ordre
        xm = x + v * dt * math.cos(psi + 0.5 * w * dt)
        ym = y + v * dt * math.sin(psi + 0.5 * w * dt)
        return (xm, ym, psi1)
    return (x + v / w * (math.sin(psi1) - math.sin(psi)),
            y - v / w * (math.cos(psi1) - math.cos(psi)),
            psi1)
