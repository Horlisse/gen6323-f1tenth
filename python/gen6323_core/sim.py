"""Boucle d'essai MATLAB-équivalente (actionnement idéal v = v_c, delta = delta_c)."""
import math
import numpy as np
from .vehicle import Limits, saturate, step
from .path import Progress, project
from .controllers import pure_pursuit, stanley


def speed_profile(v_target, s_rem, closed, a_brake=2.5):
    if closed:
        return v_target
    if s_rem < 0.03:
        return 0.0
    return min(v_target, math.sqrt(2 * a_brake * s_rem))


def run_trial(ref, path, q0, v_target, law, gains, lim=Limits(), delay=0, noise=None, seed=1):
    """delay : retard de commande en pas de dt (la consigne calculée au pas k est
    appliquée au pas k + delay ; consignes nulles avant).
    noise : (sigma_xy [m], sigma_psi [rad]) bruit gaussien sur la pose MESURÉE par le
    contrôleur ; réalisations pré-tirées avec la graine `seed` (identiques pour les deux lois)."""
    q = q0; v_c = d_c = 0.0; v_app = d_app = 0.0
    prog = Progress(); prog_f = Progress(); prog_ref = Progress()
    T_max = 2 * ref.length / v_target + 10
    n_max = int(T_max / lim.dt) + 2
    rng = np.random.default_rng(seed)
    bruit = rng.standard_normal((n_max, 3))
    if noise:
        bruit *= np.array([noise[0], noise[0], noise[1]])
    else:
        bruit[:] = 0.0
    file = [(0.0, 0.0)] * delay
    log = {"t": [0.0], "e": [], "vc": [0.0], "dc": [0.0], "xy": [q[:2]]}
    prog_ref, _, _, e = project(ref, q[:2], prog_ref); log["e"].append(abs(e))
    status = "timeout"; k = 0
    while k * lim.dt < T_max:
        qm = (q[0] + bruit[k, 0], q[1] + bruit[k, 1], q[2] + bruit[k, 2])   # pose mesurée
        if law == "pp":
            dg, prog, _ = pure_pursuit(qm, v_app, path, prog, lim.L, **gains)
            s_now = prog.s_tot
        else:
            dg, prog_f, _ = stanley(qm, v_app, path, prog_f, lim.L, **gains)
            s_now = prog_ref.s_tot
        v_des = speed_profile(v_target, path.length - s_now, path.closed)
        v_c, d_c, _ = saturate(v_des, dg, v_c, d_c, lim)
        file.append((v_c, d_c))
        v_app, d_app = file.pop(0)            # consigne effectivement appliquée
        q = step(q, v_app, d_app, lim)
        k += 1
        prog_ref, _, _, e = project(ref, q[:2], prog_ref)
        log["t"].append(k * lim.dt); log["e"].append(abs(e))
        log["vc"].append(v_c); log["dc"].append(d_c); log["xy"].append(q[:2])
        if not all(map(math.isfinite, q)):
            status = "non fini"; break
        if abs(e) > 2.0:
            status = "erreur > 2 m"; break
        if ref.closed and prog_ref.s_tot >= ref.length:
            status = "ok"; break
        if not ref.closed and math.hypot(q[0] - 20, q[1]) < 0.2 and v_app < 0.05:
            status = "ok"; break
    t = np.array(log["t"]); e = np.array(log["e"]); dt = np.diff(t)
    T = t[-1] - t[0]
    vc = np.array(log["vc"]); dc = np.array(log["dc"])
    return dict(status=status, T=T,
                e_rms=math.sqrt(np.sum(e[1:] ** 2 * dt) / T),
                e_max=e.max(),
                J_d=np.sum(np.diff(dc) ** 2 / dt) / T,
                J_v=np.sum(np.diff(vc) ** 2 / dt) / T,
                s_frac=prog_ref.s_tot / ref.length, log=log)
