"""La logique du nœud, sans ROS, reproduit la campagne MATLAB (étape 7, réglages gelés)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from gen6323_controle.logique import SuiviPP   # noqa: E402
from gen6323_core.path import Progress, initial_pose, project, source_reference   # noqa: E402
from gen6323_core.vehicle import Limits, step   # noqa: E402


@pytest.mark.parametrize('nom, v, T_ml, erms_ml, Jd_ml', [
    ('huit', 2, 24.71, 0.0061176711, 0.0054445300),
    ('cercle', 3, 10.97, 0.0000010486, 0.0187062438),
])
def test_logique_contre_matlab(nom, v, T_ml, erms_ml, Jd_ml):
    lim = Limits(); ref = source_reference(nom); q = initial_pose(nom, ref)
    L = SuiviPP(ref, ref, 0.3, 0.2, v, lim)
    _, _, _, e0 = project(ref, q[:2], Progress())
    ts, e, dc, v_app = [0.0], [abs(e0)], [0.0], 0.0
    for _ in range(20000):
        r = L.commande(q[0], q[1], q[2], v_app)
        if r['termine']:
            break
        q = step(q, r['vc'], r['dc'], lim); v_app = r['vc']
        ts.append(ts[-1] + lim.dt); dc.append(r['dc'])
        _, _, _, ek = project(ref, q[:2], Progress(L.prog_ref.seg, L.prog_ref.s, L.prog_ref.s_tot))
        e.append(abs(ek))
    ts, e = np.array(ts), np.array(e); dt = np.diff(ts); T = ts[-1]
    assert abs(T - T_ml) <= 0.006
    assert abs(math.sqrt(np.sum(e[1:] ** 2 * dt) / T) - erms_ml) <= 1e-6
    assert abs(np.sum(np.diff(dc) ** 2 / dt) / T - Jd_ml) <= 1e-6


def test_arret_securite_vitesse_nulle_immediate():
    ref = source_reference('cercle'); L = SuiviPP(ref, ref, 0.3, 0.2, 2.0)
    for _ in range(100):
        L.commande(5.0, 0.0, math.pi / 2, 0.0)
    assert L.vc > 0.5
    L.arret_securite('test')
    assert L.commande(5.0, 0.0, math.pi / 2, 1.0)['vc'] == 0.0
